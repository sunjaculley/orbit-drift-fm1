/* SPDX-License-Identifier: GPL-3.0-only */
/* ORBIT DRIFT for the M-VAVE FM-1: a polymetric FM synth and an FM drum machine on one clock, sharing a twin
 * delay and a space reverb. Built from ORBITAL (lanes, FM voice, contour filter), Scenedrift (locks, trig
 * conditions, XY morph, twin delay) and FM-16 (drums, echoes), with sweeps, mod envelopes and fractional ratios.
 *
 *   SEQ           the steps; again: SYNTH / DRUMS (hold SEQ + white key 1-4: bank A-D, waits for the bar while playing)
 *   HOME          the orbit view (again: SYSTEM, the CPU and voice meter)      ARP  PERFORM
 *   EDIT          synth OSC / TONE, drums SOUND / SWEEP     SEL  synth MOD, drums PERF     ENV  envelope
 *   LFO           the LFO: one wave to FILTER, INDEX, PITCH, LEVEL, PAN, DELAY, DRUM PITCH or DRUM LEVEL; free or in time
 *   FX            DELAY / SPACE / BUS (sends) / STEP MOD (a drum track moves the synth)   GLO  GLOBAL / GLOBAL 2
 *   ALGORITHM     synth puck X, on every page               PRESETS  synth puck Y: blends the sounds in corners A-D
 *                 a sound knob turned on a corner changes that corner; turned in between, all four together
 *   SELECT        lane or track (steps held: trig condition)   KNOB1-4  cyan, lime, white, orange on every page
 *
 *   synth         white keys: steps of the lane (tap: off, short, medium, long, longest). OCT- / OCT+: 1-16 / 17-32
 *                 hold steps: KNOB1-4 lock PITCH, VELOCITY, INDEX, SEND; EDIT locks on / off
 *                 black keys F#3..A#4: lanes 1-8 (hold one: KNOB1-4 = LENGTH, DIRECTION, DEGREE, OCTAVE)
 *                 C#5 RANDOM lane (hold 1 s: all)   D#5 CLEAR lane (steps held: their locks)
 *   drums         white keys: steps (tap on / off; hold + KNOB1-4: lock the page's values)
 *                 F#3..D#4: KICK SNARE HAT PERC CLAP (tap: play; REC on while playing: record). Hold one: KNOB1 LENGTH
 *                 F#4 COPY  G#4 PASTE  A#4 CLEAR LOCKS (held steps)  C#5 RANDOM  D#5 CLEAR   OCT- / OCT+ track
 *   F#5           hold + lane or track key: mute
 *   perform       F#3..C#4 corners A-D (tap: jump; SAVE + tap: store)  C#5 intensity to centre  D#5 puck to centre
 *                 white 9-13 drum mutes, 14 synth mute, 15 delay freeze (hold), 16 unmute all
 *   hold FX       effects on the white keys, mutes on the black, the Perform macros on the knobs
 *   hold SEQ      banks and lane / track tools; hold GLO: tap, restart, solo, tempo on SELECT
 *   hold LFO      turn a knob: it springs back when LFO is let go     hold SAVE: undo (again: redo)
 *   SAVE          store in flash (stopped)   SAVE + D#5: INIT
 *
 * For the chip: integers only (no FPU), no 64-bit division. Up to 6 drum voices and 4-10 synth voices: the synth
 * starts at 6 and the CPU guard adds or takes away two at a time from the meter in the header. */
#include "fm1.h"

#define LANES 8
#define MAXSTEP 32
#define TRACKS 5
#define DSTEPS 16
#define BANKS 4
#define SUB 16
#define MAXSV 10
#define DV 6
#define EVQ 48
#define DLEN 32768u
#define DMASK (DLEN - 1u)

/* ================================================================ settings */

enum { K_ALGO, K_RATIO, K_INDEX, K_FB, K_CUT, K_CONTOUR, K_RESO, K_DRIVE, K_MDEC, K_MEND, K_ATK, K_HOLD, K_REL, K_SUB,
       K_DTA, K_DTB, K_DFB, K_DMODE, K_SPACE, K_TRAIL, K_DIRT, K_MIX, K_SSD, K_SSS, K_SDD, K_SDS,
       K_BPM, K_SWING, K_SCALE, K_ROOT, K_SYNC, K_INT, K_KIT, K_THROW, K_FILT,
       K_LDEST, K_LRATE, K_LDEPTH, K_LSHAPE, K_MSRC, K_MDEST, K_MSHAPE, K_MDEPTH,
       K_MATK, K_MVEL, K_KTRK, K_MLOOP, K_OP3, K_DRIFT, K_UNI, K_UDET, K_USPR, K_UMIX, K_TLEN, K_TEVERY, K_TCUR, K_SONG, NK };
enum { U_ALGO, U_FRAC, U_DIV10, U_PCT, U_MS, U_SEC100, U_HZ, U_SCALE, U_NOTE, U_INT, U_SYNC, U_CENTER, U_DEST, U_LRATE, U_SHAPE, U_TRACK, U_DMODE, U_LOOP, U_CENTS, U_TLEN, U_EVERY, U_STOP, U_ONOFF };
typedef struct {
  int16_t min, max, step;
  uint8_t unit;
  const char *label;
} kdef_t;
static const kdef_t KD[NK] = {
  {0, 2, 1, U_ALGO, "ALGO"},       {0, 19, 1, U_FRAC, "RATIO"},     {0, 120, 1, U_DIV10, "INDEX"},
  {0, 100, 1, U_PCT, "FEEDBK"},    {0, 100, 1, U_HZ, "CUTOFF"},     {0, 100, 1, U_PCT, "CONTOUR"},
  {3, 140, 1, U_DIV10, "RESO"},    {0, 100, 1, U_PCT, "DRIVE"},     {10, 200, 2, U_PCT, "MOD DEC"},
  {0, 100, 1, U_PCT, "MOD END"},   {1, 300, 1, U_MS, "ATTACK"},     {0, 500, 5, U_MS, "HOLD"},
  {5, 250, 1, U_SEC100, "RELEASE"}, {0, 100, 1, U_PCT, "SUB"},
  {20, 740, 5, U_MS, "TIME A"},    {20, 740, 5, U_MS, "TIME B"},    {0, 90, 1, U_PCT, "FEEDBK"},
  {0, 1, 1, U_DMODE, "MODE"},       {0, 100, 1, U_PCT, "SPACE"},     {0, 100, 1, U_PCT, "TRAIL"},
  {0, 100, 1, U_PCT, "DIRT"},      {0, 100, 1, U_PCT, "MIX"},       {0, 100, 1, U_PCT, "S>DLY"},
  {0, 100, 1, U_PCT, "S>SPC"},   {0, 100, 1, U_PCT, "D>DLY"},   {0, 100, 1, U_PCT, "D>SPC"},
  {40, 220, 1, U_INT, "TEMPO"},    {0, 75, 1, U_PCT, "SWING"},      {0, 3, 1, U_SCALE, "SCALE"},
  {0, 11, 1, U_NOTE, "ROOT"},      {0, 1, 1, U_SYNC, "SYNC"},       {0, 100, 1, U_CENTER, "INTENS"},
  {0, 100, 1, U_CENTER, "KIT"},    {0, 100, 1, U_PCT, "THROW"},     {0, 100, 1, U_CENTER, "FILTER"},
  {0, 8, 1, U_DEST, "DEST"},       {0, 108, 1, U_LRATE, "RATE"},    {0, 100, 1, U_PCT, "DEPTH"},
  {0, 3, 1, U_SHAPE, "SHAPE"},     {0, 4, 1, U_TRACK, "SOURCE"},    {0, 8, 1, U_DEST, "DEST"},
  {20, 800, 10, U_MS, "DECAY"},    {0, 100, 1, U_PCT, "DEPTH"},
  {0, 300, 2, U_MS, "ATTACK"},     {0, 100, 1, U_PCT, "VEL>IDX"},   {0, 100, 1, U_PCT, "KEYTRK"},
  {0, 4, 1, U_LOOP, "LOOP"},       {0, 200, 2, U_PCT, "OP3"},       {0, 100, 1, U_PCT, "DRIFT"},
  {1, 3, 1, U_INT, "VOICES"},      {0, 50, 1, U_CENTS, "DETUNE"},   {0, 100, 1, U_PCT, "SPREAD"},
  {0, 100, 1, U_PCT, "MIX"},       {0, 8, 1, U_TLEN, "LENGTH"},     {0, 3, 1, U_EVERY, "EVERY"},
  {0, 7, 1, U_STOP, "STOP"},       {0, 1, 1, U_ONOFF, "SONG"},
};
static const int16_t KDEFAULT[NK] = {0,  11, 35, 10, 65, 50, 15, 15, 90, 10, 6,   20, 60, 20, 270, 400, 40, 0,
                                     60, 55, 10, 30, 50, 40, 15, 25, 120, 0, 1,  2,  0,  50, 50, 0,   50, 1, 40, 0, 0, 0, 4, 200, 0,
                                     0, 30, 30, 0, 100, 0, 1, 12, 30, 70, 0, 0, 0, 0};

/* the XY morph blends these between the four corners */
#define NMORPH 10
static const uint8_t MORPH_K[NMORPH] = {K_RATIO, K_INDEX, K_FB, K_DRIVE, K_CUT, K_CONTOUR, K_RESO, K_REL, K_MEND, K_SUB};
static const int16_t CORNER_DEFAULT[4][NMORPH] = {
  {11, 35, 10, 15, 65, 50, 15, 60, 10, 20},  /* A: the starting sound */
  {5, 20, 0, 5, 85, 20, 8, 140, 40, 0},      /* B: bright, long, mellow */
  {15, 70, 30, 30, 40, 80, 40, 35, 0, 35},   /* C: dark bell, deep contour */
  {8, 100, 60, 70, 55, 65, 90, 25, 20, 50},  /* D: hard, resonant */
};

/* fractional ratios: every stop is a musical interval */
#define NFRAC 20
static const uint32_t FRAC_Q16[NFRAC] = {16384,  21845,  32768,  43691,  49152,  65536,  81920,
                                         87381,  98304,  109227, 114688, 131072, 163840, 196608,
                                         229376, 262144, 327680, 393216, 458752, 524288};
static const char *const FRAC_NAME[NFRAC] = {"1/4", "1/3", "1/2", "2/3", "3/4", "1",   "5/4",
                                             "4/3", "3/2", "5/3", "7/4", "2",   "5/2", "3",
                                             "7/2", "4",   "5",   "6",   "7",   "8"};

static const uint8_t SCALE_LEN[4] = {7, 7, 7, 5};
static const uint8_t SCALES[4][7] = {
  {0, 2, 3, 5, 7, 8, 10}, {0, 2, 3, 5, 7, 9, 10}, {0, 1, 3, 5, 7, 8, 10}, {0, 2, 4, 7, 9, 0, 0},
};
static const char *const SCALE_NAME[4] = {"MINOR", "DORIAN", "PHRYG", "PENT"};
static const char *const NOTE_NAME[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
static const char *const DIR_NAME[4] = {"FWD", "BACK", "PONG", "RAND"};
static const uint16_t DUR_MULT[4] = {50, 100, 180, 320}; /* x100 */

/* trig conditions: always, chances, then A:B (plays on pass A of every B) */
#define NCOND 16
static const char *const COND_NAME[NCOND] = {"ALL", "10%", "30%", "50%", "70%", "90%", "1:2", "2:2",
                                             "1:3", "2:3", "3:3", "1:4", "2:4", "3:4", "4:4", "FILL"};
static const uint8_t COND_A[NCOND] = {0, 10, 30, 50, 70, 90, 1, 2, 1, 2, 3, 1, 2, 3, 4, 0};
static const uint8_t COND_B[NCOND] = {0, 0, 0, 0, 0, 0, 2, 2, 3, 3, 3, 4, 4, 4, 4, 0};

/* drums */
enum { P_CAR, P_RAT, P_IDX, P_PITCH, P_SWD, P_SWT, P_MDEC, P_MEND, P_ATK, P_REL, P_ACC, P_PROB, P_ECHO, P_EDEC, P_WAIT, DP };
enum { F_NONE, F_FRAC, F_DIV10, F_PITCH, F_SEMI, F_MS2, F_MS5, F_PCT, F_MS, F_ACC, F_PCT5, F_INT, F_HZ50, F_NTYPE, F_TICK };
typedef struct {
  uint8_t min, max, kind;
  const char *label;
} pdef_t;
static const pdef_t FM_DEF[DP] = {
  {0, 19, F_FRAC, "CARRIER"}, {0, 19, F_FRAC, "RATIO"},    {0, 100, F_DIV10, "INDEX"}, {0, 24, F_PITCH, "PITCH"},
  {0, 48, F_SEMI, "SWEEP"},   {1, 100, F_MS2, "SW TIME"},  {1, 100, F_MS5, "MOD DEC"}, {0, 100, F_PCT, "MOD END"},
  {1, 150, F_MS, "ATTACK"},   {4, 180, F_MS5, "RELEASE"},  {10, 40, F_ACC, "ACCENT"},  {0, 20, F_PCT5, "PROB"},
  {0, 4, F_INT, "ECHO"},      {0, 18, F_PCT5, "E.DECAY"},  {0, 5, F_TICK, "WAIT"},
};
static const pdef_t NZ_DEF[DP] = {
  {4, 180, F_HZ50, "FILTER"}, {1, 150, F_DIV10, "RESO"},   {0, 4, F_NTYPE, "TYPE"},    {0, 0, F_NONE, ""},
  {0, 0, F_NONE, ""},         {0, 0, F_NONE, ""},          {0, 0, F_NONE, ""},         {0, 0, F_NONE, ""},
  {1, 100, F_MS, "ATTACK"},   {4, 100, F_MS5, "RELEASE"},  {10, 40, F_ACC, "ACCENT"},  {0, 20, F_PCT5, "PROB"},
  {0, 4, F_INT, "ECHO"},      {0, 18, F_PCT5, "E.DECAY"},  {0, 5, F_TICK, "WAIT"},
};
static const char *const NTYPE_NAME[5] = {"WHITE", "GB", "METAL", "GRAIN", "RESO"};
static const char *const TRACK_NAME[TRACKS] = {"KICK", "SNARE", "HAT", "PERC", "CLAP"};
static const uint16_t TRACK_HZ[TRACKS] = {80, 200, 600, 340, 0};
#define IS_NOISE(t) ((t) == 4)
static const pdef_t *defs(int t) { return IS_NOISE(t) ? NZ_DEF : FM_DEF; }
static const uint8_t DDEFAULT[TRACKS][DP] = {
  /* car rat idx pitch swd swt mdec mend atk rel acc prob echo edec */
  {5, 2, 20, 12, 24, 20, 15, 0, 1, 44, 20, 20, 0, 10, 0},
  {5, 13, 50, 12, 7, 10, 20, 20, 1, 28, 20, 20, 0, 10, 0},
  {11, 18, 60, 12, 0, 5, 10, 30, 1, 12, 20, 20, 0, 10, 0},
  {10, 12, 40, 12, 12, 8, 30, 10, 1, 40, 20, 20, 0, 10, 0},
  {80, 10, 0, 0, 0, 1, 1, 0, 2, 24, 20, 20, 0, 10, 0},
};
static const uint8_t DVOLUME[TRACKS] = {85, 78, 66, 72, 74};
#define UNLOCKED 255

/* ---- what survives power-off */
typedef struct {
  uint8_t dc; /* bits 0-2 length (0 off, 1-4), bit 3 locked, bits 4-7 trig condition */
  int8_t pitch;
  uint8_t vel;
  int8_t index;
  uint8_t send;
} sstep_t;
typedef struct {
  uint8_t len, dir, deg, oct; /* oct: 0..4 = -2..+2 */
} lane_t;
typedef struct {
  sstep_t st[LANES][MAXSTEP];
  lane_t lane[LANES];
  uint16_t on[TRACKS];
  uint8_t dlen[TRACKS];
  uint8_t dcond[TRACKS][DSTEPS];
  uint8_t lock[TRACKS][DSTEPS][DP];
  uint8_t xb[LANES][MAXSTEP]; /* lock set B: bits 0-2 wait (ticks late), 3-4 echoes, 5-6 ladder, 7 slow fade */
} bank_t;
typedef struct {
  uint8_t used, xy[2], mac[4], dmute, smute, smut;
} snap_t;

#define MAGIC_07 0x4F440005u /* 0.7: the same layout up to song_mute */
#define MAGIC_08 0x4F440006u /* 0.8: the same layout up to mix */
#define MAGIC_09 0x4F440007u /* 0.9: the same layout up to pump */
#define MAGIC 0x4F440008u
typedef struct {
  uint32_t magic;
  int16_t k[NK];
  int16_t corner[4][NMORPH];
  uint8_t xy[2], pad[2];
  uint8_t def[TRACKS][DP];
  uint8_t vol[TRACKS];
  int8_t tseq[8];                          /* the transpose sequence, in scale steps */
  uint8_t song_n, song_bank[16], song_rep[16];
  snap_t snap[8];
  bank_t bank[BANKS];
  /* 0.8, after the banks so a 0.7 save still loads */
  uint8_t song_mute[16]; /* per song block: bits 0-4 drum tracks, 5 the synth */
  uint8_t path_len, pad3[3];
  uint8_t path[128][2]; /* the morph path, a puck position each step */
  /* 0.9 */
  uint8_t mix[4];  /* levels 0-127 (100 = 0 dB): synth, drums, master */
  uint8_t llev[8]; /* each synth lane */
  /* 0.10 */
  uint8_t pump[4]; /* depth 0-100, time 0-100, source 0 kick / 1 beat, target 0 synth / 1 synth + its sends */
} store_t;
static store_t S;
static volatile uint8_t pump_hit; /* a kick (or the beat) since the pump last looked */
#define OLD07_SIZE 11860u
#define OLD08_SIZE 12136u
_Static_assert(__builtin_offsetof(store_t, mix) == OLD08_SIZE, "0.8 layout must stay a prefix");
#define OLD09_SIZE 12148u
_Static_assert(__builtin_offsetof(store_t, pump) == OLD09_SIZE, "0.9 layout must stay a prefix");
static void mix_defaults(void) {
  S.mix[0] = S.mix[1] = S.mix[2] = 100, S.mix[3] = 0;
  for (int l = 0; l < 8; l++) S.llev[l] = 100;
  S.pump[0] = 0, S.pump[1] = 30, S.pump[2] = 0, S.pump[3] = 0;
}
_Static_assert(__builtin_offsetof(store_t, song_mute) == OLD07_SIZE, "0.7 layout must stay a prefix");
#define HEAD_SIZE __builtin_offsetof(store_t, bank) /* settings, sounds, song: what sound undo covers */
_Static_assert(sizeof(store_t) <= SETTINGS_MAX, "settings too big");

#define DUR(st) (((st)->dc & 7) == 7 ? 0 : (st)->dc & 7) /* 7: a trigless step */
#define TRIGLESS(st) (((st)->dc & 7) == 7)
#define LOCKED(st) (((st)->dc >> 3) & 1)
#define COND(st) ((st)->dc >> 4)

static void fill(void *p, uint8_t v, uint32_t n) {
  uint8_t *d = (uint8_t *)p;
  while (n--) *d++ = v;
}
static void clear_sstep(sstep_t *st) {
  st->dc = 0;
  st->pitch = 0;
  st->vel = 85;
  st->index = 0;
  st->send = 50;
}
static void init_bank(bank_t *b) {
  for (int l = 0; l < LANES; l++) {
    for (int s = 0; s < MAXSTEP; s++) clear_sstep(&b->st[l][s]);
    b->lane[l].len = (uint8_t)(8 + l % 3 * 4);
    b->lane[l].dir = 0;
    b->lane[l].deg = (uint8_t)l;
    b->lane[l].oct = 2;
  }
  for (int t = 0; t < TRACKS; t++) {
    b->on[t] = 0;
    b->dlen[t] = DSTEPS;
  }
  fill(b->dcond, 0, sizeof b->dcond);
  fill(b->lock, UNLOCKED, sizeof b->lock);
  fill(b->xb, 0, sizeof b->xb);
}
static void init_all(void) {
  fill(&S, 0, sizeof S);
  S.magic = MAGIC;
  mix_defaults();
  for (int i = 0; i < NK; i++) S.k[i] = KDEFAULT[i];
  for (int c = 0; c < 4; c++)
    for (int i = 0; i < NMORPH; i++) S.corner[c][i] = CORNER_DEFAULT[c][i];
  S.xy[0] = S.xy[1] = 0; /* on corner A */
  for (int t = 0; t < TRACKS; t++) {
    S.vol[t] = DVOLUME[t];
    for (int p = 0; p < DP; p++) S.def[t][p] = DDEFAULT[t][p];
  }
  for (int b = 0; b < BANKS; b++) init_bank(&S.bank[b]);
  /* a starting point in bank A: a few synth notes and a beat */
  bank_t *b = &S.bank[0];
  static const uint8_t SEED[LANES][4] = {{0, 4, 9, 255}, {2, 7, 255, 255}, {0, 6, 255, 255}, {3, 11, 255, 255},
                                         {5, 255, 255, 255}, {1, 9, 255, 255}, {255, 255, 255, 255}, {7, 255, 255, 255}};
  static const uint8_t SEEDLEN[4] = {1, 2, 3, 2};
  for (int l = 0; l < LANES; l++)
    for (int i = 0; i < 4; i++)
      if (SEED[l][i] != 255 && SEED[l][i] < b->lane[l].len) b->st[l][SEED[l][i]].dc = SEEDLEN[(l + i) & 3];
  b->on[0] = 1u << 0 | 1u << 4 | 1u << 8 | 1u << 12;
  b->on[1] = 1u << 4 | 1u << 12;
  b->on[2] = 1u << 2 | 1u << 6 | 1u << 10 | 1u << 14;
  b->on[4] = 1u << 12;
  b->lock[2][14][P_IDX] = 90;
  b->lock[2][14][P_REL] = 6;
}

/* ================================================================ tables */

static const int16_t TANH10[1025] = {0,320,640,960,1279,1599,1918,2236,2555,2873,3190,3506,3822,4138,4452,4766,5079,5390,5701,6011,6320,6627,6933,7238,7542,7845,8145,8445,8743,9039,9334,9627,9919,10209,10497,10783,11067,11350,11631,11909,12186,12461,12733,13004,13273,13539,13803,14065,14325,14583,14839,15092,15343,15591,15838,16082,16324,16563,16800,17035,17267,17497,17725,17950,18173,18393,18611,18826,19040,19250,19459,19665,19868,20069,20268,20464,20658,20850,21039,21226,21411,21593,21773,21950,22126,22299,22469,22638,22804,22968,23129,23289,23446,23601,23754,23905,24053,24200,24344,24487,24627,24765,24901,25035,25168,25298,25426,25553,25677,25799,25920,26039,26156,26271,26385,26496,26606,26714,26821,26925,27029,27130,27230,27328,27425,27520,27613,27705,27796,27885,27972,28058,28143,28226,28308,28389,28468,28545,28622,28697,28771,28844,28915,28985,29054,29122,29189,29254,29319,29382,29444,29505,29565,29624,29682,29739,29795,29850,29904,29957,30009,30060,30110,30160,30208,30256,30302,30348,30393,30438,30481,30524,30566,30607,30647,30687,30726,30764,30802,30839,30875,30911,30945,30980,31013,31046,31079,31111,31142,31173,31203,31232,31261,31290,31318,31345,31372,31398,31424,31450,31475,31499,31523,31547,31570,31593,31615,31637,31658,31680,31700,31721,31740,31760,31779,31798,31817,31835,31852,31870,31887,31904,31920,31936,31952,31968,31983,31998,32013,32027,32041,32055,32069,32082,32095,32108,32121,32133,32145,32157,32169,32180,32192,32203,32214,32224,32235,32245,32255,32265,32274,32284,32293,32302,32311,32320,32328,32337,32345,32353,32361,32369,32377,32384,32391,32399,32406,32413,32420,32426,32433,32439,32446,32452,32458,32464,32470,32475,32481,32486,32492,32497,32502,32507,32512,32517,32522,32527,32531,32536,32540,32545,32549,32553,32557,32561,32565,32569,32573,32577,32581,32584,32588,32591,32595,32598,32601,32604,32607,32611,32614,32617,32619,32622,32625,32628,32630,32633,32636,32638,32641,32643,32646,32648,32650,32652,32655,32657,32659,32661,32663,32665,32667,32669,32671,32673,32675,32676,32678,32680,32682,32683,32685,32686,32688,32689,32691,32692,32694,32695,32697,32698,32699,32701,32702,32703,32704,32706,32707,32708,32709,32710,32711,32712,32713,32715,32716,32717,32717,32718,32719,32720,32721,32722,32723,32724,32725,32725,32726,32727,32728,32729,32729,32730,32731,32731,32732,32733,32733,32734,32735,32735,32736,32737,32737,32738,32738,32739,32739,32740,32740,32741,32742,32742,32742,32743,32743,32744,32744,32745,32745,32746,32746,32746,32747,32747,32748,32748,32748,32749,32749,32749,32750,32750,32750,32751,32751,32751,32752,32752,32752,32753,32753,32753,32753,32754,32754,32754,32754,32755,32755,32755,32755,32756,32756,32756,32756,32756,32757,32757,32757,32757,32757,32758,32758,32758,32758,32758,32758,32759,32759,32759,32759,32759,32759,32760,32760,32760,32760,32760,32760,32760,32761,32761,32761,32761,32761,32761,32761,32761,32761,32762,32762,32762,32762,32762,32762,32762,32762,32762,32762,32763,32763,32763,32763,32763,32763,32763,32763,32763,32763,32763,32763,32763,32764,32764,32764,32764,32764,32764,32764,32764,32764,32764,32764,32764,32764,32764,32764,32764,32764,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32765,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32766,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767}; /* tanh(i * 10 / 1024), Q15 */
static const uint32_t EXP2[257] = {32768,32857,32946,33035,33125,33215,33305,33395,33486,33576,33667,33759,33850,33942,34034,34126,34219,34312,34405,34498,34591,34685,34779,34874,34968,35063,35158,35253,35349,35445,35541,35637,35734,35831,35928,36025,36123,36221,36319,36417,36516,36615,36715,36814,36914,37014,37114,37215,37316,37417,37518,37620,37722,37824,37927,38030,38133,38236,38340,38444,38548,38653,38757,38863,38968,39074,39180,39286,39392,39499,39606,39714,39821,39929,40037,40146,40255,40364,40473,40583,40693,40804,40914,41025,41136,41248,41360,41472,41584,41697,41810,41923,42037,42151,42265,42380,42495,42610,42726,42841,42958,43074,43191,43308,43425,43543,43661,43780,43898,44017,44137,44256,44376,44497,44617,44738,44859,44981,45103,45225,45348,45471,45594,45718,45842,45966,46091,46216,46341,46467,46593,46719,46846,46973,47100,47228,47356,47484,47613,47742,47871,48001,48131,48262,48393,48524,48655,48787,48920,49052,49185,49319,49452,49586,49721,49856,49991,50126,50262,50399,50535,50672,50810,50947,51085,51224,51363,51502,51642,51782,51922,52063,52204,52346,52488,52630,52773,52916,53059,53203,53347,53492,53637,53782,53928,54074,54221,54368,54515,54663,54811,54960,55109,55258,55408,55558,55709,55860,56012,56163,56316,56468,56622,56775,56929,57083,57238,57393,57549,57705,57861,58018,58176,58333,58491,58650,58809,58968,59128,59289,59449,59611,59772,59934,60097,60260,60423,60587,60751,60916,61081,61247,61413,61579,61746,61914,62081,62250,62419,62588,62757,62928,63098,63269,63441,63613,63785,63958,64132,64306,64480,64655,64830,65006,65182,65359,65536};     /* 2^(i / 256), Q15 */
static const int32_t LOG2_1P7C[101] = {0,6397,12389,18023,23340,28374,33154,37704,42044,46194,50170,53985,57652,61183,64586,67871,71045,74117,77091,79976,82774,85492,88135,90705,93207,95645,98022,100340,102603,104813,106972,109084,111149,113170,115148,117086,118986,120847,122673,124464,126222,127948,129643,131308,132944,134553,136134,137690,139220,140726,142208,143668,145105,146521,147915,149290,150645,151981,153298,154597,155878,157142,158390,159621,160837,162037,163222,164392,165548,166691,167819,168934,170036,171126,172203,173268,174321,175362,176392,177411,178420,179417,180404,181381,182348,183305,184253,185191,186120,187039,187950,188853,189747,190632,191509,192378,193240,194093,194939,195777,196608}; /* log2(1 + 7 c / 100), Q16 */
#define LOG2_250 522046
#define LOG2_12000 888062
#define LOG2_60 390974
#define REL_K 1854286438u /* -ln(0.001), Q28: fall 60 dB over the time given */
#define INC_K 24932236u   /* phase step per frame = hz_q8 * INC_K >> 16 */

static uint32_t exp2_q16(int32_t l) { /* 2^(l / 65536) */
  int i = l >> 16;
  uint32_t v = EXP2[(l >> 8) & 255];
  return i >= 15 ? v << (i - 15) : v >> (15 - i);
}
/* 2^(n/12) in Q16, n = -12..12 */
static const int32_t SEMI[25] = {32768, 34716, 36781, 38968, 41285, 43740, 46341, 49097, 52016,
                                 55109, 58386, 61858, 65536, 69433, 73562, 77936, 82570, 87480,
                                 92682, 98193, 104032, 110218, 116772, 123715, 131072};
static const uint32_t METAL_INC[6] = {19994485u, 29645987u, 35995917u, 50906563u, 52591437u, 77913240u}; /* 808 hats */

/* ================================================================ shared: loop() <-> audio() */

static volatile uint8_t running, play_bank, cur_bank;
static volatile int8_t queued_bank = -1;
static volatile int8_t spos[LANES] = {-1, -1, -1, -1, -1, -1, -1, -1};
static volatile int8_t dpos[TRACKS] = {-1, -1, -1, -1, -1};
static volatile uint32_t pulses;
static volatile uint16_t load_pct;
static volatile uint8_t vcap = 8;
static volatile uint8_t smute, dmute, synth_mute, freeze;
static volatile uint8_t preview_req, sprev;
/* the FX layer, loop -> audio: echo and space throws, a filter push, intensity, repeat; and a note counter back */
static volatile uint8_t fx_td, fx_ts, rpt, notes_n, fill_on, kreq, song_pos, t_pos;
static volatile int8_t kddeg, t_shift;
static volatile int8_t int_ovr = -1;
static volatile int32_t fx_mf;

/* ================================================================ the synth voice */

typedef struct {
  uint8_t active, stage, four, para, fmod, lane, dstage, nuni, gate, key;
  uint32_t cphu[2], cincu[2];
  int32_t umix, ugain, dainc, lcount, lframes;
  uint32_t cph, cinc, cph2, cinc2, mph, minc, m3ph, m3inc, sph, sinc;
  int32_t dev, dev2, dev3;           /* deviation at full envelope, phase-step units */
  int32_t denv, d3env, mend;         /* modulation envelopes and where they settle, Q24 */
  int32_t dcoef, d3coef;             /* per slice, Q28 */
  int32_t amp, peak, ainc, rcoef;    /* Q24, Q28 */
  int32_t hold;                      /* frames at the peak */
  int32_t fbk, m1, m2;               /* modulator feedback */
  int32_t sgain;                     /* Q15 */
  int32_t kq, inv;                   /* shaper */
  int32_t lfc, lbase, lstep;         /* filter cutoff, log2 Hz Q16 */
  int32_t f, q, low, band;
  int16_t gl, gr;
  int32_t send;                      /* Q15 */
} svoice_t;
static svoice_t SV[MAXSV];
/* modulation, set once a slice from the LFO and the step mod. Destinations: 1 FILTER 2 INDEX 3 PITCH 4 LEVEL 5 PAN 6 DELAY */
static int32_t m_filt, m_idx = 32768, m_pitch, sm_env, lfo_sh, m_dpm = 32768, m_dlev = 32768;
static uint32_t lfo_ph, lfo_prev;
static volatile uint32_t lfo_ph_out;
static int32_t lfo_shape(int sh, uint32_t ph) { /* -32767..32767 */
  if (sh == 1) {
    int32_t t = (int32_t)(ph >> 15);
    return t < 65536 ? t - 32768 : 98304 - t;
  }
  if (sh == 2) return ph & 0x80000000u ? -32767 : 32767;
  if (sh == 3) return (int32_t)(((ph >> 29) * 40503u + 7919u) & 0xFFFFu) - 32768; /* for the picture: eight steps */
  return sine(ph);
}

static int32_t frames(int32_t ms) { return ms * 441 / 10; }
static int32_t coef_slice(int32_t n) { /* fall to 0.001 over n frames, applied once per slice */
  int32_t sl = n / SUB;
  if (sl < 1) sl = 1;
  return (int32_t)((1u << 28) - REL_K / (uint32_t)sl);
}
static int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }
static int32_t clamp16(int32_t v) { return v > 32767 ? 32767 : v < -32767 ? -32767 : v; }
static int32_t dev_of(uint32_t inc, int32_t idx, int32_t div) { /* inc x idx / div, kept in range: 32-bit division only */
  uint64_t prod = (uint64_t)(inc >> 8) * (uint32_t)idx;
  uint32_t d = prod < 0xFFFFFFFFull ? (uint32_t)prod / (uint32_t)div : ((uint32_t)(prod >> 8) / (uint32_t)div) << 8;
  if (d > (1u << 22)) d = 1u << 22;
  return (int32_t)(d << 8);
}

static void filter_coefs(svoice_t *v) {
  uint32_t fc = exp2_q16(v->lfc);
  if (fc > 12000) fc = 12000;
  v->f = sine(fc * 24348u); /* 2 sin(pi fc / 88200), Q14: the filter runs twice a frame */
}

static int lane_note(int deg, int oct) {
  const int16_t *k = S.k;
  int sc = k[K_SCALE], sl = SCALE_LEN[sc];
  return 48 + k[K_ROOT] + SCALES[sc][deg % sl] + 12 * (deg / sl) + 12 * (oct - 2);
}

static int32_t frames_per_step(void);
static void synth_incs(svoice_t *v, int l, const sstep_t *st, int ddeg, int nov) { /* the note's phase steps */
  const int16_t *k = S.k;
  const lane_t *ln = &S.bank[play_bank].lane[l];
  int lk = LOCKED(st), pitch = lk ? st->pitch : 0;
  int note = nov >= 0 ? clampi(nov, 12, 118) : clampi(lane_note(clampi(ln->deg + ddeg + t_shift, 0, 40), ln->oct) + pitch, 12, 118);
  v->cinc = note_inc(note);
  v->sinc = note_inc(note - 12);
  v->minc = (uint32_t)(((uint64_t)v->cinc * FRAC_Q16[k[K_RATIO]]) >> 16);
  v->m3inc = (uint32_t)(((uint64_t)v->minc * 3u) >> 1);
  if (v->para) v->cinc2 = note_inc(nov >= 0 ? clampi(note + 4, 12, 118) : clampi(lane_note(clampi(ln->deg + ddeg + t_shift + 2, 0, 40), ln->oct) + pitch, 12, 118));
  uint32_t up = exp2_q16(k[K_UDET] * 65536 / 1200 + (15 << 16)), dn = exp2_q16((15 << 16) - k[K_UDET] * 65536 / 1200);
  v->cincu[0] = (uint32_t)(((uint64_t)v->cinc * up) >> 15);
  v->cincu[1] = (uint32_t)(((uint64_t)v->cinc * dn) >> 15);
}
static svoice_t *fire_synth(int l, const sstep_t *st, int32_t velq, int ddeg, int nov) { /* nov: a MIDI note, or -1 */
  const int16_t *k = S.k;
  const lane_t *ln = &S.bank[play_bank].lane[l];
  int cap = vcap, pick = -1;
  for (int i = 0; i < cap; i++)
    if (!SV[i].active) {
      pick = i;
      break;
    }
  if (pick < 0) { /* the quietest gives way */
    pick = 0;
    for (int i = 1; i < cap; i++)
      if (SV[i].amp < SV[pick].amp) pick = i;
  }
  svoice_t *v = &SV[pick];
  v->active = 0;
  int lk = LOCKED(st);
  int pitch = lk ? st->pitch : 0, vel = lk ? st->vel : 85, send = lk ? st->send : 50;
  int32_t idx = clampi(k[K_INDEX] + (lk ? st->index : 0), 0, 200);
  int note = nov >= 0 ? clampi(nov, 12, 118) : clampi(lane_note(clampi(ln->deg + ddeg + t_shift, 0, 40), ln->oct) + pitch, 12, 118);
  idx = idx * (100 - k[K_MVEL] + k[K_MVEL] * vel / 100) / 100;            /* velocity: harder is brighter */
  idx = idx * clampi(100 - k[K_KTRK] * (note - 60) / 48, 25, 200) / 100;  /* key tracking: high notes get less */
  if (k[K_DRIFT]) idx += idx * ((int32_t)(random32() % (uint32_t)(2 * k[K_DRIFT] + 1)) - k[K_DRIFT]) / 200;
  idx = clampi(idx, 0, 250);
  v->para = k[K_ALGO] == 2;
  synth_incs(v, l, st, ddeg, nov);
  uint32_t frac = FRAC_Q16[k[K_RATIO]];
  int32_t ratio10 = (int32_t)((frac * 10u) >> 16);
  if (ratio10 < 2) ratio10 = 2;
  int dur = DUR(st);
  int32_t decay = k[K_REL] * DUR_MULT[dur - 1] / 10; /* ms */
  if (decay < 60) decay = 60;
  int32_t nd = frames(decay);
  v->four = k[K_ALGO] == 1;
  v->para = k[K_ALGO] == 2;
  int32_t nmod = nd / 100 * k[K_MDEC];
  if (!v->four) {
    v->dev = dev_of(v->cinc, idx, 500);
    v->dcoef = coef_slice(nmod * 9 / 10);
    if (v->para) v->dev2 = dev_of(v->cinc2, idx, 500); /* a second carrier two scale steps up, on the same modulator */
  } else { /* op3 -> op2 -> carrier */
    v->dev = dev_of(v->cinc, idx * 18, 1000);
    v->dev3 = dev_of(v->cinc, idx * ratio10 * 108 / 100 * k[K_OP3] / 100, 1000);
    v->dcoef = coef_slice(nmod * 85 / 100);
    v->d3coef = coef_slice(nmod * 7 / 10);
  }
  if (k[K_MATK]) { /* the index swells in, then falls */
    v->denv = v->d3env = 0, v->dstage = 1;
    int32_t sl = frames(k[K_MATK]) / SUB;
    v->dainc = (1 << 24) / (sl > 0 ? sl : 1) + 1;
  } else {
    v->denv = v->d3env = 1 << 24, v->dstage = 0;
  }
  static const uint8_t LOOPS[5] = {0, 1, 2, 4, 8};
  v->lframes = LOOPS[k[K_MLOOP]] * frames_per_step(), v->lcount = v->lframes;
  v->nuni = (uint8_t)(k[K_UNI] - 1);
  v->umix = k[K_UMIX] * 327;
  v->ugain = (int32_t)(1073741824 / (32768 + v->nuni * v->umix));
  v->cphu[0] = random32(), v->cphu[1] = random32();
  v->lane = (uint8_t)l;
  v->mend = k[K_MEND] * 167772;
  v->fbk = k[K_FB] * 313;
  v->m1 = v->m2 = 0;
  v->sgain = k[K_SUB] * 22938 / 100;
  int32_t drive100 = 100 + k[K_DRIVE] * 9;
  v->kq = drive100 * 2097;
  int32_t di = drive100 * 1024 / 1000;
  v->inv = (int32_t)(1073709056 / TANH10[di > 1024 ? 1024 : di]);
  /* tone: from base x (1 + 7 contour) down to base; Perform FILTER moves the base +-2.5 octaves */
  int32_t lbase = clampi(LOG2_250 + k[K_CUT] * 3277 + (k[K_FILT] - 50) * 3277, LOG2_60, LOG2_12000);
  int32_t lpeak = clampi(lbase + LOG2_1P7C[k[K_CONTOUR]], LOG2_60, LOG2_12000);
  int32_t sweep = frames(decay * 65 / 100 > 30 ? decay * 65 / 100 : 30) / SUB;
  v->lfc = lpeak;
  v->lbase = lbase;
  v->lstep = (lpeak - lbase) / (sweep > 0 ? sweep : 1);
  int32_t q = 163840 / k[K_RESO];
  v->q = q > 31130 ? 31130 : q < 1170 ? 1170 : q;
  v->low = v->band = 0;
  filter_coefs(v);
  int32_t peak15 = (int32_t)(((int64_t)(vel * 16384 / 100) * velq) >> 15);
  v->peak = peak15 << 9;
  v->amp = 0;
  int32_t atk = frames(k[K_ATK]);
  v->ainc = v->peak / (atk > 0 ? atk : 1) + 1;
  v->hold = frames(k[K_HOLD] * DUR_MULT[dur - 1] / 100);
  v->rcoef = (int32_t)((1u << 28) - REL_K / (uint32_t)(nd > 0 ? nd : 1));
  v->stage = 0;
  v->fmod = 0;
  static uint8_t alt;
  alt ^= 1;
  int32_t pan = (l * 32768 / 7 - 16384) * 8 / 10; /* -0.4..0.4 by lane, equal power; SPREAD alternates notes */
  pan = clampi(pan + (alt ? 1 : -1) * k[K_USPR] * 180, -29490, 29490);
  uint32_t ang = (uint32_t)(pan + 32768) << 14;
  v->gl = (int16_t)sine(ang + 1073741824u);
  v->gr = (int16_t)sine(ang);
  v->send = send * 32767 / 100;
  v->cph2 = v->cph;
  v->gate = 0, v->key = 0;
  v->active = 1;
  notes_n++;
  return v;
}

static inline int32_t shaper(const svoice_t *v, int32_t x) {
  if (x > 32767) x = 32767;
  if (x < -32767) x = -32767;
  int32_t ax = x < 0 ? -x : x;
  int32_t iq = (int32_t)(((int64_t)ax * v->kq) >> 18);
  int32_t i = iq >> 8, fr = iq & 255;
  int32_t t = i >= 1024 ? 32767 : TANH10[i] + (((TANH10[i + 1] - TANH10[i]) * fr) >> 8);
  t = (t * v->inv) >> 15;
  return x < 0 ? -t : t;
}

static int32_t synth_g12 = 4096;
static void render_synth(int32_t *out, int32_t *sbus, uint32_t n) {
  for (int vi = 0; vi < MAXSV; vi++) {
    svoice_t *v = &SV[vi];
    if (!v->active) continue;
    int32_t lg = synth_g12 * (v->lane < 8 ? S.llev[v->lane] : 100) / 100; /* the mix: synth x lane, Q12 */
    if (v->lframes) { /* the index envelope loops in time */
      v->lcount -= (int32_t)n;
      if (v->lcount <= 0) {
        v->lcount += v->lframes;
        if (S.k[K_MATK]) v->denv = v->d3env = 0, v->dstage = 1;
        else v->denv = v->d3env = 1 << 24;
      }
    }
    if (v->dstage) {
      v->denv += v->dainc, v->d3env = v->denv;
      if (v->denv >= 1 << 24) v->denv = v->d3env = 1 << 24, v->dstage = 0;
    } else {
      v->denv = v->mend + (int32_t)(((int64_t)(v->denv - v->mend) * v->dcoef) >> 28);
    }
    int32_t dev = (int32_t)(((int64_t)v->dev * (v->denv >> 9)) >> 15);
    dev = (int32_t)(((int64_t)dev * m_idx) >> 15);
    int32_t dev2 = v->para ? (int32_t)(((int64_t)v->dev2 * (v->denv >> 9)) >> 15) : 0;
    int32_t dev3 = 0;
    if (v->four) {
      if (!v->dstage) v->d3env = v->mend + (int32_t)(((int64_t)(v->d3env - v->mend) * v->d3coef) >> 28);
      dev3 = (int32_t)(((int64_t)v->dev3 * (v->d3env >> 9)) >> 15);
    }
    if (v->lfc > v->lbase) {
      v->lfc -= v->lstep;
      if (v->lfc < v->lbase) v->lfc = v->lbase;
      filter_coefs(v);
    }
    if (m_filt) { /* the cutoff moved by the LFO or the step mod */
      int32_t keep = v->lfc;
      v->lfc = clampi(keep + m_filt, LOG2_60, LOG2_12000);
      filter_coefs(v);
      v->lfc = keep;
      v->fmod = 1;
    } else if (v->fmod) {
      filter_coefs(v);
      v->fmod = 0;
    }
    int32_t padd = (int32_t)(((int64_t)v->cinc * m_pitch) >> 15);
    int32_t amp = v->amp;
    for (uint32_t f = 0; f < n; f++) {
      if (v->stage == 0) {
        amp += v->ainc;
        if (amp >= v->peak) amp = v->peak, v->stage = 1;
      } else if (v->stage == 1) {
        if (!v->gate && --v->hold <= 0) v->stage = 2; /* a held key or MIDI note stays at the peak */
      } else {
        amp = (int32_t)(((int64_t)amp * v->rcoef) >> 28);
        if (amp < 4096) {
          v->active = 0;
          break;
        }
      }
      uint32_t minc = v->minc;
      if (v->four) {
        minc += (uint32_t)((sine(v->m3ph) * (int64_t)dev3) >> 15);
        v->m3ph += v->m3inc;
      }
      int32_t m = sine(v->mph + (uint32_t)(((v->m1 + v->m2) >> 1) * v->fbk));
      v->m2 = v->m1;
      v->m1 = m;
      v->mph += minc;
      v->cph += v->cinc + (uint32_t)padd + (uint32_t)(int32_t)(((int64_t)m * dev) >> 15);
      int32_t x = sine(v->cph);
      if (v->nuni) { /* unison: detuned copies of the carrier on the same modulator */
        uint32_t pm = (uint32_t)padd + (uint32_t)(int32_t)(((int64_t)m * dev) >> 15);
        int32_t u = 0;
        for (int q = 0; q < v->nuni; q++) v->cphu[q] += v->cincu[q] + pm, u += sine(v->cphu[q]);
        x = ((x + ((u * v->umix) >> 15)) * v->ugain) >> 15;
      }
      if (v->para) {
        v->cph2 += v->cinc2 + (uint32_t)(int32_t)(((int64_t)m * dev2) >> 15);
        x = (x + sine(v->cph2)) >> 1;
      }
      if (v->sgain) {
        x += (sine(v->sph) * v->sgain) >> 15;
        v->sph += v->sinc;
      }
      x = shaper(v, x) >> 4;
      for (int o = 0; o < 2; o++) {
        v->low += (v->f * v->band) >> 14;
        int32_t high = x - v->low - ((v->q * v->band) >> 14);
        v->band += (v->f * high) >> 14;
      }
      int32_t y = v->low << 4;
      if (y > 65534) y = 65534;
      if (y < -65534) y = -65534;
      y = (y * (amp >> 9)) >> 15;
      y = (y * lg) >> 12;
      int32_t l = (y * v->gl) >> 15, r = (y * v->gr) >> 15;
      out[2 * f] += l;
      out[2 * f + 1] += r;
      sbus[f] += (((l + r) >> 1) * v->send) >> 15;
    }
    v->amp = amp;
  }
}

/* ================================================================ the drum voice */

typedef struct {
  uint8_t active, stage, noise, ntype, trk;
  uint32_t cph, cinc, mph, minc, ceff, rat;
  int32_t midx;                   /* x10 */
  int32_t menv, mend, mcoef;      /* index envelope Q15, Q28 per slice */
  int32_t swenv, swcoef, swd;     /* pitch sweep: Q15, Q28, semitones */
  int32_t amp, peak, ainc, rcoef;
  int32_t f, q, low, band;
  uint32_t lfsr, mph6[6];
} dvoice_t;
static dvoice_t DVo[DV];

static int32_t kit_scale(int32_t v) { /* KIT: 0..100, 50 = as set; shorter / longer and softer / brighter */
  int32_t x = S.k[K_KIT];
  return x < 50 ? v * (50 + x) / 100 : v * (100 + (x - 50) * 2) / 100;
}

static void fire_drum(int t, const uint8_t *p, int32_t vel) {
  if ((dmute >> t) & 1) return;
  if (t == S.k[K_MSRC]) sm_env = 32767; /* the step mod follows this track */
  int32_t g = (int32_t)S.vol[t] * p[P_ACC];
  int32_t peak = (int32_t)(((uint32_t)g * (uint32_t)vel) / (IS_NOISE(t) ? 2500u : 2222u));
  if (peak > 32767) peak = 32767;
  if (peak < 64) return;
  int pick = 0;
  for (int i = 0; i < DV; i++) {
    if (!DVo[i].active) {
      pick = i;
      break;
    }
    if (DVo[i].amp < DVo[pick].amp) pick = i;
  }
  dvoice_t *v = &DVo[pick];
  v->active = 0;
  v->noise = (uint8_t)IS_NOISE(t);
  v->trk = (uint8_t)t;
  if (t == 0 && vel > 8000) pump_hit = 1; /* the kick drives the pump */
  v->peak = peak << 9;
  int32_t atk = frames(p[P_ATK]);
  if (atk < 1) atk = 1;
  v->amp = 0;
  v->ainc = v->peak / atk;
  if (v->ainc < 1) v->ainc = 1;
  int32_t rel = frames(kit_scale(p[P_REL] * 5));
  if (rel < 882) rel = 882;
  v->rcoef = (int32_t)((1u << 28) - REL_K / (uint32_t)rel);
  v->stage = 0;
  if (v->noise) {
    uint32_t fc = (uint32_t)p[P_CAR] * 50u;
    v->f = sine(fc * 24348u);
    int32_t q = 163840 / (p[P_RAT] ? p[P_RAT] : 1);
    v->q = q > 31130 ? 31130 : q;
    v->low = v->band = 0;
    v->ntype = (uint8_t)(p[P_IDX] > 4 ? 4 : p[P_IDX]);
    if (v->ntype == 4) v->q = 600; /* RESO: noise rings a narrow filter */
    v->lfsr = 0x7F;
  } else {
    uint32_t hz_q8 = (uint32_t)(((uint64_t)TRACK_HZ[t] * 256u * FRAC_Q16[p[P_CAR]]) >> 16);
    hz_q8 = (uint32_t)(((uint64_t)hz_q8 * (uint32_t)SEMI[p[P_PITCH]]) >> 16);
    v->cinc = (uint32_t)(((uint64_t)hz_q8 * INC_K) >> 16);
    v->rat = FRAC_Q16[p[P_RAT]];
    v->minc = (uint32_t)(((uint64_t)v->cinc * v->rat) >> 16);
    v->ceff = v->cinc;
    v->midx = kit_scale(p[P_IDX]);
    v->cph = v->mph = 0;
    v->menv = 32767;
    v->mend = p[P_MEND] * 327;
    v->mcoef = coef_slice(frames(p[P_MDEC] * 5));
    v->swd = p[P_SWD];
    v->swenv = 32767;
    v->swcoef = coef_slice(frames(p[P_SWT] * 2));
  }
  v->active = 1;
  notes_n++;
}

static void drum_retune(int t, const uint8_t *p) { /* a trigless drum step: the ringing hit takes this step's sound */
  for (int i = 0; i < DV; i++) {
    dvoice_t *v = &DVo[i];
    if (!v->active || v->trk != t) continue;
    if (v->noise) {
      v->f = sine((uint32_t)p[P_CAR] * 50u * 24348u);
      if (v->ntype != 4) {
        int32_t q = 163840 / (p[P_RAT] ? p[P_RAT] : 1);
        v->q = q > 31130 ? 31130 : q;
      }
    } else {
      uint32_t hz_q8 = (uint32_t)(((uint64_t)TRACK_HZ[t] * 256u * FRAC_Q16[p[P_CAR]]) >> 16);
      hz_q8 = (uint32_t)(((uint64_t)hz_q8 * (uint32_t)SEMI[p[P_PITCH]]) >> 16);
      v->cinc = (uint32_t)(((uint64_t)hz_q8 * INC_K) >> 16);
      v->rat = FRAC_Q16[p[P_RAT]];
      v->midx = kit_scale(p[P_IDX]);
    }
  }
}

static void render_drums(int32_t *dbus, uint32_t n);
static struct { int32_t *bus; uint32_t n; } drum_job_arg;
static void drum_job(uint32_t a) { /* the drums, on the second core while the first renders the synth */
  (void)a;
  render_drums(drum_job_arg.bus, drum_job_arg.n);
}
static void render_drums(int32_t *dbus, uint32_t n) {
  for (int i = 0; i < DV; i++) {
    dvoice_t *v = &DVo[i];
    if (!v->active) continue;
    int32_t midx = 0;
    if (!v->noise) { /* once a slice: the pitch sweep and the index envelope */
      uint64_t c = ((uint64_t)v->cinc * (uint32_t)m_dpm) >> 15; /* the LFO / step mod moving drum pitch */
      if (v->swd) {
        v->swenv = (int32_t)(((int64_t)v->swenv * v->swcoef) >> 28);
        uint32_t mult = exp2_q16(v->swd * v->swenv / 6 + (15 << 16)); /* 2^(depth x env / 12), Q15 */
        c = (c * mult) >> 15;
      }
      v->ceff = c > 0x7FFFFFFFu ? 0x7FFFFFFFu : (uint32_t)c;
      v->menv = v->mend + (int32_t)(((int64_t)(v->menv - v->mend) * v->mcoef) >> 28);
      midx = (v->midx * v->menv) >> 15;
    }
    uint32_t minc = v->noise ? 0 : (uint32_t)(((uint64_t)v->ceff * v->rat) >> 16); /* the modulator follows */
    int32_t amp = v->amp;
    for (uint32_t k = 0; k < n; k++) {
      if (v->stage == 0) {
        amp += v->ainc;
        if (amp >= v->peak) amp = v->peak, v->stage = 1;
      } else {
        amp = (int32_t)(((int64_t)amp * v->rcoef) >> 28);
        if (amp < 4096) {
          v->active = 0;
          break;
        }
      }
      int32_t s;
      if (v->noise) {
        int32_t in;
        uint32_t r;
        switch (v->ntype) {
          case 1: { /* 7-bit LFSR: the buzzy short loop of a handheld */
            uint32_t b = (v->lfsr ^ (v->lfsr >> 1)) & 1u;
            v->lfsr = (v->lfsr >> 1) | (b << 6);
            in = (v->lfsr & 1u) ? 1024 : -1024;
            break;
          }
          case 2: /* six square waves */
            in = 0;
            for (int o = 0; o < 6; o++) in += (v->mph6[o] & 0x80000000u) ? 170 : -170, v->mph6[o] += METAL_INC[o];
            break;
          case 3: /* sparse grains */
            r = random32();
            in = (r & 7u) == 0 ? (int32_t)r >> 20 : 0;
            break;
          case 4: in = (int32_t)random32() >> 25; break;
          default: in = (int32_t)random32() >> 21;
        }
        for (int o = 0; o < 2; o++) {
          v->low += (v->f * v->band) >> 14;
          int32_t high = in - v->low - ((v->q * v->band) >> 14);
          v->band += (v->f * high) >> 14;
        }
        s = clamp16(((v->band * v->q) >> 14) * 32);
      } else {
        int32_t m = sine(v->mph);
        uint32_t off = (uint32_t)(m * midx) * 2086u;
        s = sine(v->cph + off);
        v->cph += v->ceff;
        v->mph += minc;
      }
      dbus[k] += (s * (amp >> 9)) >> 16;
    }
    v->amp = amp;
  }
}

/* ================================================================ sequencing */

typedef struct {
  int32_t wait;
  int32_t vel; /* Q15 */
  uint8_t used, kind, who; /* kind 0 note, 1 drum hit, 2 trigless (the lane's note moves, no new note), 3 trigless drum */
  int8_t ddeg;             /* echo ladder: scale steps */
  uint8_t p[DP];
  sstep_t st;
} event_t;
static event_t Q[EVQ];

static int32_t frames_per_step(void) {
  uint32_t bpm100 = clock_bpm();
  if (bpm100 < 2000) bpm100 = (uint32_t)S.k[K_BPM] * 100u;
  return (int32_t)(66150000u / bpm100);
}
static event_t *ev_new(void) {
  for (int i = 0; i < EVQ; i++)
    if (!Q[i].used) return &Q[i];
  return 0;
}
static void enqueue_synth(int l, const sstep_t *st, int32_t wait, int32_t vel, int ddeg, int kind) {
  event_t *e = ev_new();
  if (!e) return;
  e->kind = (uint8_t)kind, e->who = (uint8_t)l, e->st = *st, e->wait = wait, e->vel = vel, e->ddeg = (int8_t)ddeg;
  e->used = 1;
}
static void enqueue_drum(int t, const uint8_t *p, int32_t vel, int32_t wait) {
  event_t *e = ev_new();
  if (!e) return;
  e->kind = 1, e->who = (uint8_t)t, e->wait = wait, e->vel = vel;
  for (int k = 0; k < DP; k++) e->p[k] = p[k];
  e->used = 1;
}
static void resolve(const bank_t *b, int t, int s, uint8_t *p) {
  for (int k = 0; k < DP; k++) {
    uint8_t l = b->lock[t][s][k];
    p[k] = l == UNLOCKED ? S.def[t][k] : l;
  }
}
static void hit(int t, const uint8_t *p, int32_t wait) { /* a hit and its echoes, half a step apart */
  wait += p[P_WAIT] * frames_per_step() / 6;
  enqueue_drum(t, p, 32767, wait);
  int n = p[P_ECHO];
  int32_t spacing = frames_per_step() / 2, keep = (100 - p[P_EDEC] * 5) * 32768 / 100, vel = 32767;
  for (int i = 1; i <= n; i++) {
    vel = (vel * keep) >> 15;
    if (vel < 655) break;
    enqueue_drum(t, p, vel, wait + spacing * i);
  }
}
static int cond_ok(int c, uint32_t loops) {
  c &= 15;
  if (!c) return 1;
  if (c == 15) return fill_on; /* FILL: only while held */
  if (!COND_B[c]) return random32() % 100u < COND_A[c];
  return loops % COND_B[c] == (uint32_t)(COND_A[c] - 1);
}
static int lane_period(const lane_t *ln) {
  int len = ln->len < 1 ? 1 : ln->len;
  return ln->dir == 2 && len > 1 ? 2 * len - 2 : len;
}
static int lane_pos(const lane_t *ln, uint32_t c, uint32_t *loops) {
  int len = ln->len < 1 ? 1 : ln->len, per = lane_period(ln);
  *loops = c / (uint32_t)per;
  int p = (int)(c % (uint32_t)per);
  switch (ln->dir) {
    case 1: return len - 1 - p;
    case 2: return p < len ? p : per - p;
    case 3: return (int)(random32() % (uint32_t)len);
    default: return p;
  }
}

/* notes asked for by loop(): the white keys in KEYS and MIDI in. One writer (loop), one reader (audio) */
typedef struct {
  uint8_t type, who, id, vel; /* type 0 synth note on, 1 note off (by id), 2 drum hit */
  int8_t deg;
  int16_t note; /* a MIDI note, or -1: the lane's degree + deg */
} nreq_t;
static nreq_t NR[16];
static volatile uint8_t nr_w, nr_r;
static void note_req(int type, int who, int id, int deg, int note, int vel) {
  uint8_t w = nr_w, nx = (uint8_t)((w + 1) & 15);
  if (nx == nr_r) return;
  NR[w].type = (uint8_t)type, NR[w].who = (uint8_t)who, NR[w].id = (uint8_t)id, NR[w].vel = (uint8_t)vel;
  NR[w].deg = (int8_t)deg, NR[w].note = (int16_t)note;
  __asm__ volatile("" ::: "memory");
  nr_w = nx;
}

static uint32_t tick, pulse;
static void do_pulse(void) {
  if (pulse % 4 == 0 && S.pump[2]) pump_hit = 1; /* the pump on the beat */
  if (pulse % 16 == 0) { /* the bar line: song, queued bank, transpose */
    static uint8_t sidx, srep;
    if (S.k[K_SONG] && S.song_n) {
      if (pulse == 0) sidx = 0, srep = 0;
      else if (++srep >= S.song_rep[sidx]) srep = 0, sidx = (uint8_t)((sidx + 1) % S.song_n);
      play_bank = cur_bank = S.song_bank[sidx];
      if (pulse == 0 || sidx != song_pos) dmute = S.song_mute[sidx] & 31, synth_mute = (S.song_mute[sidx] >> 5) & 1;
      song_pos = sidx;
      queued_bank = -1;
    } else if (queued_bank >= 0) {
      play_bank = cur_bank = (uint8_t)queued_bank;
      queued_bank = -1;
    }
    int tl = S.k[K_TLEN], every = 1 << S.k[K_TEVERY];
    if (tl) {
      uint32_t bar = pulse / 16;
      t_pos = (uint8_t)(bar / (uint32_t)every % (uint32_t)tl);
      t_shift = S.tseq[t_pos];
    } else {
      t_shift = 0;
    }
  }
  int32_t wait = (pulse & 1) ? frames_per_step() * S.k[K_SWING] * 2 / 300 : 0;
  const bank_t *b = &S.bank[play_bank];
  static uint32_t rpt_from;
  static uint8_t rpt_on;
  if (rpt && !rpt_on) rpt_on = 1, rpt_from = pulse >= 2 ? pulse - 2 : 0;
  if (!rpt) rpt_on = 0;
  uint32_t pp = rpt_on ? rpt_from + ((pulse - rpt_from) & 1) : pulse; /* repeat: the last two steps, over and over */
  for (int l = 0; l < LANES; l++) {
    uint32_t loops;
    int s = lane_pos(&b->lane[l], pp, &loops);
    spos[l] = (int8_t)s;
    const sstep_t *st = &b->st[l][s];
    if (synth_mute || ((smute >> l) & 1) || !cond_ok(COND(st), loops)) continue;
    uint8_t x = b->xb[l][s];
    int32_t w = wait + (x & 7) * frames_per_step() / 6; /* lock set B: a little late */
    if (TRIGLESS(st)) {
      enqueue_synth(l, st, w, 32767, 0, 2);
    } else if (DUR(st)) {
      enqueue_synth(l, st, w, 32767, 0, 0);
      int n = (x >> 3) & 3, shape = (x >> 5) & 3; /* echo ladder: repeats half a step apart, climbing or falling */
      int32_t vel = 32767, keep = x & 128 ? 27853 : 19661, sp = frames_per_step() / 2;
      for (int i = 1; i <= n; i++) {
        vel = (vel * keep) >> 15;
        int dd = shape == 1 ? i : shape == 2 ? -i : shape == 3 ? (int)(random32() % 5u) - 2 : 0;
        enqueue_synth(l, st, w + sp * i, vel, dd, 0);
      }
    }
  }
  int in = int_ovr >= 0 ? int_ovr : S.k[K_INT];
  for (int t = 0; t < TRACKS; t++) {
    int len = b->dlen[t] ? b->dlen[t] : DSTEPS;
    int s = (int)(pp % (uint32_t)len);
    dpos[t] = (int8_t)s;
    if (!((b->on[t] >> s) & 1) && (b->dcond[t][s] & 16)) { /* trigless: the ringing hit takes this step's locks */
      if (cond_ok(b->dcond[t][s], pp / (uint32_t)len)) {
        uint8_t p[DP];
        resolve(b, t, s, p);
        enqueue_drum(t, p, 0, wait + p[P_WAIT] * frames_per_step() / 6);
        for (int i = EVQ - 1; i >= 0; i--)
          if (Q[i].used && Q[i].kind == 1 && Q[i].who == t && Q[i].vel == 0) Q[i].kind = 3;
      }
      continue;
    }
    if (!((b->on[t] >> s) & 1)) { /* intensity above centre adds ghost hats, percs and snares */
      if (in > 50 && t >= 1 && t <= 3 && (int)(random32() % 100u) < (in - 50) * (t == 1 ? 2 : 6) / 10) {
        uint8_t p[DP];
        resolve(b, t, s, p);
        p[P_ECHO] = 0;
        enqueue_drum(t, p, 14746, wait);
      }
      continue;
    }
    if (!cond_ok(b->dcond[t][s], pp / (uint32_t)len)) continue;
    uint8_t p[DP];
    resolve(b, t, s, p);
    int32_t prob = p[P_PROB] * 5;
    if (t > 0 && in < 50) prob = prob * in / 50; /* below centre thins all but the kick */
    if ((int32_t)(random32() % 100u) >= prob) continue;
    hit(t, p, wait);
  }
  pulses = pulse;
  pulse++;
}

static void on_clock(int what) {
  if (what == CLOCK_START) {
    tick = pulse = 0;
    lfo_ph = 0;
    queued_bank = -1;
    running = 1;
  } else if (what == CLOCK_CONTINUE) {
    running = 1;
  } else if (what == CLOCK_STOP) {
    running = 0;
    for (int l = 0; l < LANES; l++) spos[l] = -1;
    for (int t = 0; t < TRACKS; t++) dpos[t] = -1;
  } else if (what == CLOCK_TICK && running) {
    if (tick % 6 == 0) do_pulse();
    tick++;
  }
}

/* ================================================================ effects */

static int16_t dly_a[DLEN] POOL, dly_b[DLEN] POOL;
static uint32_t dw;
static int32_t ta_q8, tb_q8, lp_a, lp_b;

/* space: two allpass diffusers into four delay lines mixed through a Hadamard matrix */
#define AP1 225
#define AP2 341
static const int16_t RV_LEN[4] = {3203, 3529, 3911, 4373};
#define RV_TOTAL (AP1 + AP2 + 3203 + 3529 + 3911 + 4373)
static int16_t rvb[RV_TOTAL] POOL;
static int16_t *const rv_ap1 = rvb, *const rv_ap2 = rvb + AP1;
/* fixed at build time: on the FM-1, audio() starts before setup() runs */
static int16_t *const rv_line[4] = {rvb + AP1 + AP2, rvb + AP1 + AP2 + 3203, rvb + AP1 + AP2 + 3203 + 3529,
                                    rvb + AP1 + AP2 + 3203 + 3529 + 3911};
static volatile uint8_t ready; /* setup() has run: until then audio() plays silence */
static int32_t rv_i1, rv_i2, rv_w[4], rv_lp[4], rv_hold, rv_held;

static int32_t lim_env = 26000;
static volatile uint16_t pk_syn, pk_drm, pk_out, pk_hdr; /* meters: the loudest sample since the panel last looked */
static volatile uint16_t sec_pct[3]; /* share of the audio time: synth voices, drums, effects and mix */
static volatile uint16_t pump_show; /* the pump's duck now, 0-32768, for the picture */
static int32_t glide(int32_t *s, int32_t t) { /* a setting's smoothed value, once a block: no steps on fast turns */
  int32_t d = t - *s;
  *s += d > 3 || d < -3 ? d / 4 : d;
  return *s;
}
static volatile uint8_t limiting;

static int32_t dly_read(const int16_t *buf, int32_t t_q8) {
  uint32_t pos = dw - (uint32_t)(t_q8 >> 8);
  int32_t fr = t_q8 & 255;
  int32_t a = buf[pos & DMASK], b = buf[(pos - 1) & DMASK];
  return a + (((b - a) * fr) >> 8);
}

void audio(int32_t *out, uint32_t nframes) {
  if (!ready) {
    for (uint32_t i = 0; i < 2 * nframes; i++) out[i] = 0;
    return;
  }
  uint32_t t0 = micros();
  const clock_event_t *ev;
  int nev = clock_events(&ev), e = 0;
  const int16_t *k = S.k;
  static int32_t sbus[SUB], dbus[SUB];
  uint8_t req = preview_req;
  if (req) {
    preview_req = 0;
    for (int t = 0; t < TRACKS; t++)
      if ((req >> t) & 1) {
        uint8_t p[DP];
        for (int i = 0; i < DP; i++) p[i] = S.def[t][i];
        fire_drum(t, p, 32767);
      }
  }
  if (sprev) { /* a lane key was tapped: sound its note */
    sstep_t st;
    clear_sstep(&st);
    st.dc = 2;
    fire_synth(sprev - 1, &st, 32767, 0, -1);
    sprev = 0;
  }
  while (nr_r != nr_w) { /* keys and MIDI: held notes stay at the peak until let go */
    const nreq_t *r = &NR[nr_r];
    if (r->type == 0) {
      sstep_t st;
      clear_sstep(&st);
      st.dc = 2;
      svoice_t *v = fire_synth(r->who, &st, clampi(r->vel * 258, 0, 32767), r->deg, r->note);
      v->gate = 1, v->key = r->id, v->lane = 255;
    } else if (r->type == 1) {
      for (int vi = 0; vi < MAXSV; vi++)
        if (SV[vi].active && SV[vi].gate && SV[vi].key == r->id) SV[vi].gate = 0, SV[vi].hold = 1;
    } else if (r->who < TRACKS) {
      fire_drum(r->who, S.def[r->who], clampi(r->vel * 258, 0, 32767));
    }
    __asm__ volatile("" ::: "memory");
    nr_r = (uint8_t)((nr_r + 1) & 15);
  }
  int32_t th = k[K_THROW];
  int32_t gsd = clampi(k[K_SSD] + th, 0, 100) * 328, gss = clampi(k[K_SSS] + th, 0, 100) * 328;
  int32_t gdd = clampi(k[K_SDD] + th, 0, 100) * 328, gds = clampi(k[K_SDS] + th, 0, 100) * 328;
  if (fx_td) gsd = gdd = 32767; /* throws: everything into the delay, or the space, while held */
  if (fx_ts) gss = gds = 32767;
  static int32_t g_sd, g_ss, g_dd, g_ds, g_fb, g_rm, g_rf, g_dp, g_dg, g_mg, g_sg;
  gsd = glide(&g_sd, gsd), gss = glide(&g_ss, gss), gdd = glide(&g_dd, gdd), gds = glide(&g_ds, gds);
  int32_t dfb = glide(&g_fb, k[K_DFB] * 328);
  int field = k[K_DMODE];
  /* the echo field (ORBITAL's): four taps of one send, the shortest fed back through a damping low-pass.
   * TIME A sets its size, TIME B its gravity (feedback), FEEDBK its trail (send and brightness) */
  int32_t fsz = clampi(k[K_DTA] * 256 / 400, 64, 358), grav = clampi((k[K_DTB] - 20) * 100 / 720, 0, 100);
  int32_t trail = k[K_DFB] * 100 / 90;
  int32_t fsend = 8192 + trail * 164, ffb = 9830 + grav * 203, ffc = 600 + (100 - trail) * 40;
  int32_t fw = ffc * 4669 / 1000, fa = fw - ((fw * fw) >> 16) + ((((fw * fw) >> 15) * fw) >> 15) / 6;
  uint32_t ft0 = 9305u * (uint32_t)fsz >> 8, ft1 = 14862u * (uint32_t)fsz >> 8, ft2 = 18478u * (uint32_t)fsz >> 8,
           ft3 = 23064u * (uint32_t)fsz >> 8;
  int frz = freeze;
  /* space */
  int32_t eff[4];
  for (int i = 0; i < 4; i++) eff[i] = RV_LEN[i] * (40 + k[K_SPACE] * 6 / 10) / 100;
  int32_t rfb = glide(&g_rf, 18000 + k[K_TRAIL] * 128);
  int32_t damp = glide(&g_dp, 24000 - k[K_TRAIL] * 100 - k[K_DIRT] * 60);
  int32_t rmix = glide(&g_rm, k[K_MIX] * 655);
  synth_g12 = glide(&g_sg, S.mix[0] * 4096 / 100);
  /* the pump: a duck on each kick (or beat), back over TIME */
  static int32_t penv;
  int32_t pcoef = coef_slice(frames(30 + S.pump[1] * 57 / 10)), pdep = S.pump[0] * 328, pg = 32768;
  int dirt = k[K_DIRT], dhold = 1 + dirt / 15;
  int32_t dmask = dirt ? ~((1 << (dirt / 11)) - 1) : ~0;
  uint32_t lfo_inc;
  if (k[K_LRATE] <= 100) {
    lfo_inc = exp2_q16(k[K_LRATE] * 5661 + (10 << 16)) * 76u; /* 0.05 Hz to 20 Hz, per slice */
  } else { /* in time: 4 bars down to 1/32 */
    static const uint8_t ST2[8] = {128, 64, 32, 16, 8, 4, 2, 1}; /* half steps a cycle */
    uint32_t per = (uint32_t)frames_per_step() * ST2[k[K_LRATE] - 101] / 2u;
    lfo_inc = (0xFFFFFFFFu / (per ? per : 1u)) * SUB;
  }
  int32_t ldep = k[K_LDEPTH] * 328, mdep = k[K_MDEPTH] * 328, smcoef = coef_slice(frames(k[K_MSHAPE]));
  int ld = ldep ? k[K_LDEST] : 0, md = mdep ? k[K_MDEST] : 0;
  int32_t gl_s = 32768, gr_s = 32768, dmod = 0;
  int32_t dg12 = S.mix[1] * 4096 / 100, mg12 = S.mix[2] * 4096 / 100, bs = 0, bd = 0, bo = 0;
  uint32_t t_syn = 0, t_drm = 0;

  for (uint32_t at = 0; at < nframes; at += SUB) {
    uint32_t n = nframes - at < SUB ? nframes - at : SUB;
    while (e < nev && ev[e].at < at + n) on_clock(ev[e++].what);
    for (int i = 0; i < EVQ; i++) {
      event_t *q = &Q[i];
      if (!q->used) continue;
      if (q->wait < (int32_t)n) {
        if (q->kind == 1) {
          fire_drum(q->who, q->p, q->vel);
        } else if (q->kind == 3) {
          drum_retune(q->who, q->p);
        } else if (q->kind == 2) { /* trigless: the lane's sounding notes glide to this step's pitch */
          for (int vi = 0; vi < MAXSV; vi++)
            if (SV[vi].active && SV[vi].lane == q->who) synth_incs(&SV[vi], q->who, &q->st, 0, -1);
        } else {
          fire_synth(q->who, &q->st, q->vel, q->ddeg, -1);
        }
        q->used = 0;
      } else {
        q->wait -= (int32_t)n;
      }
    }
    for (uint32_t f = 0; f < n; f++) sbus[f] = dbus[f] = 0;
    int32_t *o = out + 2 * at;
    { /* LFO and step mod, once a slice */
      lfo_ph += lfo_inc;
      if (lfo_ph < lfo_prev) lfo_sh = (int32_t)(random32() >> 16) - 32768;
      lfo_prev = lfo_ph;
      lfo_ph_out = lfo_ph;
      int32_t lv = k[K_LSHAPE] == 3 ? lfo_sh : lfo_shape(k[K_LSHAPE], lfo_ph);
      lv = (lv * (ldep >> 1)) >> 14;
      sm_env = (int32_t)(((int64_t)sm_env * smcoef) >> 28);
      int32_t sv = (sm_env * (mdep >> 1)) >> 14;
      m_filt = (ld == 1 ? lv * 4 : 0) - (md == 1 ? sv * 4 : 0) + fx_mf;
      m_idx = clampi(32768 + (ld == 2 ? lv : 0) - (md == 2 ? sv : 0), 0, 65536);
      m_pitch = (((ld == 3 ? lv : 0) - (md == 3 ? sv : 0)) * 1928) >> 15;
      int32_t lev = clampi(32768 - (ld == 4 ? (ldep - lv) >> 1 : 0) - (md == 4 ? sv : 0), 0, 32768);
      int32_t pan = ld == 5 ? lv : 0;
      gl_s = (lev * ((pan > 0 ? 32768 - pan : 32768) >> 1)) >> 14;
      gr_s = (lev * ((pan < 0 ? 32768 + pan : 32768) >> 1)) >> 14;
      dmod = ((ld == 6 ? lv : 0) - (md == 6 ? sv : 0)) / 37 * 256; /* +-20 ms */
      int32_t dp = (ld == 7 ? lv : 0) - (md == 7 ? sv : 0);   /* drum pitch, +-7 semitones */
      m_dpm = (int32_t)exp2_q16(dp * 14 / 12 * 7 / 7 + (15 << 16));
      m_dlev = clampi(32768 - (ld == 8 ? (ldep - lv) >> 1 : 0) - (md == 8 ? sv : 0), 0, 32768);
      if (pump_hit) pump_hit = 0, penv = 32768;
      else penv = (int32_t)(((int64_t)penv * pcoef) >> 28);
      pg = 32768 - ((pdep * (penv >> 1)) >> 14);
      gl_s = (int32_t)(((int64_t)gl_s * pg) >> 15), gr_s = (int32_t)(((int64_t)gr_s * pg) >> 15);
      pump_show = (uint16_t)(32768 - pg);
    }
    uint32_t t1 = micros();
    drum_job_arg.bus = dbus, drum_job_arg.n = n;
    int two = cpu1_run(drum_job, 0); /* the second core takes the drums, if it runs */
    render_synth(o, sbus, n);
    if (S.pump[3] && pg < 32768)
      for (uint32_t f = 0; f < n; f++) sbus[f] = (int32_t)(((int64_t)sbus[f] * pg) >> 15); /* the sends duck too */
    uint32_t t2 = micros();
    t_syn += t2 - t1;
    if (!two) {
      render_drums(dbus, n);
    } else if (cpu1_wait()) { /* it was retired mid-job: start the drums again here */
      for (uint32_t f = 0; f < n; f++) dbus[f] = 0;
      render_drums(dbus, n);
    }
    t_drm += micros() - t2; /* (with two cores: only the wait) */
    int32_t want_a = frames(k[K_DTA]) * 256 + dmod, want_b = frames(k[K_DTB]) * 256 + dmod;
    if (want_a < 25600) want_a = 25600;
    if (want_b < 25600) want_b = 25600;
    ta_q8 += (want_a - ta_q8) >> 4;
    tb_q8 += (want_b - tb_q8) >> 4;
    for (uint32_t f = 0; f < n; f++) {
      int32_t sb = clamp16(sbus[f]), db = (((clamp16(dbus[f]) * m_dlev) >> 15) * dg12) >> 12;
      int32_t dsend = clamp16(((sb * gsd) >> 15) + ((db * gdd) >> 15));
      int32_t wl, wr;
      if (field) {
        if (!frz) dly_a[dw & DMASK] = (int16_t)((dsend * fsend) >> 15);
        int32_t y0 = dly_b[(dw - ft0) & DMASK], y1 = dly_a[(dw - ft1) & DMASK];
        int32_t y2 = dly_a[(dw - ft2) & DMASK], y3 = dly_a[(dw - ft3) & DMASK];
        lp_a += (int32_t)(((int64_t)(((((y0 + y1 + y2 + y3) >> 2) * ffb) >> 13) - lp_a) * fa) >> 15);
        dly_b[dw & DMASK] = (int16_t)clamp16(dly_a[dw & DMASK] + lp_a);
        dw++;
        wl = (((y0 * 3 + y1 + y2 * 3 + y3) >> 2) * 19660) >> 15; /* taps leaning left, right, left, right */
        wr = (((y0 + y1 * 3 + y2 + y3 * 3) >> 2) * 19660) >> 15;
      } else {
      /* twin delay: A leans left, B right, each low-passed in its loop; freeze holds what is in it */
      int32_t ya = dly_read(dly_a, ta_q8), yb = dly_read(dly_b, tb_q8);
      if (frz) {
        dly_a[dw & DMASK] = (int16_t)ya;
        dly_b[dw & DMASK] = (int16_t)yb;
        lp_a = ya, lp_b = yb;
      } else {
        lp_a += ((ya - lp_a) * 8817) >> 15;
        lp_b += ((yb - lp_b) * 6680) >> 15;
        dly_a[dw & DMASK] = (int16_t)clamp16(dsend + ((lp_a * dfb) >> 15));
        dly_b[dw & DMASK] = (int16_t)clamp16(dsend + ((lp_b * dfb) >> 15));
      }
      dw++;
      wl = (((lp_a * 26214 + lp_b * 13107) >> 15) * 11469) >> 15; /* the return: the sends set how much */
      wr = (((lp_a * 13107 + lp_b * 26214) >> 15) * 11469) >> 15;
      }
      /* space, with dirt: held and coarser input */
      int32_t x = clamp16(((sb * gss) >> 15) + ((db * gds) >> 15)) >> 1;
      if (dirt) {
        if (++rv_held >= dhold) rv_held = 0, rv_hold = x & dmask;
        x = rv_hold;
      }
      int32_t d1 = rv_ap1[rv_i1], y1 = d1 - ((x * 19661) >> 15);
      rv_ap1[rv_i1] = (int16_t)clamp16(x + ((y1 * 19661) >> 15));
      if (++rv_i1 >= AP1) rv_i1 = 0;
      int32_t d2 = rv_ap2[rv_i2], y2 = d2 - ((y1 * 19661) >> 15);
      rv_ap2[rv_i2] = (int16_t)clamp16(y1 + ((y2 * 19661) >> 15));
      if (++rv_i2 >= AP2) rv_i2 = 0;
      for (int i = 0; i < 4; i++) {
        int32_t r = rv_w[i] - eff[i];
        if (r < 0) r += RV_LEN[i];
        rv_lp[i] += ((rv_line[i][r] - rv_lp[i]) * damp) >> 15;
      }
      int32_t a = rv_lp[0] + rv_lp[1], b = rv_lp[0] - rv_lp[1], c = rv_lp[2] + rv_lp[3], d = rv_lp[2] - rv_lp[3];
      int32_t h[4] = {(a + c) >> 1, (b + d) >> 1, (a - c) >> 1, (b - d) >> 1};
      for (int i = 0; i < 4; i++) {
        rv_line[i][rv_w[i]] = (int16_t)clamp16(y2 + ((h[i] * rfb) >> 15));
        if (++rv_w[i] >= RV_LEN[i]) rv_w[i] = 0;
      }
      int32_t rl = (int32_t)(((int64_t)(rv_lp[0] + rv_lp[2]) * rmix) >> 15), rr = (int32_t)(((int64_t)(rv_lp[1] + rv_lp[3]) * rmix) >> 15);
      /* mix, master 0.9, then the limiter: ceiling 2 dB under full scale, release ~190 ms */
      int32_t l = (int32_t)(((((int64_t)o[2 * f] * 19661) >> 15) * gl_s) >> 15) + db + wl + rl;
      int32_t r = (int32_t)(((((int64_t)o[2 * f + 1] * 19661) >> 15) * gr_s) >> 15) + db + wr + rr;
      { /* meters, before the master */
        int32_t sv = (int32_t)(((int64_t)o[2 * f] * 19661) >> 15), sa = sv < 0 ? -sv : sv, da = db < 0 ? -db : db;
        if (sa > bs) bs = sa;
        if (da > bd) bd = da;
      }
      l = (int32_t)(((int64_t)l * 29491 >> 15) * mg12 >> 12);
      r = (int32_t)(((int64_t)r * 29491 >> 15) * mg12 >> 12);
      int32_t pk = l < 0 ? -l : l, pr = r < 0 ? -r : r;
      if (pr > pk) pk = pr;
      if (pk > lim_env) lim_env = pk;
      else lim_env -= (lim_env >> 13) + 1;
      if (lim_env < 26000) lim_env = 26000;
      int32_t g = (26000 * 32768) / lim_env;
      o[2 * f] = (int32_t)(((int64_t)l * g) >> 15);
      o[2 * f + 1] = (int32_t)(((int64_t)r * g) >> 15);
      int32_t oa = o[2 * f] < 0 ? -o[2 * f] : o[2 * f];
      if (oa > bo) bo = oa;
    }
  }
  if (bs > pk_syn) pk_syn = (uint16_t)(bs > 65535 ? 65535 : bs);
  if (bd > pk_drm) pk_drm = (uint16_t)(bd > 65535 ? 65535 : bd);
  if (bo > pk_out) pk_out = (uint16_t)(bo > 65535 ? 65535 : bo);
  if (bo > pk_hdr) pk_hdr = (uint16_t)(bo > 65535 ? 65535 : bo);
  if (lim_env > 26600) limiting = 1;
  uint32_t used = micros() - t0, budget = nframes * 22676u / 1000u;
  load_pct = (uint16_t)((load_pct * 7u + (budget ? used * 100u / budget : 0)) / 8u);
  if (budget) {
    uint32_t fx = used > t_syn + t_drm ? used - t_syn - t_drm : 0;
    sec_pct[0] = (uint16_t)((sec_pct[0] * 7u + t_syn * 100u / budget) / 8u);
    sec_pct[1] = (uint16_t)((sec_pct[1] * 7u + t_drm * 100u / budget) / 8u);
    sec_pct[2] = (uint16_t)((sec_pct[2] * 7u + fx * 100u / budget) / 8u);
  }
}

/* ================================================================ the panel */

#define C_BG RGB(5, 5, 7)
#define C_PANEL RGB(17, 17, 23)
#define C_PANEL2 RGB(26, 26, 36)
#define C_LINE RGB(36, 36, 46)
#define C_LINE2 RGB(50, 50, 64)
#define C_DIM RGB(110, 110, 124)
#define C_MUTE RGB(154, 154, 168)
#define C_TEXT RGB(242, 242, 245)
#define C_VIO RGB(139, 108, 255)
#define C_CYAN RGB(77, 225, 240)
#define C_CORAL RGB(255, 107, 87)
#define C_AMBER RGB(255, 181, 71)
#define C_MINT RGB(61, 242, 160)
#define C_LIME RGB(200, 242, 90)
#define C_PINK RGB(255, 79, 154)

static const uint16_t LANE_COL[LANES] = {RGB(139, 108, 255), RGB(127, 123, 255), RGB(114, 144, 255),
                                         RGB(100, 167, 255), RGB(85, 190, 248),  RGB(77, 225, 240),
                                         RGB(95, 230, 216),  RGB(126, 235, 198)};
static const uint16_t TRACK_COL[TRACKS] = {RGB(255, 107, 87), RGB(255, 181, 71), RGB(255, 215, 138),
                                           RGB(255, 154, 122), RGB(255, 95, 143)};
static const uint16_t CORNER_COL[4] = {RGB(139, 108, 255), RGB(255, 107, 87), RGB(77, 225, 240), RGB(255, 181, 71)};

static uint16_t mix565(uint16_t a, uint16_t b, int t) { /* t 0..256: a -> b */
  int ar = a >> 11, ag = (a >> 5) & 63, ab = a & 31, br = b >> 11, bg = (b >> 5) & 63, bb = b & 31;
  return (uint16_t)(((ar + ((br - ar) * t >> 8)) << 11) | ((ag + ((bg - ag) * t >> 8)) << 5) | (ab + ((bb - ab) * t >> 8)));
}

/* rounded ends: how far row j of a shape of height h comes in at corner radius r */
static uint8_t INS[9][9];
static int isqrt(int v) {
  int r = 0;
  while ((r + 1) * (r + 1) <= v) r++;
  return r;
}
static int inset(int h, int r, int j) {
  if (r > h / 2) r = h / 2;
  if (r > 8) r = 8;
  if (j < r) return INS[r][j];
  if (j >= h - r) return INS[r][h - 1 - j];
  return 0;
}
static void capsule(int x, int y, int w, int h, int r, uint16_t c) {
  for (int j = 0; j < h; j++) {
    int in = inset(h, r, j);
    if (in * 2 < w) rect(x + in, y + j, w - 2 * in, 1, c);
  }
}
static void disk(int cx, int cy, int r, uint16_t c) {
  for (int dy = -r; dy <= r; dy++) {
    int dx = isqrt(r * r - dy * dy);
    rect(cx - dx, cy + dy, 2 * dx + 1, 1, c);
  }
}

static void arc_at(int cx, int cy, int r0, int r1, int u0, int u1, uint16_t c) { /* angles: 1/4096 turn, 0 at the top */
  int n = r1 * (u1 - u0) / 326 + 2;
  for (int i = 0; i <= n; i++) {
    uint32_t ph = (uint32_t)(u0 + (u1 - u0) * i / n) << 20;
    int32_t sn = sine(ph), cs = sine(ph + 1073741824u);
    for (int r = r0; r <= r1; r++) pixel(cx + ((r * sn + 16384) >> 15), cy - ((r * cs + 16384) >> 15), c);
  }
}
static void ring_dots(int cx, int cy, int r, uint16_t c) {
  for (int u = 0; u < 4096; u += 128) arc_at(cx, cy, r, r, u, u + 56, c);
}
/* KNOB1-4 are cyan, lime, white and orange on every page: what a knob moves on screen wears its colour */
static const uint16_t KC[4] = {RGB(77, 225, 240), RGB(200, 242, 90), RGB(242, 242, 245), RGB(255, 138, 61)};

enum { V_HOME, V_SYS, V_SYNTH, V_DRUM, V_PAGE, V_PERF, V_SONG };
enum { PG_OSC, PG_TONE, PG_MOD, PG_ENV, PG_DSND, PG_DSWP, PG_DPRF, PG_DENV, PG_DLY, PG_SPC, PG_BUS, PG_GLO, PG_GLO2,
       PG_PERF, PG_LFO, PG_SMOD, PG_MOD2, PG_UNI, PG_MIX, PG_PUMP, NPG };
static const char *const PAGE_NAME[NPG] = {"OSC", "TONE", "MOD", "ENV", "SOUND", "SWEEP", "PERF", "ENV",
                                           "DELAY", "SPACE", "BUS", "GLOBAL", "TRANSPOSE", "PERFORM", "LFO", "STEP MOD", "INDEX", "UNISON", "MIX", "PUMP"};
#define NPGI 15 /* EDIT 0-1, SEL 2-4, ENV 5, LFO 6, FX 7-12, GLO 13-14 */
static const uint8_t SLIST[NPGI] = {PG_OSC, PG_TONE, PG_MOD, PG_MOD2, PG_UNI, PG_ENV, PG_LFO, PG_DLY, PG_SPC, PG_BUS, PG_MIX, PG_PUMP, PG_SMOD, PG_GLO, PG_GLO2};
static const uint8_t DLIST[NPGI] = {PG_DSND, PG_DSWP, PG_DPRF, PG_DPRF, PG_DPRF, PG_DENV, PG_LFO, PG_DLY, PG_SPC, PG_BUS, PG_MIX, PG_PUMP, PG_SMOD, PG_GLO, PG_GLO2};
#define FX0 7
#define FX1 12
#define GLO0 13
static const char *const DEST_NAME[9] = {"OFF", "FILTER", "INDEX", "PITCH", "LEVEL", "PAN", "DELAY", "DRM PIT", "DRM LVL"};
static const char *const DEST3[9] = {"OFF", "FLT", "IDX", "PIT", "LVL", "PAN", "DLY", "DPT", "DLV"};
#define X_VOL 200
#define X_LEN 201
#define X_TSHIFT 202
#define X_SLEV 203
#define X_DLEV 204
#define X_CH 205 /* the selected lane's level, or the selected drum's volume */
#define X_MAST 206
#define X_PDEP 207 /* the pump: depth, time, source, target */
#define X_PTIME 208
#define X_PSRC 209
#define X_PTGT 210
#define D(p) (100 + (p))
static const uint8_t PAGE_SLOT[NPG][4] = {
  {K_ALGO, K_RATIO, K_INDEX, K_FB},       {K_CUT, K_CONTOUR, K_RESO, K_DRIVE},  {K_MATK, K_MDEC, K_MEND, K_MVEL},
  {K_ATK, K_HOLD, K_REL, K_SUB},          {D(P_CAR), D(P_RAT), D(P_IDX), D(P_PITCH)},
  {D(P_SWD), D(P_SWT), D(P_MDEC), D(P_MEND)}, {D(P_ACC), D(P_PROB), D(P_ECHO), D(P_EDEC)},
  {D(P_ATK), D(P_REL), X_VOL, X_LEN},     {K_DMODE, K_DTA, K_DTB, K_DFB},        {K_SPACE, K_TRAIL, K_DIRT, K_MIX},
  {K_SSD, K_SSS, K_SDD, K_SDS},           {K_BPM, K_SWING, K_SCALE, K_ROOT},    {K_TLEN, K_TEVERY, K_TCUR, X_TSHIFT},
  {K_INT, K_KIT, K_THROW, K_FILT},        {K_LDEST, K_LRATE, K_LDEPTH, K_LSHAPE},
  {K_MSRC, K_MDEST, K_MSHAPE, K_MDEPTH},  {K_KTRK, K_MLOOP, K_OP3, K_DRIFT},     {K_UNI, K_UDET, K_USPR, K_UMIX},
  {X_SLEV, X_DLEV, X_CH, X_MAST},
  {X_PDEP, X_PTIME, X_PSRC, X_PTGT},
};
static const int16_t DB10[128] = {-999, -400, -340, -305, -280, -260, -244, -231, -219, -209, -200, -192, -184, -177, -171, -165, -159, -154, -149, -144, -140, -136, -132, -128, -124, -120, -117, -114, -111, -108, -105, -102, -99, -96, -94, -91, -89, -86, -84, -82, -80, -77, -75, -73, -71, -69, -67, -66, -64, -62, -60, -58, -57, -55, -54, -52, -50, -49, -47, -46, -44, -43, -42, -40, -39, -37, -36, -35, -33, -32, -31, -30, -29, -27, -26, -25, -24, -23, -22, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 16, 17, 17, 18, 19, 19, 20, 21}; /* level 0-127 in tenths of a dB, 100 = 0 dB */
static const uint8_t NOISE_SND[4] = {D(P_IDX), D(P_CAR), D(P_RAT), 255};

static int8_t white_of[KEY_COUNT], black_of[KEY_COUNT];
static int black_key[11];
enum { BK_RND = 8, BK_CLR = 9, BK_F5 = 10, BK_COPY = 5, BK_PASTE = 6, BK_CLRLOCK = 7 };

static uint8_t view = V_HOME, drawn_view = 255, mode, lane, track, spage, rec, playing, pgi, lockset, keys_on, song_cur;
static uint32_t held;
static uint8_t held_used, seq_used, save_used, safe_mode;
static uint32_t rnd_since, f5_since, puck_say;
static char msg[30];
static uint32_t msg_until;
enum { D_HEAD = 1, D_MAIN = 2, D_PAGES = 4, D_CTX = 8, D_VALS = 16, D_GRAPH = 32, D_POP = 64, D_ALL = 127 };
static uint8_t dirty = D_ALL;
static uint8_t hist[54], hist_n;

static struct {
  uint8_t ok, len;
  uint16_t on;
  uint8_t cond[DSTEPS];
  uint8_t lock[DSTEPS][DP];
} clip;

static void say(const char *s) {
  int i = 0;
  for (; s[i] && i < 29; i++) msg[i] = s[i];
  msg[i] = 0;
  msg_until = millis() + 1400;
  dirty |= D_CTX;
}
static char *put(char *d, const char *s) {
  while (*s) *d++ = *s++;
  *d = 0;
  return d;
}
static char *put_int(char *d, int32_t v) {
  char b[12];
  format_int(b, v);
  return put(d, b);
}
static char *put_dec(char *d, int32_t v, int places) {
  int32_t div = places == 1 ? 10 : 100;
  if (v < 0) *d++ = '-', v = -v;
  d = put_int(d, v / div);
  *d++ = '.';
  if (places == 2) *d++ = (char)('0' + v / 10 % 10);
  *d++ = (char)('0' + v % 10);
  *d = 0;
  return d;
}
static char *put_signed(char *d, int32_t v) {
  if (v > 0) *d++ = '+';
  return put_int(d, v);
}
static void center(int x, int w, int y, const char *s, uint16_t c, int font) {
  text(x + (w - text_width(s, font)) / 2, y, s, c, font);
}
static int msg_live(void) { return msg[0] && (int32_t)(msg_until - millis()) > 0; }
static bank_t *CB(void) { return &S.bank[cur_bank]; }
static int cur_page(void) { return view == V_PERF ? PG_PERF : mode ? DLIST[pgi] : SLIST[pgi]; }
static int slot_id(int pg, int i) {
  if (IS_NOISE(track) && pg == PG_DSND) return NOISE_SND[i];
  if (IS_NOISE(track) && pg == PG_DSWP) return 255;
  return PAGE_SLOT[pg][i];
}

static void fmt_k(int i, int v, char *val, const char **unit) {
  *unit = "";
  switch (KD[i].unit) {
    case U_ALGO: put(val, v == 0 ? "2-OP" : v == 1 ? "4-OP" : "PARA"); break;
    case U_FRAC: put(val, FRAC_NAME[clampi(v, 0, NFRAC - 1)]); break;
    case U_DIV10: put_dec(val, v, 1); break;
    case U_PCT: put_int(val, v), *unit = "%"; break;
    case U_MS: put_int(val, v), *unit = "MS"; break;
    case U_SEC100: put_dec(val, v, 2), *unit = "S"; break;
    case U_HZ: {
      uint32_t hz = exp2_q16(LOG2_250 + v * 3277);
      if (hz >= 1000) put_dec(val, (int32_t)(hz / 100), 1), *unit = "K";
      else put_int(val, (int32_t)hz), *unit = "HZ";
      break;
    }
    case U_SCALE: put(val, SCALE_NAME[clampi(v, 0, 3)]); break;
    case U_NOTE: put(val, NOTE_NAME[clampi(v, 0, 11)]); break;
    case U_INT: put_int(val, v); break;
    case U_SYNC: put(val, v ? "EXT" : "INT"); break;
    case U_CENTER: put_signed(val, v - 50); break;
    case U_DEST: put(val, DEST_NAME[clampi(v, 0, 6)]); break;
    case U_SHAPE: put(val, v == 0 ? "SINE" : v == 1 ? "TRI" : v == 2 ? "SQR" : "S+H"); break;
    case U_TRACK: put(val, TRACK_NAME[clampi(v, 0, TRACKS - 1)]); break;
    case U_DMODE: put(val, v ? "FIELD" : "TWIN"); break;
    case U_LOOP: {
      static const char *const LN[5] = {"OFF", "1/16", "1/8", "1/4", "1/2"};
      put(val, LN[clampi(v, 0, 4)]);
      break;
    }
    case U_CENTS: put_int(val, v), *unit = "CT"; break;
    case U_TLEN: v ? put_int(val, v) : put(val, "OFF"); break;
    case U_EVERY: put_int(val, 1 << v), *unit = v ? "BARS" : "BAR"; break;
    case U_STOP: put_int(val, v + 1); break;
    case U_ONOFF: put(val, v ? "ON" : "OFF"); break;
    case U_LRATE: {
      if (v > 100) {
        static const char *const SY[8] = {"4BAR", "2BAR", "1BAR", "1/2", "1/4", "1/8", "1/16", "1/32"};
        put(val, SY[v - 101]);
        break;
      }
      int32_t hz100 = (int32_t)(exp2_q16(v * 5661 + (10 << 16)) * 5u / 1024u);
      if (hz100 >= 1000) put_dec(val, hz100 / 10, 1);
      else put_dec(val, hz100, 2);
      *unit = "HZ";
      break;
    }
    default: val[0] = 0;
  }
}
static void fmt_d(int kind, int t, char *val, const char **unit) {
  *unit = "";
  switch (kind) {
    case F_FRAC: put(val, FRAC_NAME[clampi(t, 0, NFRAC - 1)]); break;
    case F_DIV10: put_dec(val, t, 1); break;
    case F_PITCH: put_signed(val, t - 12), *unit = "ST"; break;
    case F_SEMI: put_signed(val, t), *unit = "ST"; break;
    case F_MS2: put_int(val, t * 2), *unit = "MS"; break;
    case F_MS5: put_int(val, t * 5), *unit = "MS"; break;
    case F_PCT: put_int(val, t), *unit = "%"; break;
    case F_MS: put_int(val, t), *unit = "MS"; break;
    case F_ACC: put_dec(val, t * 5, 2); break;
    case F_PCT5: put_int(val, t * 5), *unit = "%"; break;
    case F_INT: put_int(val, t); break;
    case F_HZ50:
      if (t * 50 >= 1000) put_dec(val, t * 50 / 100, 1), *unit = "K";
      else put_int(val, t * 50), *unit = "HZ";
      break;
    case F_NTYPE: put(val, NTYPE_NAME[t > 4 ? 4 : t]); break;
    case F_TICK: put_int(val, t), *unit = "/6"; break;
    default: put(val, "-");
  }
}

/* ---- the header: mode, page name, a dot per page under that button, bank, CPU, tempo */
static int hm_drawn;
static void draw_head(void) {
  char b[24], *d;
  rect(0, 0, SCREEN_W, 18, C_BG);
  capsule(3, 1, 30, 16, 8, mode ? C_CORAL : C_VIO);
  center(3, 30, 1, mode ? "DRM" : "SYN", C_BG, FONT_SMALL);
  const char *nm = view == V_HOME ? "ORBIT" : view == V_SYS ? "SYSTEM" : view == V_SONG ? "SONG" : PAGE_NAME[cur_page()];
  int x = text(38, 1, nm, C_TEXT, FONT_SMALL) + 4;
  if (view >= V_SYNTH && view != V_PERF) {
    int n = 1, at = 0;
    if (pgi <= 1) n = 2, at = pgi;
    else if (pgi <= 4) n = mode ? 1 : 3, at = mode ? 0 : pgi - 2;
    else if (pgi >= FX0 && pgi <= FX1) n = FX1 - FX0 + 1, at = pgi - FX0;
    else if (pgi >= GLO0) n = 2, at = pgi - GLO0;
    if (n > 1) {
      for (int i = 0; i < n; i++) disk(x + 3 + i * 8, 9, 2, i == at ? C_TEXT : C_DIM);
      x += n * 8 + 4;
    }
  }
  if (rec) disk(x + 4, 9, 3, C_PINK), x += 10;
  if (keys_on && !mode) text(x + 4, 1, "KEYS", C_LIME, FONT_SMALL);
  d = b;
  *d++ = (char)('A' + cur_bank);
  if (queued_bank >= 0) *d++ = '>', *d++ = (char)('A' + queued_bank);
  *d++ = ' ';
  d = put_int(d, load_pct);
  d = put(d, "% ");
  if (S.k[K_SYNC]) put(d, "EXT");
  else put_int(d, S.k[K_BPM]);
  text(SCREEN_W - 9 - text_width(b, FONT_SMALL), 1, b, load_pct > 80 ? C_PINK : playing ? C_LIME : C_DIM, FONT_SMALL);
  rect(0, 18, SCREEN_W, 1, C_LINE);
  hm_drawn = -1;
}
/* the output meter at the header's right end, on every page: pink while the limiter works */
static int hm_drawn = -1;
static uint16_t hm_v;
static uint8_t hm_lim;
static void draw_hmeter(void) {
  uint16_t p = pk_hdr;
  pk_hdr = 0;
  hm_v = p > hm_v ? p : hm_v - (hm_v >> 3);
  int h = 0;
  uint32_t q = hm_v;
  while (q > 64 && h < 16) q >>= 1, h += 2;
  h = h * 14 / 16;
  uint8_t lim = hm_v > 25000;
  if (h == hm_drawn && lim == hm_lim) return;
  hm_drawn = h, hm_lim = lim;
  rect(234, 2, 3, 14, C_PANEL);
  if (h) rect(234, 16 - h, 3, h, lim ? C_PINK : C_LIME);
}

/* ================================================================ morph: the puck's X and Y blend the sounds of
 * the four corners. A top left, B top right, C bottom left, D bottom right. */
static void morph_xy(int *mx, int *my) { *mx = S.xy[0], *my = S.xy[1]; }
static int on_corner(void) { return S.xy[0] % 100 == 0 && S.xy[1] % 100 == 0; }
static void apply_morph(void) {
  int x, y;
  morph_xy(&x, &y);
  int32_t w[4] = {(100 - x) * (100 - y), x * (100 - y), (100 - x) * y, x * y};
  for (int i = 0; i < NMORPH; i++) {
    int32_t v = 0;
    for (int c = 0; c < 4; c++) v += S.corner[c][i] * w[c];
    int id = MORPH_K[i];
    S.k[id] = (int16_t)clampi((v + 5000) / 10000, KD[id].min, KD[id].max);
  }
}
static int nearest_corner(void) {
  int x, y;
  morph_xy(&x, &y);
  return (x >= 50) | (y >= 50) << 1;
}

/* ================================================================ SYNTH: eight lanes as rounded columns */
#define LY 20
#define LW 25
#define SP 5 /* pixels a step */
static int LX(int l) { return 4 + l * 29; }
static int8_t cap_start[MAXSTEP];
static uint8_t cap_span[MAXSTEP];
static int8_t drawn_s[LANES] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int8_t drawn_d[TRACKS] = {-1, -1, -1, -1, -1};
static uint32_t drawn_pulse = 0xFFFFFFFFu;

static void compute_caps(int l) {
  const bank_t *b = CB();
  int len = b->lane[l].len;
  for (int s = 0; s < MAXSTEP; s++) cap_start[s] = -1;
  for (int s = 0; s < len; s++) {
    int d = DUR(&b->st[l][s]);
    if (!d) continue;
    int span = d;
    for (int j = 1; j < span; j++)
      if (s + j >= len || DUR(&b->st[l][s + j])) {
        span = j;
        break;
      }
    cap_span[s] = (uint8_t)span;
    for (int j = 0; j < span; j++) cap_start[s + j] = (int8_t)s;
  }
}
static int sph(int l) { return playing ? spos[l] : -1; }

static void draw_prow(int l, int py) {
  const bank_t *b = CB();
  int len = b->lane[l].len, x0 = LX(l), bot = LY + len * SP + 2;
  rect(x0, py, LW, 1, C_BG);
  if (py < LY || py >= bot) return;
  int ci = inset(bot - LY, 3, py - LY);
  int s = py > LY ? (py - LY - 1) / SP : -1, ph = sph(l);
  if (s >= len) s = -1;
  uint16_t bg = l == lane ? C_PANEL2 : C_PANEL;
  if ((smute >> l) & 1) bg = C_BG;
  if (s >= 0 && s == ph) bg = C_LINE2;
  if (s >= 0 && l == lane && !mode && ((held >> s) & 1)) bg = RGB(70, 26, 50);
  rect(x0 + ci, py, LW - 2 * ci, 1, bg);
  if (s < 0) return;
  if (TRIGLESS(&b->st[l][s])) { /* a trigless step: a hollow outline */
    int sj = (py - LY - 1) % SP;
    uint16_t tc = mix565(C_PANEL, LANE_COL[l], 200);
    if (sj == 0 || sj == SP - 2) rect(x0 + 6, py, 13, 1, tc);
    else if (sj < SP - 1) rect(x0 + 6, py, 1, 1, tc), rect(x0 + 18, py, 1, 1, tc);
    return;
  }
  int k = cap_start[s];
  if (k < 0) return;
  int ctop = LY + 1 + k * SP, ch = cap_span[k] * SP - 1, cj = py - ctop;
  if (cj >= ch) return;
  uint16_t col = LANE_COL[l];
  if (ph >= k && ph < k + cap_span[k]) col = mix565(col, C_TEXT, 150);
  else if ((smute >> l) & 1) col = C_LINE2;
  else if (l != lane) col = mix565(C_PANEL, col, 200);
  int cin = inset(ch, 4, cj);
  rect(x0 + 4 + cin, py, 17 - 2 * cin, 1, col);
  if (cj < 3) {
    const sstep_t *st = &b->st[l][k];
    if (LOCKED(st)) rect(x0 + 22, py, 2, 1, C_PINK);
    if (COND(st)) rect(x0 + 1, py, 2, 1, C_CYAN);
  }
}
static void draw_lane_frame(int l) {
  const bank_t *b = CB();
  int x0 = LX(l), bot = LY + b->lane[l].len * SP + 2;
  rect(x0 - 2, LY, 1, MAXSTEP * SP + 3, C_BG);
  rect(x0 + LW + 1, LY, 1, MAXSTEP * SP + 3, C_BG);
  if (l != lane || mode) return;
  uint16_t dimc = mix565(C_BG, C_LIME, 90);
  rect(x0 - 2, LY + 2, 1, bot - LY - 4, dimc);
  rect(x0 + LW + 1, LY + 2, 1, bot - LY - 4, dimc);
  int s0 = spage ? 16 : 0, s1 = spage ? 32 : 16, len = b->lane[l].len;
  if (s1 > len) s1 = len;
  if (s1 > s0) rect(x0 - 2, LY + 1 + s0 * SP, 1, (s1 - s0) * SP, C_LIME);
}
static void draw_lane(int l) {
  if (view != V_SYNTH) return;
  compute_caps(l);
  for (int py = LY; py < LY + MAXSTEP * SP + 2; py++) draw_prow(l, py);
  draw_lane_frame(l);
  drawn_s[l] = (int8_t)sph(l);
}
static void redraw_sstep(int l, int s) { /* the step's rows, or the whole capsule it sits in */
  if (s < 0 || view != V_SYNTH) return;
  compute_caps(l);
  int a = s, z = s;
  if (cap_start[s] >= 0) a = cap_start[s], z = a + cap_span[a] - 1;
  for (int py = LY + 1 + a * SP; py < LY + 1 + (z + 1) * SP; py++) draw_prow(l, py);
}

static int lane_key_held(void) {
  if (mode) return -1;
  for (int i = 0; i < LANES; i++)
    if (key_held(black_key[i])) return i;
  return -1;
}
static int track_key_held(void) {
  if (!mode) return -1;
  for (int i = 0; i < TRACKS; i++)
    if (key_held(black_key[i])) return i;
  return -1;
}
static int first_held(void) {
  for (int s = 0; s < MAXSTEP; s++)
    if ((held >> s) & 1) return s;
  return -1;
}

static int path_state(void);
static void path_ctx(char *b);
static void draw_ctx(int y) {
  char b[40], *d;
  rect(0, y, SCREEN_W, 17, C_BG);
  if (msg_live()) {
    center(0, SCREEN_W, y, msg, C_LIME, FONT_SMALL);
    return;
  }
  msg[0] = 0;
  const bank_t *bk = CB();
  if (view == V_SONG) {
    d = put(b, S.k[K_SONG] ? "SONG ON" : "SONG OFF");
    if (S.k[K_SONG] && playing && S.song_n) {
      d = put(d, "  BLOCK ");
      d = put_int(d, song_pos + 1);
      d = put(d, "/");
      put_int(d, S.song_n);
    } else {
      put(d, "  1-4 ADD  BLACK MUTE");
    }
    center(0, SCREEN_W, y, b, S.k[K_SONG] ? C_LIME : C_DIM, FONT_SMALL);
    return;
  }
  if (view == V_PERF && path_state()) {
    path_ctx(b);
    center(0, SCREEN_W, y, b, C_LIME, FONT_SMALL);
    return;
  }
  if (view == V_PERF) {
    d = put(b, on_corner() ? "ON " : "NEAR ");
    *d++ = (char)('A' + nearest_corner());
    if (on_corner()) {
      put(d, ": KNOBS EDIT THIS SOUND");
    } else {
      d = put(d, "  X ");
      d = put_int(d, S.xy[0]);
      d = put(d, "  Y ");
      put_int(d, S.xy[1]);
    }
    center(0, SCREEN_W, y, b, CORNER_COL[nearest_corner()], FONT_SMALL);
    return;
  }
  int s = first_held();
  if (!mode) {
    int lk = lane_key_held();
    const lane_t *ln = &bk->lane[lk >= 0 ? lk : lane];
    if (lk >= 0) {
      d = put(b, "LANE ");
      d = put_int(d, lk + 1);
      put(d, " SETTINGS");
      center(0, SCREEN_W, y, b, LANE_COL[lk], FONT_SMALL);
    } else if (s >= 0) {
      const sstep_t *st = &bk->st[lane][s];
      d = put(b, "STEP ");
      d = put_int(d, s + 1);
      if (lockset) d = put(d, " LOCKS B");
      else if (TRIGLESS(st)) d = put(d, " TRIGLESS");
      else d = put(d, LOCKED(st) ? " LOCKED " : " ");
      if (!lockset) put(d, COND_NAME[COND(st)]);
      center(0, SCREEN_W, y, b, LOCKED(st) ? C_PINK : C_MUTE, FONT_SMALL);
    } else {
      int note = lane_note(ln->deg, ln->oct);
      d = put(b, "LANE ");
      d = put_int(d, lane + 1);
      d = put(d, "  ");
      d = put(d, NOTE_NAME[note % 12]);
      d = put_int(d, note / 12 - 1);
      d = put(d, "  ");
      d = put(d, DIR_NAME[ln->dir]);
      d = put(d, "  ");
      put_int(d, ln->len);
      center(0, SCREEN_W, y, b, LANE_COL[lane], FONT_SMALL);
    }
  } else {
    int tk = track_key_held();
    if (tk >= 0) {
      d = put(b, TRACK_NAME[tk]);
      d = put(d, ": KNOB1 = LENGTH ");
      put_int(d, bk->dlen[tk]);
      center(0, SCREEN_W, y, b, TRACK_COL[tk], FONT_SMALL);
    } else if (s >= 0) {
      int any = 0;
      for (int p = 0; p < DP; p++) any |= bk->lock[track][s][p] != UNLOCKED;
      d = put(b, TRACK_NAME[track]);
      d = put(d, " STEP ");
      d = put_int(d, s + 1);
      if (lockset) d = put(d, " B");
      if (bk->dcond[track][s] & 16) d = put(d, " TRIGLESS ");
      else d = put(d, any ? " LOCKED " : " ");
      put(d, COND_NAME[bk->dcond[track][s] & 15]);
      center(0, SCREEN_W, y, b, any ? C_PINK : C_MUTE, FONT_SMALL);
    } else {
      d = put(b, TRACK_NAME[track]);
      d = put(d, (dmute >> track) & 1 ? "  MUTED" : "");
      d = put(d, rec ? "  REC: TAP THE TRACK KEY" : "");
      center(0, SCREEN_W, y, b, TRACK_COL[track], FONT_SMALL);
    }
  }
}

static int cell_id; /* the setting behind the last cell, or -1 */
static int first_held(void);
static int cell_slot(int i) { /* the drum setting KNOB i edits now: lock set B while held steps say so */
  static const uint8_t DSETB[4] = {100 + P_WAIT, 100 + P_ECHO, 100 + P_EDEC, 100 + P_PROB};
  if (mode && lockset && first_held() >= 0 && view != V_PERF) return DSETB[i];
  return slot_id(cur_page(), i);
}
static int layer_shown(void);
enum { L_NONE, L_FX, L_SEQ, L_GLO };
/* what KNOB i does right now: its label, value, unit and 0..100 position. 0: nothing */
static int get_cell(int i, const char **label, char *val, const char **unit, int *norm) {
  const bank_t *bk = CB();
  int s = first_held();
  if (s < 0 && rec && playing && view == V_SYNTH && !mode && spos[lane] >= 0) s = spos[lane]; /* recording knobs */
  *unit = "", *norm = 0, val[0] = 0, cell_id = -1;
  if (view < V_SYNTH) return 0;
  int lay = layer_shown(), lid = -1;
  if (lay == L_FX) {
    static const uint8_t F[4] = {K_INT, K_KIT, K_THROW, K_FILT};
    lid = F[i];
  } else if (lay == L_GLO) {
    static const uint8_t G[4] = {K_BPM, K_SWING, K_SCALE, K_ROOT};
    lid = G[i];
  } else if (lay == L_SEQ) {
    static const char *const L[4] = {"LENGTH", "HITS", "ROTATE", "SWING"};
    int len = mode ? bk->dlen[track] : bk->lane[lane].len, hits = 0;
    for (int q = 0; q < len; q++) hits += mode ? (bk->on[track] >> q) & 1 : DUR(&bk->st[lane][q]) != 0;
    *label = L[i];
    if (i == 0) put_int(val, len), *norm = len * 100 / (mode ? DSTEPS : MAXSTEP);
    if (i == 1) put_int(val, hits), *norm = hits * 100 / (len ? len : 1);
    if (i == 2) put(val, "< >"), *norm = 50;
    if (i == 3) {
      cell_id = K_SWING;
      fmt_k(K_SWING, S.k[K_SWING], val, unit), *norm = S.k[K_SWING] * 100 / 75;
    }
    return 1;
  }
  if (lid >= 0) {
    cell_id = lid;
    fmt_k(lid, S.k[lid], val, unit);
    *label = KD[lid].label;
    *norm = (S.k[lid] - KD[lid].min) * 100 / (KD[lid].max - KD[lid].min);
    return 1;
  }
  if (view != V_PERF && lane_key_held() >= 0) {
    static const char *const L[4] = {"LENGTH", "DIR", "DEGREE", "OCTAVE"};
    const lane_t *ln = &bk->lane[lane_key_held()];
    *label = L[i];
    if (i == 0) put_int(val, ln->len), *norm = ln->len * 100 / MAXSTEP;
    if (i == 1) put(val, DIR_NAME[ln->dir]), *norm = ln->dir * 33;
    if (i == 2) put_int(val, ln->deg + 1), *norm = ln->deg * 100 / 13;
    if (i == 3) put_signed(val, ln->oct - 2), *norm = ln->oct * 25;
    return 1;
  }
  if (view == V_SONG) {
    static const char *const L[4] = {"MOVE", "BANK", "REPEAT", "SONG"};
    *label = L[i];
    if (i == 0) put_int(val, song_cur + 1), *norm = song_cur * 100 / 15;
    if (i == 1) val[0] = S.song_n ? (char)('A' + S.song_bank[song_cur]) : '-', val[1] = 0, *norm = S.song_bank[song_cur] * 33;
    if (i == 2) put_int(val, S.song_n ? S.song_rep[song_cur] : 0), *unit = "BAR", *norm = S.song_rep[song_cur] * 100 / 8;
    if (i == 3) cell_id = K_SONG, put(val, S.k[K_SONG] ? "ON" : "OFF"), *norm = S.k[K_SONG] * 100;
    return 1;
  }
  if (view != V_PERF && !mode && s >= 0 && lockset) {
    static const char *const L[4] = {"WAIT", "ECHOES", "LADDER", "FADE"};
    static const char *const SH[4] = {"SAME", "UP", "DOWN", "RAND"};
    uint8_t x = bk->xb[lane][s];
    *label = L[i];
    if (i == 0) put_int(val, x & 7), *unit = "/6", *norm = (x & 7) * 20;
    if (i == 1) put_int(val, (x >> 3) & 3), *norm = ((x >> 3) & 3) * 33;
    if (i == 2) put(val, SH[(x >> 5) & 3]), *norm = ((x >> 5) & 3) * 33;
    if (i == 3) put(val, x & 128 ? "SLOW" : "FAST"), *norm = x & 128 ? 100 : 30;
    return 2;
  }
  if (view != V_PERF && !mode && s >= 0) {
    static const char *const L[4] = {"PITCH", "VEL", "INDEX", "SEND"};
    const sstep_t *st = &bk->st[lane][s];
    int lk = LOCKED(st), p = lk ? st->pitch : 0, v = lk ? st->vel : 85, x = lk ? st->index : 0, sd = lk ? st->send : 50;
    *label = L[i];
    if (i == 0) put_signed(val, p), *unit = "ST", *norm = (p + 12) * 100 / 24;
    if (i == 1) put_int(val, v), *norm = v;
    if (i == 2) put_dec(val, x, 1), *norm = (x + 80) * 100 / 160;
    if (i == 3) put_int(val, sd), *norm = sd;
    return 2;
  }
  int id = cell_slot(i);
  if (id == 255) return 0;
  if (id >= X_PDEP && id <= X_PTGT) {
    static const char *const PL[4] = {"DEPTH", "TIME", "SOURCE", "TARGET"};
    int v = S.pump[id - X_PDEP];
    *label = PL[id - X_PDEP];
    if (id == X_PDEP) put_int(val, v), *unit = "%", *norm = v;
    if (id == X_PTIME) put_int(val, 30 + v * 57 / 10), *unit = "MS", *norm = v;
    if (id == X_PSRC) put(val, v ? "BEAT" : "KICK"), *norm = v * 100;
    if (id == X_PTGT) put(val, v ? "SYN+FX" : "SYNTH"), *norm = v * 100;
    return 1;
  }
  if (id >= X_SLEV && id <= X_MAST) {
    static const char *const ML[4] = {"SYNTH", "DRUMS", "LANE", "MASTER"};
    int v = id == X_SLEV ? S.mix[0] : id == X_DLEV ? S.mix[1] : id == X_MAST ? S.mix[2] : mode ? S.vol[track] : S.llev[lane];
    *label = id == X_CH && mode ? TRACK_NAME[track] : ML[id - X_SLEV];
    if (id == X_CH && mode) {
      put_int(val, v), *unit = "%", *norm = v;
    } else if (!v) {
      put(val, "OFF");
    } else {
      int t = DB10[v], a = t < 0 ? -t : t;
      char *d = val;
      if (t > 0) *d++ = '+';
      if (t < 0) *d++ = '-';
      d = put_int(d, a / 10);
      *d++ = '.';
      put_int(d, a % 10);
      *unit = "DB";
    }
    if (!(id == X_CH && mode)) *norm = v * 100 / 127;
    return 1;
  }
  if (id == X_TSHIFT) {
    int v = S.tseq[S.k[K_TCUR]];
    put_signed(val, v), *unit = "ST", *label = "SHIFT", *norm = (v + 7) * 100 / 14;
    return 1;
  }
  if (S.k[K_DMODE] && (id == K_DTA || id == K_DTB || id == K_DFB)) { /* the echo field's names for them */
    int v = id == K_DTA ? clampi(S.k[K_DTA] * 100 / 400, 25, 140) : id == K_DTB ? clampi((S.k[K_DTB] - 20) * 100 / 720, 0, 100) : S.k[K_DFB] * 100 / 90;
    cell_id = id;
    put_int(val, v), *unit = "%", *label = id == K_DTA ? "SIZE" : id == K_DTB ? "GRAVITY" : "TRAIL", *norm = v > 100 ? 100 : v;
    return 1;
  }
  if (id < 100) {
    cell_id = id;
    fmt_k(id, S.k[id], val, unit);
    *label = KD[id].label;
    *norm = (S.k[id] - KD[id].min) * 100 / (KD[id].max - KD[id].min);
    return 1;
  }
  if (id == X_VOL) {
    put_int(val, S.vol[track]), *unit = "%", *label = "VOLUME", *norm = S.vol[track];
    return 1;
  }
  if (id == X_LEN) {
    put_int(val, bk->dlen[track]), *label = "LENGTH", *norm = bk->dlen[track] * 100 / DSTEPS;
    return 1;
  }
  int p = id - 100, t = S.def[track][p], r = 1;
  const pdef_t *pd = &defs(track)[p];
  if (pd->kind == F_NONE) return 0;
  if (s >= 0 && s < DSTEPS && mode && view != V_PERF && bk->lock[track][s][p] != UNLOCKED) t = bk->lock[track][s][p], r = 2;
  fmt_d(pd->kind, t, val, unit);
  *label = pd->label;
  *norm = pd->max > pd->min ? (t - pd->min) * 100 / (pd->max - pd->min) : 0;
  return r;
}

static int mod_hits(int dest, int id) { /* does destination dest move setting id (a K id, or a slot id) */
  static const int16_t A[9] = {-1, K_CUT, K_INDEX, -1, X_SLEV, -1, K_DTA, 100 + P_PITCH, X_DLEV};
  return dest > 0 && dest < 9 && (A[dest] == id || (dest == 6 && id == K_DTB) || (dest == 1 && id == K_FILT) ||
                                  (dest == 8 && id == X_CH && mode) || (dest == 7 && id == 100 + P_CAR && mode));
}
static int mod_mark(int id, int *depth) { /* 1: the LFO moves this setting, 2: the step mod does; depth 0-100 */
  if (id < 0) return 0;
  if (S.k[K_LDEPTH] && mod_hits(S.k[K_LDEST], id)) return *depth = S.k[K_LDEPTH], 1;
  if (S.k[K_MDEPTH] && mod_hits(S.k[K_MDEST], id)) return *depth = S.k[K_MDEPTH], 2;
  return 0;
}
static void draw_strip(void) { /* per knob: its colour, the name, the value. A ring marks what is being modulated */
  rect(0, 203, SCREEN_W, 37, C_BG);
  for (int i = 0; i < 4; i++) {
    const char *l, *u;
    char v[24], b[8], t[12], vv[32];
    int n, j = 0, n2 = 0, r = get_cell(i, &l, v, &u, &n);
    if (!r) continue;
    for (int q = 0; l[q] && n2 < 11; q++)
      if (l[q] != ' ') t[n2++] = l[q];
    for (int q = n2 - 1; q > 0 && n2 > 6; q--)
      if (t[q] == 'A' || t[q] == 'E' || t[q] == 'I' || t[q] == 'O' || t[q] == 'U') {
        for (int z = q; z < n2 - 1; z++) t[z] = t[z + 1];
        n2--;
      }
    for (; j < n2 && j < 6; j++) b[j] = t[j];
    b[j] = 0;
    int dep = 0, mk = mod_mark(cell_id >= 0 ? cell_id : cell_slot(i), &dep);
    if (mk) { /* a range ring: how far the LFO (pink) or step mod (amber) moves it */
      ring_dots(i * 60 + 5, 212, 5, C_LINE2);
      arc_at(i * 60 + 5, 212, 4, 6, 0, 40 + dep * 4056 / 100, mk == 1 ? C_PINK : C_AMBER);
    }
    disk(i * 60 + 5, 212, mk ? 2 : 3, KC[i]);
    text(i * 60 + 11, 204, b, C_MUTE, FONT_SMALL);
    char *d = put(vv, v);
    put(d, u);
    vv[7] = 0;
    text(i * 60 + 2, 221, vv, r == 2 ? C_PINK : KC[i], FONT_SMALL);
  }
}
static void show_pop(int i) {
  (void)i;
  dirty |= D_VALS;
}

/* ================================================================ DRUMS */
static int step_x(int s) { return 4 + (s >> 2) * 59 + (s & 3) * 14; }
static int dph(int t) { return playing ? dpos[t] : -1; }
static int acc_at(const bank_t *b, int t, int s) {
  uint8_t l = b->lock[t][s][P_ACC];
  return l == UNLOCKED ? S.def[t][P_ACC] : l;
}
static void draw_tabs(void) {
  for (int t = 0; t < TRACKS; t++) {
    int x = t * 48;
    rect(x, 20, 48, 17, C_BG);
    int m = (dmute >> t) & 1;
    if (t == track) capsule(x + 2, 20, 44, 17, 8, TRACK_COL[t]);
    center(x, 48, 20, TRACK_NAME[t], t == track ? C_BG : m ? C_LINE2 : mix565(C_BG, TRACK_COL[t], 170), FONT_SMALL);
    if (m) rect(x + 6, 28, 36, 1, C_PINK);
  }
}
static void draw_pad(int s) {
  if (view != V_DRUM) return;
  const bank_t *b = CB();
  int x = step_x(s), len = b->dlen[track];
  rect(x, 40, 12, 51, C_BG);
  if (s >= len) {
    frame(x, 40, 12, 50, C_PANEL);
    return;
  }
  int ph = dph(track) == s;
  frame(x, 40, 12, 50, ph ? C_TEXT : (held >> s) & 1 ? C_PINK : C_LINE);
  rect(x + 1, 41, 10, 48, C_PANEL);
  if ((b->on[track] >> s) & 1) {
    int h = 48 * acc_at(b, track, s) / 40;
    rect(x + 1, 89 - h, 10, h, ph ? mix565(TRACK_COL[track], C_TEXT, 120) : TRACK_COL[track]);
  }
  int any = 0;
  for (int p = 0; p < DP; p++) any |= b->lock[track][s][p] != UNLOCKED;
  if (any) rect(x + 8, 42, 2, 2, C_PINK);
  if (b->dcond[track][s] & 15) rect(x + 2, 42, 2, 2, C_CYAN);
  if (b->dcond[track][s] & 16) frame(x + 2, 60, 8, 26, TRACK_COL[track]); /* trigless: a hollow bar */
}
static void draw_ov_cell(int t, int s) {
  if (view != V_DRUM) return;
  const bank_t *b = CB();
  int x = step_x(s), y = 96 + t * 9;
  if (s >= b->dlen[t]) {
    rect(x, y, 12, 6, C_BG);
    return;
  }
  int on = (b->on[t] >> s) & 1, ph = dph(t) == s;
  uint16_t c = on ? (t == track ? TRACK_COL[t] : mix565(C_BG, TRACK_COL[t], 150)) : mix565(C_BG, TRACK_COL[t], 34);
  if (ph) c = on ? C_TEXT : C_LINE2;
  if ((dmute >> t) & 1 && on && !ph) c = C_LINE;
  rect(x, y, 12, 6, c);
}
static void drum_wave(int yc, int amp, int yp);
static uint16_t PBG;
static void draw_dface_graph(void) {
  PBG = C_BG;
  rect(0, 141, SCREEN_W, 44, C_BG);
  drum_wave(160, 15, 183);
}
static void draw_drum_main(void) {
  rect(0, 19, SCREEN_W, 166, C_BG);
  draw_tabs();
  for (int s = 0; s < DSTEPS; s++) draw_pad(s);
  for (int t = 0; t < TRACKS; t++)
    for (int s = 0; s < DSTEPS; s++) draw_ov_cell(t, s);
  for (int t = 0; t < TRACKS; t++) drawn_d[t] = (int8_t)dph(t);
  draw_dface_graph();
}

/* ================================================================ HOME: the orbit */
#define OCX 120
#define OCY 130
static int ring_r(int l) { return 24 + l * 10; }
static void arc_px(int r0, int r1, int u0, int u1, uint16_t c) { arc_at(OCX, OCY, r0, r1, u0, u1, c); }
static void draw_seg(int l, int s, int erase) {
  const bank_t *b = CB();
  int len = b->lane[l].len, r = ring_r(l);
  int u0 = s * 4096 / len, u1 = (s + 1) * 4096 / len, gap = 978 / r;
  if (u1 - u0 > 2 * gap + 4) u0 += gap / 2 + 1, u1 -= gap / 2 + 1;
  if (erase) arc_px(r - 2, r + 2, u0, u1, C_BG);
  int on = DUR(&b->st[l][s]) != 0, ph = sph(l) == s;
  uint16_t c = on ? LANE_COL[l] : C_LINE;
  if (ph) c = on ? C_TEXT : RGB(90, 90, 104);
  arc_px(ph ? r - 2 : r - 1, ph ? r + 2 : r + 1, u0, u1, c);
}
static void draw_halo(int s) {
  const bank_t *b = CB();
  uint32_t ph = (uint32_t)(s * 256) << 20;
  int x = OCX + ((104 * sine(ph) + 16384) >> 15), y = OCY - ((104 * sine(ph + 1073741824u) + 16384) >> 15);
  disk(x, y, 4, C_BG);
  int n = 0;
  for (int t = 0; t < TRACKS; t++) n += (b->on[t] >> s) & 1;
  if (dph(0) == s) disk(x, y, 3, C_TEXT);
  else if (n) disk(x, y, n > 1 ? 3 : 2, (b->on[0] >> s) & 1 ? C_CORAL : C_AMBER);
  else disk(x, y, 1, C_LINE2);
}
static uint32_t gcd32(uint32_t a, uint32_t b) {
  while (b) {
    uint32_t t = a % b;
    a = b, b = t;
  }
  return a;
}
static void draw_orbit_text(void) {
  char b[24], *d;
  const bank_t *bk = CB();
  uint32_t per = 1;
  for (int l = 0; l < LANES; l++) {
    if (bk->lane[l].dir == 3) continue;
    uint32_t p = (uint32_t)lane_period(&bk->lane[l]);
    per = per / gcd32(per, p) * p;
    if (per > 100000u) break;
  }
  rect(0, 20, 76, 34, C_BG);
  text(4, 20, "REALIGN", C_DIM, FONT_SMALL);
  if (per > 100000u) put(b, "NEVER");
  else if (per <= 1) put(b, "IN STEP");
  else {
    uint32_t rem = playing ? per - pulses % per : per;
    d = put_int(b, (int32_t)((rem + 15) / 16));
    put(d, (rem + 15) / 16 == 1 ? " BAR" : " BARS");
  }
  text(4, 36, b, C_LIME, FONT_SMALL);
  int ns = 0, nd = 0;
  for (int i = 0; i < MAXSV; i++) ns += SV[i].active;
  for (int i = 0; i < DV; i++) nd += DVo[i].active;
  rect(164, 20, 76, 34, C_BG);
  d = put(b, "SYN ");
  d = put_int(d, ns);
  d = put(d, "/");
  put_int(d, vcap);
  text(236 - text_width(b, FONT_SMALL), 20, b, C_VIO, FONT_SMALL);
  d = put(b, "DRM ");
  d = put_int(d, nd);
  put(d, "/6");
  text(236 - text_width(b, FONT_SMALL), 36, b, C_CORAL, FONT_SMALL);
}
static void draw_orbit(void) {
  rect(0, 19, SCREEN_W, SCREEN_H - 19, C_BG);
  const bank_t *b = CB();
  for (int l = 0; l < LANES; l++) {
    for (int s = 0; s < b->lane[l].len; s++) draw_seg(l, s, 0);
    drawn_s[l] = (int8_t)sph(l);
  }
  for (int s = 0; s < DSTEPS; s++) draw_halo(s);
  drawn_d[0] = (int8_t)dph(0);
  capsule(OCX - 14, OCY - 18, 28, 36, 6, C_PANEL2);
  char c[2] = {(char)('A' + cur_bank), 0};
  center(OCX - 14, 28, OCY - 16, c, mode ? C_CORAL : C_VIO, FONT_BIG);
  draw_orbit_text();
}

/* ================================================================ SYSTEM */
static void draw_sys(void) {
  char b[24], *d;
  rect(0, 19, SCREEN_W, SCREEN_H - 19, C_BG);
  put_int(b, load_pct);
  int x = text(12, 24, b, C_TEXT, FONT_BIG);
  text(x + 4, 38, "% CPU", C_MUTE, FONT_SMALL);
  { /* where the time goes */
    static const char *const SN[3] = {"SYN", "DRM", "FX"};
    static const uint16_t SC[3] = {C_VIO, C_CORAL, C_MINT};
    int xx = 12;
    for (int i = 0; i < 3; i++) {
      d = put(b, SN[i]);
      *d++ = ' ';
      put_int(d, sec_pct[i]);
      xx = text(xx, 58, b, SC[i], FONT_SMALL) + 10;
    }
    static const char *const C1[5] = {"1 CORE", "2 CORES", "2ND RETIRED", "2ND HELD", "1 CORE"};
    int st = cpu1_state();
    text(SCREEN_W - 12 - text_width(C1[st], FONT_SMALL), 24, C1[st], st == 1 ? C_LIME : st == 2 || st == 3 ? C_PINK : C_DIM, FONT_SMALL);
    if (st != 4 && cpu1_option(-1) != (st >= 1 && st <= 3)) text(SCREEN_W - 12 - text_width("NEXT START", FONT_SMALL), 40, "NEXT START", C_AMBER, FONT_SMALL);
  }
  int nd = 0, ns = 0;
  for (int i = 0; i < 16; i++) {
    int col = i % 8, row = i / 8, xx = 12 + col * 27, yy = 80 + row * 27;
    uint16_t c;
    if (i < DV) c = DVo[i].active ? (nd++, C_CORAL) : RGB(60, 28, 24);
    else if (i - DV < vcap) c = SV[i - DV].active ? (ns++, C_VIO) : RGB(36, 30, 70);
    else c = C_PANEL;
    capsule(xx, yy, 22, 22, 5, c);
  }
  d = put(b, "DRUMS ");
  put_int(d, nd);
  text(12, 138, b, C_CORAL, FONT_SMALL);
  d = put(b, "SYNTH ");
  put_int(d, ns);
  text(92, 138, b, C_VIO, FONT_SMALL);
  d = put(b, "FREE ");
  put_int(d, DV + vcap - nd - ns);
  text(172, 138, b, C_DIM, FONT_SMALL);
  capsule(8, 160, 224, 56, 6, C_PANEL);
  for (int i = 0; i < hist_n; i++) {
    int h = hist[i] * 48 / 100;
    if (h > 48) h = 48;
    rect(12 + i * 4, 212 - h, 3, h, hist[i] > 80 ? C_PINK : C_LIME);
  }
  text(8, 222, "27 S AGO", C_DIM, FONT_SMALL);
  text(232 - text_width("NOW", FONT_SMALL), 222, "NOW", C_TEXT, FONT_SMALL);
}

/* ================================================================ PERFORM */
#define XS 8
#define YS 22
#define XYZ 112
static uint8_t trail[16][2], trail_n;
static int puck_x(int v) { return XS + 8 + v * (XYZ - 16) / 100; }
static int puck_y(int v) { return YS + 8 + v * (XYZ - 16) / 100; }
static void draw_path(void);
static void draw_ring(void) { /* the XY square: ALGORITHM is X, PRESETS is Y; the four sounds sit in its corners */
  static const char *const CN[4] = {"A", "B", "C", "D"};
  int nc = nearest_corner();
  rect(0, 19, 142, 117, C_BG);
  capsule(XS, YS, XYZ, XYZ, 8, C_PANEL);
  for (int i = 1; i < 4; i++) {
    rect(XS + i * 28, YS + 4, 1, XYZ - 8, C_LINE);
    rect(XS + 4, YS + i * 28, XYZ - 8, 1, C_LINE);
  }
  for (int i = 0; i < trail_n; i++) disk(puck_x(trail[i][0]), puck_y(trail[i][1]), 1, mix565(C_PANEL, C_LIME, 40 + i * 120 / 16));
  for (int c = 0; c < 4; c++) {
    int cx = puck_x(c & 1 ? 100 : 0), cy = puck_y(c & 2 ? 100 : 0);
    disk(cx, cy, 10, CORNER_COL[c]);
    disk(cx, cy, 8, c == nc ? CORNER_COL[c] : C_PANEL);
    center(cx - 8, 17, cy - 8, CN[c], c == nc ? C_BG : CORNER_COL[c], FONT_SMALL);
  }
  draw_path();
  int px = puck_x(S.xy[0]), py = puck_y(S.xy[1]);
  disk(px, py, 6, mix565(C_PANEL, C_LIME, 80));
  disk(px, py, 4, mix565(C_PANEL, C_LIME, 160));
  disk(px, py, 2, C_TEXT);
}
static void draw_pills(void) {
  static const uint8_t PK[4] = {K_INT, K_KIT, K_THROW, K_FILT};
#define PC KC
  for (int k = 0; k < 4; k++) {
    int x = 146 + k * 23, v = S.k[PK[k]], top = XYZ - XYZ * v / 100;
    rect(x, YS, 16, XYZ, C_BG);
    for (int j = 0; j < XYZ; j++) {
      int in = inset(XYZ, 8, j);
      rect(x + in, YS + j, 16 - 2 * in, 1, j >= top ? PC[k] : C_PANEL2);
    }
    if (KD[PK[k]].unit == U_CENTER) rect(x - 2, YS + XYZ / 2, 2, 1, C_DIM);
  }
}
/* snapshots on white keys 1-8: hold to play, SAVE + key to store */
static snap_t snap_live;
static int8_t snap_held = -1;
static void draw_snaps(void) {
  for (int i = 0; i < 8; i++) {
    int x = i * 30;
    rect(x, 137, 30, 16, C_BG);
    const snap_t *sp = &S.snap[i];
    uint16_t c = !sp->used ? C_PANEL : snap_held == i ? C_LIME : mix565(C_PANEL, C_LIME, 90);
    capsule(x + 2, 137, 26, 16, 6, c);
    char b[3] = {(char)('1' + i), 0};
    center(x + 2, 26, 137, b, !sp->used ? C_DIM : snap_held == i ? C_BG : C_LIME, FONT_SMALL);
  }
}
static void snap_take(snap_t *sp) {
  sp->used = 1, sp->xy[0] = S.xy[0], sp->xy[1] = S.xy[1];
  sp->mac[0] = (uint8_t)S.k[K_INT], sp->mac[1] = (uint8_t)S.k[K_KIT], sp->mac[2] = (uint8_t)S.k[K_THROW], sp->mac[3] = (uint8_t)S.k[K_FILT];
  sp->dmute = dmute, sp->smute = smute, sp->smut = synth_mute;
}
static void snap_put(const snap_t *sp) {
  S.xy[0] = sp->xy[0], S.xy[1] = sp->xy[1];
  S.k[K_INT] = sp->mac[0], S.k[K_KIT] = sp->mac[1], S.k[K_THROW] = sp->mac[2], S.k[K_FILT] = sp->mac[3];
  dmute = sp->dmute, smute = sp->smute, synth_mute = sp->smut;
  apply_morph();
}

/* the morph path: REC on Perform records the puck a step at a time (up to 8 bars) and plays it back in a loop */
#define path S.path
#define path_len S.path_len
static uint8_t path_rec, path_play;
static uint32_t path_pulse = 0xFFFFFFFFu, path_from;
static int path_state(void) { return path_rec ? 2 : path_play ? 1 : 0; }
static void path_ctx(char *b) {
  char *d = put(b, path_rec ? "PATH REC  " : "PATH  ");
  d = put_int(d, (path_rec ? path_len : (int)((pulses - path_from) % (uint32_t)(path_len ? path_len : 1))) / 16 + 1);
  d = put(d, " / ");
  d = put_int(d, path_rec ? 8 : (path_len + 15) / 16);
  put(d, " BARS");
}
static void draw_path(void) {
  for (int i = 1; i < path_len; i++)
    line(puck_x(path[i - 1][0]), puck_y(path[i - 1][1]), puck_x(path[i][0]), puck_y(path[i][1]), path_rec ? C_PINK : mix565(C_PANEL, C_LIME, 150));
}

static void draw_mutes(void) {
  static const char *const MN[8] = {"KCK", "SNR", "HAT", "PRC", "CLP", "SYN", "FRZ", "ALL"};
  for (int i = 0; i < 8; i++) {
    int x = i * 30, on;
    rect(x, 157, 30, 17, C_BG);
    if (i < 5) on = !((dmute >> i) & 1);
    else if (i == 5) on = !synth_mute;
    else on = i == 6 ? freeze : 0;
    if (i == 6 && freeze) capsule(x + 1, 157, 28, 17, 8, C_MINT);
    uint16_t c = i == 6 ? (freeze ? C_BG : C_DIM) : i == 7 ? C_DIM : on ? (i < 5 ? TRACK_COL[i] : C_VIO) : C_PINK;
    center(x, 30, 157, MN[i], c, FONT_SMALL);
    if (i < 6 && !on) rect(x + 4, 165, 22, 1, C_PINK);
  }
}

/* ================================================================ pictures: one drawing a page, rows 20-165, on the
 * black background. Whatever KNOB i moves is drawn in KC[i], and one thing
 * on most pages moves: page_anim() redraws only that small part. */
static int an_prev = -1, an_u, an_r;
static uint8_t an_lvl[128], an_notes, smw[128], dey[224], dec_[224];
static void dashes(int x0, int x1, int y, uint16_t c) {
  for (int x = x0; x < x1; x += 4) rect(x, y, 2, 1, c);
}
static void vdashes(int x, int y0, int y1, uint16_t c) {
  for (int y = y0; y < y1; y += 4) rect(x, y, 1, 2, c);
}
static void pad_row(int y, int n, int sel, const char *const *names, uint16_t c);
static void pad_rows(int y, int n, int sel, const char *const *names, uint16_t c) { /* two rows when there are many */
  int a = (n + 1) / 2;
  for (int r = 0; r < 2; r++) {
    int i0 = r ? a : 0, cnt = r ? n - a : a, w = 224 / a;
    for (int i = 0; i < cnt; i++) {
      int j = i0 + i;
      capsule(8 + i * w, y + r * 19, w - 2, 17, 8, j == sel ? c : C_PANEL);
      center(8 + i * w, w - 2, y + r * 19, names[j], j == sel ? C_BG : C_DIM, FONT_SMALL);
    }
  }
}
static void pad_row(int y, int n, int sel, const char *const *names, uint16_t c) {
  int w = 224 / n;
  for (int i = 0; i < n; i++) {
    capsule(8 + i * w, y, w - 1, 17, 8, i == sel ? c : C_PANEL);
    center(8 + i * w, w - 1, y, names[i], i == sel ? C_BG : C_DIM, FONT_SMALL);
  }
}
static void vpill(int x, int y, int h, int pct, uint16_t c) {
  int top = h - h * clampi(pct, 0, 100) / 100;
  for (int j = 0; j < h; j++) {
    int in = inset(h, 5, j);
    rect(x + in, y + j, 10 - 2 * in, 1, j >= top ? c : C_PANEL2);
  }
}
static void orbit_mark(int cx, int cy, int r, int u, uint16_t c) {
  uint32_t ph = (uint32_t)u << 20;
  disk(cx + ((r * sine(ph)) >> 15), cy - ((r * sine(ph + 1073741824u)) >> 15), 2, c);
}

static void pic_pills(void) {
  for (int i = 0; i < 4; i++) {
    const char *l, *u;
    char v[28];
    int n;
    if (!get_cell(i, &l, v, &u, &n)) continue;
    int x = i * 60 + 14, top = 136 - 136 * clampi(n, 0, 100) / 100;
    for (int j = 0; j < 136; j++) {
      int in = inset(136, 8, j);
      rect(x + in, 24 + j, 32 - 2 * in, 1, j >= top ? KC[i] : C_PANEL2);
    }
  }
}

#define OX 112
#define OY 92
static uint32_t isqrt64(uint64_t v) { /* shifts only: no 64-bit division on the chip */
  uint64_t r = 0, b = 1ull << 62;
  while (b > v) b >>= 2;
  while (b) {
    if (v >= r + b) v -= r + b, r = (r >> 1) + b;
    else r >>= 1;
    b >>= 2;
  }
  return (uint32_t)r;
}
static int tone_y(int32_t lf, int32_t lfc, int reso) {
  int32_t a = lf - lfc + (8 << 16);
  uint32_t w = a < 0 ? 0 : a >= (12 << 16) ? 4096u : exp2_q16(a);
  if (w > 4096u) w = 4096u;
  int32_t re = 256 - (int32_t)((w * w) >> 8);
  uint32_t im = w * 10u / (uint32_t)reso;
  uint32_t den = isqrt64((uint64_t)((int64_t)re * re) + (uint64_t)im * im);
  uint32_t m = 65536u / (den ? den : 1u);
  int l = 0;
  if (m < 1u) m = 1u;
  while (m >= 512u) m >>= 1, l += 16;
  while (m < 256u) m <<= 1, l -= 16;
  l += (int)(m - 256u) * 16 / 256;
  return clampi(104 - l * 12 / 16, 32, 163);
}
/* ---- TONE: the filter curve. Cutoff line, contour start in dashes, resonance dot, drive band */
static void pic_tone(void) {
  const int16_t *k = S.k;
  int32_t lbase = clampi(LOG2_250 + k[K_CUT] * 3277 + (k[K_FILT] - 50) * 3277, LOG2_60, LOG2_12000);
  int32_t lpeak = clampi(lbase + LOG2_1P7C[k[K_CONTOUR]], LOG2_60, LOG2_12000), span = LOG2_12000 - LOG2_60;
  int h = 1 + k[K_DRIVE] * 9 / 100, py = 0, pq = 0;
  uint16_t fillc = mix565(PBG, KC[0], 36);
  rect(8, 22, 224, h, mix565(PBG, KC[3], 70));
  dashes(8, 232, 22 + h, KC[3]);
  for (int x = 0; x < 224; x++) {
    int32_t lf = LOG2_60 + (span / 223) * x;
    int y = tone_y(lf, lbase, k[K_RESO]), q = tone_y(lf, lpeak, k[K_RESO]);
    rect(8 + x, y, 1, 165 - y, fillc);
    if (x) {
      line(7 + x, py, 8 + x, y, KC[0]);
      line(7 + x, py + 1, 8 + x, y + 1, KC[0]);
      if ((x & 7) < 4) line(7 + x, pq, 8 + x, q, KC[1]);
    }
    py = y, pq = q;
  }
  int xc = 8 + (lbase - LOG2_60) / (span / 223), yc = tone_y(lbase, lbase, k[K_RESO]);
  rect(xc, yc, 1, 165 - yc, KC[0]);
  disk(xc, yc, 4, KC[2]);
}

/* ---- MOD: one turn of the ring is one note. The index starts bright at the top and fades clockwise (cyan tick =
 * decay time) to its end level (the lime inner ring). Moving: a marker going round, as bright as the index there */
#define MX 120
#define MY 92
static void pic_mod(void) {
  const int16_t *k = S.k;
  int n = clampi(128 * k[K_MDEC] / 200, 2, 128), na = clampi(128 * k[K_MATK] / 800, 0, 60);
  int32_t a = 32768, fr = clampi(226000 / n, 0, 32768), end = k[K_MEND] * 328;
  for (int i = 0; i < 128; i++) {
    int32_t lvl = end + ((((32768 - end) >> 1) * (a >> 1)) >> 13);
    if (i < na) lvl = 32768 * i / na; /* the attack: it swells in */
    an_lvl[i] = (uint8_t)clampi(lvl >> 7, 0, 255);
    arc_at(MX, MY, 52, 57, i * 32, i * 32 + 30, mix565(PBG, KC[0], 30 + an_lvl[i] * 226 / 256));
    if (i >= na) a -= (a * fr) >> 15;
  }
  for (int v = 0; v < 3; v++) { /* velocity: the index at soft, medium and hard steps */
    int vel = 30 + v * 35, h = 30 * (100 - k[K_MVEL] + k[K_MVEL] * vel / 100) / 100;
    rect(206 + v * 9, 160 - h, 6, h, KC[3]);
  }
  if (na) arc_at(MX, MY, 59, 64, na * 32 - 16, na * 32 + 16, KC[0]);
  if (end) arc_at(MX, MY, 43, 45, 0, 4096, mix565(PBG, KC[1], 60 + k[K_MEND] * 196 / 100));
  else ring_dots(MX, MY, 44, C_LINE);
  int ud = n * 32;
  arc_at(MX, MY, 59, 64, ud - 16, ud + 16, KC[0]);
  line(MX, MY, MX, MY - 50, C_LINE2);
  disk(MX, MY - 54, 5, C_TEXT);
  disk(MX, MY, 16, C_TEXT);
  disk(MX, MY, 14, C_PANEL2);
  center(MX - 16, 32, MY - 8, "IDX", C_MUTE, FONT_SMALL);
}

/* ---- ENV: the envelope, each segment in its knob's colour; sub level beneath */
static void pic_env(void) {
  const int16_t *k = S.k;
  int32_t atk = k[K_ATK], hold = k[K_HOLD], rel = k[K_REL] * 10, total = atk + hold + rel;
  int yb = 142, yt = 30, xa = 10 + clampi(220 * atk / total, 3, 120), xh = clampi(xa + 220 * hold / total, xa, 200);
  int n = 230 - xh, py = yb;
  int32_t a = 32768, fr = clampi(226000 / (n > 0 ? n : 1), 0, 32768);
  uint16_t fillc = mix565(PBG, C_VIO, 60);
  for (int x = 10; x <= 230; x++) {
    int y;
    uint16_t c;
    if (x <= xa) y = yb - (yb - yt) * (x - 10) / (xa - 10), c = KC[0];
    else if (x <= xh) y = yt, c = KC[1];
    else a -= (a * fr) >> 15, y = yb - (((yb - yt) * a) >> 15), c = KC[2];
    rect(x, y, 1, yb - y, fillc);
    if (x > 10) line(x - 1, py, x, y, c), line(x - 1, py + 1, x, y + 1, c);
    py = y;
  }
  rect(xa - 4, yt - 4, 8, 8, KC[0]);
  rect(xh - 4, yt - 4, 8, 8, KC[1]);
  rect(226, yb - 4, 8, 8, KC[2]);
  capsule(10, 152, 220, 10, 5, C_PANEL2);
  if (k[K_SUB]) capsule(10, 152, clampi(220 * k[K_SUB] / 100, 10, 220), 10, 5, KC[3]);
}

/* ---- LFO: where it goes (pads), how fast, how deep (band), its shape (glyphs). Moving: where the LFO is now */
static void lfo_mark(void) {
  int x = 8 + (int)(lfo_ph_out >> 24) * 112 / 256;
  rect(4, 122, 232, 9, PBG);
  rect(8, 126, 224, 1, C_LINE);
  disk(x, 126, 3, C_TEXT);
  disk(x + 112, 126, 3, C_TEXT);
}
static void pic_lfo(void) {
  const int16_t *k = S.k;
  int amp = 3 + k[K_LDEPTH] * 26 / 100, py = 0;
  if (amp > 22) amp = 22;
  pad_rows(22, 9, k[K_LDEST], DEST3, KC[0]);
  dashes(8, 232, 90 - amp, KC[2]);
  dashes(8, 232, 90 + amp, KC[2]);
  for (int x = 0; x < 224; x++) {
    int y = 90 - ((lfo_shape(k[K_LSHAPE], (uint32_t)x * 38347922u) * amp) >> 15);
    if (x) line(7 + x, py, 8 + x, y, KC[1]), line(7 + x, py + 1, 8 + x, y + 1, KC[1]);
    py = y;
  }
  for (int i = 0; i < 4; i++) {
    int x0 = 28 + i * 52, qy = 0;
    for (int x = 0; x < 32; x++) {
      int y = 150 - ((lfo_shape(i, (uint32_t)x * 134217728u) * 9) >> 15);
      if (x) line(x0 + x - 1, qy, x0 + x, y, i == k[K_LSHAPE] ? KC[3] : C_LINE2);
      qy = y;
    }
  }
  lfo_mark();
}

/* ---- DELAY: repeats bounce left (time A) and right (time B), fading with the feedback; wet as a pill.
 * Moving: the repeat under the playhead lights up, a step at a time */
static void delay_dot(int i, int hl) {
  const int16_t *k = S.k;
  int n = 1 + k[K_DFB] * 5 / 90, y = 30 + i * 24, r = 6 - i;
  int xa = 111 - k[K_DTA] * 96 / 740, xb = 129 + k[K_DTB] * 84 / 740;
  disk(xa, y, r + 2, hl ? C_TEXT : PBG);
  disk(xa, y, r, i < n ? KC[1] : mix565(PBG, KC[1], 50));
  disk(xb, y + 10, r + 2, hl ? C_TEXT : PBG);
  disk(xb, y + 10, r, i < n ? KC[2] : mix565(PBG, KC[2], 50));
}
static void pic_field(void);
static void pic_delay(void) {
  const int16_t *k = S.k;
  if (k[K_DMODE]) {
    pic_field();
    return;
  }
  capsule(8, 26, 44, 17, 8, KC[0]);
  center(8, 44, 26, "TWIN", C_BG, FONT_SMALL);
  int n = 1 + k[K_DFB] * 5 / 90;
  rect(119, 24, 1, 138, C_LINE);
  rect(118, 26, 3, n * 24 - 8, KC[3]);
  for (int i = 0; i < 6; i++) delay_dot(i, 0);
  vpill(224, 24, 138, k[K_DFB] * 100 / 90, KC[3]);
}

/* ---- SPACE: a burst. Rays = space, trailing dots = trail, speckles = dirt, core = mix. Moving: the core pulses
 * with every note */
#define SX 120
#define SY 92
static int space_core(void) { return 3 + S.k[K_MIX] * 6 / 100; }
static void pic_space(void) {
  const int16_t *k = S.k;
  int nt = k[K_TRAIL] / 25, nd = k[K_DIRT] / 3;
  uint32_t r = 777u;
  for (int i = 0; i < 32; i++) {
    r = r * 1103515245u + 12345u;
    int var = 50 + (int)((r >> 16) % 50u), len = 5 + k[K_SPACE] * 28 / 100 * var / 100;
    uint32_t ph = (uint32_t)i << 27;
    int32_t sn = sine(ph), cs = sine(ph + 1073741824u);
    line(SX + ((14 * sn) >> 15), SY - ((14 * cs) >> 15), SX + (((14 + len) * sn) >> 15), SY - (((14 + len) * cs) >> 15), KC[0]);
    for (int j = 0; j < nt; j++) {
      int rr = 14 + len + 5 + j * 5;
      disk(SX + ((rr * sn) >> 15), SY - ((rr * cs) >> 15), 1, mix565(PBG, KC[1], 220 - j * 50));
    }
  }
  for (int i = 0; i < nd; i++) {
    r = r * 1103515245u + 12345u;
    int x = 20 + (int)((r >> 8) % 200u), y = 24 + (int)((r >> 18) % 136u);
    if ((x - SX) * (x - SX) + (y - SY) * (y - SY) > 196) rect(x, y, 2, 2, KC[2]);
  }
  disk(SX, SY, space_core(), KC[3]);
}

/* ---- BUS: four flows, four knobs; each band is as wide as its send. Moving: dots travel along the open sends */
static const uint8_t BY0[4] = {44, 72, 114, 142}, BY1[4] = {44, 114, 72, 142};
static const uint8_t BUS_ID[4] = {K_SSD, K_SSS, K_SDD, K_SDS};
static int bus_y(int i, int x) {
  int32_t s = (32767 - sine((uint32_t)x * 13765696u + 1073741824u)) >> 1;
  return BY0[i] + (((BY1[i] - BY0[i]) * s) >> 15);
}
static void bus_col(int x) {
  if (x < 0 || x > 155) return;
  rect(42 + x, 26, 1, 136, PBG);
  for (int i = 0; i < 4; i++) {
    int v = S.k[BUS_ID[i]], th = 1 + v * 13 / 100, y = bus_y(i, x);
    if (v || (x & 3) < 2) rect(42 + x, y - th / 2, 1, th, v ? KC[i] : C_LINE2);
  }
}
static void pic_bus(void) {
  capsule(8, 26, 34, 60, 6, C_VIO);
  center(8, 34, 48, "SYN", C_BG, FONT_SMALL);
  capsule(8, 100, 34, 60, 6, C_CORAL);
  center(8, 34, 122, "DRM", C_BG, FONT_SMALL);
  capsule(198, 26, 34, 60, 6, C_MINT);
  capsule(199, 27, 32, 58, 5, PBG);
  center(198, 34, 48, "DLY", C_MINT, FONT_SMALL);
  capsule(198, 100, 34, 60, 6, C_CYAN);
  capsule(199, 101, 32, 58, 5, PBG);
  center(198, 34, 122, "SPC", C_CYAN, FONT_SMALL);
  for (int x = 0; x < 156; x++) bus_col(x);
}

/* ---- STEP MOD: a flow from the source drum track to what it moves. The band pinches on every hit of the source
 * (orange edges: depth) and swells back (white body: decay). Moving: the playhead along the band */
static void smod_col(int x, int hl) {
  int w = smw[x];
  rect(56 + x, 70, 1, 45, PBG);
  rect(56 + x, 92 - w, 1, 2 * w + 1, hl ? C_TEXT : mix565(PBG, KC[2], 120));
  pixel(56 + x, 92 - w, KC[3]);
  pixel(56 + x, 92 + w, KC[3]);
}
static void pic_smod(void) {
  const int16_t *k = S.k;
  const bank_t *b = CB();
  int src = k[K_MSRC], len = b->dlen[src] ? b->dlen[src] : DSTEPS;
  int32_t npx = clampi(k[K_MSHAPE] * 8 * k[K_BPM] / 15000, 1, 400), fr = clampi(226000 / npx, 0, 32768), a = 0;
  for (int pass = 0; pass < 2; pass++)
    for (int x = 0; x < 128; x++) {
      int st = (x >> 3) % len;
      if ((x & 7) == 0 && ((b->on[src] >> st) & 1)) a = 32768;
      if (pass) smw[x] = (uint8_t)(18 - ((17 * k[K_MDEPTH] / 100 * a) >> 15));
      a -= (a * fr) >> 15;
    }
  capsule(6, 62, 48, 60, 8, KC[0]);
  capsule(7, 63, 46, 58, 7, PBG);
  center(6, 48, 84, TRACK_NAME[src], KC[0], FONT_SMALL);
  capsule(186, 62, 48, 60, 8, KC[1]);
  capsule(187, 63, 46, 58, 7, PBG);
  center(186, 48, 84, DEST3[k[K_MDEST]], k[K_MDEST] ? KC[1] : C_DIM, FONT_SMALL);
  for (int s = 0; s < 16; s++)
    if ((b->on[src] >> (s % len)) & 1) rect(56 + s * 8, 60, 2, 6, KC[0]);
  for (int x = 0; x < 128; x++) smod_col(x, 0);
  center(0, SCREEN_W, 128, k[K_MDEPTH] ? "EACH HIT PULLS IT DOWN" : "DEPTH 0: OFF", C_DIM, FONT_SMALL);
}

/* ---- GLOBAL: tempo, swing as shifted off-beats, the scale on one octave with its root. Moving: the beat */
static void glo_dot(int i, int hl) {
  int x = 18 + i * 28 + (i & 1 ? S.k[K_SWING] * 14 / 75 : 0), r = i & 1 ? 3 : 4;
  disk(x, 80, r + 2, hl ? C_TEXT : PBG);
  disk(x, 80, r, i & 1 ? KC[1] : C_LINE2);
}
static void pic_glo(void) {
  const int16_t *k = S.k;
  char b[8];
  put_int(b, k[K_BPM]);
  int x = text(10, 22, b, KC[0], FONT_BIG);
  text(x + 6, 36, "BPM", C_MUTE, FONT_SMALL);
  for (int i = 0; i < 8; i++) glo_dot(i, 0);
  int sc = k[K_SCALE];
  static const uint8_t BLK[12] = {0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0};
  for (int n = 0; n < 12; n++) {
    int in = 0;
    for (int j = 0; j < SCALE_LEN[sc]; j++) in |= (k[K_ROOT] + SCALES[sc][j]) % 12 == n;
    capsule(10 + n * 18, BLK[n] ? 100 : 108, 16, BLK[n] ? 38 : 52, 4, n == k[K_ROOT] ? KC[3] : in ? KC[2] : C_PANEL2);
  }
}

/* ---- drums */
static void drum_wave(int yc, int amp, int yp) { /* the selected track's hit: wave and pitch line, or its noise */
  const uint8_t *p = S.def[track];
  if (IS_NOISE(track)) {
    uint32_t r = 12345u + p[P_IDX] * 977u;
    int hmax = amp * 2 + 8;
    for (int i = 0; i < 56; i++) {
      r = r * 1103515245u + 12345u;
      int v = (int)((r >> 16) % 100u), h;
      switch (p[P_IDX] > 4 ? 4 : p[P_IDX]) {
        case 1: h = v & 1 ? 90 : 30; break;
        case 2: h = i % 3 == 0 ? 85 : 12; break;
        case 3: h = i % 4 == 0 ? v + 5 : 3; break;
        case 4: h = 5 + (int)((sine((uint32_t)i * 300000000u) >> 8) * (56 - i) / 56 * 90 / 128 + 0) * (sine((uint32_t)i * 300000000u) < 0 ? -1 : 1); break;
        default: h = v + 5;
      }
      h = h * hmax / 105;
      rect(8 + i * 4, yp - h, 2, h, i < p[P_CAR] * 56 / 180 ? KC[1] : mix565(PBG, KC[1], 90));
    }
    rect(8 + p[P_CAR] * 224 / 180, yp - hmax, 1, hmax, KC[2]);
    return;
  }
  int32_t atk = p[P_ATK], rel = kit_scale(p[P_REL] * 5), total = atk + rel;
  int32_t fa = clampi(6907 * total / 224 * 33 / (rel ? rel : 1), 0, 32768);
  int32_t fs = clampi(6907 * total / 224 * 33 / (p[P_SWT] * 2), 0, 32768);
  int32_t fm = clampi(6907 * total / 224 * 33 / (p[P_MDEC] * 5), 0, 32768);
  int32_t a = 32768, sw = 32767, me = 32767, mend = p[P_MEND] * 327;
  uint32_t inc = (uint32_t)(((uint64_t)76695844u * FRAC_Q16[p[P_CAR]]) >> 16);
  inc = (uint32_t)(((uint64_t)inc * (uint32_t)SEMI[p[P_PITCH]]) >> 16);
  if (inc > 600000000u) inc = 600000000u;
  uint32_t pc = 0, pm = 0;
  int py = yc, pp = 0;
  rect(8, yc, 224, 1, C_LINE);
  for (int x = 0; x < 224; x++) {
    int32_t t = total * x / 224, av;
    if (t < atk) av = 32768 * t / (atk ? atk : 1);
    else a -= (a * fa) >> 15, av = a;
    sw -= (sw * fs) >> 15;
    me = mend + (me - mend) - (((me - mend) * fm) >> 15);
    uint32_t mult = p[P_SWD] ? exp2_q16(p[P_SWD] * sw / 6 + (15 << 16)) : 32768u;
    uint32_t ci = (uint32_t)(((uint64_t)inc * mult) >> 15);
    int32_t midx = (kit_scale(p[P_IDX]) * me) >> 15;
    int32_t sv = sine(pc + (uint32_t)(sine(pm) * midx) * 2086u);
    pc += ci;
    pm += (uint32_t)(((uint64_t)ci * FRAC_Q16[p[P_RAT]]) >> 16);
    int y = yc - ((((sv * (av >> 1)) >> 14) * amp) >> 15);
    int q = yp - p[P_PITCH] * amp / 80 - ((p[P_SWD] * amp / 44 * sw) >> 15);
    if (x) {
      line(7 + x, py, 8 + x, y, KC[0]);
      line(7 + x, pp, 8 + x, q, KC[3]);
    }
    py = y, pp = q;
  }
}
/* SOUND: wave = carrier, tag = ratio, ticks = index, lower line = pitch */
static void pic_drum(void) {
  const uint8_t *p = S.def[track];
  if (IS_NOISE(track)) {
    center(0, SCREEN_W, 24, NTYPE_NAME[p[P_IDX] > 4 ? 4 : p[P_IDX]], KC[0], FONT_SMALL);
    drum_wave(98, 50, 162);
    return;
  }
  capsule(8, 24, 44, 18, 9, KC[1]);
  capsule(9, 25, 42, 16, 8, PBG);
  center(8, 44, 25, FRAC_NAME[p[P_RAT]], KC[1], FONT_SMALL);
  for (int i = 0; i < 12; i++) rect(60 + i * 6, 27, 4, 12, i * 100 < p[P_IDX] * 12 ? KC[2] : C_PANEL2);
  drum_wave(92, 40, 162);
}
/* SWEEP: a falling trail of dots, one per moment of the hit. Height = pitch (cyan: sweep depth, lime line: sweep
 * time), size = FM index (white: how fast it shrinks; an orange ring once it has settled at its end size) */
static void pic_dswp(void) {
  const uint8_t *p = S.def[track];
  if (IS_NOISE(track)) {
    center(0, SCREEN_W, 84, "NOISE HAS NO SWEEP", C_DIM, FONT_SMALL);
    return;
  }
  /* the window: one and a half times the longer of the sweep and the index decay, so the trail always has room */
  int32_t win = clampi(3 * (p[P_SWT] * 2 > p[P_MDEC] * 5 ? p[P_SWT] * 2 : p[P_MDEC] * 5) / 2, 30, 600);
  int dpx = p[P_SWD] * 110 / 48, nsw = clampi(p[P_SWT] * 2 * 27 / win, 1, 200), nme = clampi(p[P_MDEC] * 5 * 27 / win, 1, 400);
  int32_t sw = 32767, me = 32767, mend = p[P_MEND] * 327;
  int32_t fs = clampi(226000 / nsw, 0, 32768), fm = clampi(226000 / nme, 0, 32768);
  int16_t yy[27];
  uint8_t rr[27], fin[27];
  for (int i = 0; i < 27; i++) {
    yy[i] = (int16_t)(150 - ((dpx * sw) >> 15));
    rr[i] = (uint8_t)(1 + ((6 * me) >> 15));
    fin[i] = me - mend < 1638;
    sw -= (sw * fs) >> 15;
    me = mend + (me - mend) - (((me - mend) * fm) >> 15);
  }
  rect(8, 160, 224, 1, C_LINE);
  vdashes(clampi(14 + nsw * 8, 14, 228), 26, 160, KC[1]);
  for (int i = 1; i < 27; i++) {
    line(6 + i * 8, yy[i - 1], 14 + i * 8, yy[i], KC[0]);
    line(6 + i * 8, yy[i - 1] + 1, 14 + i * 8, yy[i] + 1, KC[0]);
  }
  for (int i = 0; i < 27; i++) {
    if (fin[i] && p[P_MEND]) disk(14 + i * 8, yy[i], rr[i] + 2, KC[3]);
    disk(14 + i * 8, yy[i], rr[i], KC[2]);
  }
}
/* PERF: each hit sized by accent with its chance as dots above; the echoes below and how they fade.
 * Moving: a tick under the hit that is playing */
static void pic_dprf(void) {
  const bank_t *b = CB();
  const uint8_t *p = S.def[track];
  for (int s = 0; s < DSTEPS; s++) {
    int x = step_x(s) + 6;
    if (s >= b->dlen[track]) continue;
    if (!((b->on[track] >> s) & 1)) {
      disk(x, 98, 1, C_LINE2);
      continue;
    }
    uint8_t lk = b->lock[track][s][P_PROB];
    int prob = lk == UNLOCKED ? p[P_PROB] : lk, np = (prob + 3) / 4;
    for (int j = 0; j < 5; j++) disk(x, 84 - j * 10, 2, j < np ? KC[1] : C_PANEL2);
    disk(x, 98, 2 + acc_at(b, track, s) / 8, KC[0]);
  }
  int32_t keep = (100 - p[P_EDEC] * 5) * 328, sz = 6 << 8;
  disk(20, 142, 6, C_TEXT);
  for (int j = 1; j <= 4; j++) {
    sz = (sz * (keep >> 1)) >> 14;
    disk(20 + j * 34, 142, j <= p[P_ECHO] ? clampi(sz >> 8, 1, 6) : 1, j <= p[P_ECHO] ? KC[2] : C_LINE2);
  }
  line(20, 130, 156, 130 + p[P_EDEC] * 12 / 18, KC[3]);
}
/* ENV: the bar as columns: every hit rises (cyan: attack) and falls (lime: release) up to the track's volume (white
 * line); orange ticks are its length. Moving: a tick under the step that is playing */
static void pic_denv(void) {
  const bank_t *b = CB();
  const uint8_t *p = S.def[track];
  int len = b->dlen[track];
  int32_t step_ms = 15000 / S.k[K_BPM];
  int32_t apx = clampi(p[P_ATK] * 14 / step_ms, 1, 14), rpx = clampi(p[P_REL] * 5 * 14 / step_ms, 1, 220);
  int32_t fr = clampi(226000 / rpx, 0, 32768), a = 0, peak = 0;
  int t = 999;
  for (int pass = 0; pass < 2; pass++)
    for (int x = 0; x < 224; x++) {
      int s = x / 14;
      if (x % 14 == 0 && s < len && ((b->on[track] >> s) & 1)) {
        t = 0;
        peak = clampi(32767 * S.vol[track] / 100 * acc_at(b, track, s) / 20, 0, 32767); /* accent 1.00 = the volume */
      }
      int32_t lv;
      if (t < apx) lv = peak * t / apx;
      else {
        if (t == apx) a = peak;
        lv = a;
        a -= (a * fr) >> 15;
      }
      if (pass) dey[x] = (uint8_t)(150 - ((120 * lv) >> 15)), dec_[x] = (uint8_t)(lv < 300 ? 0 : t < apx ? 1 : 2);
      t++;
    }
  dashes(8, 232, 150 - 120 * S.vol[track] / 100, KC[2]);
  for (int x = 0; x < 224; x++) {
    if (!dec_[x]) continue;
    rect(8 + x, dey[x], 1, 150 - dey[x], mix565(PBG, TRACK_COL[track], x & 1 ? 70 : 120));
    rect(8 + x, dey[x], 1, 2, dec_[x] == 1 ? KC[0] : KC[1]);
  }
  rect(8, 150, 224, 1, C_LINE);
  for (int s = 0; s < DSTEPS; s++) rect(8 + s * 14, 153, 11, 4, s < len ? KC[3] : C_PANEL2);
}

/* ---- OSC: the modulator's path round the carrier, traced. Petals and loops = ratio (fractions close after several
 * turns), reach = index, a rough line = feedback; 4-op rides a third loop on it, parallel adds a second trace.
 * Moving: a dot runs along the trace */
static const uint8_t RATIO_TURNS[NFRAC] = {4, 3, 2, 3, 4, 1, 4, 3, 2, 3, 4, 1, 2, 1, 2, 1, 1, 1, 1, 1};
static int16_t spx[481], spy[481];
static int sp_n;
static void spiro(int cx, int cy, int rc, int rm, int r3, uint32_t turns, int fb, int store, uint16_t c) {
  uint32_t frac = FRAC_Q16[S.k[K_RATIO]];
  int n = (int)(120u * turns), px = 0, py = 0;
  for (int i = 0; i <= n; i++) {
    uint32_t th = (uint32_t)((uint64_t)i * 0x100000000ull / 120u); /* carrier angle, wraps per turn */
    uint32_t tm = (uint32_t)(((uint64_t)i * 0x100000000ull / 120u) * frac >> 16);
    int32_t rmm = rm + (fb ? ((sine(tm * 3u) >> 12) * fb) / 100 : 0);
    int32_t x = cx + ((rc * sine(th)) >> 15) + ((rmm * sine(tm)) >> 15);
    int32_t y = cy - ((rc * sine(th + 1073741824u)) >> 15) - ((rmm * sine(tm + 1073741824u)) >> 15);
    if (r3) {
      uint32_t t3 = tm + (tm >> 1);
      x += (r3 * sine(t3)) >> 15, y -= (r3 * sine(t3 + 1073741824u)) >> 15;
    }
    if (i) line(px, py, x, y, c);
    if (store && i <= 480) spx[i] = (int16_t)x, spy[i] = (int16_t)y;
    px = x, py = y;
  }
  if (store) sp_n = n > 480 ? 480 : n;
}
static void pic_osc(void) {
  static const char *const AN[3] = {"2", "4", "P"};
  const int16_t *k = S.k;
  for (int i = 0; i < 3; i++) {
    int sel = k[K_ALGO] == i;
    capsule(8, 26 + i * 22, 20, 18, 5, sel ? KC[0] : C_PANEL);
    center(8, 20, 27 + i * 22, AN[i], sel ? C_BG : C_DIM, FONT_SMALL);
  }
  int rm = 6 + k[K_INDEX] * 30 / 120, rc = 64 - rm, r3 = k[K_ALGO] == 1 ? rm / 3 : 0;
  if (r3) rc -= r3;
  if (k[K_ALGO] == 2) spiro(OX + 8, OY, rc * 2 / 3, rm, 0, RATIO_TURNS[k[K_RATIO]], k[K_FB], 0, mix565(C_BG, KC[2], 80));
  spiro(OX + 8, OY, rc, rm, r3, RATIO_TURNS[k[K_RATIO]], k[K_FB], 1, KC[1]);
  disk(OX + 8, OY, 3, C_TEXT);
  text(8, 146, FRAC_NAME[k[K_RATIO]], KC[1], FONT_SMALL);
  if (k[K_FB]) text(232 - text_width("FB", FONT_SMALL), 146, "FB", KC[3], FONT_SMALL);
}
static void osc_anim(void) {
  if (sp_n < 2) return;
  if (an_prev >= 0) {
    int ex = spx[an_prev], ey = spy[an_prev];
    disk(ex, ey, 3, C_BG);
    for (int i = 1; i <= sp_n; i++) /* mend the trace where the dot was */
      if ((spx[i] - ex) * (spx[i] - ex) + (spy[i] - ey) * (spy[i] - ey) < 64 ||
          (spx[i - 1] - ex) * (spx[i - 1] - ex) + (spy[i - 1] - ey) * (spy[i - 1] - ey) < 64)
        line(spx[i - 1], spy[i - 1], spx[i], spy[i], KC[1]);
    disk(OX + 8, OY, 3, C_TEXT);
  }
  an_u = (an_u + 2) % sp_n;
  disk(spx[an_u], spy[an_u], 3, C_TEXT);
  an_prev = an_u;
}

/* ---- INDEX (SEL x2): four small pictures, one a knob. Key tracking: the index across the keyboard; loop: the
 * envelope repeating in time; op 3: the third operator's reach; drift: how far notes scatter */
static void pic_mod2(void) {
  const int16_t *k = S.k;
  int i;
  rect(119, 24, 1, 136, C_LINE);
  rect(8, 92, 224, 1, C_LINE);
  for (i = 0; i < 12; i++) { /* key tracking */
    int h = 8 + 50 * clampi(100 - k[K_KTRK] * (i * 5 - 24) / 48, 25, 200) / 200;
    rect(10 + i * 9, 86 - h, 6, h, KC[0]);
  }
  static const uint8_t LOOPS[5] = {0, 1, 2, 4, 8};
  int per = LOOPS[k[K_MLOOP]] * 6, py = 30; /* loop */
  for (int x = 0; x < 104; x++) {
    int t = per ? x % per : x, y = 30 + clampi(t * 7, 0, 54);
    if (x) line(129 + x, py, 130 + x, y, KC[1]);
    py = y;
  }
  int r3 = 4 + k[K_OP3] * 24 / 200; /* op 3 */
  disk(62, 126, 6, C_PANEL2);
  ring_dots(62, 126, r3 + 6, KC[2]);
  disk(62 + r3 + 6, 126, 3, KC[2]);
  uint32_t r = 4321u; /* drift */
  rect(130, 126, 100, 1, C_LINE2);
  for (i = 0; i < 18; i++) {
    r = r * 1103515245u + 12345u;
    int dy = (int)((r >> 16) % 61u) - 30;
    disk(134 + i * 5, 126 + dy * k[K_DRIFT] / 100, 1, KC[3]);
  }
}

/* ---- UNISON: the voices as glowing bands; detune spreads them apart, spread alternates notes left and right */
static void pic_uni(void) {
  const int16_t *k = S.k;
  static const uint16_t UC[3] = {RGB(139, 108, 255), RGB(77, 225, 240), RGB(255, 79, 154)};
  for (int v = 0; v < k[K_UNI]; v++) {
    int off = (v == 0 ? 0 : v == 1 ? 1 : -1) * k[K_UDET] * 2 / 5, py = 0;
    uint16_t c = v ? mix565(C_BG, UC[v], 80 + k[K_UMIX] * 170 / 100) : UC[0];
    for (int x = 0; x < 224; x++) {
      int dx = x - 112 + off, q = dx * dx, t = q < 3600 ? 3600 - q : 0; /* a soft peak */
      int y = 124 - t * t / 185143;
      if (x) line(7 + x, py, 8 + x, y, c);
      py = y;
    }
  }
  for (int i = 0; i < 7; i++) disk(16 + i * 14, 30, 4, i < k[K_UNI] ? KC[0] : C_PANEL2);
  int w = 20 + k[K_USPR] * 80 / 100;
  rect(120 - w, 146, 2 * w, 2, KC[2]);
  disk(120 - w, 147, 4, KC[2]);
  disk(120 + w, 147, 4, KC[2]);
  text(120 - w - 14, 139, "L", C_DIM, FONT_SMALL);
  text(120 + w + 8, 139, "R", C_DIM, FONT_SMALL);
  char b[12], *d = put_int(b, k[K_UDET]);
  put(d, " CT");
  center(0, SCREEN_W, 124, b, KC[1], FONT_SMALL);
}

/* ---- the delay as the echo field: four taps orbiting the send; size spreads the orbits, gravity keeps repeats
 * going, trail is the sun */
static void pic_field(void) {
  const int16_t *k = S.k;
  int sz = clampi(k[K_DTA] * 100 / 400, 25, 140), grav = clampi((k[K_DTB] - 20) * 100 / 720, 0, 100);
  static const uint8_t TR[4] = {20, 32, 40, 50};
  for (int t = 0; t < 4; t++) {
    int r = 14 + TR[t] * sz / 100;
    if (r > 68) r = 68;
    ring_dots(120, 92, r, mix565(C_BG, KC[1], 120));
    int nd = 1 + grav / 25;
    for (int j = 0; j < nd; j++) orbit_mark(120, 92, r, (t * 1100 + j * 380) & 4095, mix565(C_BG, KC[2], 255 - j * 50));
  }
  disk(120, 92, 4 + k[K_DFB] / 12, KC[3]);
  capsule(8, 26, 50, 17, 8, KC[0]);
  center(8, 50, 26, "FIELD", C_BG, FONT_SMALL);
}

/* ---- SONG: the chain as tape. Blocks pass under the head; the reels turn while it plays */
static void song_reels(int ang) {
  for (int r = 0; r < 2; r++) {
    int cx = r ? 196 : 44, cy = 52, rr = r ? 12 + (S.song_n ? 10 * song_pos / S.song_n : 0) : 22 - (S.song_n ? 10 * song_pos / S.song_n : 0);
    disk(cx, cy, 26, C_PANEL);
    disk(cx, cy, rr, C_PANEL2);
    for (int a = 0; a < 3; a++) orbit_mark(cx, cy, rr - 5 > 3 ? rr - 5 : 3, (ang + a * 1365) & 4095, C_BG);
    disk(cx, cy, 3, C_BG);
  }
}
static void draw_song(void) {
  rect(0, 19, SCREEN_W, 166, C_BG);
  song_reels(0);
  rect(44, 80, 152, 2, C_LINE2);
  int first = song_cur > 5 ? song_cur - 5 : 0;
  for (int i = 0; i < 6; i++) {
    int j = first + i, x = 8 + i * 38;
    if (j >= S.song_n) {
      frame(x, 100, 34, 44, C_PANEL);
      continue;
    }
    static const uint16_t BC[4] = {RGB(139, 108, 255), RGB(255, 107, 87), RGB(77, 225, 240), RGB(255, 181, 71)};
    uint16_t c = BC[S.song_bank[j] & 3];
    int now = S.k[K_SONG] && playing && j == song_pos;
    capsule(x, 100, 34, 44, 6, now ? c : C_PANEL);
    if (!now) frame(x, 100, 34, 44, c);
    char b[2] = {(char)('A' + S.song_bank[j]), 0};
    center(x, 34, 104, b, now ? C_BG : c, FONT_BIG);
    for (int m = 0; m < 6; m++) { /* what plays in this block: the five tracks, then the synth */
      int off = (S.song_mute[j] >> m) & 1;
      uint16_t mc = m < 5 ? TRACK_COL[m] : RGB(139, 108, 255);
      rect(x + 3 + m * 5, 138, 4, 3, off ? (now ? c : C_PANEL2) : (now ? C_BG : mc));
    }
    char r[6], *d = put(r, "x");
    put_int(d, S.song_rep[j]);
    center(x, 34, 146, r, j == song_cur ? C_TEXT : C_DIM, FONT_SMALL);
    if (j == song_cur) rect(x + 4, 162, 26, 2, KC[0]);
  }
  if (!S.song_n) center(0, SCREEN_W, 112, "EMPTY: WHITE 1-4 ADD A-D", C_DIM, FONT_SMALL);
}

/* ---- TRANSPOSE (GLO x2): the stops as a route; each stem is as tall as its shift. Moving: the stop playing */
static void tr_stop(int i, int hl) {
  const int16_t *k = S.k;
  int x = 20 + i * 28, v = S.tseq[i], on = i < k[K_TLEN], y = 110 - v * 6;
  rect(x - 8, 56, 17, 88, C_BG);
  rect(x - 8, 108, 17, 5, on ? (i < k[K_TLEN] - 1 ? KC[1] : C_PANEL2) : C_PANEL2);
  if (on && v) rect(x, v > 0 ? y : 110, 2, v > 0 ? 110 - y : y - 110, KC[1]);
  if (hl) disk(x, y, 8, C_TEXT);
  disk(x, y, i == k[K_TCUR] ? 6 : 4, !on ? C_PANEL2 : i == k[K_TCUR] ? KC[3] : KC[1]);
  char b[4];
  put_int(b, i + 1);
  center(x - 8, 17, 128, b, i == k[K_TCUR] ? KC[2] : C_DIM, FONT_SMALL);
}
static void pic_glo2(void) {
  const int16_t *k = S.k;
  char b[12], *d;
  int tv = k[K_TLEN] ? S.tseq[t_pos] : 0, x = 10;
  if (tv) x = text(x, 30, tv > 0 ? "+" : "-", C_TEXT, FONT_SMALL);
  put_int(b, tv < 0 ? -tv : tv);
  x = text(x, 22, b, C_TEXT, FONT_BIG);
  text(x + 8, 30, k[K_TLEN] ? "SCALE STEPS NOW" : "OFF: TURN LENGTH", C_MUTE, FONT_SMALL);
  rect(12, 110, 216, 1, C_LINE);
  for (int i = 0; i < 8; i++) tr_stop(i, 0);
  d = put(b, "EVERY ");
  d = put_int(d, 1 << k[K_TEVERY]);
  put(d, k[K_TEVERY] ? " BARS" : " BAR");
  text(10, 146, b, KC[1], FONT_SMALL);
}

/* ---- MIX: two big faders (synth, drums), a small one for every lane and drum, and the master with its meter.
 * The selected lane or drum is lit; the meters move with what plays */
#define MIXB 146 /* the faders' floor */
static uint16_t mt_s, mt_d, mt_o; /* meters, falling */
static uint8_t lim_show;
static int lvl_h(int v, int full) { return v * full / 127; }
static void mix_fader(int x, int w, int full, int v, uint16_t c, int unity) {
  rect(x, MIXB - full, w, full, C_PANEL);
  int h = lvl_h(v, full);
  if (h) rect(x, MIXB - h, w, h, c);
  if (unity) rect(x - 2, MIXB - lvl_h(100, full), w + 4, 1, C_DIM);
}
static int meter_h(uint16_t pk, int full) { /* the loudest sample, on a dB-like curve: top = full scale */
  int h = 0;
  uint32_t p = pk;
  while (p > 64 && h < 16) p >>= 1, h += 2; /* 6 dB a step */
  if (p > 45) h++;
  return h * full / 19;
}
static void mix_meter(int x, int full, uint16_t pk, uint16_t c) {
  int h = meter_h(pk, full);
  rect(x, MIXB - full, 4, full - h, C_BG);
  if (h) rect(x, MIXB - h, 4, h, c);
}
static void mix_meters(void) {
  uint16_t s = pk_syn, d = pk_drm, o = pk_out;
  pk_syn = pk_drm = pk_out = 0;
  mt_s = s > mt_s ? s : mt_s - (mt_s >> 3);
  mt_d = d > mt_d ? d : mt_d - (mt_d >> 3);
  mt_o = o > mt_o ? o : mt_o - (mt_o >> 3);
  mix_meter(32, 110, mt_s, KC[0]);
  mix_meter(68, 110, mt_d, KC[1]);
  mix_meter(226, 110, mt_o, mt_o > 25000 ? C_PINK : KC[3]);
  uint8_t lim = limiting;
  limiting = 0;
  if (lim != lim_show) {
    lim_show = lim;
    capsule(196, 148, 40, 16, 6, lim ? C_PINK : C_BG);
    center(196, 40, 148, "LIMIT", lim ? C_BG : C_BG, FONT_SMALL);
  }
}
static void pic_mix(void) {
  mix_fader(16, 14, 110, S.mix[0], KC[0], 1);
  mix_fader(52, 14, 110, S.mix[1], KC[1], 1);
  text(10, 148, "SYN", KC[0], FONT_SMALL);
  text(46, 148, "DRM", KC[1], FONT_SMALL);
  text(92, 22, "LANES", C_DIM, FONT_SMALL);
  for (int l = 0; l < LANES; l++) { /* the lanes */
    int x = 88 + l * 8, sel = !mode && l == lane;
    mix_fader(x, 5, 90, S.llev[l], sel ? C_TEXT : mix565(C_PANEL, LANE_COL[l], 190), 0);
    if (sel) rect(x, MIXB + 3, 5, 2, KC[2]);
  }
  text(154, 22, "DRUMS", C_DIM, FONT_SMALL);
  for (int t = 0; t < TRACKS; t++) { /* the drums: their volumes */
    int x = 156 + t * 8, sel = mode && t == track;
    mix_fader(x, 5, 90, S.vol[t] * 127 / 100, sel ? C_TEXT : mix565(C_PANEL, TRACK_COL[t], 190), 0);
    if (sel) rect(x, MIXB + 3, 5, 2, KC[2]);
  }
  rect(88, MIXB - lvl_h(100, 90), 102, 1, C_LINE2);
  mix_fader(210, 14, 110, S.mix[2], KC[3], 1);
  text(202, 22, "OUT", KC[3], FONT_SMALL);
  lim_show = 255;
  mix_meters();
}

/* ---- PUMP: the synth's level over a bar, ducked at each kick (or beat) and back over TIME; the bar moves with it */
static int pump_drawn;
static void pic_pump(void) {
  const bank_t *b = CB();
  int dep = S.pump[0], tms = 30 + S.pump[1] * 57 / 10, py = 0;
  int32_t step_ms = 15000 / (S.k[K_BPM] ? S.k[K_BPM] : 120);
  rect(8, 40, 224, 1, C_LINE2);
  rect(8, 120, 224, 1, C_LINE);
  for (int x = 0; x < 224; x++) {
    int s = x * 16 / 224, since = 99;
    for (int j = 0; j <= s; j++) { /* the last trigger at or before this step */
      int hit = S.pump[2] ? (j % 4 == 0) : (j < b->dlen[0] && ((b->on[0] >> j) & 1));
      if (hit) since = s - j;
    }
    int ms = since == 99 ? 9999 : since * step_ms + (x * 16 % 224) * step_ms / 224;
    int duck = ms >= tms * 3 ? 0 : dep * (tms * 3 - ms) / (tms * 3);
    int y = 40 + duck * 80 / 100;
    if (x) line(7 + x, py, 8 + x, y, KC[0]);
    py = y;
  }
  for (int j = 0; j < 16; j++) { /* the triggers */
    int hit = S.pump[2] ? (j % 4 == 0) : (j < b->dlen[0] && ((b->on[0] >> j) & 1));
    if (hit) disk(8 + j * 14, 132, 3, S.pump[2] ? KC[2] : C_CORAL);
  }
  text(8, 144, S.pump[2] ? "EVERY BEAT" : "KICK HITS", S.pump[2] ? KC[2] : C_CORAL, FONT_SMALL);
  text(232 - text_width(S.pump[3] ? "SYNTH + FX" : "SYNTH", FONT_SMALL), 144, S.pump[3] ? "SYNTH + FX" : "SYNTH", KC[3], FONT_SMALL);
  pump_drawn = -1;
}
static int pump_drawn = -1;
static void pump_anim(void) { /* the duck now, as a bar at the left edge */
  int h = pump_show * 78 / 32768;
  if (h == pump_drawn) return;
  pump_drawn = h;
  rect(1, 41, 4, 80, C_BG);
  if (h) rect(1, 41, 4, h, KC[1]);
}

/* ---- the steps of the lane or track, under every picture */
static void draw_mini(void) {
  const bank_t *b = CB();
  rect(0, 166, SCREEN_W, 19, C_BG);
  if (!mode) {
    int len = b->lane[lane].len, pw = len <= 16 ? 14 : 7, ph = sph(lane);
    compute_caps(lane);
    for (int s = 0; s < len; s++)
      rect(8 + s * pw, 168, pw - 1, 14, (held >> s) & 1 ? RGB(70, 26, 50) : s == ph ? C_LINE2 : C_PANEL);
    for (int s = 0; s < len; s++) {
      if (cap_start[s] != s) continue;
      int on = ph >= s && ph < s + cap_span[s];
      capsule(8 + s * pw, 170, cap_span[s] * pw - 1, 10, 4, on ? mix565(LANE_COL[lane], C_TEXT, 150) : LANE_COL[lane]);
      if (LOCKED(&b->st[lane][s])) rect(8 + s * pw + 1, 168, 3, 2, C_PINK);
      if (COND(&b->st[lane][s])) rect(8 + s * pw + 1, 180, 3, 2, C_CYAN);
    }
  } else {
    for (int s = 0; s < DSTEPS; s++) {
      int x = step_x(s), ph = dph(track) == s;
      if (s >= b->dlen[track]) {
        frame(x, 167, 12, 16, C_PANEL);
        continue;
      }
      frame(x, 167, 12, 16, ph ? C_TEXT : (held >> s) & 1 ? C_PINK : C_LINE);
      rect(x + 1, 168, 10, 14, C_PANEL);
      if ((b->on[track] >> s) & 1) {
        int h = 14 * acc_at(b, track, s) / 40;
        rect(x + 1, 182 - h, 10, h, ph ? mix565(TRACK_COL[track], C_TEXT, 120) : TRACK_COL[track]);
      }
    }
  }
}

static void draw_pic(void) {
  int pg = cur_page();
  PBG = C_BG;
  rect(0, 19, SCREEN_W, 147, PBG);
  an_prev = -1;
  switch (pg) {
    case PG_OSC: pic_osc(); break;
    case PG_TONE: pic_tone(); break;
    case PG_MOD: pic_mod(); break;
    case PG_ENV: pic_env(); break;
    case PG_LFO: pic_lfo(); break;
    case PG_DLY: pic_delay(); break;
    case PG_SPC: pic_space(); break;
    case PG_BUS: pic_bus(); break;
    case PG_SMOD: pic_smod(); break;
    case PG_GLO: pic_glo(); break;
    case PG_GLO2: pic_glo2(); break;
    case PG_MOD2: pic_mod2(); break;
    case PG_UNI: pic_uni(); break;
    case PG_MIX: pic_mix(); break;
    case PG_PUMP: pic_pump(); break;
    case PG_DSND: pic_drum(); break;
    case PG_DSWP: pic_dswp(); break;
    case PG_DPRF: pic_dprf(); break;
    case PG_DENV: pic_denv(); break;
    default: pic_pills();
  }
}

/* the one moving thing on each page; called every 40 ms */
static void page_anim(void) {
  int pg = cur_page(), idx;
  switch (pg) {
    case PG_OSC: osc_anim(); break;
    case PG_GLO2:
      idx = playing && S.k[K_TLEN] ? t_pos : -1;
      if (idx != an_prev) {
        if (an_prev >= 0) tr_stop(an_prev, 0);
        if (idx >= 0) tr_stop(idx, 1);
        an_prev = idx;
      }
      break;
    case PG_MOD:
      if (an_prev >= 0) orbit_mark(MX, MY, 68, an_prev, PBG);
      an_u = (an_u + 48) & 4095;
      orbit_mark(MX, MY, 68, an_u, mix565(PBG, KC[0], 40 + an_lvl[an_u >> 5] * 216 / 256));
      an_prev = an_u;
      break;
    case PG_LFO: lfo_mark(); break;
    case PG_MIX: mix_meters(); break;
    case PG_PUMP: pump_anim(); break;
    case PG_DLY:
      if (S.k[K_DMODE]) break;
      idx = playing ? (int)(pulses % 6u) : -1;
      if (idx != an_prev) {
        if (an_prev >= 0) delay_dot(an_prev, 0);
        if (idx >= 0) delay_dot(idx, 1);
        an_prev = idx;
      }
      break;
    case PG_SPC: {
      int r0 = space_core();
      if (notes_n != an_notes) an_notes = notes_n, an_r = 4;
      else if (an_r > 0) an_r--;
      if (an_r != an_prev) {
        disk(SX, SY, r0 + 4, PBG);
        disk(SX, SY, r0 + an_r, an_r ? C_TEXT : KC[3]);
        if (an_r) disk(SX, SY, r0, KC[3]);
        an_prev = an_r;
      }
      break;
    }
    case PG_BUS:
      if (an_prev >= 0) bus_col(an_prev), bus_col(an_prev + 1);
      an_u = (an_u + 3) % 154;
      for (int i = 0; i < 4; i++)
        if (S.k[BUS_ID[i]]) rect(42 + an_u, bus_y(i, an_u) - 1, 2, 3, C_TEXT);
      an_prev = an_u;
      break;
    case PG_SMOD:
      idx = dph(S.k[K_MSRC]);
      idx = idx >= 0 ? (idx % 16) * 8 + 3 : -1;
      if (idx != an_prev) {
        if (an_prev >= 0) smod_col(an_prev, 0), smod_col(an_prev + 1, 0);
        if (idx >= 0) smod_col(idx, 1), smod_col(idx + 1, 1);
        an_prev = idx;
      }
      break;
    case PG_GLO:
      idx = playing ? (int)(pulses / 2u % 8u) : -1;
      if (idx != an_prev) {
        if (an_prev >= 0) glo_dot(an_prev, 0);
        if (idx >= 0) glo_dot(idx, 1);
        an_prev = idx;
      }
      break;
    case PG_DPRF:
    case PG_DENV: {
      int y = pg == PG_DPRF ? 110 : 160;
      idx = dph(track);
      if (idx != an_prev) {
        if (an_prev >= 0) rect(step_x(an_prev) + (pg == PG_DPRF ? 3 : 4), y, pg == PG_DPRF ? 7 : 11, 2, PBG);
        if (idx >= 0) rect(step_x(idx) + (pg == PG_DPRF ? 3 : 4), y, pg == PG_DPRF ? 7 : 11, 2, C_TEXT);
        an_prev = idx;
      }
      break;
    }
  }
}

/* ================================================================ edits */
static int accel(int e, int32_t det) {
  static uint32_t last[ENC_COUNT];
  uint32_t now = millis(), dt = now - last[e];
  if (!det) return 0;
  last[e] = now;
  return det * (dt < 25 ? 4 : dt < 60 ? 2 : 1);
}
static int sgn(int d) { return d > 0 ? 1 : -1; }

static void big_show(int id);
static void edit_k(int id, int d) {
  if (id == 255 || !d) return;
  int u = KD[id].unit == U_INT && id == K_BPM ? U_PCT : KD[id].unit, dd = (u == U_ALGO || u == U_SCALE || u == U_NOTE || u == U_SYNC || u == U_FRAC || u == U_DEST || u == U_SHAPE || u == U_TRACK || u == U_DMODE || u == U_LOOP ||
            u == U_TLEN || u == U_EVERY || u == U_STOP || u == U_ONOFF || u == U_INT) ? sgn(d) : d;
  int was = S.k[id];
  S.k[id] = (int16_t)clampi(S.k[id] + dd * KD[id].step, KD[id].min, KD[id].max);
  if (S.k[id] != was && (u == U_FRAC || u == U_DEST || u == U_SHAPE || u == U_DMODE || u == U_LOOP || u == U_SCALE ||
                         u == U_NOTE || u == U_ALGO || (u == U_LRATE && S.k[id] > 100)))
    big_show(id);
  for (int i = 0; i < NMORPH; i++) {
    if (MORPH_K[i] != id) continue;
    if (on_corner()) {
      S.corner[nearest_corner()][i] = S.k[id];
    } else {
      for (int c = 0; c < 4; c++) S.corner[c][i] = (int16_t)clampi(S.corner[c][i] + S.k[id] - was, KD[id].min, KD[id].max);
    }
  }
  if (id == K_BPM) clock_tempo((uint32_t)S.k[K_BPM] * 100u), dirty |= D_HEAD;
  if (id == K_SYNC) clock_follow(S.k[K_SYNC] ? MIDI_ANY : 0), dirty |= D_HEAD;
  if (id == K_SCALE || id == K_ROOT) dirty |= D_CTX;
  if (id >= K_INT && view == V_PERF) dirty |= D_MAIN;
}
static void snapshot_head(int gesture);
/* a stepped setting's new value, large over the picture for a moment */
static uint32_t big_until;
static char big_t[16];
static uint16_t big_c;
static void big_draw(void) {
  int w = text_width(big_t, FONT_BIG) + 24;
  if (w > 232) w = 232;
  capsule(120 - w / 2, 64, w, 44, 10, C_PANEL2);
  center(120 - w / 2, w, 70, big_t, big_c, FONT_BIG);
}
static void big_show(int id) {
  if (view != V_PAGE) return;
  for (int i = 0; i < 4; i++) {
    if (cell_slot(i) != id) continue;
    const char *l, *u;
    char v[24], t[28];
    int n;
    if (!get_cell(i, &l, v, &u, &n)) return;
    char *d = put(t, v);
    put(d, u);
    t[12] = 0;
    put(big_t, t), big_c = KC[i];
    big_draw();
    big_until = millis() + 700;
    return;
  }
}
static void edit_slot(int id, int d) {
  bank_t *b = CB();
  if (id == 255) return;
  if (id != X_LEN) snapshot_head(id);
  if (id < 100) {
    edit_k(id, d);
  } else if (id == X_VOL) {
    S.vol[track] = (uint8_t)clampi(S.vol[track] + d, 0, 100);
  } else if (id >= X_PDEP && id <= X_PTGT) {
    uint8_t *p = &S.pump[id - X_PDEP];
    *p = (uint8_t)(id <= X_PTIME ? clampi(*p + d, 0, 100) : d > 0);
    if (id >= X_PSRC) big_show(id);
    dirty |= D_MAIN;
  } else if (id >= X_SLEV && id <= X_MAST) {
    if (id == X_CH && mode) S.vol[track] = (uint8_t)clampi(S.vol[track] + d, 0, 100);
    else {
      uint8_t *p = id == X_SLEV ? &S.mix[0] : id == X_DLEV ? &S.mix[1] : id == X_MAST ? &S.mix[2] : &S.llev[lane];
      *p = (uint8_t)clampi(*p + d, 0, 127);
    }
    dirty |= D_MAIN;
  } else if (id == X_TSHIFT) {
    int c = S.k[K_TCUR];
    S.tseq[c] = (int8_t)clampi(S.tseq[c] + sgn(d), -7, 7);
    dirty |= D_MAIN;
  } else if (id == X_LEN) {
    b->dlen[track] = (uint8_t)clampi(b->dlen[track] + sgn(d), 1, DSTEPS);
    dirty |= D_MAIN;
  } else {
    int p = id - 100;
    const pdef_t *pd = &defs(track)[p];
    if (pd->kind == F_NONE) return;
    int dd = (pd->kind == F_FRAC || pd->kind == F_INT || pd->kind == F_NTYPE || pd->kind == F_PITCH || pd->kind == F_TICK) ? sgn(d) : d;
    S.def[track][p] = (uint8_t)clampi(S.def[track][p] + dd, pd->min, pd->max);
    if (pd->kind == F_NTYPE || pd->kind == F_FRAC) big_show(100 + p);
    if (p == P_ACC) dirty |= D_MAIN;
  }
  dirty |= D_VALS | D_GRAPH;
}
static void edit_slock(int k, int d) {
  bank_t *b = CB();
  for (int s = 0; s < MAXSTEP; s++) {
    if (!((held >> s) & 1)) continue;
    sstep_t *st = &b->st[lane][s];
    if (!DUR(st) && !TRIGLESS(st)) continue;
    if (k == 0) st->pitch = (int8_t)clampi(st->pitch + sgn(d), -12, 12);
    if (k == 1) st->vel = (uint8_t)clampi(st->vel + d, 10, 100);
    if (k == 2) st->index = (int8_t)clampi(st->index + d, -80, 80);
    if (k == 3) st->send = (uint8_t)clampi(st->send + d, 0, 100);
    st->dc |= 8;
    redraw_sstep(lane, s);
  }
  held_used = 1;
  dirty |= D_VALS | D_CTX;
}
static void edit_slockB(int k, int d) { /* lock set B on held synth steps */
  bank_t *b = CB();
  for (int s = 0; s < MAXSTEP; s++) {
    if (!((held >> s) & 1)) continue;
    uint8_t x = b->xb[lane][s];
    if (k == 0) x = (uint8_t)((x & ~7) | clampi((x & 7) + sgn(d), 0, 5));
    if (k == 1) x = (uint8_t)((x & ~0x18) | clampi(((x >> 3) & 3) + sgn(d), 0, 3) << 3);
    if (k == 2) x = (uint8_t)((x & ~0x60) | ((((x >> 5) & 3) + sgn(d) + 4) & 3) << 5);
    if (k == 3) x = (uint8_t)(d > 0 ? x | 128 : x & 127);
    b->xb[lane][s] = x;
  }
  held_used = 1;
  dirty |= D_VALS | D_CTX;
}
static void edit_dlock(int id, int d) {
  bank_t *b = CB();
  if (id < 100 || id >= 200) {
    say("NO LOCK ON THIS PAGE");
    held_used = 1;
    return;
  }
  int p = id - 100;
  const pdef_t *pd = &defs(track)[p];
  if (pd->kind == F_NONE) return;
  int dd = (pd->kind == F_FRAC || pd->kind == F_INT || pd->kind == F_NTYPE || pd->kind == F_PITCH || pd->kind == F_TICK) ? sgn(d) : d;
  for (int s = 0; s < DSTEPS; s++) {
    if (!((held >> s) & 1)) continue;
    uint8_t cur = b->lock[track][s][p];
    if (cur == UNLOCKED) cur = S.def[track][p];
    b->lock[track][s][p] = (uint8_t)clampi(cur + dd, pd->min, pd->max);
    draw_pad(s);
  }
  held_used = 1;
  dirty |= D_VALS | D_CTX;
}
static void edit_cond(int d) {
  bank_t *b = CB();
  for (int s = 0; s < MAXSTEP; s++) {
    if (!((held >> s) & 1)) continue;
    if (!mode) {
      sstep_t *st = &b->st[lane][s];
      if (!DUR(st)) continue;
      int c = (COND(st) + sgn(d) + NCOND) % NCOND;
      st->dc = (uint8_t)((st->dc & 15) | c << 4);
      redraw_sstep(lane, s);
    } else if (s < DSTEPS) {
      b->dcond[track][s] = (uint8_t)((b->dcond[track][s] & 16) | (((b->dcond[track][s] & 15) + sgn(d) + NCOND) % NCOND));
      draw_pad(s);
    }
  }
  held_used = 1;
  dirty |= D_CTX;
}

static void randomize_lane(int l) { /* downbeats likely, others sparse; longer notes rarer */
  bank_t *b = CB();
  int len = b->lane[l].len;
  for (int s = 0; s < MAXSTEP; s++) clear_sstep(&b->st[l][s]);
  for (int s = 0; s < len; s++) {
    int w = len % 4 == 0 ? (s % 4 == 0 ? 60 : s % 2 == 0 ? 30 : 10) : (s % 2 == 0 ? 35 : 12);
    if ((int)(random32() % 100u) < w) {
      uint32_t r = random32() % 100u;
      b->st[l][s].dc = (uint8_t)(r < 45 ? 1 : r < 75 ? 2 : r < 92 ? 3 : 4);
    }
  }
}
static void randomize_track(int t) {
  static const uint8_t W[TRACKS][4] = {{85, 5, 20, 5}, {10, 5, 5, 5}, {40, 15, 60, 15}, {20, 25, 20, 25}, {10, 5, 10, 5}};
  bank_t *b = CB();
  b->on[t] = 0;
  for (int s = 0; s < DSTEPS; s++) {
    int w = W[t][s & 3];
    if (t == 1 && (s == 4 || s == 12)) w = 90;
    if (t == 4 && (s == 4 || s == 12)) w = 50;
    if ((int)(random32() % 100u) < w) b->on[t] |= (uint16_t)(1u << s);
  }
  fill(b->lock[t], UNLOCKED, sizeof b->lock[t]);
  fill(b->dcond[t], 0, sizeof b->dcond[t]);
}

static bank_t undo_bank;
static uint8_t undo_head[HEAD_SIZE], undo_smute[16], undo_mix[16];
static uint8_t undo_ok, undo_i, undo_redo, undo_kind; /* kind 0 a bank, 1 the sound and song */
static void cpy(void *d, const void *src, uint32_t n) {
  uint8_t *a = (uint8_t *)d;
  const uint8_t *b = (const uint8_t *)src;
  while (n--) *a++ = *b++;
}
static void snapshot(void) {
  cpy(&undo_bank, CB(), sizeof(bank_t));
  undo_i = cur_bank, undo_ok = 1, undo_redo = 0, undo_kind = 0;
}
static void snapshot_head(int gesture) { /* before a sound or song edit: once per knob gesture */
  static uint32_t last_at;
  static int last_g = -1;
  uint32_t now = millis();
  if (gesture == last_g && now - last_at < 1500 && undo_kind == 1 && undo_ok) {
    last_at = now;
    return;
  }
  last_g = gesture, last_at = now;
  cpy(undo_head, &S, HEAD_SIZE);
  cpy(undo_smute, S.song_mute, 16);
  cpy(undo_mix, S.mix, 16);
  undo_ok = 1, undo_redo = 0, undo_kind = 1;
}
static void swap_bytes(uint8_t *a, uint8_t *b, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    uint8_t t = a[i];
    a[i] = b[i], b[i] = t;
  }
}
static void undo(void) {
  if (!undo_ok) {
    say("NOTHING TO UNDO");
    return;
  }
  if (undo_kind == 1) { /* the puck stays where it is */
    uint8_t x = S.xy[0], y = S.xy[1];
    swap_bytes(undo_head, (uint8_t *)&S, HEAD_SIZE);
    swap_bytes(undo_smute, S.song_mute, 16);
    swap_bytes(undo_mix, S.mix, 16);
    S.xy[0] = x, S.xy[1] = y;
    apply_morph();
    clock_tempo((uint32_t)S.k[K_BPM] * 100u);
    clock_follow(S.k[K_SYNC] ? MIDI_ANY : 0);
    if (song_cur >= S.song_n) song_cur = S.song_n ? S.song_n - 1 : 0;
  } else {
    swap_bytes((uint8_t *)&undo_bank, (uint8_t *)&S.bank[undo_i], sizeof(bank_t));
  }
  undo_redo ^= 1;
  say(undo_redo ? "UNDO" : "REDO");
  dirty = D_ALL;
}

static void song_edit(int k, int d) {
  snapshot_head(300 + k);
  if (k == 0) song_cur = (uint8_t)clampi(song_cur + sgn(d), 0, S.song_n ? S.song_n - 1 : 0);
  if (k == 1 && S.song_n) S.song_bank[song_cur] = (uint8_t)clampi(S.song_bank[song_cur] + sgn(d), 0, BANKS - 1);
  if (k == 2 && S.song_n) S.song_rep[song_cur] = (uint8_t)clampi(S.song_rep[song_cur] + sgn(d), 1, 8);
  if (k == 3) S.k[K_SONG] = (int16_t)(d > 0), say(d > 0 ? "SONG ON: PLAYS THE CHAIN" : "SONG OFF");
  dirty |= D_MAIN | D_VALS | D_CTX;
}
static void song_key(int w, int bk) {
  snapshot_head(-2 - (w >= 0 ? w : 10 + bk)); /* each key press is its own step */
  if (bk >= 0 && bk <= TRACKS && S.song_n) { /* black keys: this block's mutes (KICK..CLAP, then the synth) */
    S.song_mute[song_cur] ^= (uint8_t)(1u << bk);
    dirty |= D_MAIN;
    return;
  }
  if (w >= 0 && w < BANKS) {
    if (S.song_n >= 16) {
      say("16 BLOCKS IS THE MOST");
      return;
    }
    S.song_bank[S.song_n] = (uint8_t)w, S.song_rep[S.song_n] = 1, S.song_mute[S.song_n] = 0;
    song_cur = S.song_n++;
  } else if (bk == BK_CLR && S.song_n) {
    for (int i = song_cur; i < S.song_n - 1; i++)
      S.song_bank[i] = S.song_bank[i + 1], S.song_rep[i] = S.song_rep[i + 1], S.song_mute[i] = S.song_mute[i + 1];
    S.song_n--;
    if (song_cur >= S.song_n && song_cur) song_cur--;
  }
  dirty |= D_MAIN | D_VALS | D_CTX;
}

static void set_view(int v) {
  if (view != v) view = (uint8_t)v, dirty = D_ALL;
}
static void inst_view(void) { set_view(mode ? V_DRUM : V_SYNTH); } /* the steps */
static void set_pgi(int i) { /* a page button: its picture */
  pgi = (uint8_t)((i + NPGI) % NPGI);
  set_view(V_PAGE);
  dirty |= D_HEAD | D_MAIN | D_VALS | D_CTX;
}
static void page_button(int a, int b) { /* first press opens a, a second press b */
  if (view == V_PAGE && pgi == a && b >= 0) set_pgi(b);
  else set_pgi(a);
}
static void select_lane(int l) {
  int old = lane;
  lane = (uint8_t)l;
  held = 0;
  if (view == V_SYNTH) draw_lane(old), draw_lane(lane);
  dirty |= D_CTX | D_VALS;
}
static void select_track(int t) {
  track = (uint8_t)((t + TRACKS) % TRACKS);
  held = 0;
  if (view == V_DRUM || view == V_PAGE) dirty |= D_MAIN;
  dirty |= D_CTX | D_VALS | D_HEAD;
}
static void set_bank(int bnk) {
  if (playing) {
    queued_bank = (int8_t)bnk;
    say("BANK CHANGES AT THE BAR");
  } else {
    cur_bank = play_bank = (uint8_t)bnk;
    dirty = D_ALL;
  }
  dirty |= D_HEAD;
}
static void main_dirty(void) { dirty |= D_MAIN | D_CTX | D_VALS; }

static void black_down(int bk) {
  uint32_t now = millis();
  bank_t *b = CB();
  if (view == V_SONG) {
    song_key(-1, bk);
    return;
  }
  if (bk == BK_F5) {
    f5_since = now | 1;
    return;
  }
  if (key_held(black_key[BK_F5]) && bk < (mode ? TRACKS : LANES) && view != V_PERF) { /* mutes */
    f5_since = 0;
    if (mode) dmute ^= (uint8_t)(1u << bk), say((dmute >> bk) & 1 ? "TRACK MUTED" : "TRACK ON");
    else smute ^= (uint8_t)(1u << bk), say((smute >> bk) & 1 ? "LANE MUTED" : "LANE ON");
    main_dirty();
    return;
  }
  if (key_held(black_key[BK_CLR]) || bk == BK_CLR) {
    if (button_held(BTN_SAVE)) {
      save_used = 1;
      if (playing) playing = 0, clock_stop();
      init_all();
      clock_tempo((uint32_t)S.k[K_BPM] * 100u);
      held = 0;
      dirty = D_ALL;
      say("INIT");
      return;
    }
  }
  if (view == V_PERF) {
    if (bk < 4) {
      if (button_held(BTN_SAVE)) {
        for (int i = 0; i < NMORPH; i++) S.corner[bk][i] = S.k[MORPH_K[i]];
        save_used = 1;
        say("SOUND STORED IN CORNER");
      } else {
        S.xy[0] = bk & 1 ? 100 : 0;
        S.xy[1] = bk & 2 ? 100 : 0;
        apply_morph();
      }
      dirty |= D_MAIN | D_CTX | D_VALS;
    } else if (bk == BK_RND) {
      S.k[K_INT] = 50, dirty |= D_MAIN | D_VALS;
    } else if (bk == BK_CLR) {
      S.xy[0] = S.xy[1] = 50, apply_morph(), dirty |= D_MAIN | D_CTX;
    }
    return;
  }
  if (bk == BK_RND || bk == BK_CLR || (mode && (bk == BK_PASTE || bk == BK_CLRLOCK))) snapshot();
  if (!mode) {
    if (bk < LANES) {
      select_lane(bk);
      sprev = (uint8_t)(bk + 1);
    } else if (bk == BK_RND) {
      randomize_lane(lane);
      rnd_since = now | 1;
      say("RANDOM (HOLD: ALL LANES)");
      main_dirty();
    } else if (bk == BK_CLR) {
      if (held) {
        for (int s = 0; s < MAXSTEP; s++)
          if ((held >> s) & 1) {
            sstep_t *st = &b->st[lane][s];
            uint8_t d = DUR(st);
            clear_sstep(st);
            st->dc = d;
          }
        held_used = 1;
        say("LOCKS CLEARED");
      } else {
        for (int s = 0; s < MAXSTEP; s++) clear_sstep(&b->st[lane][s]);
        say("LANE CLEARED");
      }
      main_dirty();
    }
    return;
  }
  if (bk < TRACKS) {
    if (bk != track) select_track(bk);
    preview_req |= (uint8_t)(1u << bk);
    if (rec && playing && dpos[bk] >= 0) {
      b->on[bk] |= (uint16_t)(1u << dpos[bk]);
      main_dirty();
    }
  } else if (bk == BK_COPY) {
    clip.ok = 1, clip.len = b->dlen[track], clip.on = b->on[track];
    for (int s = 0; s < DSTEPS; s++) {
      clip.cond[s] = b->dcond[track][s];
      for (int p = 0; p < DP; p++) clip.lock[s][p] = b->lock[track][s][p];
    }
    say("TRACK COPIED");
  } else if (bk == BK_PASTE) {
    if (!clip.ok) {
      say("NOTHING COPIED");
      return;
    }
    b->dlen[track] = clip.len, b->on[track] = clip.on;
    for (int s = 0; s < DSTEPS; s++) {
      b->dcond[track][s] = clip.cond[s];
      for (int p = 0; p < DP; p++) b->lock[track][s][p] = clip.lock[s][p];
    }
    say("PASTED");
    main_dirty();
  } else if (bk == BK_CLRLOCK || (bk == BK_CLR && held)) {
    for (int s = 0; s < DSTEPS; s++)
      if ((held >> s) & 1) fill(b->lock[track][s], UNLOCKED, DP), b->dcond[track][s] = 0;
    held_used = 1;
    say("LOCKS CLEARED");
    main_dirty();
  } else if (bk == BK_RND) {
    randomize_track(track);
    say("RANDOM TRACK");
    main_dirty();
  } else if (bk == BK_CLR) {
    b->on[track] = 0;
    fill(b->lock[track], UNLOCKED, sizeof b->lock[track]);
    fill(b->dcond[track], 0, sizeof b->dcond[track]);
    say("TRACK CLEARED");
    main_dirty();
  }
}

static uint16_t keys_down, keys_latched; /* KEYS: white keys held, notes latched */
static uint8_t keys_latch;
static void white_down(int w) {
  if (view == V_SONG) {
    song_key(w, -1);
    return;
  }
  if (view == V_PERF) {
    if (w >= 8 && w <= 12) dmute ^= (uint8_t)(1u << (w - 8));
    else if (w == 13) synth_mute ^= 1;
    else if (w == 14) freeze = 1;
    else if (w == 15) dmute = 0, smute = 0, synth_mute = 0, say("ALL ON");
    else if (button_held(BTN_SAVE)) snap_take(&S.snap[w]), save_used = 1, say("SNAPSHOT STORED");
    else if (S.snap[w].used) snap_take(&snap_live), snap_put(&S.snap[w]), snap_held = (int8_t)w;
    else say("EMPTY: SAVE + KEY STORES IT");
    dirty |= D_MAIN | D_VALS;
    return;
  }
  if (keys_on && !mode && (view == V_SYNTH || view == V_PAGE)) { /* KEYS: the white keys play the scale */
    const bank_t *b = CB();
    if ((keys_latched >> w) & 1) { /* a latched note: this press lets it go */
      keys_latched &= (uint16_t)~(1u << w);
      note_req(1, 0, 1 + w, 0, -1, 0);
      return;
    }
    keys_down |= (uint16_t)(1u << w);
    note_req(0, lane, 1 + w, w - b->lane[lane].deg + 7 * (2 - b->lane[lane].oct), -1, 127);
    if (rec && playing) { /* recording: the note goes to the lane of that degree, at its playhead */
      for (int l = 0; l < LANES; l++)
        if (b->lane[l].deg == w && spos[l] >= 0) {
          snapshot();
          CB()->st[l][spos[l]].dc = (uint8_t)((CB()->st[l][spos[l]].dc & ~7) | 1);
          dirty |= D_MAIN;
          break;
        }
    }
    return;
  }
  int s = mode ? w : w + (spage ? 16 : 0);
  if (!held) held_used = 0, snapshot();
  held |= 1u << s;
  if (view == V_SYNTH && !mode) redraw_sstep(lane, s);
  if (view == V_DRUM && mode) draw_pad(s);
  dirty |= D_CTX | D_VALS;
}
static void keys_release_all(void) {
  for (int w = 0; w < 16; w++)
    if ((keys_latched >> w) & 1) note_req(1, 0, 1 + w, 0, -1, 0);
  keys_latched = 0;
}
static void white_up(int w) {
  keys_down &= (uint16_t)~(1u << w);
  if (keys_latch && keys_on && !mode && (view == V_SYNTH || view == V_PAGE)) keys_latched |= (uint16_t)(1u << w);
  else note_req(1, 0, 1 + w, 0, -1, 0); /* a KEYS note ends when its key is let go */
  if (view == V_PERF) {
    if (w == 14) freeze = 0, dirty |= D_MAIN;
    if (w == snap_held) snap_put(&snap_live), snap_held = -1, dirty |= D_MAIN | D_VALS;
    return;
  }
  if (view == V_SONG || (keys_on && !mode && (view == V_SYNTH || view == V_PAGE))) return;
  int s = mode ? w : w + (spage ? 16 : 0);
  if (!((held >> s) & 1)) return;
  held &= ~(1u << s);
  if (!held) lockset = 0;
  bank_t *b = CB();
  if (!held_used) {
    if (!mode) {
      if (s < b->lane[lane].len) {
        sstep_t *st = &b->st[lane][s];
        int d = DUR(st);
        if (d && d < 4) st->dc = (uint8_t)((st->dc & ~7) | (d + 1));
        else if (!d) st->dc = (uint8_t)((st->dc & ~7) | 1);
        else clear_sstep(st);
      }
    } else if (s < b->dlen[track]) {
      b->on[track] ^= (uint16_t)(1u << s);
      b->dcond[track][s] &= 15; /* a tap makes a trigless step a hit again */
    }
  }
  if (view == V_SYNTH) draw_lane(lane);
  if (view == V_DRUM) {
    draw_pad(s);
    for (int t = 0; t < TRACKS; t++) draw_ov_cell(t, s);
  }
  if (view == V_HOME) dirty |= D_MAIN;
  dirty |= D_CTX | D_VALS;
}

/* ================================================================ hold layers, as on Felucca: hold FX, SEQ or GLO
 * and the keys and knobs do that layer's jobs; a map of the keys shows on screen. A quick tap is the page button */
static const uint8_t LBTN[4] = {0, BTN_FX, BTN_SEQ, BTN_GLO};
static uint32_t lb_down[4];
static uint8_t lb_used[4], lay_drawn, lay_snap;
static uint16_t fx_white;
static uint32_t taps[4];
static uint8_t ntap;
static const char *const FXW[16] = {"FRZ", "ECHO", "SPC", "LOW", "HIGH", "THIN", "BUSY", "RPT", "FILL", 0, 0, 0, "A", "B", "C", "D"};
static const char *const SQW[16] = {"A", "B", "C", "D", "CLR", "REV", "<", ">", "RND", "COOK", "DBL", "HALF", "COPY", "PSTE", 0, 0};
static const char *const GLW[16] = {"TAP", "TOP", "SYNC", "KEYS", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "ALL"};
static uint8_t lay_latch; /* hold a layer's button + OCT+: it stays open until its button is tapped again */
static int layer_held(void) {
  if (button_held(BTN_SEQ)) return L_SEQ;
  if (button_held(BTN_FX)) return L_FX;
  if (button_held(BTN_GLO)) return L_GLO;
  return lay_latch;
}
static int layer_shown(void) {
  int l = layer_held();
  if (l && l == lay_latch && !button_held(LBTN[l])) return l;
  return l && (lb_used[l] || millis() - lb_down[l] > 250) ? l : L_NONE;
}
static const char *const *lay_names(int l) { return l == L_FX ? FXW : l == L_SEQ ? SQW : GLW; }
static uint16_t lay_col(int l) { return l == L_FX ? C_MINT : l == L_SEQ ? (mode ? C_CORAL : C_VIO) : C_AMBER; }
static uint16_t lay_wcol(int l, int i) {
  if (l == L_FX && i >= 12) return CORNER_COL[i - 12];
  if (l == L_SEQ && i < 4) return C_TEXT;
  return lay_col(l);
}
static int lay_black(int l) { return l == L_SEQ ? 0 : mode ? TRACKS : LANES; } /* black keys the layer uses */
static uint16_t wheld(void) {
  uint16_t m = 0;
  for (int k = 0; k < KEY_COUNT; k++)
    if (white_of[k] >= 0 && key_held(k)) m |= (uint16_t)(1u << white_of[k]);
  return m;
}
static void draw_layer(int l) {
  static const char *const LN[4] = {"", "FX", "SEQ", "GLO"};
  static const uint8_t BAFTER[11] = {0, 1, 2, 4, 5, 7, 8, 9, 11, 12, 14};
  const char *const *nm = lay_names(l);
  uint16_t c = lay_col(l), hw = wheld();
  rect(0, 19, SCREEN_W, 183, C_BG);
  capsule(4, 22, 40, 17, 8, c);
  center(4, 40, 22, LN[l], C_BG, FONT_SMALL);
  text(52, 22, l == L_FX ? "EFFECTS WHILE HELD" : l == L_SEQ ? (mode ? "BANKS, TRACK TOOLS" : "BANKS, LANE TOOLS") : "TEMPO, SOLO", C_MUTE, FONT_SMALL);
  for (int i = 0; i < 16; i++) {
    if (!nm[i]) continue;
    char b[10], *d = put_int(b, i + 1);
    *d++ = ' ';
    put(d, nm[i]);
    text(4 + (i & 3) * 59, 44 + (i >> 2) * 16, b, (hw >> i) & 1 ? C_TEXT : lay_wcol(l, i), FONT_SMALL);
  }
  const char *bl = l == L_FX ? (mode ? "BLACK: MUTE TRACKS" : "BLACK: MUTE LANES") : l == L_GLO ? (mode ? "BLACK: SOLO TRACKS" : "BLACK: SOLO LANES") : "";
  text(4, 110, bl, C_DIM, FONT_SMALL);
  for (int i = 0; i < 16; i++) {
    uint16_t kc = !nm[i] ? C_PANEL : (hw >> i) & 1 ? lay_wcol(l, i) : mix565(C_PANEL, lay_wcol(l, i), 110);
    rect(8 + i * 14, 130, 13, 52, kc);
  }
  int nb = lay_black(l);
  for (int bk = 0; bk < 11; bk++) {
    int x = 8 + (BAFTER[bk] + 1) * 14 - 6;
    uint16_t kc = C_LINE;
    if (bk < nb) {
      int m = mode ? (dmute >> bk) & 1 : (smute >> bk) & 1;
      kc = m ? C_PINK : mode ? TRACK_COL[bk] : LANE_COL[bk];
      if (key_held(black_key[bk])) kc = C_TEXT;
    }
    rect(x - 1, 130, 12, 33, C_BG);
    rect(x, 130, 10, 31, kc);
  }
  static const uint8_t MARK[4] = {0, 4, 8, 12};
  for (int i = 0; i < 4; i++) {
    char b[4];
    put_int(b, MARK[i] + 1);
    center(8 + MARK[i] * 14, 13, 185, b, C_DIM, FONT_SMALL);
  }
}

typedef struct {
  uint8_t on, cond;
  uint8_t lock[DP];
} dstep_t;
static void dget(int s, dstep_t *d) {
  const bank_t *b = CB();
  d->on = (uint8_t)((b->on[track] >> s) & 1), d->cond = b->dcond[track][s];
  for (int p = 0; p < DP; p++) d->lock[p] = b->lock[track][s][p];
}
static void dset(int s, const dstep_t *d) {
  bank_t *b = CB();
  b->on[track] = (uint16_t)((b->on[track] & ~(1u << s)) | (uint32_t)d->on << s);
  b->dcond[track][s] = d->cond;
  for (int p = 0; p < DP; p++) b->lock[track][s][p] = d->lock[p];
}
static struct {
  uint8_t ok;
  lane_t ln;
  sstep_t st[MAXSTEP];
} lclip;
static void euclid(int hits) { /* hits spread as evenly as they go over the length */
  bank_t *b = CB();
  int len = mode ? b->dlen[track] : b->lane[lane].len;
  hits = clampi(hits, 0, len);
  for (int s = 0; s < len; s++) {
    int on = hits && (s * hits) % len < hits;
    if (mode) b->on[track] = (uint16_t)((b->on[track] & ~(1u << s)) | (uint32_t)on << s);
    else b->st[lane][s].dc = (uint8_t)((b->st[lane][s].dc & ~7) | (on ? (DUR(&b->st[lane][s]) ? DUR(&b->st[lane][s]) : 1) : 0));
  }
}
static void tool_move(int w) { /* reverse, shift left, shift right, within the length */
  bank_t *b = CB();
  int len = mode ? b->dlen[track] : b->lane[lane].len;
  if (len < 2) return;
  if (!mode) {
    sstep_t t[MAXSTEP], *st = b->st[lane];
    for (int s = 0; s < len; s++) t[s] = st[w == 5 ? len - 1 - s : w == 6 ? (s + 1) % len : (s + len - 1) % len];
    for (int s = 0; s < len; s++) st[s] = t[s];
  } else {
    dstep_t t[DSTEPS];
    for (int s = 0; s < len; s++) dget(w == 5 ? len - 1 - s : w == 6 ? (s + 1) % len : (s + len - 1) % len, &t[s]);
    for (int s = 0; s < len; s++) dset(s, &t[s]);
  }
}
static void seq_tool(int w) {
  bank_t *b = CB();
  snapshot();
  int len = mode ? b->dlen[track] : b->lane[lane].len, mx = mode ? DSTEPS : MAXSTEP;
  switch (w) {
    case 4:
      if (mode) b->on[track] = 0, fill(b->lock[track], UNLOCKED, sizeof b->lock[track]), fill(b->dcond[track], 0, DSTEPS);
      else
        for (int s = 0; s < MAXSTEP; s++) clear_sstep(&b->st[lane][s]);
      say("CLEARED");
      break;
    case 5: tool_move(5), say("REVERSED"); break;
    case 6: tool_move(6), say("SHIFTED LEFT"); break;
    case 7: tool_move(7), say("SHIFTED RIGHT"); break;
    case 8: mode ? randomize_track(track) : randomize_lane(lane), say("NEW RHYTHM"); break;
    case 9: /* cook: a small change each press */
      for (int i = 0; i < 2; i++) {
        int s = (int)(random32() % (uint32_t)len);
        if (mode) {
          if (track == 0 && s == 0) continue;
          b->on[track] ^= (uint16_t)(1u << s);
        } else {
          sstep_t *st = &b->st[lane][s];
          int d = DUR(st);
          st->dc = (uint8_t)((st->dc & ~7) | (d && random32() % 2u ? 0 : 1 + random32() % 3u));
        }
      }
      say("COOKED");
      break;
    case 10:
      if (len * 2 > mx) {
        say(mode ? "16 STEPS IS THE MOST" : "32 STEPS IS THE MOST");
        break;
      }
      for (int s = 0; s < len; s++) {
        if (mode) {
          dstep_t d;
          dget(s, &d), dset(s + len, &d);
        } else {
          b->st[lane][s + len] = b->st[lane][s];
        }
      }
      if (mode) b->dlen[track] = (uint8_t)(len * 2);
      else b->lane[lane].len = (uint8_t)(len * 2);
      say("DOUBLED");
      break;
    case 11:
      if (len > 1) {
        if (mode) b->dlen[track] = (uint8_t)(len / 2);
        else b->lane[lane].len = (uint8_t)(len / 2);
      }
      say("HALVED");
      break;
    case 12:
      if (mode) {
        clip.ok = 1, clip.len = b->dlen[track], clip.on = b->on[track];
        for (int s = 0; s < DSTEPS; s++) {
          clip.cond[s] = b->dcond[track][s];
          for (int p = 0; p < DP; p++) clip.lock[s][p] = b->lock[track][s][p];
        }
      } else {
        lclip.ok = 1, lclip.ln = b->lane[lane];
        for (int s = 0; s < MAXSTEP; s++) lclip.st[s] = b->st[lane][s];
      }
      say("COPIED");
      break;
    case 13:
      if (mode && clip.ok) {
        b->dlen[track] = clip.len, b->on[track] = clip.on;
        for (int s = 0; s < DSTEPS; s++) {
          b->dcond[track][s] = clip.cond[s];
          for (int p = 0; p < DP; p++) b->lock[track][s][p] = clip.lock[s][p];
        }
        say("PASTED");
      } else if (!mode && lclip.ok) {
        uint8_t deg = b->lane[lane].deg, oct = b->lane[lane].oct; /* the lane keeps its own note */
        b->lane[lane] = lclip.ln, b->lane[lane].deg = deg, b->lane[lane].oct = oct;
        for (int s = 0; s < MAXSTEP; s++) b->st[lane][s] = lclip.st[s];
        say("PASTED");
      } else {
        say("NOTHING COPIED");
      }
      break;
  }
}
static void fx_key(int w, int on) {
  switch (w) {
    case 0: freeze = (uint8_t)on; break;
    case 1: fx_td = (uint8_t)on; break;
    case 2: fx_ts = (uint8_t)on; break;
    case 3: fx_mf = on ? -(3 << 16) : 0; break;
    case 4: fx_mf = on ? 2 << 16 : 0; break;
    case 5: int_ovr = (int8_t)(on ? 0 : -1); break;
    case 6: int_ovr = (int8_t)(on ? 100 : -1); break;
    case 7: rpt = (uint8_t)on; break;
    case 8: fill_on = (uint8_t)on; break;
    default:
      if (on && w >= 12) {
        S.xy[0] = (w - 12) & 1 ? 100 : 0;
        S.xy[1] = (w - 12) & 2 ? 100 : 0;
        apply_morph();
      }
  }
}
static void layer_key(int l, int w, int bk) {
  uint32_t now = millis();
  if (l == L_FX) {
    if (w >= 0) {
      if (!FXW[w]) return;
      fx_white |= (uint16_t)(1u << w);
      fx_key(w, 1);
    } else if (bk < lay_black(l)) {
      if (mode) dmute ^= (uint8_t)(1u << bk);
      else smute ^= (uint8_t)(1u << bk);
    }
  } else if (l == L_SEQ) {
    if (w < 0) return;
    if (w < BANKS) set_bank(w);
    else if (SQW[w]) seq_tool(w);
  } else {
    if (w == 0) { /* tap tempo */
      if (ntap && now - taps[ntap - 1] > 2000) ntap = 0;
      if (ntap == 4) {
        for (int i = 0; i < 3; i++) taps[i] = taps[i + 1];
        ntap = 3;
      }
      taps[ntap++] = now;
      if (ntap >= 2) {
        uint32_t per = (taps[ntap - 1] - taps[0]) / (uint32_t)(ntap - 1);
        if (per) {
          S.k[K_BPM] = (int16_t)clampi((int32_t)(60000u / per), 40, 220);
          clock_tempo((uint32_t)S.k[K_BPM] * 100u);
        }
      }
    } else if (w == 1) {
      if (playing) clock_stop(), clock_start();
      say("FROM THE TOP");
    } else if (w == 2) {
      S.k[K_SYNC] ^= 1;
      clock_follow(S.k[K_SYNC] ? MIDI_ANY : 0);
      say(S.k[K_SYNC] ? "SYNC: FOLLOWS MIDI CLOCK" : "SYNC: ITS OWN TEMPO");
    } else if (w == 3) {
      keys_on ^= 1;
      say(keys_on ? "KEYS: WHITE KEYS PLAY" : "KEYS OFF: WHITE KEYS ARE STEPS");
    } else if (w == 15) {
      dmute = smute = synth_mute = 0;
      say("ALL ON");
    } else if (w < 0 && bk < lay_black(l)) { /* solo: again to hear everything */
      uint8_t all = (uint8_t)((1u << lay_black(l)) - 1u), m = (uint8_t)(all & ~(1u << bk));
      volatile uint8_t *mu = mode ? &dmute : &smute;
      *mu = *mu == m ? 0 : m;
    }
  }
  dirty |= D_MAIN | D_HEAD | D_VALS;
}
static void layer_knob(int l, int k, int d) {
  bank_t *b = CB();
  if (l == L_FX) {
    static const uint8_t F[4] = {K_INT, K_KIT, K_THROW, K_FILT};
    edit_k(F[k], d);
  } else if (l == L_GLO) {
    static const uint8_t G[4] = {K_BPM, K_SWING, K_SCALE, K_ROOT};
    edit_k(G[k], d);
  } else {
    if (!lay_snap && k < 3) snapshot(), lay_snap = 1;
    int len = mode ? b->dlen[track] : b->lane[lane].len, hits = 0;
    for (int q = 0; q < len; q++) hits += mode ? (b->on[track] >> q) & 1 : DUR(&b->st[lane][q]) != 0;
    if (k == 0) {
      if (mode) b->dlen[track] = (uint8_t)clampi(len + sgn(d), 1, DSTEPS);
      else b->lane[lane].len = (uint8_t)clampi(len + sgn(d), 1, MAXSTEP);
    }
    if (k == 1) euclid(hits + sgn(d));
    if (k == 2) tool_move(d > 0 ? 7 : 6);
    if (k == 3) edit_k(K_SWING, d);
  }
  dirty |= D_VALS;
}

static uint8_t oct_prev_spage;
void setup(void) {
  static const uint8_t IS_BLACK[12] = {0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0}; /* from F */
  int w = 0, b = 0;
  for (int k = 0; k < KEY_COUNT; k++) {
    white_of[k] = black_of[k] = -1;
    if (IS_BLACK[k % 12]) black_key[b] = k, black_of[k] = (int8_t)b++;
    else white_of[k] = (int8_t)w++;
  }
  for (int r = 1; r <= 8; r++)
    for (int j = 0; j < r; j++) {
      int dy2 = 2 * r - 2 * j - 1, dx2 = isqrt(4 * r * r - dy2 * dy2);
      INS[r][j] = (uint8_t)(r - (dx2 + 1) / 2);
    }
  if (platform_last_crashed()) { /* a crash or the watchdog last time: start clean, keep the save as it is */
    init_all();
    safe_mode = 1;
    say("SAFE START: SAVE LOADS WORK");
  } else if (settings_load(&S, sizeof S) == 0 && S.magic == MAGIC) {
    say("RESTORED");
  } else if (settings_load(&S, OLD09_SIZE) == 0 && S.magic == MAGIC_09) { /* a 0.9 save: keep it, add the pump */
    S.magic = MAGIC;
    S.pump[0] = 0, S.pump[1] = 30, S.pump[2] = 0, S.pump[3] = 0;
    say("RESTORED FROM 0.9");
  } else if (settings_load(&S, OLD08_SIZE) == 0 && S.magic == MAGIC_08) { /* a 0.8 save: keep it, add the mix */
    S.magic = MAGIC;
    mix_defaults();
    say("RESTORED FROM 0.8");
  } else if (settings_load(&S, OLD07_SIZE) == 0 && S.magic == MAGIC_07) { /* a 0.7 save: keep it, add the rest */
    S.magic = MAGIC;
    fill(S.song_mute, 0, sizeof S - OLD07_SIZE);
    mix_defaults();
    say("RESTORED FROM 0.7");
  } else {
    init_all();
  }
  clear(C_BG);
  clock_tempo((uint32_t)S.k[K_BPM] * 100u);
  clock_send(1);
  clock_follow(S.k[K_SYNC] ? MIDI_ANY : 0);
  led_backlight(1);
  ready = 1;
  print("ORBIT DRIFT ready");
}

static void draw_all(void) {
  int ls = layer_shown();
  if (ls != lay_drawn) {
    if (!ls) drawn_view = 255; /* the layer closed: draw the page again */
    lay_drawn = (uint8_t)ls;
    dirty |= D_MAIN | D_VALS | D_HEAD;
  }
  uint8_t d = dirty;
  dirty = 0;
  if (ls) {
    if (d & D_HEAD) draw_head();
    if (d & D_MAIN) draw_layer(ls);
    if (d & D_VALS) draw_strip();
    return;
  }
  if (drawn_view != view) {
    clear(C_BG);
    drawn_view = view;
    d = D_ALL;
  }
  if (d & D_HEAD) draw_head();
  switch (view) {
    case V_HOME:
      if (d & D_MAIN) draw_orbit();
      if (msg_live()) {
        rect(40, 222, 160, 17, C_BG);
        center(40, 160, 222, msg, C_LIME, FONT_SMALL);
      } else if (d & D_CTX) {
        if (msg[0]) msg[0] = 0, draw_orbit();
      }
      break;
    case V_SYS:
      if (d & D_MAIN) draw_sys();
      break;
    case V_SYNTH:
      if (d & D_MAIN) {
        rect(0, 19, SCREEN_W, 166, C_BG);
        for (int l = 0; l < LANES; l++) draw_lane(l);
      }
      break;
    case V_DRUM:
      if (d & D_MAIN) draw_drum_main();
      else if (d & D_VALS) draw_dface_graph();
      break;
    case V_PAGE:
      if (d & (D_MAIN | D_VALS)) {
        draw_pic(), draw_mini();
        if (big_until) big_draw(); /* a stepped value's name stays on top for its moment */
      }
      break;
    case V_PERF:
      if (d & D_MAIN) {
        rect(0, 19, SCREEN_W, 166, C_BG);
        draw_ring(), draw_pills(), draw_snaps(), draw_mutes();
      }
      break;
    case V_SONG:
      if (d & D_MAIN) draw_song();
      break;
  }
  if (view >= V_SYNTH) {
    if (d & D_CTX) draw_ctx(185);
    if (d & D_VALS) draw_strip();
  }
}

static void update_playheads(void) {
  if (lay_drawn) return;
  if (view == V_SYNTH) {
    for (int l = 0; l < LANES; l++) {
      int ph = sph(l);
      if (ph == drawn_s[l]) continue;
      int old = drawn_s[l];
      drawn_s[l] = (int8_t)ph;
      redraw_sstep(l, old);
      redraw_sstep(l, ph);
    }
  } else if (view == V_HOME) {
    for (int l = 0; l < LANES; l++) {
      int ph = sph(l);
      if (ph == drawn_s[l]) continue;
      int old = drawn_s[l];
      drawn_s[l] = (int8_t)ph;
      if (old >= 0 && old < CB()->lane[l].len) draw_seg(l, old, 1);
      if (ph >= 0) draw_seg(l, ph, 1);
    }
    int ph = dph(0);
    if (ph != drawn_d[0]) {
      int old = drawn_d[0];
      drawn_d[0] = (int8_t)ph;
      if (old >= 0) draw_halo(old);
      if (ph >= 0) draw_halo(ph);
    }
    if (pulses != drawn_pulse) drawn_pulse = pulses, draw_orbit_text();
  } else if (view == V_DRUM) {
    for (int t = 0; t < TRACKS; t++) {
      int ph = dph(t);
      if (ph == drawn_d[t]) continue;
      int old = drawn_d[t];
      drawn_d[t] = (int8_t)ph;
      if (old >= 0) draw_ov_cell(t, old);
      if (ph >= 0) draw_ov_cell(t, ph);
      if (t == track) {
        if (old >= 0) draw_pad(old);
        if (ph >= 0) draw_pad(ph);
      }
    }
  }
}

void loop(void) {
  uint32_t now = millis();
  /* keys */
  int lh = layer_held();
  for (int k = 0; k < KEY_COUNT; k++) {
    if (key_pressed(k)) {
      if (lh) lb_used[lh] = 1, layer_key(lh, white_of[k], black_of[k]);
      else if (white_of[k] >= 0) white_down(white_of[k]);
      else black_down(black_of[k]);
    }
    if (key_released(k)) {
      int w = white_of[k];
      if (w >= 0 && ((fx_white >> w) & 1)) { /* an effect ends when its key is let go */
        fx_white &= (uint16_t)~(1u << w);
        fx_key(w, 0);
        dirty |= D_MAIN;
      } else if (lh) {
        if (w >= 0) white_up(w);
        dirty |= D_MAIN;
      } else if (w >= 0) {
        white_up(w);
      } else {
        int bk = black_of[k];
        if (bk == BK_RND) rnd_since = 0;
        if (bk == BK_F5) f5_since = 0;
        dirty |= D_CTX | D_VALS;
      }
    }
  }
  if (rnd_since && now - rnd_since > 1000) {
    rnd_since = 0;
    for (int l = 0; l < LANES; l++) randomize_lane(l);
    say("ALL LANES RANDOM");
    main_dirty();
  }
  if (f5_since && view == V_PERF && now - f5_since > 1000) {
    f5_since = 0;
    dmute = smute = synth_mute = 0;
    say("ALL ON");
    dirty |= D_MAIN;
  }

  /* buttons */
  if (button_pressed(BTN_PLAY)) {
    playing = !playing;
    playing ? clock_start() : clock_stop();
    dirty |= D_HEAD;
  }
  for (int l = 1; l <= 3; l++) {
    int bt = LBTN[l];
    if (button_pressed(bt)) lb_down[l] = now, lb_used[l] = 0, lay_snap = 0;
    static uint32_t last_tap[4];
    (void)last_tap;
    if (button_released(bt) && !lb_used[l] && now - lb_down[l] < 400 && lay_latch == l) { /* closes a kept layer */
      lay_latch = 0, last_tap[l] = 0, dirty = D_ALL;
    } else if (button_released(bt) && !lb_used[l] && now - lb_down[l] < 400) { /* a tap: the page button */
      last_tap[l] = now;
      if (l == L_SEQ) { /* to the steps; from the steps, the other instrument */
        if (view == V_SYNTH || view == V_DRUM) mode = !mode;
        held = 0;
        inst_view();
        dirty = D_ALL;
      } else if (l == L_FX) {
        int fx = view == V_PAGE && pgi >= FX0 && pgi <= FX1;
        set_pgi(fx ? (pgi == FX1 ? FX0 : pgi + 1) : FX0);
      } else {
        page_button(GLO0, GLO0 + 1);
      }
    }
  }
  static int16_t mom_k[NK], mom_corner[4][NMORPH];
  static uint8_t mom_def[TRACKS][DP], mom_vol[TRACKS], mom_used, mom_said;
  static uint32_t mom_down, save_down;
  if (button_pressed(BTN_LFO)) { /* hold LFO and turn a knob: the value springs back when LFO is let go */
    cpy(mom_k, S.k, sizeof mom_k), cpy(mom_corner, S.corner, sizeof mom_corner);
    cpy(mom_def, S.def, sizeof mom_def), cpy(mom_vol, S.vol, sizeof mom_vol);
    mom_used = mom_said = 0, mom_down = now;
  }
  if (button_held(BTN_LFO) && !mom_used && !mom_said && now - mom_down > 400) mom_said = 1, say("TURN A KNOB: IT SPRINGS BACK");
  if (button_released(BTN_LFO)) {
    if (mom_used) {
      cpy(S.k, mom_k, sizeof mom_k), cpy(S.corner, mom_corner, sizeof mom_corner);
      cpy(S.def, mom_def, sizeof mom_def), cpy(S.vol, mom_vol, sizeof mom_vol);
      clock_tempo((uint32_t)S.k[K_BPM] * 100u);
      dirty = D_ALL;
    } else if (now - mom_down < 400) {
      page_button(6, -1);
    }
  }
  if (button_pressed(BTN_SAVE)) save_used = 0, save_down = now;
  if (button_held(BTN_SAVE) && !save_used && now - save_down > 600) save_used = 1, undo();
  if (button_released(BTN_SAVE) && !save_used && safe_mode) { /* after a safe start: SAVE brings the saved work back */
    safe_mode = 0;
    if (settings_load(&S, sizeof S) == 0 && S.magic == MAGIC) say("YOUR WORK IS BACK"), apply_morph();
    else say("NO SAVE TO LOAD");
    clock_tempo((uint32_t)S.k[K_BPM] * 100u);
    dirty = D_ALL;
    save_used = 1;
  }
  if (button_released(BTN_SAVE) && !save_used) {
    if (playing) say("STOP TO SAVE");
    else say(settings_save(&S, sizeof S) == 0 ? "SAVED" : "SAVE FAILED");
  }
  if (button_pressed(BTN_HOME)) set_view(view == V_HOME ? V_SONG : view == V_SONG ? V_SYS : V_HOME);
  if (button_pressed(BTN_ARP)) {
    if (view == V_PERF) inst_view();
    else set_view(V_PERF);
  }
  if (button_pressed(BTN_EDIT)) {
    if (held && view != V_PERF) {
      bank_t *b = CB();
      if (!mode) {
        int on = !LOCKED(&b->st[lane][first_held()]);
        for (int s = 0; s < MAXSTEP; s++)
          if (((held >> s) & 1) && DUR(&b->st[lane][s])) b->st[lane][s].dc = (uint8_t)(on ? b->st[lane][s].dc | 8 : b->st[lane][s].dc & ~8);
        say(on ? "LOCKS ON" : "LOCKS OFF");
      } else {
        for (int s = 0; s < DSTEPS; s++)
          if ((held >> s) & 1) fill(b->lock[track][s], UNLOCKED, DP);
        say("LOCKS CLEARED");
      }
      held_used = 1;
      main_dirty();
    } else {
      page_button(0, 1);
    }
  }
  if (button_pressed(BTN_SEL)) {
    if (held && view != V_PERF && view != V_SONG) { /* with steps held: lock set A / B */
      lockset ^= 1, held_used = 1;
      say(lockset ? "LOCKS B" : "LOCKS A");
      dirty |= D_VALS | D_CTX;
    } else if (mode) {
      set_pgi(2);
    } else {
      set_pgi(view == V_PAGE && pgi >= 2 && pgi <= 4 ? (pgi == 4 ? 2 : pgi + 1) : 2);
    }
  }
  if (button_pressed(BTN_ENV)) page_button(5, -1);
  static uint32_t rec_down;
  if (button_pressed(BTN_REC)) {
    rec_down = now;
    if (held && view != V_PERF && view != V_SONG) { /* hold steps + REC: trigless */
      if (mode) {
        bank_t *b = CB();
        int on = !(b->dcond[track][first_held()] & 16);
        for (int s = 0; s < DSTEPS; s++)
          if ((held >> s) & 1) {
            if (on) b->on[track] &= (uint16_t)~(1u << s), b->dcond[track][s] |= 16;
            else b->dcond[track][s] &= 15;
          }
        say(on ? "TRIGLESS: MOVES THE HIT" : "TRIGLESS OFF");
        main_dirty();
      } else {
        bank_t *b = CB();
        int on = !TRIGLESS(&b->st[lane][first_held()]);
        for (int s = 0; s < MAXSTEP; s++)
          if ((held >> s) & 1) b->st[lane][s].dc = (uint8_t)((b->st[lane][s].dc & ~7) | (on ? 7 : 0));
        say(on ? "TRIGLESS: MOVES THE NOTE" : "TRIGLESS OFF");
        main_dirty();
      }
      held_used = 1;
    } else if (view == V_PERF) {
      if (path_rec) path_rec = 0, path_play = path_len > 0, path_from = pulses;
      else if (path_len && !path_play) path_play = 1, path_from = pulses;
      else if (!playing) say("PRESS PLAY TO RECORD A PATH");
      else path_rec = 1, path_play = 0, path_len = 0;
      dirty |= D_MAIN | D_CTX;
    } else {
      rec = !rec, dirty |= D_HEAD | D_CTX;
    }
  }
  if (view == V_PERF && button_held(BTN_REC) && rec_down && now - rec_down > 1000) {
    rec_down = 0, path_rec = path_play = 0, path_len = 0;
    say("PATH CLEARED");
    dirty |= D_MAIN | D_CTX;
  }
  if (playing && pulses != path_pulse) { /* the morph path, a step at a time */
    path_pulse = pulses;
    if (path_rec) {
      path[path_len][0] = S.xy[0], path[path_len][1] = S.xy[1];
      if (++path_len >= 128) path_rec = 0, path_play = 1, path_from = pulses + 1;
      if (view == V_PERF) dirty |= D_CTX;
    } else if (path_play && path_len) {
      uint32_t i = (pulses - path_from) % path_len;
      if (S.xy[0] != path[i][0] || S.xy[1] != path[i][1]) {
        S.xy[0] = path[i][0], S.xy[1] = path[i][1];
        apply_morph();
        if (view == V_PERF) dirty |= D_MAIN | D_CTX;
      }
    }
  }
  if (button_pressed(BTN_OCT_UP) && lh && button_held(LBTN[lh])) { /* hold a layer's button + OCT+: it stays open */
    lay_latch = (uint8_t)lh, lb_used[lh] = 1;
    say("LAYER KEPT: TAP TO CLOSE");
  } else if (lh) {
  } else if (keys_on && !mode && (view == V_SYNTH || view == V_PAGE) &&
             ((button_pressed(BTN_OCT_DOWN) && button_held(BTN_OCT_UP)) || (button_pressed(BTN_OCT_UP) && button_held(BTN_OCT_DOWN)))) {
    keys_latch ^= 1; /* both octave buttons: KEYS latch */
    if (!keys_latch) keys_release_all();
    spage = oct_prev_spage;
    say(keys_latch ? "LATCH: NOTES HOLD" : "LATCH OFF");
    dirty |= D_HEAD | D_MAIN;
  } else if (button_pressed(BTN_OCT_DOWN) || button_pressed(BTN_OCT_UP)) {
    int up = button_pressed(BTN_OCT_UP);
    oct_prev_spage = spage;
    if (mode) {
      select_track(track + (up ? 1 : -1));
    } else {
      spage = (uint8_t)up;
      held = 0;
      if (view == V_SYNTH) draw_lane(lane);
      say(up ? "STEPS 17-32" : "STEPS 1-16");
    }
  }

  /* encoders */
  int ex = accel(ENC_ALGORITHM, encoder(ENC_ALGORITHM)), ey = accel(ENC_PRESETS, encoder(ENC_PRESETS));
  if (ex || ey) {
    if (trail_n < 16) trail_n++;
    for (int i = 0; i < trail_n - 1; i++) trail[i][0] = trail[i + 1][0], trail[i][1] = trail[i + 1][1];
    trail[trail_n - 1][0] = S.xy[0], trail[trail_n - 1][1] = S.xy[1];
    if (path_play && !path_rec) path_play = 0, say("PATH PAUSED: REC PLAYS IT");
    S.xy[0] = (uint8_t)clampi(S.xy[0] + ex * 2, 0, 100); /* ALGORITHM: X */
    S.xy[1] = (uint8_t)clampi(S.xy[1] + ey * 2, 0, 100); /* PRESETS: Y */
    apply_morph();
    if (view == V_PERF || view == V_PAGE) dirty |= D_MAIN | D_CTX;
    if (view != V_PERF && now - puck_say > 120) {
      char b[28], *d = put(b, on_corner() ? "CORNER " : "MORPH NEAR ");
      *d++ = (char)('A' + nearest_corner());
      d = put(d, "  X ");
      d = put_int(d, S.xy[0]);
      d = put(d, " Y ");
      put_int(d, S.xy[1]);
      say(b);
      puck_say = now;
    }
    dirty |= D_VALS;
  }
  int sel = encoder(ENC_SELECT);
  if (sel && button_held(BTN_GLO)) { /* hold GLO and turn SELECT: tempo */
    lb_used[L_GLO] = 1;
    edit_k(K_BPM, accel(ENC_SELECT, sel));
    dirty |= D_VALS | D_HEAD;
    sel = 0;
  }
  if (sel) {
    if (held && view != V_PERF) edit_cond(sel);
    else if (view >= V_SYNTH && view != V_PERF) { /* the lane or the track */
      if (mode) {
        select_track(clampi(track + sgn(sel), 0, TRACKS - 1));
      } else if (lane + sgn(sel) >= 0 && lane + sgn(sel) < LANES) {
        select_lane(lane + sgn(sel));
        sprev = (uint8_t)(lane + 1);
      }
    }
  }
  int lk = lane_key_held(), tk = track_key_held(), s0 = first_held();
  for (int k = 0; k < 4; k++) {
    int d = accel(ENC_KNOB1 + k, encoder(ENC_KNOB1 + k));
    if (!d) continue;
    if (lh) {
      lb_used[lh] = 1;
      layer_knob(lh, k, d);
      continue;
    }
    if (button_held(BTN_LFO)) mom_used = 1;
    if (view == V_SONG) {
      song_edit(k, d);
      continue;
    }
    if (view == V_SYS && k == 3 && cpu1_state() == 4) {
      say("2ND CORE: NOT IN THIS BUILD");
      continue;
    }
    if (view == V_SYS && k == 3) { /* KNOB4 on SYSTEM: the second core, from the next start */
      int on = cpu1_option(d > 0);
      say(on ? "2ND CORE ON: NEXT START" : "2ND CORE OFF: NEXT START");
      dirty |= D_MAIN;
      continue;
    }
    if (view == V_HOME || view == V_SYS) {
      say(view == V_SYS ? "KNOB4: 2ND CORE (BETA)" : "SEQ: STEPS  EDIT: SOUND");
      continue;
    }
    if (view == V_PERF) {
      edit_k(PAGE_SLOT[PG_PERF][k], d);
      dirty |= D_MAIN;
      show_pop(k);
      continue;
    }
    if (lk >= 0) {
      lane_t *ln = &CB()->lane[lk];
      if (k == 0) ln->len = (uint8_t)clampi(ln->len + sgn(d), 1, MAXSTEP);
      if (k == 1) ln->dir = (uint8_t)((ln->dir + sgn(d) + 4) % 4);
      if (k == 2) ln->deg = (uint8_t)clampi(ln->deg + sgn(d), 0, 13);
      if (k == 3) ln->oct = (uint8_t)clampi(ln->oct + sgn(d), 0, 4);
      if (k == 0) draw_lane(lk);
      dirty |= D_CTX;
    } else if (tk >= 0) {
      if (k == 0) CB()->dlen[tk] = (uint8_t)clampi(CB()->dlen[tk] + sgn(d), 1, DSTEPS), main_dirty();
    } else if (s0 >= 0) {
      if (!mode && lockset) edit_slockB(k, d);
      else if (!mode) edit_slock(k, d);
      else edit_dlock(cell_slot(k), d);
    } else if (rec && playing && view == V_SYNTH && !mode && spos[lane] >= 0) {
      /* REC while playing, on the lanes: KNOB1-4 write PITCH, VEL, INDEX and SEND on the step under the playhead */
      uint32_t keep = held;
      held = 1u << spos[lane];
      edit_slock(k, d);
      held = keep;
    } else if (rec && playing && view == V_PAGE && (mode ? cell_slot(k) >= 100 && cell_slot(k) < 200 && dpos[track] >= 0 :
                                                   cell_slot(k) == K_INDEX && spos[lane] >= 0)) {
      /* REC while playing: the knob writes a lock on the step under the playhead */
      uint32_t keep = held;
      held = 1u << (mode ? dpos[track] : spos[lane]);
      if (mode) edit_dlock(cell_slot(k), d);
      else edit_slock(2, d);
      held = keep;
      say("RECORDED ON THE STEP");
    } else {
      edit_slot(cell_slot(k), d);
    }
    show_pop(k);
    if (view == V_PAGE) dirty |= D_MAIN;
  }
  if (S.k[K_SYNC] && playing != running) playing = running, dirty |= D_HEAD;
  { /* MIDI in: notes play the selected lane's sound; channel 10 plays the drums */
    static const int8_t GM[13] = {0, 0, 4, 1, 2, 1, 3, 2, 3, 2, 3, 2, 3}; /* notes 35-47 */
    uint8_t st, d1, d2;
    for (int guard = 0; guard < 32 && midi_read(&st, &d1, &d2); guard++) {
      uint8_t ty = st & 0xF0, ch = st & 15;
      if (ty == 0x90 && d2) {
        if (ch == 9) {
          if (d1 >= 35 && d1 <= 47) note_req(2, GM[d1 - 35], 0, 0, -1, d2);
          else if (d1 == 48 || d1 == 50) note_req(2, 3, 0, 0, -1, d2);
        } else {
          note_req(0, lane, 32 + (d1 & 127), 0, d1, d2);
        }
      } else if ((ty == 0x80 || ty == 0x90) && ch != 9) {
        note_req(1, 0, 32 + (d1 & 127), 0, -1, 0);
      }
    }
  }
  { /* recording knobs on the lanes: the values follow the step under the playhead */
    static int8_t rec_sp = -1;
    int sp = rec && playing && view == V_SYNTH && !mode && !held ? spos[lane] : -1;
    if (sp != rec_sp) rec_sp = (int8_t)sp, dirty |= D_VALS | D_CTX;
  }

  /* CPU guard: the synth gets fewer notes at once when the chip runs short, more back when there is room */
  static uint32_t over_since, under_since, head_at, hist_at;
  uint16_t load = load_pct;
  if (load > 80) {
    if (!over_since) over_since = now | 1;
    if (now - over_since > 200 && vcap > 4) vcap = (uint8_t)(vcap - 2), over_since = 0, say("CPU: FEWER SYNTH NOTES");
  } else {
    over_since = 0;
  }
  if (load < 45 && vcap < MAXSV) {
    if (!under_since) under_since = now | 1;
    if (now - under_since > 3000) vcap = (uint8_t)(vcap + 2), under_since = 0;
  } else {
    under_since = 0;
  }
  if (now - hist_at > 500) {
    hist_at = now;
    if (hist_n < 54) hist_n++;
    for (int i = 0; i < hist_n - 1; i++) hist[i] = hist[i + 1];
    hist[hist_n - 1] = (uint8_t)(load > 255 ? 255 : load);
    if (view == V_SYS) dirty |= D_MAIN;
  }
  if (now - head_at > 500) head_at = now, dirty |= D_HEAD;
  if (msg[0] && (int32_t)(msg_until - now) <= 0) dirty |= D_CTX;

  draw_all();
  update_playheads();
  if (view == V_SONG && !lay_drawn) { /* the reels turn while the song plays */
    static uint32_t reel_at;
    static uint8_t drawn_pos = 255;
    static int ang;
    if (song_pos != drawn_pos) drawn_pos = song_pos, draw_song();
    if (playing && S.k[K_SONG] && now - reel_at > 80) reel_at = now, ang = (ang + 160) & 4095, song_reels(ang);
  }
  { /* restore at power-on: an autosave, written only when stopped and left alone (at most once a minute) */
    static uint32_t sum_at, saved_sum, last_save, touched;
    if (keys() || buttons()) touched = now;
    if (!saved_sum || (!playing && now - sum_at > 2000)) {
      sum_at = now;
      uint32_t h = 2166136261u;
      const uint8_t *p = (const uint8_t *)&S;
      for (uint32_t i = 0; i < sizeof S; i++) h = (h ^ p[i]) * 16777619u;
      if (!saved_sum) saved_sum = h;
      if (h != saved_sum && !safe_mode && now - touched > 10000 && (!last_save || now - last_save > 60000)) {
        if (settings_save(&S, sizeof S) == 0) saved_sum = h, last_save = now;
      }
    }
  }
  if (view == V_PAGE) { /* the steps under the picture follow the playhead; the LFO's marker runs */
    static int8_t mini_ph = -2;
    static uint32_t lfo_at;
    int ph = mode ? dph(track) : sph(lane);
    if (!lay_drawn && ph != mini_ph) mini_ph = (int8_t)ph, draw_mini();
    if (!lay_drawn && now - lfo_at >= 40 && !big_until) lfo_at = now, page_anim();
  }

  { /* the header meter, and the big name's time up */
    static uint32_t hm_at;
    if (!lay_drawn && now - hm_at >= 40) hm_at = now, draw_hmeter();
    if (big_until && (int32_t)(now - big_until) > 0) big_until = 0, dirty |= D_MAIN;
  }
  /* LEDs */
  const bank_t *b = CB();
  uint16_t keys_lit = 0;
  if (keys_on && !mode && playing)
    for (int l = 0; l < LANES; l++) {
      int sp = sph(l);
      if (sp < 0 || !DUR(&b->st[l][sp]) || ((smute >> l) & 1)) continue;
      int kw = b->lane[l].deg + t_shift + 7 * b->lane[l].oct - 14;
      if (kw >= 0 && kw < 16) keys_lit |= (uint16_t)(1u << kw);
    }
  for (int k = 0; k < KEY_COUNT; k++) {
    int on = 0, w = white_of[k], bk = black_of[k];
    if (lay_drawn) {
      on = w >= 0 ? lay_names(lay_drawn)[w] != 0 : bk < lay_black(lay_drawn);
    } else if (view == V_SONG) {
      on = (w >= 0 && w < BANKS) || bk == BK_CLR;
    } else if (view == V_PERF && w >= 0 && w < 8) {
      on = S.snap[w].used;
    } else if (view == V_PERF) {
      if (w >= 8 && w <= 12) on = !((dmute >> (w - 8)) & 1);
      else if (w == 13) on = !synth_mute;
      else if (w == 14) on = freeze;
      else if (bk >= 0 && bk < 4) on = bk == nearest_corner();
    } else if (!mode && keys_on && (view == V_SYNTH || view == V_PAGE)) { /* KEYS: the notes the lanes play now, and held keys */
      on = w >= 0 ? (int)(((keys_lit | keys_down | keys_latched) >> w) & 1) : bk == lane;
    } else if (!mode) {
      if (w >= 0) {
        int s = w + (spage ? 16 : 0);
        on = s < b->lane[lane].len && ((DUR(&b->st[lane][s]) != 0) ^ (s == sph(lane)));
      } else {
        on = bk == lane;
      }
    } else {
      if (w >= 0) on = w < b->dlen[track] && ((((b->on[track] >> w) & 1) != 0) ^ (w == dph(track)));
      else on = bk == track;
    }
    led_key(k, on);
  }
  int pg = cur_page();
  led_button(BTN_PLAY, playing);
  led_button(BTN_REC, rec);
  led_button(BTN_SEQ, view == V_SYNTH || view == V_DRUM);
  led_button(BTN_HOME, view == V_HOME || view == V_SYS || view == V_SONG);
  led_button(BTN_ARP, view == V_PERF);
  int inst = view == V_PAGE;
  led_button(BTN_EDIT, inst && pgi <= 1);
  led_button(BTN_SEL, inst && pgi >= 2 && pgi <= 4);
  led_button(BTN_ENV, inst && pgi == 5);
  led_button(BTN_FX, (inst && pgi >= FX0 && pgi <= FX1) || lay_drawn == L_FX);
  led_button(BTN_LFO, inst && pgi == 6);
  led_button(BTN_GLO, inst && (pg == PG_GLO || pg == PG_GLO2));
  led_button(BTN_OCT_DOWN, !mode && !spage);
  led_button(BTN_OCT_UP, !mode && spage);
}
