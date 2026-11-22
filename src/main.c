/* SPDX-License-Identifier: GPL-3.0-only */
/* ORBIT DRIFT for the M-VAVE FM-1: a polymetric FM synth and an FM drum machine on one clock, sharing a twin
 * delay and a space reverb. Built from ORBITAL (lanes, FM voice, contour filter), Scenedrift (locks, trig
 * conditions, XY morph, twin delay) and FM-16 (drums, echoes), with sweeps, mod envelopes and fractional ratios.
 *
 *   SEQ           the steps; again: SYNTH / DRUMS (hold SEQ + white key 1-4: bank A-D, waits for the bar while playing)
 *   HOME          the orbit view (again: SYSTEM, the CPU and voice meter)      ARP  PERFORM
 *   EDIT          synth OSC / TONE, drums SOUND / SWEEP     SEL  synth MOD, drums PERF     ENV  envelope
 *   FX            DELAY / SPACE / BUS (sends)               GLO  GLOBAL / GLOBAL 2
 *   PRESETS       orbits the synth puck past corners A-D    ALGORITHM  pulls it to the centre (on every page)
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
       K_DTA, K_DTB, K_DFB, K_DWET, K_SPACE, K_TRAIL, K_DIRT, K_MIX, K_SSD, K_SSS, K_SDD, K_SDS,
       K_BPM, K_SWING, K_SCALE, K_ROOT, K_SYNC, K_INT, K_KIT, K_THROW, K_FILT, NK };
enum { U_ALGO, U_FRAC, U_DIV10, U_PCT, U_MS, U_SEC100, U_HZ, U_SCALE, U_NOTE, U_INT, U_SYNC, U_CENTER };
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
  {0, 100, 1, U_PCT, "WET"},       {0, 100, 1, U_PCT, "SPACE"},     {0, 100, 1, U_PCT, "TRAIL"},
  {0, 100, 1, U_PCT, "DIRT"},      {0, 100, 1, U_PCT, "MIX"},       {0, 100, 1, U_PCT, "S>DLY"},
  {0, 100, 1, U_PCT, "S>SPC"},   {0, 100, 1, U_PCT, "D>DLY"},   {0, 100, 1, U_PCT, "D>SPC"},
  {40, 220, 1, U_INT, "TEMPO"},    {0, 75, 1, U_PCT, "SWING"},      {0, 3, 1, U_SCALE, "SCALE"},
  {0, 11, 1, U_NOTE, "ROOT"},      {0, 1, 1, U_SYNC, "SYNC"},       {0, 100, 1, U_CENTER, "INTENS"},
  {0, 100, 1, U_CENTER, "KIT"},    {0, 100, 1, U_PCT, "THROW"},     {0, 100, 1, U_CENTER, "FILTER"},
};
static const int16_t KDEFAULT[NK] = {0,  11, 35, 10, 65, 50, 15, 15, 90, 10, 6,   20, 60, 20, 270, 400, 40, 30,
                                     60, 55, 10, 30, 50, 40, 15, 25, 120, 0, 1,  2,  0,  50, 50, 0,   50};

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
#define NCOND 15
static const char *const COND_NAME[NCOND] = {"ALL", "10%", "30%", "50%", "70%", "90%", "1:2", "2:2",
                                             "1:3", "2:3", "3:3", "1:4", "2:4", "3:4", "4:4"};
static const uint8_t COND_A[NCOND] = {0, 10, 30, 50, 70, 90, 1, 2, 1, 2, 3, 1, 2, 3, 4};
static const uint8_t COND_B[NCOND] = {0, 0, 0, 0, 0, 0, 2, 2, 3, 3, 3, 4, 4, 4, 4};

/* drums */
enum { P_CAR, P_RAT, P_IDX, P_PITCH, P_SWD, P_SWT, P_MDEC, P_MEND, P_ATK, P_REL, P_ACC, P_PROB, P_ECHO, P_EDEC, DP };
enum { F_NONE, F_FRAC, F_DIV10, F_PITCH, F_SEMI, F_MS2, F_MS5, F_PCT, F_MS, F_ACC, F_PCT5, F_INT, F_HZ50, F_NTYPE };
typedef struct {
  uint8_t min, max, kind;
  const char *label;
} pdef_t;
static const pdef_t FM_DEF[DP] = {
  {0, 19, F_FRAC, "CARRIER"}, {0, 19, F_FRAC, "RATIO"},    {0, 100, F_DIV10, "INDEX"}, {0, 24, F_PITCH, "PITCH"},
  {0, 48, F_SEMI, "SWEEP"},   {1, 100, F_MS2, "SW TIME"},  {1, 100, F_MS5, "MOD DEC"}, {0, 100, F_PCT, "MOD END"},
  {1, 150, F_MS, "ATTACK"},   {4, 180, F_MS5, "RELEASE"},  {10, 40, F_ACC, "ACCENT"},  {0, 20, F_PCT5, "PROB"},
  {0, 4, F_INT, "ECHO"},      {0, 18, F_PCT5, "E.DECAY"},
};
static const pdef_t NZ_DEF[DP] = {
  {4, 180, F_HZ50, "FILTER"}, {1, 150, F_DIV10, "RESO"},   {0, 3, F_NTYPE, "TYPE"},    {0, 0, F_NONE, ""},
  {0, 0, F_NONE, ""},         {0, 0, F_NONE, ""},          {0, 0, F_NONE, ""},         {0, 0, F_NONE, ""},
  {1, 100, F_MS, "ATTACK"},   {4, 100, F_MS5, "RELEASE"},  {10, 40, F_ACC, "ACCENT"},  {0, 20, F_PCT5, "PROB"},
  {0, 4, F_INT, "ECHO"},      {0, 18, F_PCT5, "E.DECAY"},
};
static const char *const NTYPE_NAME[4] = {"WHITE", "GB", "METAL", "GRAIN"};
static const char *const TRACK_NAME[TRACKS] = {"KICK", "SNARE", "HAT", "PERC", "CLAP"};
static const uint16_t TRACK_HZ[TRACKS] = {80, 200, 600, 340, 0};
#define IS_NOISE(t) ((t) == 4)
static const pdef_t *defs(int t) { return IS_NOISE(t) ? NZ_DEF : FM_DEF; }
static const uint8_t DDEFAULT[TRACKS][DP] = {
  /* car rat idx pitch swd swt mdec mend atk rel acc prob echo edec */
  {5, 2, 20, 12, 24, 20, 15, 0, 1, 44, 20, 20, 0, 10},
  {5, 13, 50, 12, 7, 10, 20, 20, 1, 28, 20, 20, 0, 10},
  {11, 18, 60, 12, 0, 5, 10, 30, 1, 12, 20, 20, 0, 10},
  {10, 12, 40, 12, 12, 8, 30, 10, 1, 40, 20, 20, 0, 10},
  {80, 10, 0, 0, 0, 1, 1, 0, 2, 24, 20, 20, 0, 10},
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
} bank_t;

#define MAGIC 0x4F440002u /* "OD" + layout 2: the morph is polar */
typedef struct {
  uint32_t magic;
  int16_t k[NK];
  int16_t corner[4][NMORPH];
  uint8_t xy[2], pad[2];
  uint8_t def[TRACKS][DP];
  uint8_t vol[TRACKS];
  bank_t bank[BANKS];
} store_t;
static store_t S;
_Static_assert(sizeof(store_t) <= SETTINGS_MAX, "settings too big");

#define DUR(st) ((st)->dc & 7)
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
}
static void init_all(void) {
  fill(&S, 0, sizeof S);
  S.magic = MAGIC;
  for (int i = 0; i < NK; i++) S.k[i] = KDEFAULT[i];
  for (int c = 0; c < 4; c++)
    for (int i = 0; i < NMORPH; i++) S.corner[c][i] = CORNER_DEFAULT[c][i];
  S.xy[0] = 224, S.xy[1] = 100; /* on the ring at corner A */
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
static volatile uint8_t vcap = 6;
static volatile uint8_t smute, dmute, synth_mute, freeze;
static volatile uint8_t preview_req, sprev;

/* ================================================================ the synth voice */

typedef struct {
  uint8_t active, stage, four, para;
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

static void fire_synth(int l, const sstep_t *st, int32_t velq) {
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
  int note = clampi(lane_note(ln->deg, ln->oct) + pitch, 12, 118);
  v->cinc = note_inc(note);
  v->sinc = note_inc(note - 12);
  uint32_t frac = FRAC_Q16[k[K_RATIO]];
  v->minc = (uint32_t)(((uint64_t)v->cinc * frac) >> 16);
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
    if (v->para) { /* a second carrier two scale steps up, on the same modulator */
      int n2 = clampi(lane_note(ln->deg + 2, ln->oct) + pitch, 12, 118);
      v->cinc2 = note_inc(n2);
      v->dev2 = dev_of(v->cinc2, idx, 500);
    }
  } else { /* op3 -> op2 -> carrier */
    v->dev = dev_of(v->cinc, idx * 18, 1000);
    v->dev3 = dev_of(v->cinc, idx * ratio10 * 108 / 100, 1000);
    v->m3inc = (uint32_t)(((uint64_t)v->minc * 3u) >> 1);
    v->dcoef = coef_slice(nmod * 85 / 100);
    v->d3coef = coef_slice(nmod * 7 / 10);
  }
  v->denv = v->d3env = 1 << 24;
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
  int32_t pan = (l * 32768 / 7 - 16384) * 8 / 10; /* -0.4..0.4 by lane, equal power */
  uint32_t ang = (uint32_t)(pan + 32768) << 14;
  v->gl = (int16_t)sine(ang + 1073741824u);
  v->gr = (int16_t)sine(ang);
  v->send = send * 32767 / 100;
  v->cph2 = v->cph;
  v->active = 1;
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

static void render_synth(int32_t *out, int32_t *sbus, uint32_t n) {
  for (int vi = 0; vi < MAXSV; vi++) {
    svoice_t *v = &SV[vi];
    if (!v->active) continue;
    v->denv = v->mend + (int32_t)(((int64_t)(v->denv - v->mend) * v->dcoef) >> 28);
    int32_t dev = (int32_t)(((int64_t)v->dev * (v->denv >> 9)) >> 15);
    int32_t dev2 = v->para ? (int32_t)(((int64_t)v->dev2 * (v->denv >> 9)) >> 15) : 0;
    int32_t dev3 = 0;
    if (v->four) {
      v->d3env = v->mend + (int32_t)(((int64_t)(v->d3env - v->mend) * v->d3coef) >> 28);
      dev3 = (int32_t)(((int64_t)v->dev3 * (v->d3env >> 9)) >> 15);
    }
    if (v->lfc > v->lbase) {
      v->lfc -= v->lstep;
      if (v->lfc < v->lbase) v->lfc = v->lbase;
      filter_coefs(v);
    }
    int32_t amp = v->amp;
    for (uint32_t f = 0; f < n; f++) {
      if (v->stage == 0) {
        amp += v->ainc;
        if (amp >= v->peak) amp = v->peak, v->stage = 1;
      } else if (v->stage == 1) {
        if (--v->hold <= 0) v->stage = 2;
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
      v->cph += v->cinc + (uint32_t)(int32_t)(((int64_t)m * dev) >> 15);
      int32_t x = sine(v->cph);
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
  uint8_t active, stage, noise, ntype;
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
    v->ntype = p[P_IDX] & 3;
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
}

static void render_drums(int32_t *dbus, uint32_t n) {
  for (int i = 0; i < DV; i++) {
    dvoice_t *v = &DVo[i];
    if (!v->active) continue;
    int32_t midx = 0;
    if (!v->noise) { /* once a slice: the pitch sweep and the index envelope */
      if (v->swd) {
        v->swenv = (int32_t)(((int64_t)v->swenv * v->swcoef) >> 28);
        uint32_t mult = exp2_q16(v->swd * v->swenv / 6 + (15 << 16)); /* 2^(depth x env / 12), Q15 */
        uint64_t c = ((uint64_t)v->cinc * mult) >> 15;
        v->ceff = c > 0x7FFFFFFFu ? 0x7FFFFFFFu : (uint32_t)c;
      }
      v->menv = v->mend + (int32_t)(((int64_t)(v->menv - v->mend) * v->mcoef) >> 28);
      midx = (v->midx * v->menv) >> 15;
    }
    uint32_t minc = v->swd ? (uint32_t)(((uint64_t)v->ceff * v->rat) >> 16) : v->minc; /* the modulator follows */
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
  uint8_t used, kind, who;
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
static void enqueue_synth(int l, const sstep_t *st, int32_t wait) {
  event_t *e = ev_new();
  if (!e) return;
  e->kind = 0, e->who = (uint8_t)l, e->st = *st, e->wait = wait, e->vel = 32767, e->used = 1;
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
  if (!c) return 1;
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

static uint32_t tick, pulse;
static void do_pulse(void) {
  if (pulse % 16 == 0 && queued_bank >= 0) {
    play_bank = cur_bank = (uint8_t)queued_bank;
    queued_bank = -1;
  }
  int32_t wait = (pulse & 1) ? frames_per_step() * S.k[K_SWING] * 2 / 300 : 0;
  const bank_t *b = &S.bank[play_bank];
  for (int l = 0; l < LANES; l++) {
    uint32_t loops;
    int s = lane_pos(&b->lane[l], pulse, &loops);
    spos[l] = (int8_t)s;
    const sstep_t *st = &b->st[l][s];
    if (DUR(st) && !synth_mute && !((smute >> l) & 1) && cond_ok(COND(st), loops)) enqueue_synth(l, st, wait);
  }
  int in = S.k[K_INT];
  for (int t = 0; t < TRACKS; t++) {
    int len = b->dlen[t] ? b->dlen[t] : DSTEPS;
    int s = (int)(pulse % (uint32_t)len);
    dpos[t] = (int8_t)s;
    if (!((b->on[t] >> s) & 1)) { /* intensity above centre adds ghost hats, percs and snares */
      if (in > 50 && t >= 1 && t <= 3 && (int)(random32() % 100u) < (in - 50) * (t == 1 ? 2 : 6) / 10) {
        uint8_t p[DP];
        resolve(b, t, s, p);
        p[P_ECHO] = 0;
        enqueue_drum(t, p, 14746, wait);
      }
      continue;
    }
    if (!cond_ok(b->dcond[t][s], pulse / (uint32_t)len)) continue;
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
static int16_t *rv_ap1 = rvb, *rv_ap2 = rvb + AP1;
static int16_t *rv_line[4];
static int32_t rv_i1, rv_i2, rv_w[4], rv_lp[4], rv_hold, rv_held;

static int32_t lim_env = 26000;

static int32_t dly_read(const int16_t *buf, int32_t t_q8) {
  uint32_t pos = dw - (uint32_t)(t_q8 >> 8);
  int32_t fr = t_q8 & 255;
  int32_t a = buf[pos & DMASK], b = buf[(pos - 1) & DMASK];
  return a + (((b - a) * fr) >> 8);
}

void audio(int32_t *out, uint32_t nframes) {
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
    fire_synth(sprev - 1, &st, 32767);
    sprev = 0;
  }
  int32_t th = k[K_THROW];
  int32_t gsd = clampi(k[K_SSD] + th, 0, 100) * 328, gss = clampi(k[K_SSS] + th, 0, 100) * 328;
  int32_t gdd = clampi(k[K_SDD] + th, 0, 100) * 328, gds = clampi(k[K_SDS] + th, 0, 100) * 328;
  int32_t dfb = k[K_DFB] * 328, dwet = k[K_DWET] * 328;
  int frz = freeze;
  /* space */
  int32_t eff[4];
  for (int i = 0; i < 4; i++) eff[i] = RV_LEN[i] * (40 + k[K_SPACE] * 6 / 10) / 100;
  int32_t rfb = 18000 + k[K_TRAIL] * 128;
  int32_t damp = 24000 - k[K_TRAIL] * 100 - k[K_DIRT] * 60;
  int32_t rmix = k[K_MIX] * 655;
  int dirt = k[K_DIRT], dhold = 1 + dirt / 15;
  int32_t dmask = dirt ? ~((1 << (dirt / 11)) - 1) : ~0;

  for (uint32_t at = 0; at < nframes; at += SUB) {
    uint32_t n = nframes - at < SUB ? nframes - at : SUB;
    while (e < nev && ev[e].at < at + n) on_clock(ev[e++].what);
    for (int i = 0; i < EVQ; i++) {
      event_t *q = &Q[i];
      if (!q->used) continue;
      if (q->wait < (int32_t)n) {
        if (q->kind) fire_drum(q->who, q->p, q->vel);
        else fire_synth(q->who, &q->st, q->vel);
        q->used = 0;
      } else {
        q->wait -= (int32_t)n;
      }
    }
    for (uint32_t f = 0; f < n; f++) sbus[f] = dbus[f] = 0;
    int32_t *o = out + 2 * at;
    render_synth(o, sbus, n);
    render_drums(dbus, n);
    int32_t want_a = frames(k[K_DTA]) * 256, want_b = frames(k[K_DTB]) * 256;
    ta_q8 += (want_a - ta_q8) >> 4;
    tb_q8 += (want_b - tb_q8) >> 4;
    for (uint32_t f = 0; f < n; f++) {
      int32_t sb = clamp16(sbus[f]), db = clamp16(dbus[f]);
      /* twin delay: A leans left, B right, each low-passed in its loop; freeze holds what is in it */
      int32_t dsend = clamp16(((sb * gsd) >> 15) + ((db * gdd) >> 15));
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
      int32_t wl = (((lp_a * 26214 + lp_b * 13107) >> 15) * dwet) >> 15;
      int32_t wr = (((lp_a * 13107 + lp_b * 26214) >> 15) * dwet) >> 15;
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
      int32_t l = (int32_t)(((int64_t)o[2 * f] * 19661) >> 15) + db + wl + rl;
      int32_t r = (int32_t)(((int64_t)o[2 * f + 1] * 19661) >> 15) + db + wr + rr;
      l = (int32_t)(((int64_t)l * 29491) >> 15);
      r = (int32_t)(((int64_t)r * 29491) >> 15);
      int32_t pk = l < 0 ? -l : l, pr = r < 0 ? -r : r;
      if (pr > pk) pk = pr;
      if (pk > lim_env) lim_env = pk;
      else lim_env -= (lim_env >> 13) + 1;
      if (lim_env < 26000) lim_env = 26000;
      int32_t g = (26000 * 32768) / lim_env;
      o[2 * f] = (int32_t)(((int64_t)l * g) >> 15);
      o[2 * f + 1] = (int32_t)(((int64_t)r * g) >> 15);
    }
  }
  uint32_t used = micros() - t0, budget = nframes * 22676u / 1000u;
  load_pct = (uint16_t)((load_pct * 7u + (budget ? used * 100u / budget : 0)) / 8u);
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

enum { V_HOME, V_SYS, V_SYNTH, V_DRUM, V_PAGE, V_PERF };
enum { PG_OSC, PG_TONE, PG_MOD, PG_ENV, PG_DSND, PG_DSWP, PG_DPRF, PG_DENV, PG_DLY, PG_SPC, PG_BUS, PG_GLO, PG_GLO2,
       PG_PERF, NPG };
static const char *const PAGE_NAME[NPG] = {"OSC", "TONE", "MOD", "ENV", "SOUND", "SWEEP", "PERF", "ENV",
                                           "DELAY", "SPACE", "BUS", "GLOBAL", "GLOBAL 2", "PERFORM"};
#define NPGI 9
static const uint8_t SLIST[NPGI] = {PG_OSC, PG_TONE, PG_MOD, PG_ENV, PG_DLY, PG_SPC, PG_BUS, PG_GLO, PG_GLO2};
static const uint8_t DLIST[NPGI] = {PG_DSND, PG_DSWP, PG_DPRF, PG_DENV, PG_DLY, PG_SPC, PG_BUS, PG_GLO, PG_GLO2};
#define X_VOL 200
#define X_LEN 201
#define D(p) (100 + (p))
static const uint8_t PAGE_SLOT[NPG][4] = {
  {K_ALGO, K_RATIO, K_INDEX, K_FB},       {K_CUT, K_CONTOUR, K_RESO, K_DRIVE},  {K_MDEC, K_MEND, 255, 255},
  {K_ATK, K_HOLD, K_REL, K_SUB},          {D(P_CAR), D(P_RAT), D(P_IDX), D(P_PITCH)},
  {D(P_SWD), D(P_SWT), D(P_MDEC), D(P_MEND)}, {D(P_ACC), D(P_PROB), D(P_ECHO), D(P_EDEC)},
  {D(P_ATK), D(P_REL), X_VOL, X_LEN},     {K_DTA, K_DTB, K_DFB, K_DWET},        {K_SPACE, K_TRAIL, K_DIRT, K_MIX},
  {K_SSD, K_SSS, K_SDD, K_SDS},           {K_BPM, K_SWING, K_SCALE, K_ROOT},    {K_SYNC, 255, 255, 255},
  {K_INT, K_KIT, K_THROW, K_FILT},
};
static const uint8_t NOISE_SND[4] = {D(P_IDX), D(P_CAR), D(P_RAT), 255};

static int8_t white_of[KEY_COUNT], black_of[KEY_COUNT];
static int black_key[11];
enum { BK_RND = 8, BK_CLR = 9, BK_F5 = 10, BK_COPY = 5, BK_PASTE = 6, BK_CLRLOCK = 7 };

static uint8_t view = V_HOME, drawn_view = 255, mode, lane, track, spage, rec, playing, pgi;
static uint32_t held;
static uint8_t held_used, seq_used, save_used;
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
    case F_NTYPE: put(val, NTYPE_NAME[t & 3]); break;
    default: put(val, "-");
  }
}

/* ---- the header: mode, page name, a dot per page under that button, bank, CPU, tempo */
static void draw_head(void) {
  char b[24], *d;
  rect(0, 0, SCREEN_W, 18, C_BG);
  capsule(3, 1, 30, 16, 8, mode ? C_CORAL : C_VIO);
  center(3, 30, 1, mode ? "DRM" : "SYN", C_BG, FONT_SMALL);
  const char *nm = view == V_HOME ? "ORBIT" : view == V_SYS ? "SYSTEM" : PAGE_NAME[cur_page()];
  int x = text(38, 1, nm, C_TEXT, FONT_SMALL) + 4;
  if (view >= V_SYNTH && view != V_PERF) {
    int n = pgi <= 1 ? 2 : pgi >= 7 ? 2 : pgi >= 4 ? 3 : 1, at = pgi <= 1 ? pgi : pgi >= 7 ? pgi - 7 : pgi - 4;
    if (n > 1) {
      for (int i = 0; i < n; i++) disk(x + 3 + i * 8, 9, 2, i == at ? C_TEXT : C_DIM);
      x += n * 8 + 4;
    }
  }
  if (rec) disk(x + 4, 9, 3, C_PINK);
  d = b;
  *d++ = (char)('A' + cur_bank);
  if (queued_bank >= 0) *d++ = '>', *d++ = (char)('A' + queued_bank);
  *d++ = ' ';
  d = put_int(d, load_pct);
  d = put(d, "% ");
  if (S.k[K_SYNC]) put(d, "EXT");
  else put_int(d, S.k[K_BPM]);
  text(SCREEN_W - 4 - text_width(b, FONT_SMALL), 1, b, load_pct > 80 ? C_PINK : playing ? C_LIME : C_DIM, FONT_SMALL);
  rect(0, 18, SCREEN_W, 1, C_LINE);
}

/* ================================================================ morph: polar. xy[0] is the angle (256 a turn,
 * 0 at the top), xy[1] the pull (100 on the ring, 0 at the centre). The ring is stretched onto the square of the
 * four corners, so the puck sitting on a corner is exactly that corner's sound. */
static const uint8_t CANG[4] = {224, 32, 160, 96}; /* A top left, B top right, C bottom left, D bottom right */
static void morph_xy(int *mx, int *my) {
  uint32_t ph = (uint32_t)S.xy[0] << 24;
  int32_t sn = sine(ph), cs = sine(ph + 1073741824u);
  int32_t as = sn < 0 ? -sn : sn, ac = cs < 0 ? -cs : cs, m = as > ac ? as : ac;
  if (m < 1) m = 1;
  *mx = 50 + sn * 50 / m * S.xy[1] / 100;
  *my = 50 - cs * 50 / m * S.xy[1] / 100;
}
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

static void draw_ctx(int y) {
  char b[40], *d;
  rect(0, y, SCREEN_W, 17, C_BG);
  if (msg_live()) {
    center(0, SCREEN_W, y, msg, C_LIME, FONT_SMALL);
    return;
  }
  msg[0] = 0;
  const bank_t *bk = CB();
  if (view == V_PERF) {
    d = put(b, "NEAR ");
    *d++ = (char)('A' + nearest_corner());
    d = put(d, "  ORBIT ");
    d = put_int(d, S.xy[0] * 360 / 256);
    d = put(d, "  PULL ");
    put_int(d, S.xy[1]);
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
      d = put(d, LOCKED(st) ? " LOCKED " : " ");
      put(d, COND_NAME[COND(st)]);
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
      d = put(d, any ? " LOCKED " : " ");
      put(d, COND_NAME[bk->dcond[track][s]]);
      center(0, SCREEN_W, y, b, any ? C_PINK : C_MUTE, FONT_SMALL);
    } else {
      d = put(b, TRACK_NAME[track]);
      d = put(d, (dmute >> track) & 1 ? "  MUTED" : "");
      d = put(d, rec ? "  REC: TAP THE TRACK KEY" : "");
      center(0, SCREEN_W, y, b, TRACK_COL[track], FONT_SMALL);
    }
  }
}

/* what KNOB i does right now: its label, value, unit and 0..100 position. 0: nothing */
static int get_cell(int i, const char **label, char *val, const char **unit, int *norm) {
  const bank_t *bk = CB();
  int s = first_held();
  *unit = "", *norm = 0, val[0] = 0;
  if (view < V_SYNTH) return 0;
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
  int id = slot_id(cur_page(), i);
  if (id == 255) return 0;
  if (id < 100) {
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

static void draw_strip(void) { /* a coloured dot and a name per knob */
  rect(0, 221, SCREEN_W, 19, C_BG);
  for (int i = 0; i < 4; i++) {
    const char *l, *u;
    char v[24], b[8];
    int n, j = 0;
    if (!get_cell(i, &l, v, &u, &n)) continue;
    char t[12]; /* six characters fit: drop spaces, then vowels from the right */
    int n2 = 0;
    for (int q = 0; l[q] && n2 < 11; q++)
      if (l[q] != ' ') t[n2++] = l[q];
    for (int q = n2 - 1; q > 0 && n2 > 6; q--)
      if (t[q] == 'A' || t[q] == 'E' || t[q] == 'I' || t[q] == 'O' || t[q] == 'U') {
        for (int r = q; r < n2 - 1; r++) t[r] = t[r + 1];
        n2--;
      }
    for (; j < n2 && j < 6; j++) b[j] = t[j];
    b[j] = 0;
    disk(i * 60 + 5, 231, 3, KC[i]);
    text(i * 60 + 10, 223, b, C_MUTE, FONT_SMALL);
  }
}

/* the value shows large only while its knob turns */
static char pop_l[12], pop_v[24], pop_u[4];
static uint16_t pop_c;
static uint32_t pop_until;
static void show_pop(int i) {
  const char *l, *u;
  int n;
  if (!get_cell(i, &l, pop_v, &u, &n)) return;
  int j = 0;
  for (; l[j] && j < 11; j++) pop_l[j] = l[j];
  pop_l[j] = 0;
  for (j = 0; u[j] && j < 3; j++) pop_u[j] = u[j];
  pop_u[j] = 0;
  pop_c = KC[i];
  pop_until = (millis() + 900) | 1;
  dirty |= D_POP;
}
static int pop_live(void) { return pop_until && (int32_t)(pop_until - millis()) > 0; }
static void draw_pop(void) {
  capsule(132, 22, 104, 52, 8, pop_c);
  capsule(133, 23, 102, 50, 7, C_PANEL);
  center(132, 104, 23, pop_l, C_MUTE, FONT_SMALL);
  int bw = text_width(pop_v, FONT_BIG), uw = text_width(pop_u, FONT_SMALL);
  if (bw + uw <= 98) {
    int x = text(132 + (104 - bw - uw) / 2, 39, pop_v, pop_c, FONT_BIG);
    text(x, 54, pop_u, C_MUTE, FONT_SMALL);
  } else {
    char b[28], *d = put(b, pop_v);
    put(d, pop_u);
    center(132, 104, 47, b, pop_c, FONT_SMALL);
  }
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
  rect(x, 42, 12, 81, C_BG);
  if (s >= len) {
    frame(x, 42, 12, 80, C_PANEL);
    return;
  }
  int ph = dph(track) == s;
  frame(x, 42, 12, 80, ph ? C_TEXT : (held >> s) & 1 ? C_PINK : C_LINE);
  rect(x + 1, 43, 10, 78, C_PANEL);
  if ((b->on[track] >> s) & 1) {
    int h = 78 * acc_at(b, track, s) / 40;
    rect(x + 1, 121 - h, 10, h, ph ? mix565(TRACK_COL[track], C_TEXT, 120) : TRACK_COL[track]);
  }
  int any = 0;
  for (int p = 0; p < DP; p++) any |= b->lock[track][s][p] != UNLOCKED;
  if (any) rect(x + 8, 44, 2, 2, C_PINK);
  if (b->dcond[track][s]) rect(x + 2, 44, 2, 2, C_CYAN);
}
static void draw_ov_cell(int t, int s) {
  if (view != V_DRUM) return;
  const bank_t *b = CB();
  int x = step_x(s), y = 132 + t * 9;
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
static void draw_drum_main(void) {
  rect(0, 19, SCREEN_W, 184, C_BG);
  draw_tabs();
  for (int s = 0; s < DSTEPS; s++) draw_pad(s);
  for (int t = 0; t < TRACKS; t++)
    for (int s = 0; s < DSTEPS; s++) draw_ov_cell(t, s);
  for (int t = 0; t < TRACKS; t++) drawn_d[t] = (int8_t)dph(t);
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
  d = put(b, "SYNTH VOICES ");
  d = put_int(d, vcap);
  put(d, "/10");
  text(12, 58, b, C_DIM, FONT_SMALL);
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
#define PCX 70
#define PCY 88
#define PR 56
#define YS 22
#define XYZ 128
static uint8_t trail[16][2], trail_n;
static void puck_pos(int a, int pull, int *x, int *y) {
  uint32_t ph = (uint32_t)a << 24;
  int r = PR * pull / 100;
  *x = PCX + ((r * sine(ph) + 16384) >> 15);
  *y = PCY - ((r * sine(ph + 1073741824u) + 16384) >> 15);
}
static void draw_ring(void) { /* the morph: PRESETS orbits the puck past the corners, ALGORITHM pulls it in */
  static const char *const CN[4] = {"A", "B", "C", "D"};
  int x, y, nc = nearest_corner();
  rect(0, 19, 142, 136, C_BG);
  arc_at(PCX, PCY, PR, PR + 1, 0, 4096, C_LINE2);
  ring_dots(PCX, PCY, PR / 2, C_LINE);
  for (int i = 0; i < trail_n; i++) {
    puck_pos(trail[i][0], trail[i][1], &x, &y);
    disk(x, y, 1, mix565(C_BG, C_LIME, 40 + i * 120 / 16));
  }
  puck_pos(S.xy[0], S.xy[1], &x, &y);
  line(PCX, PCY, x, y, mix565(C_BG, C_LIME, 90));
  for (int c = 0; c < 4; c++) {
    int cx, cy;
    puck_pos(CANG[c], 100, &cx, &cy);
    disk(cx, cy, 10, CORNER_COL[c]);
    disk(cx, cy, 8, c == nc ? CORNER_COL[c] : C_BG);
    center(cx - 8, 17, cy - 8, CN[c], c == nc ? C_BG : CORNER_COL[c], FONT_SMALL);
  }
  disk(x, y, 6, mix565(C_BG, C_LIME, 80));
  disk(x, y, 4, mix565(C_BG, C_LIME, 160));
  disk(x, y, 2, C_TEXT);
}
static void draw_pills(void) {
  static const uint8_t PK[4] = {K_INT, K_KIT, K_THROW, K_FILT};
#define PC KC
  for (int k = 0; k < 4; k++) {
    int x = 146 + k * 23, v = S.k[PK[k]], top = 128 - 128 * v / 100;
    rect(x, YS, 16, XYZ, C_BG);
    for (int j = 0; j < XYZ; j++) {
      int in = inset(XYZ, 8, j);
      rect(x + in, YS + j, 16 - 2 * in, 1, j >= top ? PC[k] : C_PANEL2);
    }
    if (KD[PK[k]].unit == U_CENTER) rect(x - 2, YS + 64, 2, 1, C_DIM);
  }
}
static void draw_mutes(void) {
  static const char *const MN[8] = {"KCK", "SNR", "HAT", "PRC", "CLP", "SYN", "FRZ", "ALL"};
  for (int i = 0; i < 8; i++) {
    int x = i * 30, on;
    rect(x, 156, 30, 17, C_BG);
    if (i < 5) on = !((dmute >> i) & 1);
    else if (i == 5) on = !synth_mute;
    else on = i == 6 ? freeze : 0;
    if (i == 6 && freeze) capsule(x + 1, 156, 28, 17, 8, C_MINT);
    uint16_t c = i == 6 ? (freeze ? C_BG : C_DIM) : i == 7 ? C_DIM : on ? (i < 5 ? TRACK_COL[i] : C_VIO) : C_PINK;
    center(x, 30, 156, MN[i], c, FONT_SMALL);
    if (i < 6 && !on) rect(x + 4, 164, 22, 1, C_PINK);
  }
}

/* ================================================================ pictures: one drawing a page */
static void pic_pills(void) { /* any page without its own picture: four meters in the knob colours */
  for (int i = 0; i < 4; i++) {
    const char *l, *u;
    char v[28];
    int n;
    if (!get_cell(i, &l, v, &u, &n)) continue;
    char b[32], *d = put(b, v);
    put(d, u);
    center(i * 60, 60, 20, b, KC[i], FONT_SMALL);
    int x = i * 60 + 14, top = 158 - 158 * clampi(n, 0, 100) / 100;
    for (int j = 0; j < 158; j++) {
      int in = inset(158, 8, j);
      rect(x + in, 42 + j, 32 - 2 * in, 1, j >= top ? KC[i] : C_PANEL2);
    }
  }
}

static void pic_osc(void) { /* the modulator orbits the carrier: orbit = ratio, beam = index, loop = feedback */
  static const char *const AN[3] = {"2", "4", "P"};
  const int16_t *k = S.k;
  int cx = 100, cy = 114, orb = 34 + k[K_RATIO] * 2;
  for (int i = 0; i < 3; i++) {
    int sel = k[K_ALGO] == i;
    capsule(8, 26 + i * 22, 20, 18, 5, sel ? KC[0] : C_PANEL);
    center(8, 20, 27 + i * 22, AN[i], sel ? C_BG : C_DIM, FONT_SMALL);
  }
  ring_dots(cx, cy, orb, KC[1]);
  uint32_t ph = 683u << 20; /* the moon sits at 60 degrees */
  int mx = cx + ((orb * sine(ph)) >> 15), my = cy - ((orb * sine(ph + 1073741824u)) >> 15);
  int th = k[K_INDEX] / 20;
  for (int j = -th; j <= th; j++) line(cx + j / 2, cy + j, mx + j / 2, my + j, KC[2]);
  if (k[K_ALGO] == 2) { /* parallel: a second carrier on the same modulator */
    for (int j = -th; j <= th; j++) line(cx - 40 + j, cy + 44, mx + j, my, mix565(C_BG, KC[2], 150));
    disk(cx - 40, cy + 44, 15, C_TEXT);
    disk(cx - 40, cy + 44, 13, C_PANEL2);
  }
  disk(cx, cy, 22, C_TEXT);
  disk(cx, cy, 20, C_PANEL2);
  center(cx - 20, 40, cy - 8, "CAR", C_MUTE, FONT_SMALL);
  if (k[K_ALGO] == 1) { /* 4-op: a third operator orbits the moon */
    uint32_t p3 = 3300u << 20;
    int x3 = mx + ((30 * sine(p3)) >> 15), y3 = my - ((30 * sine(p3 + 1073741824u)) >> 15);
    line(mx, my, x3, y3, KC[2]);
    disk(x3, y3, 7, KC[1]);
    disk(x3, y3, 5, C_PANEL2);
  }
  if (k[K_FB]) arc_at(mx, my, 16, 18, 0, k[K_FB] * 4096 / 100, KC[3]);
  disk(mx, my, 12, KC[1]);
  disk(mx, my, 10, C_PANEL2);
  text(8, 184, FRAC_NAME[k[K_RATIO]], KC[1], FONT_SMALL);
}

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
static int tone_y(int32_t lf, int32_t lfc, int reso) { /* the low-pass's level at lf, as a screen row */
  int32_t a = lf - lfc + (8 << 16);
  uint32_t w = a < 0 ? 0 : a >= (12 << 16) ? 4096u : exp2_q16(a); /* f / fc, Q8 */
  if (w > 4096u) w = 4096u;
  int32_t re = 256 - (int32_t)((w * w) >> 8);
  uint32_t im = w * 10u / (uint32_t)reso;
  uint32_t den = isqrt64((uint64_t)((int64_t)re * re) + (uint64_t)im * im);
  uint32_t m = 65536u / (den ? den : 1u); /* Q8 */
  int l = 0;
  if (m < 1u) m = 1u;
  while (m >= 512u) m >>= 1, l += 16;
  while (m < 256u) m <<= 1, l -= 16;
  l += (int)(m - 256u) * 16 / 256;
  return clampi(120 - l * 14 / 16, 44, 198);
}
static void pic_tone(void) { /* the filter: cutoff line, contour start in dashes, resonance dot, drive band */
  const int16_t *k = S.k;
  int32_t lbase = clampi(LOG2_250 + k[K_CUT] * 3277 + (k[K_FILT] - 50) * 3277, LOG2_60, LOG2_12000);
  int32_t lpeak = clampi(lbase + LOG2_1P7C[k[K_CONTOUR]], LOG2_60, LOG2_12000), span = LOG2_12000 - LOG2_60;
  int h = 2 + k[K_DRIVE] * 16 / 100, py = 0, pq = 0;
  uint16_t fillc = mix565(C_BG, KC[0], 36);
  rect(8, 22, 224, h, mix565(C_BG, KC[3], 70));
  for (int x = 8; x < 232; x += 4) rect(x, 22 + h, 2, 1, KC[3]);
  for (int x = 0; x < 224; x++) {
    int32_t lf = LOG2_60 + (span / 223) * x;
    int y = tone_y(lf, lbase, k[K_RESO]), q = tone_y(lf, lpeak, k[K_RESO]);
    rect(8 + x, y, 1, 199 - y, fillc);
    if (x) {
      line(7 + x, py, 8 + x, y, KC[0]);
      line(7 + x, py + 1, 8 + x, y + 1, KC[0]);
      if ((x & 7) < 4) line(7 + x, pq, 8 + x, q, KC[1]);
    }
    py = y, pq = q;
  }
  int xc = 8 + (lbase - LOG2_60) / (span / 223), yc = tone_y(lbase, lbase, k[K_RESO]);
  rect(xc, yc, 1, 199 - yc, KC[0]);
  disk(xc, yc, 4, KC[2]);
}

static void pic_env(void) { /* attack, hold and release in their knob colours; the sub level beneath */
  const int16_t *k = S.k;
  int32_t atk = k[K_ATK], hold = k[K_HOLD], rel = k[K_REL] * 10, total = atk + hold + rel;
  int yb = 160, yt = 40, xa = 10 + clampi(220 * atk / total, 3, 120), xh = clampi(xa + 220 * hold / total, xa, 200);
  int n = 230 - xh, py = yb;
  int32_t a = 32768, fr = clampi(226000 / (n > 0 ? n : 1), 0, 32768);
  uint16_t fillc = mix565(C_BG, C_VIO, 50);
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
  capsule(10, 178, 220, 10, 5, C_PANEL2);
  if (k[K_SUB]) capsule(10, 178, clampi(220 * k[K_SUB] / 100, 10, 220), 10, 5, KC[3]);
}

static void pic_drum(void) { /* the hit itself: wave = carrier, tag = ratio, ticks = index, lower line = pitch */
  const uint8_t *p = S.def[track];
  if (IS_NOISE(track)) {
    uint32_t r = 12345u + p[P_IDX] * 977u;
    int hmax = 40 + clampi(p[P_RAT], 0, 100);
    center(0, SCREEN_W, 24, NTYPE_NAME[p[P_IDX] & 3], KC[0], FONT_SMALL);
    for (int i = 0; i < 56; i++) {
      r = r * 1103515245u + 12345u;
      int v = (int)((r >> 16) % 100u), h;
      switch (p[P_IDX] & 3) {
        case 1: h = v & 1 ? 90 : 30; break;
        case 2: h = i % 3 == 0 ? 85 : 12; break;
        case 3: h = i % 4 == 0 ? v + 5 : 3; break;
        default: h = v + 5;
      }
      h = h * hmax / 140;
      rect(8 + i * 4, 196 - h, 2, h, i < p[P_CAR] * 56 / 180 ? KC[1] : mix565(C_BG, KC[1], 90));
    }
    rect(8 + p[P_CAR] * 224 / 180, 60, 1, 136, KC[2]);
    return;
  }
  capsule(8, 24, 44, 18, 9, KC[1]);
  capsule(9, 25, 42, 16, 8, C_BG);
  center(8, 44, 25, FRAC_NAME[p[P_RAT]], KC[1], FONT_SMALL);
  for (int i = 0; i < 12; i++) rect(60 + i * 6, 27, 4, 12, i * 100 < p[P_IDX] * 12 ? KC[2] : C_PANEL2);
  int32_t atk = p[P_ATK], rel = kit_scale(p[P_REL] * 5), total = atk + rel;
  int32_t fa = clampi(6907 * total / 224 * 33 / (rel ? rel : 1), 0, 32768);
  int32_t fs = clampi(6907 * total / 224 * 33 / (p[P_SWT] * 2), 0, 32768);
  int32_t fm = clampi(6907 * total / 224 * 33 / (p[P_MDEC] * 5), 0, 32768);
  int32_t a = 32768, sw = 32767, me = 32767, mend = p[P_MEND] * 327;
  uint32_t inc = (uint32_t)(((uint64_t)76695844u * FRAC_Q16[p[P_CAR]]) >> 16); /* four cycles across at ratio 1 */
  inc = (uint32_t)(((uint64_t)inc * (uint32_t)SEMI[p[P_PITCH]]) >> 16);
  if (inc > 600000000u) inc = 600000000u;
  uint32_t pc = 0, pm = 0;
  int py = 104, pp = 0;
  rect(8, 104, 224, 1, C_LINE);
  for (int x = 0; x < 224; x++) {
    int32_t t = total * x / 224, amp;
    if (t < atk) amp = 32768 * t / (atk ? atk : 1);
    else a -= (a * fa) >> 15, amp = a;
    sw -= (sw * fs) >> 15;
    me = mend + (me - mend) - (((me - mend) * fm) >> 15);
    uint32_t mult = p[P_SWD] ? exp2_q16(p[P_SWD] * sw / 6 + (15 << 16)) : 32768u;
    uint32_t ci = (uint32_t)(((uint64_t)inc * mult) >> 15);
    int32_t midx = (kit_scale(p[P_IDX]) * me) >> 15;
    int32_t sv = sine(pc + (uint32_t)(sine(pm) * midx) * 2086u);
    pc += ci;
    pm += (uint32_t)(((uint64_t)ci * FRAC_Q16[p[P_RAT]]) >> 16);
    int y = 104 - ((((sv * (amp >> 1)) >> 14) * 44) >> 15);
    int q = 196 - p[P_PITCH] - ((p[P_SWD] * sw) >> 15);
    if (x) {
      line(7 + x, py, 8 + x, y, KC[0]);
      line(7 + x, pp, 8 + x, q, KC[3]);
      line(7 + x, pp + 1, 8 + x, q + 1, KC[3]);
    }
    py = y, pp = q;
  }
}

static void draw_pic(void) {
  rect(0, 19, SCREEN_W, 184, C_BG);
  switch (cur_page()) {
    case PG_OSC: pic_osc(); break;
    case PG_TONE: pic_tone(); break;
    case PG_ENV: pic_env(); break;
    case PG_DSND: pic_drum(); break;
    default: pic_pills();
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

static void edit_k(int id, int d) {
  if (id == 255 || !d) return;
  int u = KD[id].unit, dd = (u == U_ALGO || u == U_SCALE || u == U_NOTE || u == U_SYNC || u == U_FRAC) ? sgn(d) : d;
  S.k[id] = (int16_t)clampi(S.k[id] + dd * KD[id].step, KD[id].min, KD[id].max);
  if (id == K_BPM) clock_tempo((uint32_t)S.k[K_BPM] * 100u), dirty |= D_HEAD;
  if (id == K_SYNC) clock_follow(S.k[K_SYNC] ? MIDI_ANY : 0), dirty |= D_HEAD;
  if (id == K_SCALE || id == K_ROOT) dirty |= D_CTX;
  if (id >= K_INT && view == V_PERF) dirty |= D_MAIN;
}
static void edit_slot(int id, int d) {
  bank_t *b = CB();
  if (id == 255) return;
  if (id < 100) {
    edit_k(id, d);
  } else if (id == X_VOL) {
    S.vol[track] = (uint8_t)clampi(S.vol[track] + d, 0, 100);
  } else if (id == X_LEN) {
    b->dlen[track] = (uint8_t)clampi(b->dlen[track] + sgn(d), 1, DSTEPS);
    dirty |= D_MAIN;
  } else {
    int p = id - 100;
    const pdef_t *pd = &defs(track)[p];
    if (pd->kind == F_NONE) return;
    int dd = (pd->kind == F_FRAC || pd->kind == F_INT || pd->kind == F_NTYPE || pd->kind == F_PITCH) ? sgn(d) : d;
    S.def[track][p] = (uint8_t)clampi(S.def[track][p] + dd, pd->min, pd->max);
    if (p == P_ACC) dirty |= D_MAIN;
  }
  dirty |= D_VALS | D_GRAPH;
}
static void edit_slock(int k, int d) {
  bank_t *b = CB();
  for (int s = 0; s < MAXSTEP; s++) {
    if (!((held >> s) & 1)) continue;
    sstep_t *st = &b->st[lane][s];
    if (!DUR(st)) continue;
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
  int dd = (pd->kind == F_FRAC || pd->kind == F_INT || pd->kind == F_NTYPE || pd->kind == F_PITCH) ? sgn(d) : d;
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
      b->dcond[track][s] = (uint8_t)((b->dcond[track][s] + sgn(d) + NCOND) % NCOND);
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
        S.xy[0] = CANG[bk];
        S.xy[1] = 100;
        apply_morph();
      }
      dirty |= D_MAIN | D_CTX | D_VALS;
    } else if (bk == BK_RND) {
      S.k[K_INT] = 50, dirty |= D_MAIN | D_VALS;
    } else if (bk == BK_CLR) {
      S.xy[1] = 0, apply_morph(), dirty |= D_MAIN | D_CTX;
    }
    return;
  }
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

static void white_down(int w) {
  if (button_held(BTN_SEQ)) {
    if (w < BANKS) set_bank(w);
    seq_used = 1;
    return;
  }
  if (view == V_PERF) {
    if (w >= 8 && w <= 12) dmute ^= (uint8_t)(1u << (w - 8));
    else if (w == 13) synth_mute ^= 1;
    else if (w == 14) freeze = 1;
    else if (w == 15) dmute = 0, smute = 0, synth_mute = 0, say("ALL ON");
    else say("SNAPSHOTS: NEXT VERSION");
    dirty |= D_MAIN;
    return;
  }
  int s = mode ? w : w + (spage ? 16 : 0);
  if (!held) held_used = 0;
  held |= 1u << s;
  if (view == V_SYNTH && !mode) redraw_sstep(lane, s);
  if (view == V_DRUM && mode) draw_pad(s);
  dirty |= D_CTX | D_VALS;
}
static void white_up(int w) {
  if (view == V_PERF) {
    if (w == 14) freeze = 0, dirty |= D_MAIN;
    return;
  }
  int s = mode ? w : w + (spage ? 16 : 0);
  if (!((held >> s) & 1)) return;
  held &= ~(1u << s);
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
  int off = AP1 + AP2;
  for (int i = 0; i < 4; i++) rv_line[i] = rvb + off, off += RV_LEN[i];
  if (settings_load(&S, sizeof S) != 0 || S.magic != MAGIC) init_all();
  clear(C_BG);
  clock_tempo((uint32_t)S.k[K_BPM] * 100u);
  clock_send(1);
  clock_follow(S.k[K_SYNC] ? MIDI_ANY : 0);
  led_backlight(1);
  print("ORBIT DRIFT ready");
}

static void draw_all(void) {
  uint8_t d = dirty;
  dirty = 0;
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
        rect(0, 19, SCREEN_W, 184, C_BG);
        for (int l = 0; l < LANES; l++) draw_lane(l);
      }
      break;
    case V_DRUM:
      if (d & D_MAIN) draw_drum_main();
      break;
    case V_PAGE:
      if (d & (D_MAIN | D_VALS)) draw_pic();
      break;
    case V_PERF:
      if (d & D_MAIN) {
        rect(0, 19, SCREEN_W, 184, C_BG);
        draw_ring(), draw_pills(), draw_mutes();
      }
      break;
  }
  if (view >= V_SYNTH) {
    if (d & D_CTX) draw_ctx(204);
    if (d & D_VALS) draw_strip();
    if (pop_live() && (d & (D_MAIN | D_VALS | D_POP))) draw_pop();
  }
}

static void update_playheads(void) {
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
  for (int k = 0; k < KEY_COUNT; k++) {
    if (key_pressed(k)) {
      if (white_of[k] >= 0) white_down(white_of[k]);
      else black_down(black_of[k]);
    }
    if (key_released(k)) {
      if (white_of[k] >= 0) {
        white_up(white_of[k]);
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
  if (button_pressed(BTN_SEQ)) seq_used = 0;
  if (button_released(BTN_SEQ) && !seq_used) { /* to the steps; from the steps, the other instrument */
    if (view == V_SYNTH || view == V_DRUM) mode = !mode;
    held = 0;
    inst_view();
    dirty = D_ALL;
  }
  if (button_pressed(BTN_SAVE)) save_used = 0;
  if (button_released(BTN_SAVE) && !save_used) {
    if (playing) say("STOP TO SAVE");
    else say(settings_save(&S, sizeof S) == 0 ? "SAVED" : "SAVE FAILED");
  }
  if (button_pressed(BTN_HOME)) set_view(view == V_HOME ? V_SYS : V_HOME);
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
  if (button_pressed(BTN_SEL)) page_button(2, -1);
  if (button_pressed(BTN_ENV)) page_button(3, -1);
  if (button_pressed(BTN_FX)) {
    int fx = view == V_PAGE && pgi >= 4 && pgi <= 6;
    set_pgi(fx ? (pgi == 6 ? 4 : pgi + 1) : 4);
  }
  if (button_pressed(BTN_GLO)) page_button(7, 8);
  if (button_pressed(BTN_LFO)) say("LFO: NEXT VERSION");
  if (button_pressed(BTN_REC)) rec = !rec, dirty |= D_HEAD | D_CTX;
  if (button_pressed(BTN_OCT_DOWN) || button_pressed(BTN_OCT_UP)) {
    int up = button_pressed(BTN_OCT_UP);
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
    S.xy[0] = (uint8_t)(S.xy[0] + ey * 2);                   /* PRESETS orbits */
    S.xy[1] = (uint8_t)clampi(S.xy[1] + ex * 2, 0, 100);     /* ALGORITHM pulls in and out */
    apply_morph();
    if (view == V_PERF || view == V_PAGE) dirty |= D_MAIN | D_CTX;
    if (view != V_PERF && now - puck_say > 120) {
      char b[28], *d = put(b, "ORBIT ");
      d = put_int(d, S.xy[0] * 360 / 256);
      d = put(d, "  PULL ");
      put_int(d, S.xy[1]);
      say(b);
      puck_say = now;
    }
    dirty |= D_VALS;
  }
  int sel = encoder(ENC_SELECT);
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
    if (view == V_HOME || view == V_SYS) {
      say("SEQ: STEPS  EDIT: SOUND");
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
      if (!mode) edit_slock(k, d);
      else edit_dlock(slot_id(cur_page(), k), d);
    } else {
      edit_slot(slot_id(cur_page(), k), d);
    }
    show_pop(k);
    if (view == V_PAGE) dirty |= D_MAIN;
  }
  if (pop_until && !pop_live()) pop_until = 0, dirty |= D_MAIN; /* the number steps back */
  if (S.k[K_SYNC] && playing != running) playing = running, dirty |= D_HEAD;

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
  static uint32_t pop_pulse;
  if (pop_live() && (view == V_SYNTH || view == V_DRUM) && pulses != pop_pulse) pop_pulse = pulses, draw_pop();

  /* LEDs */
  const bank_t *b = CB();
  for (int k = 0; k < KEY_COUNT; k++) {
    int on = 0, w = white_of[k], bk = black_of[k];
    if (view == V_PERF) {
      if (w >= 8 && w <= 12) on = !((dmute >> (w - 8)) & 1);
      else if (w == 13) on = !synth_mute;
      else if (w == 14) on = freeze;
      else if (bk >= 0 && bk < 4) on = bk == nearest_corner();
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
  led_button(BTN_HOME, view == V_HOME || view == V_SYS);
  led_button(BTN_ARP, view == V_PERF);
  int inst = view == V_PAGE;
  led_button(BTN_EDIT, inst && pgi <= 1);
  led_button(BTN_SEL, inst && pgi == 2);
  led_button(BTN_ENV, inst && pgi == 3);
  led_button(BTN_FX, inst && pg >= PG_DLY && pg <= PG_BUS);
  led_button(BTN_GLO, inst && (pg == PG_GLO || pg == PG_GLO2));
  led_button(BTN_OCT_DOWN, !mode && !spage);
  led_button(BTN_OCT_UP, !mode && spage);
}
