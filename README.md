# ORBIT DRIFT

A polymetric FM synth and an FM drum machine on one clock for the M-VAVE FM-1, sharing a twin delay and a space
reverb. Built from three HTML toys: ORBITAL (eight lanes, the FM voice and contour filter), Scenedrift (locks, trig
conditions, the XY morph, the twin delay) and FM-16 (the drums and echoes), with pitch sweeps, mod envelopes,
fractional ratios and noise types added from Fors FMS. Made with [foma](https://github.com/konsumer/foma).

**Try it in the browser, and install it on the FM-1 from Chrome or Edge:** <https://sunjaculley.github.io/orbit-drift-fm1/>

Version 0.7 adds the song chain, snapshots, morph path recording, the transpose sequence, the echo field, unison,
a second lock set, trigless steps, playing the synth from the keys, FILL, knob recording and restore at power-on.
Version 0.8 adds MIDI note input, held notes in KEYS, drum trigless steps, knob recording on the lanes, saved morph
paths, song mutes per block and undo for sound and song edits. Version 0.9 adds the MIX page. Saves from 0.7 and
0.8 carry over.

## Playing it

Each page is one picture, with the steps of the lane or track under it and the four knobs' names and values along
the bottom. KNOB1-4 are cyan, lime, white and orange on every page: whatever a knob moves in the picture wears its
colour. A pink ring around a knob's dot means the LFO is moving that setting; an amber ring, the step mod.

| Control | What it does |
| --- | --- |
| SEQ | The steps (lanes or drum pads). From the steps: switch SYNTH / DRUMS. Hold SEQ + white key 1-4: bank A-D (while playing, at the next bar) |
| EDIT · SEL · ENV · FX · GLO | The sound, as pictures. Synth: OSC / TONE · MOD / INDEX / UNISON · ENV. Drums: SOUND / SWEEP · PERF · ENV. Both: DELAY / SPACE / BUS / MIX / STEP MOD · GLOBAL / TRANSPOSE. Press a button again for its next page. The white keys still edit steps here: their lights show the pattern |
| LFO | The LFO: one wave (sine, triangle, square, sample and hold; 0.05-20 Hz, or synced from 4 bars to 1/32) to the synth's FILTER, INDEX, PITCH, LEVEL or PAN, the DELAY time, or the drums' pitch or level. STEP MOD (FX, fourth page) does the same from a drum track: each hit pushes the destination down and lets it recover, e.g. the kick ducking the synth |
| SELECT | Pick the lane or the track (lanes sound their note). With steps held: trig condition |
| ALGORITHM / PRESETS | The morph, on every page: X and Y of a puck that blends the four sounds in corners A-D. Turn a sound knob with the puck on a corner and that corner changes; turn it anywhere else and all four shift together, so no edit is lost |
| HOME | The orbit: every lane as a ring, the drums as the outer halo, and when the lanes next line up. Again: SONG. Again: SYSTEM (CPU, voices, load history) |
| ARP | PERFORM |
| Hold FX | Effects on the white keys while held: 1 freeze, 2 echo throw, 3 space throw, 4 filter low, 5 filter high, 6 thin, 7 busy, 8 repeat, 9 FILL (steps with the FILL condition play); 13-16 jump the puck to corners A-D. Black keys mute lanes (synth) or tracks (drums). KNOB1-4: the Perform macros |
| Hold SEQ | 1-4 bank A-D (at the next bar while playing); 5 clear, 6 reverse, 7 / 8 shift, 9 new rhythm, 10 cook (a small change each press), 11 double, 12 half, 13 copy, 14 paste. KNOB1 length, KNOB2 hits (spread evenly: Euclidean), KNOB3 rotate, KNOB4 swing |
| Hold GLO | 1 tap tempo, 2 restart from the top, 3 MIDI clock sync on/off, 4 KEYS (white keys play the synth in the scale, held as long as the key is; with REC while playing, a note lands on the lane of that degree), 16 everything back on; black keys solo. SELECT and KNOB1 tempo, KNOB2 swing, KNOB3 scale, KNOB4 root |
| Hold LFO | Turn any knob: the change springs back when LFO is let go |
| Hold SAVE | Undo the last edit: a pattern edit or tool, or a sound or song edit (a knob gesture counts as one). Again: redo |
| REC with steps held | Trigless, drawn hollow. Synth: the step moves the sounding note without a new one. Drums: the step changes the ringing hit (its pitch, ratio, index or filter) |
| SEL with steps held | Lock set B. Synth: WAIT (microtiming in sixths of a step), ECHOES, LADDER (same, up, down, random), FADE. Drums: WAIT, ECHO, ECHO DECAY, PROBABILITY |
| REC while playing | On the lanes, KNOB1-4 write PITCH, VEL, INDEX and SEND on the step under the playhead. On a sound page, a drum knob (or the synth's INDEX) does the same |
| MIDI in (USB, the 3.5 mm jack, Bluetooth) | Notes play the selected lane's sound, held until note off. Channel 10 plays the drums (GM: 36 kick, 38 snare, 42 hat, 39 clap, toms perc) |
| PLAY · REC · SAVE | Start, stop · live record (drums: tap a track key while playing) · store in flash (when stopped). SAVE + D#5: INIT |

Holding FX, SEQ or GLO shows a map of the keys and what each does, and the knob strip shows the layer's knobs. A quick tap is still the page button. Hold the layer's button and press OCT+ to keep the layer open; tap the button to close it.

Changes are saved by themselves when the FM-1 is stopped and left alone for 10 s (at most once a minute), and come back at power-on.

**Synth.** White keys are the steps of the selected lane: tap cycles the note length (short, medium, long, longest,
off). OCT- / OCT+ reach steps 1-16 / 17-32. Hold steps and turn KNOB1-4 to lock PITCH, VELOCITY, INDEX and SEND;
SELECT sets the trig condition; EDIT turns the locks on or off. The first eight black keys pick a lane; hold one
and KNOB1-4 set its LENGTH (1-32), DIRECTION (forward, back, ping-pong, random), DEGREE and OCTAVE. C#5 randomises
the lane (hold 1 s: all lanes), D#5 clears it (with steps held: their locks).

**Drums.** White keys are the steps of the selected track: tap on or off. Hold steps and turn a knob to lock that
page's value to them; SELECT sets the trig condition. F#3-D#4 pick KICK, SNARE, HAT, PERC, CLAP and play them;
hold one and KNOB1 sets its length. F#4 COPY, G#4 PASTE, A#4 CLEAR LOCKS, C#5 RANDOM, D#5 CLEAR. OCT- / OCT+ step
through the tracks. Hold F#5 and tap a lane or track key to mute it.

**Perform.** KNOB1-4: drum INTENSITY (below centre thins the beat, above adds ghost hits), KIT (shorter / longer,
softer / brighter), FX THROW (more of everything into the delay and the space) and FILTER. F#3-C#4 jump the puck
to corners A-D; SAVE + one copies the sound you hear into that corner; D#5 centres the puck. White keys 9-13 mute the drum tracks, 14 the synth, 15 freezes
the delay while held, 16 turns everything back on. White keys 1-8 are snapshots (puck, macros, mutes): SAVE + key stores, hold a key to
play it, let go to return. REC records the puck's path (up to 8 bars) while playing; REC again plays it back in a
loop (moving the puck pauses it, REC resumes); hold REC 1 s to clear. The path is saved with everything else.

**Mix.** FX four times. KNOB1 the synth's level, KNOB2 the drums', KNOB3 the selected lane's level (or, from the
drums, the selected drum's volume; SELECT picks), KNOB4 the master. Levels read in dB (0.0 is where they start).
Meters beside the faders show what plays; LIMIT lights when the output limiter is pulling the level down, a sign
to lower the master or the louder side.

**Song.** HOME twice. White keys 1-4 add a block of bank A-D; KNOB1 picks a block, KNOB2 its bank, KNOB3 its
repeats in bars, KNOB4 turns the song on. Black keys F#3-D#4 mute KICK-CLAP in the block and F#4 the synth (the
coloured marks on each block show what plays). D#5 deletes the block. With the song on, PLAY runs the chain in a loop.

**Index, unison, echo field, transpose.** SEL again: INDEX (key tracking, mod envelope loop, the third operator's
depth, drift) and UNISON (voices 1-3, detune, spread, mix). DELAY MODE FIELD swaps the twin delay for four taps:
SIZE, GRAVITY (how long repeats last) and TRAIL. TRANSPOSE (GLO twice): up to 8 stops that shift every lane by
scale steps every 1-8 bars; KNOB3 picks a stop, KNOB4 its shift.

## For the FM-1

- **CPU.** Up to 6 drum voices and 4-10 synth voices. The synth starts at 8; the meter in the header (and on the
  SYSTEM page) adds two when the chip has room for 3 s and takes two away when it runs over 80%.
- **Memory.** The twin delay uses 128 KiB and the space about 31 KiB of the FM-1's buffer pool; four banks of
  patterns and the sounds take 10.3 of the 12 KB of settings.

## Develop

Only Docker is needed:

```sh
./fm1 dev          # http://localhost:8000: the emulator, the installer, the rescue; rebuilds when you save
./fm1 build        # build/orbit-drift-VERSION.fwsc
```

The firmware is in `src/main.c`; what it can call is in [platform/fm1.h](platform/fm1.h). Every push to `main`
publishes the emulator and the installer to GitHub Pages; a tag (`v0.2`) also makes a release with the `.fwsc`.

Installing firmware is at your own risk; back up your FM-1 first. If it does not start, hold OCT- while switching it
on (the USB rescue), then install from the page.

## License

GPL-3.0-only. Credits and third-party parts: [LICENSING.md](LICENSING.md).
