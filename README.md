# ORBIT DRIFT

A polymetric FM synth and an FM drum machine on one clock for the M-VAVE FM-1, sharing a twin delay and a space
reverb. Built from three HTML toys: ORBITAL (eight lanes, the FM voice and contour filter), Scenedrift (locks, trig
conditions, the XY morph, the twin delay) and FM-16 (the drums and echoes), with pitch sweeps, mod envelopes,
fractional ratios and noise types added from Fors FMS. Made with [foma](https://github.com/konsumer/foma).

**Try it in the browser, and install it on the FM-1 from Chrome or Edge:** <https://sunjaculley.github.io/orbit-drift-fm1/>

This is the first pass: the core of the plan, playable end to end. Song chain, snapshots, morph path recording,
the transpose sequence, the four-tap echo field and unison come next.

## Playing it

Each page is one picture, with the steps of the lane or track under it and the four knobs' names and values along
the bottom. KNOB1-4 are cyan, lime, white and orange on every page: whatever a knob moves in the picture wears its
colour. A pink ring around a knob's dot means the LFO is moving that setting; an amber ring, the step mod.

| Control | What it does |
| --- | --- |
| SEQ | The steps (lanes or drum pads). From the steps: switch SYNTH / DRUMS. Hold SEQ + white key 1-4: bank A-D (while playing, at the next bar) |
| EDIT · SEL · ENV · FX · GLO | The sound, as pictures. Synth: OSC / TONE · MOD · ENV. Drums: SOUND / SWEEP · PERF · ENV. Both: DELAY / SPACE / BUS / STEP MOD · GLOBAL / GLOBAL 2. Press a button again for its next page. The white keys still edit steps here: their lights show the pattern |
| LFO | The LFO: one wave (sine, triangle, square, sample and hold; 0.05-20 Hz) to the synth's FILTER, INDEX, PITCH, LEVEL or PAN, or the DELAY time. STEP MOD (FX, fourth page) does the same from a drum track: each hit pushes the destination down and lets it recover, e.g. the kick ducking the synth |
| SELECT | Pick the lane or the track (lanes sound their note). With steps held: trig condition |
| ALGORITHM / PRESETS | The morph, on every page: X and Y of a puck that blends the four sounds in corners A-D. Turn a sound knob with the puck on a corner and that corner changes; turn it anywhere else and all four shift together, so no edit is lost |
| HOME | The orbit: every lane as a ring, the drums as the outer halo, and when the lanes next line up. Again: SYSTEM (CPU, voices, load history) |
| ARP | PERFORM |
| PLAY · REC · SAVE | Start, stop · live record (drums: tap a track key while playing) · store in flash (when stopped). SAVE + D#5: INIT |

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
the delay while held, 16 turns everything back on.

## For the FM-1

- **CPU.** Up to 6 drum voices and 4-10 synth voices. The synth starts at 6; the meter in the header (and on the
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
