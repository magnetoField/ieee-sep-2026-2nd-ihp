# The LED hold: 629 ms to 41.9 ms, and the counter that carries it

Two changes to the output stage, one behavioural and one a guard against a bug
that shipped in this design's sibling.

## What changed

`HOLD_FRAMES` is 2, not 16. `hold` is loaded in `S_CLASS` and decremented in
the `S_ROLL` of that same frame, so the LED covers `HOLD_FRAMES - 1` whole
frames:

| | `HOLD_FRAMES` | visible LED |
|---|---|---|
| before | 16 | 629 ms |
| now | 2 | **41.9 ms** |

A wake word is a single short event a human has to notice, so latching for
629 ms is right for one. A drone is not: it is a steady source that is still
there on the next window, so the output should track it. At 16 the LED lagged
the aircraft by 629 ms and ran two passes together into one.

`HOLD_FRAMES = 1` is not the floor it looks like. The load and the decrement
fall in the same frame, so it would blink for the few hundred clocks of
`S_CLASS` and never be seen. 2 is the minimum that produces an output.

## The counter width

`hold` was sized from the window geometry rather than from its own range:

```systemverilog
localparam FIDX_W = $clog2(NFRAME);
logic [FIDX_W:0] hold;                      // <- sized from NFRAME
...
if (fire) hold <= (FIDX_W+1)'(HOLD_FRAMES);
```

**This build was never broken by that**, and the reason is luck rather than
design. `NFRAME=16` makes `FIDX_W+1` five bits, which holds 16 comfortably. The
same source with `NFRAME=8` gives a four-bit counter, `4'(16)` truncates to
**zero**, and the LED can never assert at all -- which is exactly what happened
to the wake-word sibling of this design, and it shipped that way in a hardened
part.

So the sizing now comes from the thing it counts:

```systemverilog
localparam HOLD_W = $clog2(HOLD_FRAMES + 1);
logic [HOLD_W-1:0] hold;
...
if (fire) hold <= HOLD_W'(HOLD_FRAMES);
```

At `HOLD_FRAMES=2` that is two bits instead of five, so the fix also gives back
three flip-flops.

## Why no tool caught it in the sibling

Worth recording, because all three of the obvious defences failed:

* **A linter does not see it.** `verilator --lint-only -Wall` reports nothing on
  the pre-fix source. The truncating cast is *explicit*, and an explicit width
  cast is exactly what silences `WIDTHTRUNC`. It is a value invariant, not a
  width mismatch.
* **Synthesis does not remove it.** The counter, its decrement and the LED
  driver are all still built, correctly; `hold` simply never receives a nonzero
  value. So the part passes DRC, LVS, timing and every corner with a fully
  populated LED path that cannot assert.
* **Comparing against the golden model does not see it either**, unless the run
  actually fires. That part applies to this repo too -- see below.

## The gate-level pass must see the LED

A window has to fire before the LED can be compared to anything, and with
`NPHASE=2` the staggered phase's boundary comes into reach at frame
`NFRAME/2 - 1`. How many frames a pass gets through therefore decides what it
tests:

| pass | frames | can a window fire |
|---|---|---|
| `FRAME_LOG2=8` | 40 | yes |
| `FRAME_LOG2=16` | 8 | yes |
| gate-level baseline equivalence | 5 | **no** |
| gate-level real-fire/hold tests | 10 | **yes** |

A frame costs ~8 ms, ~2 s and several minutes respectively. The baseline
gate-level equivalence check therefore remains five frames, but it is no longer
the only gate-level evidence. Every named test executes on the netlist, and the
dedicated real-fire and hold checks run through the first staggered decision
and its release. Tests that deposit `hold` in RTL instead cause a guaranteed
real classifier fire and observe only public outputs on the netlist.

`test_detector_matches_model` runs at `trim = 58`, where this deterministic
stimulus lands exactly on the threshold. That checks that equality does not
fire and that classifier arithmetic does not invent a detection. A separate
`test_real_fire_drives_detection_outputs` uses a threshold below the output
layer's mathematical minimum, reaches a real staggered-window decision without
forcing DUT state, and checks assertion and release on every detection pin in
both RTL and gate-level simulation. The gate-level threshold-boundary test also
uses adjacent trims 56 and 57 around a known score of -15: one must fire and
the other must not.

## What holds it now

`test_hold_duration` measures the LED in the design's own frames and prints the
figure -- `LED holds 1 frame(s) = 41.9 ms at the tape-out frame length`. It
loads `hold` on a real frame boundary, exactly as the `S_CLASS` fire path does,
and parks the trim at 127 first so a spontaneous fire cannot reload the counter
mid-count.

Beside it, the `WW_ASSERT` block at the bottom of the RTL carries the guard as
an invariant rather than a test:

```systemverilog
// Measured against $bits(hold) -- the width the register was actually declared
// with -- not against HOLD_W. Checking the localparam would pass happily if
// the declaration were ever wired to some other parameter again, which is
// precisely the bug this is here to stop.
assert (HOLD_FRAMES < (1 << $bits(hold)))
  else $fatal(1, "A_HOLD_FITS: ...");
```

`scripts/assert_mutations.sh` proves that guard and six others can actually
fail. That matters more than it sounds: an assertion that has never failed may
be a tautology. Three checks in the first draft of the block were exactly that,
and a fourth -- `A_HOLD_STEP` -- permitted a counter stuck on forever, which is
a permanently lit LED. The mutation test found it, and `A_HOLD_EXPIRES` now
pins the frame boundary.

The block is compiled only under `-DWW_ASSERT`, which `test/Makefile` sets and
`src/config.json` never does. That it does not reach synthesis is verified
rather than argued: re-running the flow with the block present produced a
byte-identical netlist.

## Related

* [coverage.md](coverage.md) -- what the suite exercises, measured.
