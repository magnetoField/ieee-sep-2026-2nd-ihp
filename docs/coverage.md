# Coverage

Measured, not asserted. `./scripts/coverage.sh` builds `test/cov_tb.sv` with
`verilator --binary --coverage` and reports line, branch, expression and toggle
coverage of the RTL, twice: raw, and after the waivers in
`test/coverage_waivers.txt`.

Read the caveat first: verilator is not in the cocotb image and cocotb's
verilator backend needs it at build time, so this cannot instrument `test.py`
itself. `cov_tb.sv` drives the same *classes* of stimulus the suite uses -- a
chirp across every band, an amplitude ramp, both mic rails, Nyquist
alternation, near-silence, the trim range, a mid-frame reset, and the unused
pins moving. It answers "is any logic unexercised", which is the question that
points at a missing test. It is not a coverage figure for the cocotb suite.

## Where it stands

The DUT only: the harness excludes itself with a `coverage_off`, because its
32-bit generator counters were 170 toggle points that said nothing about the
design and its 31 always-run lines flattered the line figure (the earlier
93.2 % was that).

| metric | raw | after waivers |
|---|---|---|
| line | 85.7 % (24/28) | **100 %** (23/23) |
| branch | 88.2 % (60/68) | **100 %** (55/55) |
| expression | 85.5 % (65/76) | **100 %** (59/59) |
| toggle | 92.4 % (924/1000) | **100 %** (904/904) |

Both numbers matter and the script prints both. The raw one says how much of
the written RTL this stimulus reaches. The waived one says whether anything
*reachable* is unreached, which is the question a missing test would answer,
and it is the one to hold at 100 %.

## What a waiver is, and is not

A waiver removes a point that cannot be reached at this build's parameters and
weights. Every line of `test/coverage_waivers.txt` carries its proof, and the
format is a regex over `<type> <point> @ <source line text>` rather than a line
number, so an edit above the point does not detach the waiver from it. The
file is kept identical to the upstream multi-model repo, which is why its
lines are tagged by build; only the `drone` and `all` lines apply here.

Two rules keep the file honest:

* `scripts/cov_waive.py` **fails the run if a waiver removes no uncovered
  point.** A waiver that hides nothing is stale: the code under it changed, or
  a test now reaches it, and either way the proof beside it no longer
  describes the design. Delete it rather than carry it.
* The table it prints shows, per waiver, how many uncovered *and* how many
  covered points it took. A waiver over a whole signal (`uio_oe`) necessarily
  removes the bits that do toggle too; that is visible, not hidden.

The 99 raw-uncovered points, and where each went:

| cause | uncovered | covered also removed |
|---|---|---|
| `AVG_N=0`: `favg`, `avg_nx`, the ring update, its reset, the `(b < NBAND)` else arm | 45 | 15 |
| `MANT=1`: the mantissa-free log branch | 1 | 0 |
| `DEBUG_PINS=1`: `uio_oe` is the constant `8'hFF` | 8 | 8 |
| FSM `default` arm: `st` is 2 bits and all four encodings are named | 1 | 0 |
| `x_in[7:0]`: ±128 differ only in bits 9..8; bit 7 is 1 in both | 15 | 1 |
| `bmag[9:8]`, `bexp[4]`: \|band\| ≤ 192 (state in [-128,127], input a state or ±128, band = d − (d>>>2)) | 6 | 0 |
| `wide > FEAT_MAX`: bexp ≤ 8 and MANT=1 give wide ≤ 15 | 2 | 2 |
| `hval` clamp-to-15: HACC_W=6, HSHIFT=1 give hsh ≤ 15 | 1 | 0 |
| `hval_s[6:4]`: zero-extension of a 4-bit value | 6 | 0 |
| `trim[1:0]`: `<<< 2` | 4 | 0 |
| the literal `1'b0` term of the `_unused` sink | 1 | 0 |
| `WW_ASSERT` checkers: verification code, failure arms unreachable | 1 | 2 |
| `w2c[0]`: all four output weights non-zero (`WW_W2 = 8'hfd`) | 3 | 2 |
| `thresh[1:0]`: `WW_THRESH_PK = 14`, trim a multiple of 4 | 3 | 1 |
| `hb[3]`: biases [8, −2, −1, 9] all have bit 3 set | 1 | 1 |
| `is_tap` upper bound: TAP0+NBAND = NSTAGE, stg ≤ 8 | 1 | 0 |
| **total** | **99** | **32** |

Three of these are worth knowing about beyond the number. The `hval`
clamp-to-15 arm is dead at the shipped HACC_W/HSHIFT, so the requantiser can
never saturate high; synthesis removes it. `wide > FEAT_MAX` likewise: the log
encoder cannot exceed 15 at STATE_W=10 and K_SHIFT=2, so the saturate mux is
free. And `x_in` at 15 of 20 untoggled looks alarming and is not: the input is
one PDM bit, so `x_in` takes exactly two values, +128 and -128, which differ
only in bits 9 and 8. The waiver covers bits 7..0 only; the two that move stay
in the count.

## What the measurement actually changed

Three of the fourteen tests came out of this rather than out of guesswork:

* **`test_unused_inputs_ignored`.** The one real gap. The TinyTapeout wrapper
  wires all eight `uio` pins and `ena` into every project, this design uses
  none of them, and nothing checked that: every test parked `uio_in` at 0 and
  `ena` at 1, so all 17 of their toggle points and the `ena == 0` term of the
  `_unused` sink sat at zero. A refactor that read `uio_in[0]` as a second
  data input, or gated sampling on `ena`, would have passed the suite. The
  test runs the same stimulus twice, parked and with a counter walking
  `uio_in` while `ena` drops for 16 of every 32 ticks, and demands the second
  run be bit-exact against the golden model *and* identical to the first on
  `uo_out`/`uio_out`/`uio_oe` clock for clock, debug pins included. 18 points,
  2.3 s. It runs in every pass: at FRAME_LOG2=16 and on the netlist it takes a
  sixteenth of a frame and keeps only the clock-for-clock comparison, since the
  golden check needs whole frames. `scripts/assert_mutations.sh` carries two
  mutations that introduce exactly those bugs and checks the test fails on each.
* **`test_threshold_trim_arithmetic`.** `thresh` had 7 untoggled points because
  the suite only ever drove four trim values. `test_trim_raises_threshold`
  covers the *behaviour* but cannot sweep -- it needs whole frames per point to
  count detections. The arithmetic is combinational, so this reads `thresh`
  straight off the pins across 11 trim values chosen to toggle every bit of
  `ui_in[7:1]` in both directions, and checks it against
  `WW_THRESH_PK + ((trim-64) << 2)`. 7 -> 3 untoggled, the remainder the
  mod-4 constants now waived. Cost: 0.01 s.
* **`test_band_dynamic_range`.** `make_pdm()` holds one amplitude, so the log
  encoder never walked its exponent range. This sweeps amplitude over three
  decades, bit-exact, and asserts the feature levels span at least 4 steps. It
  did *not* move the toggle count -- `bexp` and `hval_s` turned out to be
  structurally capped, per the table -- but it is the test that established
  as much, and a real approach is a ramp rather than a plateau.

An earlier reading of the same data called `bmag`, `bexp` and `hval_s` real
gaps. That was wrong, and the bounds in the table are why. Only `thresh` and
the unused pins were test gaps.

## Coverage is not the same as the checks biting

Coverage says the logic ran. It says nothing about whether anything would have
noticed a wrong answer. `scripts/assert_mutations.sh` is the other half: it
reintroduces twelve real bugs -- seven against the named assertion and five
against focused behavioural tests -- and requires each to be caught. It has
already earned
its place by finding that `A_HOLD_STEP` permitted a hold counter stuck on
forever -- 100 % line coverage over that code would not have hinted at it. See
[hold_width.md](hold_width.md).

## Running it

```bash
./scripts/coverage.sh                  # raw and waived
KEEP=1 ./scripts/coverage.sh           # annotated source into artifacts/coverage/drone/ and drone_raw/
```

The script fails if any `WW_ASSERT` check trips during the coverage run, and
if any waiver has gone stale.
