![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Acoustic drone detector — TinyTapeout IHP 1×1

A self-contained acoustic drone detector for one TinyTapeout IHP sg13g2 tile.
A PDM microphone bitstream goes in and a detection signal comes out. The
integer signal-processing pipeline and ternary neural-network weights are
hard-wired, so the chip needs no host, memory, firmware, or model upload.

```text
 PDM data ──ui[0]──► octave filterbank ─► log levels ─► ternary NN ─► uo[3:1]
 mic clock ◄─uo[0]──       5 bands          16 frames
 trim ───ui[7:1]──────────────────────────► threshold
```

## Current build: `drone_dads_lvl12_s4` weights

The checked-in weights are `drone_dads_lvl12_s4`, trained in
`tinytapeout-mnist-nn-asic` on DADS with every clip re-levelled across a
−12…0 dB span (a mixture of loudness, so the template does not key on one
recording level). Twelve seeds were trained; seed 4 was selected on validation
AUC alone, and the test split was never used to choose. Scored with the
bit-exact chip model (`eval_header.py --tag dads_lvl12 --feat-off 8`):

| split | clips | AUC |
|---|---:|---:|
| validation | 11 051 | 99.15 % |
| test (held out) | 12 117 | **98.98 %** |

Three things changed with the weights, and the first two are fixes to the
`drone_4` build that shipped before:

- **`FEAT_OFF` now equals the header's centre (8).** The trainer centres the
  4-bit band features on the training-set mean and writes it as `centre=N`;
  the RTL subtracts `FEAT_OFF`. `drone_4` hardened `FEAT_OFF=6` against a
  `centre=8` header, which is a different model from the measured one (98.9 %
  AUC on paper, 81 % in silicon). The header now also carries
  `localparam WW_CENTRE`, the RTL asserts `FEAT_OFF == WW_CENTRE` at
  elaboration (`A_CENTRE`), and `test/drone_weights_io.py` reads the centre
  from the header so the golden model cannot drift either.
- **The classifier dot product no longer wraps.** It was held in `HACC_W` (6)
  bits, but five centred features of up to ±8 sum to ±40. The trainer, the
  scorer and the golden model all compute the dot exactly and saturate only
  the accumulator, so the silicon disagreed with them whenever |dot| > 31 --
  silence through a five-weight row, for one. `test_detector_matches_model`
  caught it on the new weights at frame 6; the dot now has its own width.
- **The shipped threshold is the validation max-accuracy point, 5**, so trim
  64 is the useful default. The trainer's threshold of 14 fired on 14 % of
  test drones.

This build hardens clean in the 1×1 tile with the same LibreLane 3.0.6 flow
(`TAG=lvl12 ./scripts/harden_local.sh`, `runs/lvl12/final/metrics.json`):

| metric | value |
|---|---:|
| final core utilisation | 93.78 % |
| instances | 2 165 (280 hold buffers) |
| max-fanout, max-cap, max-slew violations | 0 |
| routing DRC, Magic DRC, KLayout DRC | 0 |
| LVS errors, antenna violations | 0 |
| setup / hold violations, all three corners | 0 |
| worst setup slack | +8.30 ns (slow corner) |
| worst hold slack | +0.137 ns (fast corner) |

The previous `drone_4` reference hardening of the same logic completed in a
1×1 tile with:

- 94.21% final core utilization and 2,122 instances;
- **zero max-fanout violations** (14 in `drone_2`);
- zero routing, Magic DRC and KLayout DRC errors;
- zero LVS, antenna, setup, and hold violations;
- +5.62 ns worst setup slack and +0.141 ns worst hold slack.

Two changes got there:

- `CTS_SINK_BUFFER_MAX_CAP_DERATE_PCT: 50` clears every max-fanout violation
  for +98 µm². The violations were against the liberty's `default_max_fanout`,
  which an SDC constraint cannot lift, and they were all CTS clock-leaf
  buffers, which `repair_design` will not touch -- so CTS was the only lever.
- The LED hold drops from 629 ms to **41.9 ms**. A drone is a steady source
  that is still there on the next window, so the output should track it rather
  than latch; at 629 ms the LED lagged the aircraft and ran two passes together
  into one. Narrowing the hold counter to match also returned three flip-flops
  and 209 µm². See [docs/hold_width.md](docs/hold_width.md).

![drone_4 1x1 layout](docs/drone_4_layout.png)

The dataset result is not a field false-alarm guarantee. Detection distance was
not measured, and lawnmowers, motorcycles, helicopters, and unfamiliar ambient
sound may confuse an acoustic detector.

## Interface

- `ui[0]`: PDM microphone data.
- `ui[7:1]`: seven-bit threshold trim centered at 64.
- `uo[0]`: 1.5625 MHz PDM microphone clock from the 50 MHz system clock.
- `uo[3:1]`: mirrored active-high detection output.
- `uo[7:4]`: live four-bit band-level debug value.
- `uio[7:0]`: output-only frame, detection, FSM, and microphone-tick debug.

The shipped threshold is 5 at trim 64, the point that maximises accuracy on
the validation split. Each trim step changes it by four. On the held-out test
split, with the bit-exact chip model:

| trim | threshold | accuracy | recall | negative clips firing |
|---:|---:|---:|---:|---:|
| 66 | 13 | 65.8 % | 26.3 % | 0.25 % |
| 65 | 9 | 87.8 % | 74.8 % | 1.07 % |
| **64** | **5** | **96.0 %** | **96.6 %** | **4.42 %** |
| 63 | 1 | 92.4 % | 99.7 % | 13.9 % |
| 62 | −3 | 78.0 % | 100 % | 40.8 % |

Synthetic room tone never fires at the shipped threshold. Raise the trim for
fewer false alarms, lower it for recall; these are clip-level dataset figures,
not alarms per hour in a deployment.

## Verification

The cocotb tests compare RTL against a local bit-exact Python model:

```bash
python -m pip install -r test/requirements.txt
cd test
make                         # shortened frames, suitable for quick checks
FRAME_LOG2=16 make           # exact tape-out frame length
```

Fourteen tests check reset and clock behaviour,
every filterbank frame against the model, the detector output trace, the exact
mic-clock divider, the LED hold length in frames, reset of a lit LED, a mic
stuck at either rail, trim monotonicity and the threshold arithmetic across the
trim range, recovery from a mid-frame reset, bit-exactness on a quiet-to-loud
ramp, and that `uio_in` and `ena` change nothing -- the same stimulus with those
pins parked and with them moving must give identical outputs, clock for clock.
They also check every debug/output pin against a non-zero internal snapshot and
drive a guaranteed real classifier fire through all four detection outputs.
The full-length RTL pass omits six behavioural checks already covered by the
fast parameter-equivalent RTL build. The gate-level pass runs all fourteen:
tests that inspect or deposit RTL registers have public-pin gate variants, and
classifier tests run far enough to close a real staggered window. This makes
the gate job several hours long, but prevents a green netlist run whose
detection output never asserted. The unused-pins check remains shortened to a
sixteenth of a frame because its clock-for-clock comparison needs no frame
boundary. Here "complete gate-level suite" means all named public-interface
behaviours execute on the netlist; the separate 100% waived coverage figure is
an RTL logic-coverage measurement, not a standard-cell-netlist toggle claim.

Every RTL build also compiles the `WW_ASSERT` block at the bottom of the RTL:
seven elaboration-time parameter checks (including `A_CENTRE`, header centre
against `FEAT_OFF`) and eight per-cycle invariants on the
hold counter, the requantise sign, the FSM and the outputs, for ~0.5 s. Because
they hold under every stimulus, all fourteen tests are scenarios for them.
`src/config.json` never defines `WW_ASSERT`, and that the block does not reach
synthesis is verified rather than argued -- re-running the flow with it present
produced a byte-identical netlist.

```bash
./scripts/assert_mutations.sh   # reintroduce 12 real bugs; each must be caught
./scripts/coverage.sh           # verilator line/branch/expr/toggle coverage, raw and waived
TAG=lvl12 ./scripts/harden_local.sh   # the tt-gds-action LibreLane flow, locally (~1 h)
```

`harden_local.sh` writes `runs/$TAG/`; copy the unpowered
`runs/$TAG/final/nl/tt_um_hyphen133_drone_detection.nl.v` to
`test/gate_level_netlist.v` and run `GATES=yes make` for the gate-level suite
(with a stock iverilog, pass `GL_CELLS=` a copy of `sg13g2_stdcell.v` with the
`specify` blocks stripped).

The mutation test is what makes those assertions evidence rather than
decoration -- an assertion that has never failed may be a tautology, and it has
already caught one of mine that permitted a permanently lit LED. Two mutations
make the design read `uio_in` and gate on `ena`; three more disable the real fire
path, break a detection mirror, and make the threshold comparison inclusive.
The named tests must fail.

Coverage prints two figures. Raw, the RTL sits at 85.7% line, 88.2% branch,
85.5% expression and 92.4% toggle. After `test/coverage_waivers.txt` -- one
line per point that cannot be reached at the shipped parameters and weights,
each with its proof, and the script fails if a proof goes stale -- it is 100%
on all four. Three of the fourteen tests exist because that measurement found a
gap. See [docs/coverage.md](docs/coverage.md).

GitHub Actions also runs TinyTapeout precheck, GDS generation, and gate-level
simulation.

See [the generated datasheet](docs/info.md) for connection and usage details.

## License

Apache-2.0. See [LICENSE](LICENSE).
