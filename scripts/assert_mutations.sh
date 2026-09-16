#!/usr/bin/env bash
# Mutation test for the WW_ASSERT block in the RTL.
#
# An assertion that has never failed is not evidence of anything -- it may be a
# tautology, or may be misusing a system function that quietly returns the
# passing value. Three checks in the first draft of that block were exactly
# that: `hval <= 15` and the fmax range were guaranteed by their declared
# widths, and $isunknown on a concatenation always returns 1 in iverilog. So
# each mutation below reintroduces one real bug, and the assertion named beside
# it has to fire.
#
# A_HOLD_FITS guards the bug that shipped in the wake-word sibling of this
# design: `hold` sized from NFRAME instead of from HOLD_FRAMES, so 4'(16)
# truncated to zero and the LED never asserted.
#
# That bug is not directly reproducible in this build, and it is worth being
# precise about why rather than pretending otherwise: NFRAME=16 makes the old
# `[FIDX_W:0]` five bits wide, and HOLD_FRAMES is now 2, so the historical
# sizing would fit here twice over. Restoring it changes nothing observable.
# `hold_width_too_narrow` therefore tests the guard directly, by forcing HOLD_W
# below what HOLD_FRAMES needs. It drives HOLD_W rather than the declaration so
# that `hold` and the assertion block's own `hold_q` stay the same width -- a
# mutation that narrows only the declaration desyncs them and trips
# A_HOLD_STEP, which looks like a catch but is an artifact of the mutation.
# See docs/hold_width.md.
#
#   ./scripts/assert_mutations.sh        # every mutation
#   CASE=test_dc_input_bit_exact ./scripts/assert_mutations.sh
#
# Everything runs on copies under a temp dir; src/ and test/ are never touched.
set -uo pipefail
cd "$(dirname "$0")/.."

IMAGE=${IMAGE:-ww-ci-sim:latest}
# test_hold_duration walks several frames in ~0.8 s, which is enough to reach
# S_ROLL and the hold decrement. It parks the trim at 127 so that nothing
# fires, and test_detector_matches_model deliberately exercises equality at
# the threshold without firing. The fire-load mutation therefore names
# test_trim_raises_threshold, whose trim-1 pass fires and asserts that it does.
CASE=${CASE:-test_hold_duration}
RTL=src/tt_um_hyphen133_drone_detection.sv
WORK=$(mktemp -d)
cleanup() {
  chmod -R u+w "$WORK" 2>/dev/null || true
  command rm -rf -- "$WORK"
}
trap cleanup EXIT

# name | test case (- for $CASE) | pattern that must appear | perl -0777 substitution
MUTATIONS=(
"hold_width_too_narrow|-|A_HOLD_FITS|s/localparam HOLD_W   = .*/localparam HOLD_W = 1;/"
"hold_never_expires|-|A_HOLD_EXPIRES|s/if \(hold != 0\) hold <= hold - 1'b1;/if (hold != 0) hold <= hold;/"
"fire_loads_wrong_value|test_trim_raises_threshold|A_FIRE_LOADS_HOLD|s/hold <= HOLD_W'\(HOLD_FRAMES\)/hold <= HOLD_W'(1)/"
"hold_decrements_by_two|-|A_HOLD_STEP|s/hold <= hold - 1'b1;/hold <= hold - 2'd2;/"
"roll_skips_idle|-|A_ROLL_TO_IDLE|s/(if \(hold != 0\) hold <= hold - 1'b1;\s*\n\s*cnt <= cnt \+ 1'b1;\s*\n\s*st  <= )S_IDLE;/\${1}S_CASC;/"
"hval_sign_dropped|-|A_HVAL_SIGN|s/hval = acc_next\[HACC_W-1\]        \?/hval = 1'b0                     ?/"
"nframe_not_pow2|-|A_NFRAME_POW2|s/localparam FIDX_W   = \\\$clog2\(NFRAME\);/localparam FIDX_W = \\\$clog2(NFRAME) + 1;/"
# These two mutate the design rather than an assertion's subject: they make it
# depend on uio_in and on ena, which it must not, and the named *test* has to
# fail. Same discipline -- a test that has never failed is not evidence.
"uio_in_read_as_data|test_unused_inputs_ignored|test_unused_inputs_ignored *FAIL|s/x_in = pdm_bit \?/x_in = (pdm_bit ^ uio_in[0]) ?/"
"ena_gates_sampling|test_unused_inputs_ignored|test_unused_inputs_ignored *FAIL|s/\}\}\}\) pdm_bit <= ui_in\[0\];/}}} \&\& ena) pdm_bit <= ui_in[0];/"
"fire_disabled|test_real_fire_drives_detection_outputs|test_real_fire_drives_detection_outputs *FAIL|s/wire fire = /wire fire = 1'b0 \&\& /"
"detection_mirror_broken|test_debug_pin_mapping|test_debug_pin_mapping *FAIL|s/assign uo_out\[2\] = detect;/assign uo_out[2] = 1'b0;/"
"threshold_compare_inclusive|test_detector_matches_model|test_detector_matches_model *FAIL|s/osum_w > thresh/osum_w >= thresh/"
)

pass=0; fail=0
printf '%-24s %-34s %s\n' MUTATION MUST-SHOW RESULT
printf '%s\n' "$(printf '%.0s-' {1..82})"

for m in "${MUTATIONS[@]}"; do
  IFS='|' read -r name tcase want prog <<<"$m"
  [ "$tcase" = - ] && tcase=$CASE
  d="$WORK/$name"
  mkdir -p "$d/src" "$d/test"
  cp src/*.svh "$d/src/"
  cp test/Makefile test/test.py test/tb.v test/drone_model.py "$d/test/"

  perl -0777 -pe "$prog" "$RTL" > "$d/$RTL"
  if cmp -s "$d/$RTL" "$RTL"; then
    printf '%-24s %-34s %s\n' "$name" "$want" "BROKEN (pattern did not match)"
    fail=$((fail+1)); continue
  fi

  # --user keeps the build products owned by the caller; without it docker
  # writes sim_build and __pycache__ as root and the temp dir cannot be removed.
  # TESTCASE is what cocotb 1.x reads; COCOTB_TESTCASE is the 2.x name. With
  # only the latter, every mutation silently ran the whole suite.
  out=$(docker run --rm --user "$(id -u):$(id -g)" -v "$d":/work -w /work/test \
        -e TESTCASE="$tcase" -e COCOTB_TESTCASE="$tcase" -e HOME=/tmp "$IMAGE" \
        bash -lc 'timeout 300 make 2>&1' 2>&1)

  if grep -q "$want" <<<"$out"; then
    printf '%-24s %-34s %s\n' "$name" "$want" "caught"
    pass=$((pass+1))
  else
    fired=$(grep -oE 'A_[A-Z_]+:' <<<"$out" | tr -d ':' | sort -u | tr '\n' ' ')
    printf '%-24s %-34s %s\n' "$name" "$want" \
      "NOT CAUGHT${fired:+ -- fired instead: $fired}"
    fail=$((fail+1))
  fi
done

printf '%s\n' "$(printf '%.0s-' {1..82})"
echo "caught $pass of $((pass+fail))"
[ "$fail" -eq 0 ]
