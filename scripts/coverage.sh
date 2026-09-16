#!/usr/bin/env bash
# Line and toggle coverage for tt_um_hyphen133_drone_detection, via verilator + test/cov_tb.sv.
#
#   ./scripts/coverage.sh              # raw and waived
#   KEEP=1 ./scripts/coverage.sh       # keep the annotated source in artifacts/
#
# Read the caveat at the top of test/cov_tb.sv first: this instruments the RTL
# under the same stimulus *classes* as the cocotb suite, not the suite itself
# (verilator is not in the cocotb image). It answers "is any logic
# unexercised", which is what points at a missing test.
#
# Two figures come out, and both matter. The raw one counts every point in the
# DUT (the harness excludes itself with a coverage_off). The waived one drops
# the points test/coverage_waivers.txt proves unreachable at this build's
# parameters and weights -- AVG_N=0 leaves the frame-mean path dead, the drone
# weights leave w2c[0] stuck at 1 -- each with its proof beside it. Anything
# still uncovered after the waivers is a real gap, and the script fails if a
# waiver stops removing an uncovered point, because then its proof is stale.
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE=${IMAGE:-verilator/verilator:latest}
# Only one weight set exists here; the waiver file is still build-tagged, so
# it stays parameterised to keep the file identical to the upstream repo.
WEIGHTS=drone
KEEP=${KEEP:-0}
WORK=$(mktemp -d)
[ "$KEEP" = 1 ] || trap 'rm -rf "$WORK"' EXIT

DEFS="-DWW_ASSERT"

cp src/tt_um_hyphen133_drone_detection.sv src/drone_weights.svh "$WORK/"
cp test/cov_tb.sv "$WORK/"

# --coverage turns on line, toggle and user coverage. The design has no
# concurrent assertions (iverilog cannot parse them, so the checks in
# tt_um_hyphen133_drone_detection.sv are immediate), so there is no assertion coverage to
# collect.
docker run --rm --user "$(id -u):$(id -g)" -v "$WORK":/w -w /w -e HOME=/tmp "$IMAGE" \
  --binary --coverage --timing -Wno-DECLFILENAME -Wno-WIDTHTRUNC \
  $DEFS --top-module cov_tb -o cov_sim cov_tb.sv tt_um_hyphen133_drone_detection.sv \
  >"$WORK/build.log" 2>&1 || { tail -30 "$WORK/build.log"; exit 1; }

docker run --rm --user "$(id -u):$(id -g)" -v "$WORK":/w -w /w -e HOME=/tmp \
  --entrypoint ./obj_dir/cov_sim "$IMAGE" >"$WORK/run.log" 2>&1 || true
grep -q "cov_tb: done" "$WORK/run.log" || { tail -20 "$WORK/run.log"; exit 1; }
# An assertion failure during the coverage run matters more than the number:
# the stimulus reached a state that breaks an invariant.
if grep -qE "A_[A-Z_]+:" "$WORK/run.log"; then
  echo "ASSERTION FAILED during coverage run:"
  grep -oE "A_[A-Z_]+:.*" "$WORK/run.log" | sort -u
  exit 1
fi

python3 scripts/cov_waive.py --build "$WEIGHTS" --rtl "$WORK/tt_um_hyphen133_drone_detection.sv" \
  --waivers test/coverage_waivers.txt "$WORK/coverage.dat" "$WORK/coverage_waived.dat" \
  >"$WORK/waive.log" || { cat "$WORK/waive.log"; exit 1; }

summarise() {  # $1 = coverage file, $2 = annotate dir
  docker run --rm --user "$(id -u):$(id -g)" -v "$WORK":/w -w /w -e HOME=/tmp \
    --entrypoint verilator_coverage "$IMAGE" \
    --annotate "$2" --annotate-min 1 "$1" 2>&1 | sed -n '/Coverage Summary/,/fsm_arc/p'
}

echo "=== $WEIGHTS build, raw (DUT only) ==="
summarise coverage.dat annotated_raw
echo
echo "=== $WEIGHTS build, after waivers ==="
summarise coverage_waived.dat annotated
echo
cat "$WORK/waive.log"

# verilator_coverage prefixes a line with %000000 when every point on it is
# uncovered, and with ~ when only some are; the summary above has the exact
# point counts. This is the quick pointer at whole dead lines.
ann="$WORK/annotated/tt_um_hyphen133_drone_detection.sv"
if [ -f "$ann" ]; then
  n=$(grep -c '^%000000' "$ann" || true)
  echo
  echo "lines with every point uncovered after waivers in tt_um_hyphen133_drone_detection.sv: $n"
  [ "$n" -gt 0 ] && { echo; grep -n '^%000000' "$ann" | head -40; }
fi

if [ "$KEEP" = 1 ]; then
  mkdir -p artifacts/coverage
  rm -rf "artifacts/coverage/$WEIGHTS" "artifacts/coverage/${WEIGHTS}_raw"
  cp -r "$WORK/annotated" "artifacts/coverage/$WEIGHTS"
  cp -r "$WORK/annotated_raw" "artifacts/coverage/${WEIGHTS}_raw"
  echo
  echo "annotated source: artifacts/coverage/$WEIGHTS/ (waived), artifacts/coverage/${WEIGHTS}_raw/"
fi
