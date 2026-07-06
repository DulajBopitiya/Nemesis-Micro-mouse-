#!/usr/bin/env bash
# run_tests.sh - offline regression suite for lib/solver.
#
# Compiles the REAL flood-fill (lib/solver/solver.c) natively and runs it against
# the bundled real competition mazes (test/solver_sim/mazes/) with NO hardware and
# NO physical maze. Deterministic (0 faults). Green = the search still reaches the
# goal and floods home on every maze, and the learned<=optimal invariant holds.
#
# Usage:   bash test/solver_sim/run_tests.sh
# Exit:    0 = all pass, 1 = a regression (a maze failed to solve, or a phantom-
#              wall invariant violation).
#
# Requires a native C compiler (gcc). This does NOT need PlatformIO or the robot.
set -e
cd "$(dirname "$0")/../.."          # repo root (Nemisis Firmware/)

CC=${CC:-gcc}
OUT=test/solver_sim/run_sim.exe
INC="-I test/solver_sim/shim -I lib/solver -I lib/control -I lib/sensors -I lib/battery -I lib/mazestore"
SRC="lib/solver/solver.c test/solver_sim/stubs.c test/solver_sim/run_sim.c"

echo "[build] $CC ..."
$CC -std=gnu99 -O1 -w $INC $SRC -o "$OUT"

MAZES=(test/solver_sim/mazes/*.maz)

echo "[run] correctness regression (mode 0, 0 faults)"
"./$OUT" --mode 0 "${MAZES[@]}"
RC=$?

echo
echo "[run] mode 2 = confirm-optimal explorer (should be found-optimal N/N)"
"./$OUT" --mode 2 "${MAZES[@]}"
[ $? -ne 0 ] && RC=1

echo
echo "[info] diagonal speed-run benefit (route planning, ortho vs 8-way time):"
$CC -std=gnu99 -O2 -w test/solver_sim/diag_plan.c test/solver_sim/run_diag.c \
    -o test/solver_sim/run_diag.exe
"./test/solver_sim/run_diag.exe" "${MAZES[@]}" | tail -2

echo
echo "[info] fault characterisation (stochastic, not a pass/fail gate):"
for F in "5 0 0" "0 5 0" "0 0 5"; do
  printf "  faults miss/false/desync=%s -> " "$(echo "$F" | tr ' ' '/')"
  "./$OUT" --mode 0 --faults $F "${MAZES[@]}" 2>/dev/null | grep -oE "reached: [0-9]+/[0-9]+"
done

exit $RC
