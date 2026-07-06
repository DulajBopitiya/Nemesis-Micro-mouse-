# Offline solver regression harness (`lib/solver`)

Compiles the **real** flood-fill search (`lib/solver/solver.c` — the exact code the
robot runs) **natively on a PC** and runs it against real MMS competition mazes,
with **no hardware and no physical maze**. This is the maze-independent regression
net for the maze "brain": if a future change to the solver breaks solving a real
16×16 maze, this goes red.

## Run it

```bash
bash test/solver_sim/run_tests.sh
```

Needs a native C compiler (`gcc`; MinGW is fine on Windows). No PlatformIO, no robot.
Exit 0 = all pass, 1 = a regression.

Run one maze / other modes directly:

```bash
gcc -std=gnu99 -O1 -I test/solver_sim/shim -I lib/solver -I lib/control \
    -I lib/sensors -I lib/battery -I lib/mazestore \
    lib/solver/solver.c test/solver_sim/stubs.c test/solver_sim/run_sim.c -o run_sim
./run_sim --mode 0 test/solver_sim/mazes/apec2002.maz          # explore mode
./run_sim --mode 1 test/solver_sim/mazes/*.maz                 # shortest-path mode
./run_sim --mode 0 --faults 0 0 5 test/solver_sim/mazes/*.maz  # inject 5% desync
```

## How it works

`solver.c` references the motion/sensor/battery layer throughout its FSM, but a
headless `Solver_RunSim()` is pure CPU against a virtual maze and calls none of it.
So:
- `shim/main.h` — provides the HAL handle types the public driver headers name, so
  they compile without the STM32 HAL.
- `stubs.c` — link-by-name no-ops for the hardware layer (never called in the sim),
  plus the two CMSIS bits the sim path does read (`SystemCoreClock`, `DWT->CYCCNT`).
- `run_sim.c` — loads an MMS `.maz` (`x y N E S W` per line) into the sim's
  ground-truth grid and runs `Solver_RunSim`.
- `mazes/` — a fixed set of real competition mazes bundled so the suite is
  self-contained (doesn't depend on the external MMS install path).

## What it proves (and what it doesn't)

**PASS = correctness invariant:** the search reaches the goal AND floods home, and
`learned <= optimal` holds (with 0 faults the discovered map is a subset of the true
walls, so its shortest path can only be optimistic; `learned > optimal` with no
faults would be a phantom-wall **bug**).

**found-optimal** (`learned == optimal`) is a tracked **efficiency** metric. Modes 0
(explore) and 1 (shortest) get only 3/12 — they stop at the goal, so the shortest
*optimistic* path usually still runs through unsensed cells (`learned < optimal`).
**Mode 2 (`--mode 2`, "confirm-optimal") gets 12/12**: it keeps exploring until the
optimistic best-path is fully sensed, at which point it's a real drivable path and,
since optimistic ≤ true-optimal, it *is* the optimal. Cost: more exploration moves
(higher `journey`) — the price of a guaranteed-optimal search. Mode 2 is SIM-ONLY;
the live `solve` FSM still uses the explore picker (unchanged).

**Baseline (2026-07-06), mode 0, 0 faults, 12 competition mazes:**
`reached 12/12`, `found-optimal 3/12`, invariant holds everywhere.
Cross-check: `big_queue` optimal = 26 (matches the independently-recorded reference).

**Fault characterisation** (stochastic, informational — reproduces the real hardware
failure signature): at 5% single-fault, `missed-walls 12/12` (tolerant),
`phantom-walls ~4/12` (fragile), `position-desync 0/12` (**catastrophic**). Desync is
the dominant failure mode — same as the real fast-run crashes — so front-wall /
anti-desync reliability (the P3 drift work) is where hardening pays off.

**NOT covered here (MCU-only):** per-decision reflood **timing** (host has no cycle
counter, `DWT->CYCCNT` reads 0) and real **RAM** footprint — run the on-MCU `sim` /
`sim stats` console command for those. This harness covers correctness + path
optimality; the on-MCU sim covers timing + memory on the real chip.
