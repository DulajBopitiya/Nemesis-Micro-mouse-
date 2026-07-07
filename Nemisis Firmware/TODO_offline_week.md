# Offline work — no maze week (~2026-07-06 → ~2026-07-13)

Work that does **not** need the physical maze. Robot may be available for **bench** tests
(single wall, pivot-on-a-table, motor/sensor cal) — just not full maze runs.
Ordered by leverage. Check items off as we go.

---

## P1 — Telemetry / log-fetch reliability  *(highest leverage — unblocks everything)*
Every dump this session dropped ~1/3 of trace rows ("fewer trace rows than expected —
fetch again"), so we were debugging half-runs. Robot keeps the full log in RAM; the
*transfer* is what's lossy.

- [x] **App:** detect the dropped-row warning and **auto re-fetch** until row count == expected *(2026-07-06: `_eval_fetch`/`_auto_refetch` in gui.py; bounded `FETCH_MAX_RETRIES=3`, keeps most-complete session, loud console+maze status)*
- [x] **App:** per-run row-count / checksum handshake so a truncated run is obvious *(compares rows vs firmware-announced `ntrace` per run; `MultiRunFetch.incomplete_runs`)*
- [x] **Firmware:** ESP32-C3 `dump` bridge flow-control *(2026-07-06 earlier: `writeAllToClient()` + 8KB UART RX ring — see memory `link-reliability-hardening`; root cause was silent `WiFiClient.write()` truncation, now fixed)*
- [x] Verify end-to-end on the bench via **🧪 Fake dump (ESP)** — **PASSED 2026-07-06**: first fetch 133/200 → app logged SHORT → auto re-fetch → second fetch 200/200 → `COMPLETE after 1 retry`. Detect-and-recover confirmed on real hardware. *(harness: `##DUMPTEST##` in ESP main.cpp + toggle in maze_view.py)*

## P2 — On-MCU simulator hardening  *(test the brain without a maze)*
`sim` runs the REAL flood-fill on a virtual maze + fault injection. **2026-07-06:
built a HOST harness — compiles `lib/solver` natively (gcc) + runs it offline, no
robot. `bash test/solver_sim/run_tests.sh`. See `test/solver_sim/README.md`.**

- [x] Load real competition mazes (apec2002, ieee2011, seoul01/02, hitel, minos,
      maze2011, jrhmaze, big_queue, longpath) — bundled in `test/solver_sim/mazes/`
- [x] Prove solve on all, 16×16 — **reached 12/12**; invariant `learned<=optimal`
      holds (no phantom-wall bug); found-optimal 3/12 (explore under-explores = the
      known efficiency gap, tracked not gated). Cross-checked: big_queue optimal=26
- [x] Regression-test phantom-wall recovery + desync — fault sweep: missed-walls
      tolerant (12/12 @5%), phantom fragile (~4/12), **desync catastrophic (0/12)** =
      reproduces the real fast-run crash signature → confirms P3 is the priority
- [x] Save a fixed sim test-set so future changes are regression-checked — the script
      + bundled mazes are the permanent net (exit 1 on any solve regression)
- [ ] flood-fill **timing + RAM on 16×16** stays MCU-only (host has no cycle counter):
      run the on-MCU `sim stats` when the robot's powered — carry-over bench task

## P3 — Drift root cause  *(the thing crashing the fast run)*
- [x] **Offline: quantified drift across all 21 fast-run logs (2026-07-06)** via
      `NEMISIS PYTHON DEBUG APPLICATION/analyze_drift.py`. FINDING: turns
      **systematically UNDER-rotate ~8-11°** (dominant, fixable); straight heading
      fine (~3°); 350 wall-clip impacts; intermittent right-wheel stall in ~3 runs;
      battery sags to 6.78V (secondary). Full write-up: memory
      `drift-log-analysis-0704-0705`. Causal chain: under-rotate → skewed cell entry
      → clip/ram → desync → crash (desync = fatal, per the sim harness).
- [ ] **Propose ONE turn-completion fix** — correct the arc/flow-turn so it rotates
      the commanded angle (kill the ~10° debt at source), per `smooth-turn-rework-
      analysis` (trapezoidal-ω + fix early-release + post-turn heading-setpoint
      handoff). Needs a read of control.c turn-completion. Single, reversible change.
- [x] **Root cause PINNED to `control.c:1753`** (2026-07-06): `Control_Start` re-zeros
      `heading_deg` every primitive → snappy pivot releases ~4° short + ~8° PID lag,
      then the next straight adopts the shortfall as "straight ahead". Fix drafted:
      carry the residual (`hcarry` toggle). See memory `drift-log-analysis-0704-0705`.
- [x] **Turn-diagnostic LOGGING built + flashable** (2026-07-06): `benchturn <n> <deg>`
      spins N pivots in place (no walls) → `TURNDIAG`/`TURNSUM` lines; `turn` emits
      `TURNDIAG,-1,...`; app parses + `analyze_turns.py`. Builds clean.
- [ ] **Bench-verify WITHOUT walls** (doable now — user has a floor): flash, run
      `benchturn 4 90`, read the residuals to confirm the ~8° pin. THEN implement the
      `hcarry` fix and re-run to prove cumulative drift → ~0.
- [x] Characterise the intermittent right-wheel stall (2026-07-06, bench): added
      per-wheel posL/posR to TURNDIAG; `benchturn 6 360` → **R/L 0.99-1.04 (balanced)**,
      battery barely sagged. Right wheel is FINE on the bench → the stall is a FULL-RUN
      / load / drained-battery effect, not a mechanical bind. Bench diagnosis COMPLETE:
      pivot ✓ battery ✓ wheels ✓ all healthy. Remaining desync (wall-clip impacts +
      load-stall) needs WALLS/a real run → **P3 BLOCKED until the maze returns.**

## P4 — control.c consolidation  *(behaviour-preserving cleanup)*
`control.c` is a patchwork from this session (stall guards, front-stop variants, creep,
reverted-then-re-added bits). Safety net = `sim` + build.

- [ ] Map + document the move/finish/stall/front-stop state machine
- [ ] Remove dead paths, unify the front-wall reads, add comments
- [ ] Confirm no behaviour change (build size + sim regression)

## P5 — Sim-developable features
- [x] **Optimal explorer (`sim mode 2`, 2026-07-06)** — "confirm-optimal" search: keeps
      exploring until the optimistic best-path is fully sensed → **found-optimal 3/12
      → 12/12** on the real comp mazes, no robustness regression. SIM-ONLY (live `solve`
      FSM untouched by construction). See memory `solver-host-regression-harness`.
      Follow-up: bench-prove, then port into the real search FSM (opt-in first).
- [x] **Diagonal speed-run PLANNER built + validated (2026-07-06)** — ported the
      8-heading time-weighted Dijkstra to `test/solver_sim/diag_plan.c` (host-only,
      no lib/solver dep). Proven on the 12 comp mazes: **diagonals ~25% faster on
      average** (up to 36%). Route planning done; the MOTION layer (45°/half-cell
      trajectory in control.c) to actually drive them is the remaining piece (bigger,
      tuning is maze-blocked). See memory `solver-host-regression-harness`.
- [ ] Diagonal MOTION layer — 45°/half-cell trajectory generation + control-loop
      execution (needs walls to tune; can prototype the trajectory generator offline)
- [ ] Full per-segment velocity profile (Vpeak = √(2·d·L)) — extend the current cap

## P6 — App: analysis + tuning tooling
- [ ] "Run report": auto-compute per-turn stop error, pivot angles, wheel imbalance, front-wall approach IR, battery sag
- [ ] Trajectory overlay: actual path vs planned path vs wall detections on the maze view
- [ ] Compare-two-runs / parameter-sweep view (decel, brake_rev, VPROF_DECEL_FRAC, …)

## P7 — OTA firmware update (STM32 via ESP32-C3)  *(agreed worth doing; do AFTER P3)*
Goal: flash the STM32 over the existing WiFi link instead of SWD. **Stage the image
in the board's EXTERNAL flash** (not yet configured). Full design + rationale in
memory `ota-external-flash-plan.md`.
- [x] Fix the latent linker risk first: cap the app region so it can't grow into the
      top 3 persistent pages (maze/cal/settings @ 0x0807D000+) *(2026-07-07: KEY FINDING
      — PlatformIO ignores `STM32CubeIDE/*.ld`; it uses the package script from
      `tool-ldscripts-ststm32/stm32g4/STM32G474RETX_FLASH.ld`. Fix: copied that script
      into repo `linker/STM32G474RETX_FLASH.ld`, capped FLASH 512K→500K, pointed
      `board_build.ldscript` at it in platformio.ini. Build-verified: links clean
      (Flash 21.7%), verbose link shows `-T linker/STM32G474RETX_FLASH.ld`.)*
- [x] `git init` the firmware repo — already under git (branch `main`)
- [x] Configure external flash in `NEMSIS.ioc` (QSPI) *(2026-07-07: QUADSPI1 bank1
      quad lines, W25Q32JW 4MB, pins PA6/7 PB0/1/10/11. Regen wiped the ADC1/ADC4
      hardware oversampler — restored. `30afd61`)* + **driver + bus PROVEN**
      *(`lib/qspiflash` + `qspi` cmd; bench over WiFi: id=EF/60/16, `qspi test`
      round-trip PASS. `42b86f5`)*
- [x] **Tier A — transport + QSPI staging (2026-07-07, bench-PROVEN over WiFi):**
      app "Update firmware…" streams a .bin → `lib/ota` stages it in the QSPI
      incoming slot → read-back CRC verify. base64 (dodges the ESP `##..##` magic)
      + per-chunk ACK gate (covers the 256→1024 RX ring + on-the-fly sector
      erase). `otarx`/`ota` cmds; `ota_upload.py` + `connection.py` sniffer.
      Proven: 119672 B staged, `OTARX,DONE,OK crc=1C3730F4`, `ota verify` OK.
      **Cannot brick — no jump.** NEXT = Tier B bootloader.
- [x] **Tier B / B1 — app relocation + jump-only bootloader (2026-07-07, HW-PROVEN):**
      app moved to 0x08008000 (env:app_ota, linker/app_reloc.ld, explicit
      `SCB->VTOR=APP_VTOR_BASE` in main.c USER CODE); 32KB bootloader at
      0x08000000 (`bootloader/` standalone project) validates the app vector +
      jumps. Original 0x08000000 build kept intact as recovery (default_envs).
      Flash both in ONE J-Link session (`tools/flash_ota.jlink`) — separate
      pio uploads mass-erase each other. **BUG found+fixed:** bootloader must
      `__enable_irq()` before the jump or the app inherits PRIMASK=1, SysTick
      never fires, every HAL_Delay hangs (app runs but looks dead — diagnosed
      via J-Link: PC stuck in HAL_GetTick). After fix: full menu/sensors work.
- [ ] B2 bootloader (bottom of flash, never OTA'd): entry via RTC-backup
      flag + reset; receive image over PC4/PC5; CRC-verify in external flash; copy to
      internal app slot; jump. Keep a golden image in external flash for rollback
- [ ] Move app start + set VTOR; ESP-side flashing protocol (reuse chunked transfer);
      app "Update firmware" button. SWD stays as the recovery net
- **Rejected:** ROM bootloader route (needs BOOT0+NRST+PA9/PA10 wires) and dual-bank
  A/B (bank 2 = maze/cal/settings region → would wipe the saved maze)

---

### Suggested start order
**P1 (log reliability) → P2 (sim mazes) → P3 (offline drift analysis + proposed pivot fix).**
Gives a solid brain, clean logs, and a ready-to-bench-verify drift fix for when the maze is back.

### Status of the fast run as of 2026-07-05 (context)
- Front-stop RE-ENABLED for fast runs (Confident + Contact read + fires the cell-mark on stop). 2–3 of 3 recent runs complete the full there-and-back to home.
- Remaining crash cause = heading/lateral DRIFT (skewed arrival) + a right-wheel stall (battery ~6.9 V under load, suspect low-battery motor starve — charge & retest, then check right drivetrain).
- Full saga + all the reverted attempts: see memory `fastrun-emergency-frontstop`.
