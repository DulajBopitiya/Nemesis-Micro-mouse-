# control.c state machine — map (P4 #1)

Read-only map of the motion state machine in `lib/control/control.c`, produced as the
foundation for the P4 cleanup (P4 #2 = remove dead paths / unify front-wall reads;
P4 #3 = prove no behaviour change via build size + `sim`). **Nothing here changes
behaviour** — it documents what is currently live so the cleanup can be surgical.

Line numbers are approximate (as of the hcarry commit `42d72fc`); treat them as
"look near here", not exact anchors.

---

## 1. Two execution contexts

| Context | Runs in | What it does |
|---|---|---|
| **`control_tick()`** | 1 kHz TIM6 ISR (`control.c:914`) | The real-time loop: reads encoders + IMU, runs the mode logic, drives the motors. Sets the one-shot *done* flags. **Never blocks, never touches the console.** |
| **Main-loop tasks** | `Console_Task` → `Control_NavTask()` etc. | Drain done-flags, start the next primitive, print telemetry. Anything that **blocks** (gyro cal ~200 ms) lives here, never in the ISR. |

The `active` flag (`control.c:916`) is the master gate: `false` → the ISR returns
immediately and the motors are left off. `Control_Start*` sets it true; `Control_Stop()`
clears it and every mode flag.

### Layering
```
solver / launcher            (maze logic — above this file)
   └─ Control_NavTask()       sequencer: queue of steps, auto-chains on done-flags   (main loop)
        └─ Control_Start*()   ONE primitive: move / advance / pivot / arc            (entry points)
             └─ control_tick() mode logic + motor mix                                (1 kHz ISR)
```

---

## 2. The ISR pipeline (order matters)

`control_tick()` runs these in sequence every tick. The **sub-mode owners** each
`return` early — they fully own the wheels while active, so the main forward/turn
logic below them does not run.

1. **Master gate** — `if (!active) return;` then battery kill (`Battery_AllowHighCurrent`).
2. **Encoders → velocity** — wrap-safe counts, sliding-window velocity + EMA (`vel_hist`, `vL_filt/vR_filt`).
3. **Sub-mode owners (early-return, priority order):**
   1. `at_active` — auto-tune relay (`autotune_tick`)
   2. `sg_active` — spin-grip test (`spingrip_tick`)
   3. `align_mode` — front-wall square-up (`align_tick`)
   4. `recenter_mode` — reverse to cell centre (`recenter_tick`)
   5. `contact_mode` — creep to wall to measure (`contact_tick`)
4. **Heading fusion** — gyro-Z integral (`heading_deg`) complementary-filtered with the
   encoder-difference heading (`HEAD_COMP_A`). Also the **pickup/lift guard** (low |az|
   for `PICKUP_TICKS` → `Control_Stop`).
5. **`move_mode` block** (§3) — distance-limited straight: stall guards, cell marks,
   front-stop, braking, creep, finish.
6. **`arc_mode` block** (§4) — smooth-turn trapezoidal-ω sweep; may hand off to pivot.
7. **`pivot_mode` block** (§5) — turn-in-place heading slew; completion + `TURNDIAG`.
8. **Setpoint slew** — `ramp_cps` eases toward `target_cps` (accel up / decel down).
9. **Corridor centring** — IR-lateral bias onto `head_setpoint` (straight runs only).
10. **Heading PID → `trim`** — arc path (FF + filtered-error tracker) vs straight/pivot
    path (`pid_head` + rate-error D). Pivot breakaway floor.
11. **Common-mode speed** — `ramp_cps` normally; pivot forward-hold (`PIVOT_FWD_KP`) else.
12. **Mixer → motor duty** — `spL/spR = common ∓ trim`; arc uses a **decoupled** duty
    path (steering straight to duty + fast gyro-rate damping, steering-priority clamp),
    straight/pivot use the per-wheel velocity loop.

---

## 3. `move_mode` — distance-limited straight (the workhorse)

Entered by `Control_StartMove` / `Control_StartAdvance`. `nav_mode` is an **overlay** on
top of `move_mode` that adds per-cell marks + the front-wall early stop (the `advance`/
`cell`/solver path). Plain `move` has `nav_mode=false`.

Key state: `move_target_counts`, `move_braking`, `move_hit_wall`, `move_stalled`,
`move_done_flag/_mm`, `ramp_cps`, `target_cps`.

**Two independent stall guards** (either ends the move with `move_stalled+move_hit_wall`):
- **Hard stall** (`control.c:1006`): commanding `ramp_cps>STALL_MIN_CPS` but `vmag<STALL_SPEED_CPS`
  for `STALL_TICKS` → jammed against a wall.
- **Progress stall** (`control.c:1023`, windowed): must cover `PROG_WIN_MIN` every
  `PROG_WIN_MS`; an inching/skewed jam is caught in ~0.5 s.

**Cell marks** (`nav_mode`, `control.c:1059`): fire `nav_mark_flag` at each cell centre
so the main loop latches walls. Suppressed while `move_creep_en && Walls_FrontContact()`
(fast-run phantom-mark fix — a nose jammed on a wall is not crossing a centre).

**Front-wall stop** (`control.c:1072`): the most-patched region. Ramps `target_cps→0`
(active brake, never collapses the distance target) when a wall is seen ahead:
- **Search** (`move_creep_en=false`): `in_boundary_band && Walls_FrontRaw()` — sensitive
  single-diagonal read, gated to the boundary band (frac `0.30–0.58` of the cell) so a
  mid-cell side-graze phantom is ignored.
- **Fast run** (`move_creep_en=true`): `Walls_FrontConfident() || Walls_FrontContact()`
  — both-diagonals OR either-at-contact; no band gate (works through odometry drift).
  Fires the final cell mark **with** the stop so believed position doesn't lag.

**Braking / creep / finish** (`control.c:1128–1198`):
- `brake` distance = `v²/2a`; when `target-dist ≤ brake` → start braking.
- Optional **active reverse brake** (`brake_rev_cps`) below stop threshold.
- **Creep-to-centre** (fast run only): if slowed to ~rest but still `>MOVE_REACH_MARGIN`
  short and no wall close (`Walls_FrontClose`), creep `MOVE_CREEP_CPS` to land AT centre.
- **Overshoot guard** (fast run): if a wall is ahead and still rolling, hold the finish
  and active-brake instead of coasting into the wall.
- **Finish** (`control.c:1190`): `dist ≥ target` (and not brake-to-stop), OR braked to a
  stop (gated on `reached || wall_block` when creep is on). Sets `move_done_flag`, `Stop`.

---

## 4. `arc_mode` — smooth (flow) turn

Entered by `Control_StartSmoothTurn` (bench `arc`), `Control_StartFlowTurn` (bench
`flowturn`), or `Control_StartFlowTurnInline` (fast-run non-stop corner — the only entry
that does **not** go through `Control_Start`, so it swaps state under `__disable_irq`).

Geometry: optional **entry** lead-in (hold straight `arc_entry_counts`) → **trapezoidal-ω
sweep** (`arc_omega_dps = v/R`, ramps at `arc_alpha_dps2` over `arc_trans_mm` of floor, so
the swept angle integrates to exactly the target at any speed) → **exit**.

Completion has two shapes:
- **Bench arc** (`arc_exit_counts==0`): brake at geometric sweep-complete, settle in place
  (`ARC_SETTLE_TICKS`), distance-backstop + timeout fail-safes.
- **Flow turn** (`arc_exit_counts>0`): brake to a stop at E and require the **measured**
  heading to have arrived. If it's still short at the stop → **pivot-finish handoff**
  (`arc_pivot_finish=true`, flips into the pivot block same tick) to actively spin the
  residual out — the arc trim alone can't beat stiction at ~1 % duty.

---

## 5. `pivot_mode` — turn in place

Entered by `Control_StartPivot` (`turn`/`benchturn`), or reached via the arc pivot-finish
handoff. Forward speed 0; `head_setpoint` slews to `head_target` and the heading PID does
the rotating; common-mode is the forward-hold (`PIVOT_FWD_KP`).

- **Slew**: trapezoidal (`turn_rate_mag` ramps to `turn_rate_dps`, decels within braking
  angle, `TURN_MIN_RATE` creep floor).
- **Completion** (`control.c:1381`): `turn_sp_mag≥tgt && |herr|<tol && |rate|<settle`, OR
  timeout. Tolerances are looser under `pivot_snappy` (fast run early-release).
- Latches `TURNDIAG` diagnostics + (post-hcarry) stashes the carry residual mid-run.
- **hcarry** (see `[[drift-log-analysis]]` / the hcarry commit): a mid-run turn stashes
  `heading_deg - head_target`; the next `Control_Start` seeds `heading_deg` with it so the
  next segment rotates the debt out. Toggle `hcarry` (default ON).

---

## 6. Entry points, done-flags, teardown

**Entry points** (all except the inline flow turn route through `Control_Start`, which
zeros/​seeds state, samples gyro bias unless `seq_skip_gyro_cal`, arms `active`):
`Control_StartMove`, `Control_StartAdvance`, `Control_StartPivot`, `Control_StartSmoothTurn`,
`Control_StartFlowTurn`, `Control_StartFlowTurnInline`, plus sub-modes
`Control_StartFrontAlign / Recenter / ContactProbe / SpinGrip`.

**Done-flag handshake** (ISR sets, main loop drains via `Pop*`):
- `move_done_flag` / `move_done_mm` → `Control_PopMoveDone`
- `turn_done_flag` / `turn_done_deg` → `Control_PopTurnDone`
- `nav_mark_flag` / `nav_mark_cell` → cell-centre wall latch
- `nav_seq_done_flag`, `nav_evt_start_flag` → sequencer progress

**Sequencer** (`Control_NavTask`, `control.c:2450`): runs the `nav_seq[]` queue of
`{ADV|TURN, arg, spd}` steps. Waits for the running step's done-flag, `idx++`, starts the
next `Control_StartAdvance/Pivot` (blocks on gyro cal here, robot stopped). Wrapped by
`Control_RunBegin/RunEnd` (one gyro cal for the whole run; `seq_skip_gyro_cal`).

**Teardown** (`Control_Stop`, `control.c:2519`): clears `active` + **every** mode flag
(`move/nav/pivot/arc/at/sg`), resets `head_setpoint`, restores the gentle `pid_head`
clamp, zeros duty. This is the single funnel every completion/abort passes through.

---

## 7. Observations for the P4 #2 cleanup (candidates — NOT yet changed)

Recorded here so the cleanup has a checklist. Each needs confirming against `sim` + a
build-size diff before touching.

- **Front-wall reads are fragmented** — five predicates in play, each with a distinct
  meaning: `Walls_FrontRaw` (sensitive, search), `Walls_FrontConfident` (both diagonals),
  `Walls_FrontContact` (either at contact), `Walls_Front` (calibrated latched, ~90 mm),
  `Walls_FrontClose` (~45 mm). The move block mixes them by hand at 4+ sites (front-stop,
  creep-block, overshoot guard, mark-suppress). **Unify into named intents** (e.g.
  `front_should_stop()`, `front_blocks_creep()`) without changing the underlying reads.
- **Two stall guards** (hard + windowed-progress) set the identical
  `move_hit_wall+move_stalled+move_done` triplet via copy-paste — factor a small
  `end_move_stalled(dist)` helper.
- **`move_braking` is set from ~4 places** (brake distance, front-stop, brake-to-stop,
  overshoot) — confirm none are redundant now that the overshoot guard exists.
- **`in_boundary_band`** is only consumed by the search front-stop path; verify the fast
  path truly never needs it before simplifying the block.
- **Dead/again toggles** — `arc_kd` default 0, `brake_rev_cps`, `move_creep_en` gating:
  confirm which are live in the shipped profiles vs bench-only, and comment or remove.
- **Magic completion predicate** at `control.c:1190` is a 3-way boolean — a short truth
  table in a comment (or a named helper) would make it reviewable.

None of the above is a bug; they are the patchwork seams from the fast-run debugging
sessions. The safety net for the cleanup is: `sim` regression (host + on-MCU `sim stats`)
+ `pio run` flash-size delta ≈ 0 + a bench `benchturn`/`flowturn` spot check.
