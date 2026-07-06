/**
 ******************************************************************************
 * @file    control.h
 * @brief   Closed-loop motion controller for the Nemisis micromouse.
 *
 * A fixed-rate (1 kHz, TIM6 interrupt) cascade controller for driving straight:
 *
 *     target speed -> [L velocity PID] -> L duty -> PWM (TIM3)
 *                  -> [R velocity PID] -> R duty -> PWM (TIM3)
 *                          ^                 ^
 *                     wheel speed       (TIM1 / TIM2 quadrature)
 *
 *     heading hold -> [heading PID] -> speed trim (+R / -L)
 *                          ^
 *                 fused gyro yaw (ICM-42688) + encoder difference
 *
 * Units:
 *   - wheel speed / setpoints : encoder counts per second (CPS)
 *   - velocity PID output     : motor duty (0..MOTOR_PWM_MAX)
 *   - heading                 : degrees (gyro-integrated, encoder-corrected)
 *   - heading PID output      : speed trim in CPS
 *
 * Everything runs in the TIM6 ISR. Telemetry is buffered there and drained /
 * printed from the main loop (see Control_PopTelem) so the ISR stays light.
 *
 * Tuning is done live from the console / Python app:
 *   drive <cps>            start a straight run at <cps> counts/s
 *   speed <cps>            change target while running
 *   stop                   stop the loop, coast
 *   pid <loop> <kp ki kd>  set gains (loop: vel | lvel | rvel | head)
 *   tlm <loop> [on|off]    stream "TLM,<t_ms>,<setpoint>,<measured>,<output>"
 ******************************************************************************
 */
#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Which control loop a command/telemetry selection refers to. */
typedef enum
{
  CTRL_LOOP_VEL = 0,   /* both wheel velocity loops at once (set only) */
  CTRL_LOOP_LVEL,      /* left wheel velocity                          */
  CTRL_LOOP_RVEL,      /* right wheel velocity                         */
  CTRL_LOOP_HEAD       /* heading / straightness                       */
} CtrlLoop;

/* Peripheral handles the controller drives. Fill from the CubeMX handles. */
typedef struct
{
  TIM_HandleTypeDef *htim_mot;    /* motor PWM        (htim3)             */
  TIM_HandleTypeDef *htim_encL;   /* left  quadrature (htim1)            */
  TIM_HandleTypeDef *htim_encR;   /* right quadrature (htim2)            */
  SPI_HandleTypeDef *hspi_imu;    /* ICM-42688 gyro   (hspi1)            */
} ControlCtx;

/** Bind handles, init motor PWM, and arm the 1 kHz TIM6 loop (idle until a
 *  run is started). Call once after the MX_* inits. */
void Control_Init(const ControlCtx *ctx);

/** Begin a straight run at the given target wheel speed (encoder counts/s).
 *  Re-zeros the integrators, encoders and heading, and re-calibrates the gyro
 *  bias (keep the robot still for ~100 ms). Returns false if blocked by the
 *  battery safety gate. */
bool Control_Start(int target_cps);

/** Stop the loop and coast both motors. Safe to call any time (e.g. 'stop'). */
void Control_Stop(void);

/** Mark the start / end of an autonomous MULTI-STEP run (a 'path' sequence or a
 *  'solve'/fast run). RunBegin samples the gyro bias ONCE (robot must be stationary)
 *  and makes every subsequent Control_Start skip its own ~250 ms recal, so the mouse
 *  doesn't sit dead before each turn/advance. RunEnd re-arms per-start recal for
 *  ordinary single-shot commands. ALWAYS pair them: call RunEnd on every run-end
 *  path (completion AND abort). */
void Control_RunBegin(void);
void Control_RunEnd(void);

/** Change the target speed of an in-progress run (CPS). No effect if idle. */
void Control_SetSpeed(int target_cps);

/** Start a closed-loop straight run that auto-stops after `distance_mm`. Uses
 *  the same velocity + heading-hold controller as Control_Start, but eases the
 *  setpoint back to zero as it nears the target so it brakes to a stop rather
 *  than cutting out at speed. Returns false if blocked by the battery gate. */
bool Control_StartMove(int distance_mm, int target_cps);

/** Returns true exactly once after a move auto-completes, with the distance
 *  actually travelled (mm). Poll from the main loop to announce arrival. */
bool Control_PopMoveDone(int32_t *mm_travelled);

/** True if the last (advance) move was cut short by the front-wall stop rather
 *  than completing its distance. The solver uses this to record the wall at the
 *  CURRENT cell and NOT advance its position (the cause of the 1-cell map shift
 *  when a cell-centre sense misses a wall the front-stop then catches). Valid
 *  after Control_PopMoveDone until the next move starts. */
bool Control_LastMoveHitWall(void);

/** True if the last move ended because the wheels STALLED (jammed against a wall
 *  the sensors missed) rather than a clean front-stop. The solver backs off after
 *  one so the next pivot clears the wall. Valid until the next move starts. */
bool Control_LastMoveStalled(void);

/** Enable/disable the front-wall stop for subsequent advances. The solver turns
 *  it OFF when hopping back into a passage it already drove through (confirmed-
 *  open): there's no wall there, so a front read is the diagonal sensors seeing a
 *  side wall from off-centre - stopping on it would falsely seal the cell. */
void Control_SetFrontStop(bool enable);

/* Snappy pivots (fast run): finish a pivot the instant it's commanded-done and the
 *  heading has essentially arrived, rather than sitting still to settle - the next
 *  cell's forward flow trims the last degree while moving. Removes the per-turn
 *  dead-time. OFF (default) = the tight completion the search uses. */
void Control_SetPivotSnappy(bool on);
bool Control_GetPivotSnappy(void);

/* --- Recenter (reverse a fixed distance) ----------------------------------
 * After a front-stop the mouse is jammed against the wall ahead. Backing up by
 * the distance the blocked advance travelled returns it to the cell centre, so
 * the next turn/advance starts square instead of overshooting into a wall. */

/** Reverse `mm` straight back at a gentle speed, then stop. False for mm<=0 or
 *  if battery-gated. Non-blocking; runs in the 1 kHz loop. */
bool Control_StartRecenter(int mm);

/** True while a recenter is running. */
bool Control_RecenterActive(void);

/** Drains once when a recenter finishes. */
bool Control_PopRecenterDone(void);

/* --- Contact probe ---------------------------------------------------------
 * Creep forward (front-stop OFF) until the front IR reads "wall right in front"
 * (well above its stop-distance ref) or a safety cap, then stop. Used by the
 * front-wall distance calibration to learn centre->wall by driving from the
 * cell centre to the wall and back. Needs 'ircal front'. */
bool Control_StartContactProbe(void);

/** True while a contact probe is creeping. */
bool Control_ContactActive(void);

/** Drains once when the probe stops; mm = distance driven to reach the wall. */
bool Control_PopContactDone(int32_t *mm);

/* --- Cell-based navigation (the `advance` primitive) ----------------------
 * A multi-cell straight that does NOT stop between cells: it cruises the whole
 * corridor and only eases to a stop at the final cell (or earlier if a front
 * wall appears - needs 'ircal front'). At each cell CENTRE it raises a "cell
 * mark" so the caller can latch the walls seen at that decision point. This is
 * the motion primitive the maze solver drives later. */

/** Drive `cells` cells forward continuously at `target_cps`, easing to a stop at
 *  the last cell. Stops early and centred if Walls_Front() trips. Returns false
 *  for cells<=0 or if blocked by the battery gate. */
bool Control_StartAdvance(int cells, int target_cps);

/** Returns true exactly once per cell centre reached during an advance, with the
 *  cell index (1..cells). Poll from the main loop and latch Walls_* there. */
bool Control_PopCellMark(int32_t *cell_index);

/** Brake the in-flight advance to a stop immediately (active brake, same path as
 *  the front-wall stop). Used to halt at a decision cell while flowing through a
 *  straight run; move_done fires once stopped. No effect if no move is running. */
void Control_AdvanceBrakeNow(void);

/** Cell pitch (mm) used by 'advance'. Classic micromouse = 180 mm; set yours for
 *  the home maze. A value <= 0 is ignored. */
void  Control_SetCellMM(float mm);
float Control_GetCellMM(void);

/* --- Nav sequencer (the `path` primitive / maze-solver API) ---------------
 * Queue motion steps (advance N cells / turn D degrees) and run them back-to-
 * back: each step auto-starts when the previous finishes. Build the queue with
 * Reset + Add*, kick it with Start, and pump Control_NavTask() from the main
 * loop. This is what the flood-fill solver will drive; `path` is the manual
 * front-end. Turns still stop-pivot-go; straights within one advance flow. */

/** Clear the step queue (call before building a new sequence). */
void Control_NavReset(void);

/** Append an advance step (drive `cells` cells at `cps`). False if full / cells<=0. */
bool Control_NavAddAdvance(int cells, int cps);

/** Append a pivot step (turn `degrees`, signed; `rate_dps`<=0 = tuned default).
 *  False if the queue is full or degrees==0. */
bool Control_NavAddTurn(int degrees, int rate_dps);

/** Begin executing the queued sequence. False if empty or battery-gated. */
bool Control_NavStart(void);

/** True while a queued sequence is running. */
bool Control_NavActive(void);

/** Abort the sequence and coast (what 'stop' calls). */
void Control_NavAbort(void);

/** Pump the sequencer - call every main-loop pass. Auto-starts the next step
 *  when the current one completes (starting a step briefly blocks on gyro cal,
 *  which is why this is main-loop, not ISR). */
void Control_NavTask(void);

/** Drains once when a step starts: idx (0-based), total steps, type (0=advance,
 *  1=turn), arg (cells or degrees). For progress reporting. */
bool Control_NavPopStepStart(int *idx, int *total, int *type, int *arg);

/** Drains once when the whole sequence ends; ok=false means it was aborted. */
bool Control_NavPopDone(bool *ok);

/** Pivot (turn-in-place) by `degrees` (signed: + / - select the two rotation
 *  senses; which is "left" is determined on the bench). Forward speed is held
 *  at zero while the heading setpoint is slewed to the target at `rate_dps`
 *  (<=0 uses the tunable default) and eased to a stop, with the gyro as the
 *  sole heading source - encoder difference is meaningless while scrubbing.
 *  Reuses the heading PID. Returns false if blocked by the battery gate. */
bool Control_StartPivot(int degrees, int rate_dps);

/** Smooth (arc) turn by `degrees` while driving FORWARD at `target_cps`, curving
 *  along a circle of radius `radius_mm` (heading advances with forward distance:
 *  dtheta = ds/R). Unlike a pivot the mouse keeps cruising through the corner -
 *  the basis of a fast run. `exit_mm`: 0 = stop when the arc completes (bench
 *  geometry test); >0 = re-zero the heading to the NEW direction and drive on
 *  that many mm WITHOUT stopping (the no-stop hand-off into the next straight, so
 *  speed is carried through the corner). Completion (the arc sweep) is reported
 *  via Control_PopTurnDone. Returns false on a bad arg or the battery gate. */
bool Control_StartSmoothTurn(int degrees, int target_cps, int radius_mm, int exit_mm);

/** Flow (maze) smooth turn: lead-in straight (entry_mm) -> arc through <degrees> at
 *  radius_mm -> lead-out straight (exit_mm). The lead-in starts the curve a radius
 *  BEFORE the corner-cell centre so the mouse lands centred on the next corridor.
 *  cont=false stops centred at the end (bench geometry test via 'flowturn');
 *  cont=true re-zeros the heading and resumes a normal forward flow at the end (the
 *  fast run flows on without stopping). Returns false on a bad arg / battery gate. */
bool Control_StartFlowTurn(int degrees, int target_cps, int radius_mm,
                           int entry_mm, int exit_mm, bool cont);

/** INLINE flow turn - same as Control_StartFlowTurn but starts FROM THE CURRENT
 *  MOTION (no stop, speed/gyro preserved), for the fast-run solver to curve through
 *  a corner non-stop. Must be called while a run is active and the mouse is flowing
 *  straight. cont=true resumes the forward flow at the end. */
bool Control_StartFlowTurnInline(int degrees, int cps, int radius_mm,
                                 int entry_mm, int exit_mm, bool cont);

/** Returns true exactly once after a pivot/arc auto-completes, with the heading
 *  actually reached (deg). Poll from the main loop to announce arrival. */
bool Control_PopTurnDone(int32_t *deg_turned);

/** Diagnostics latched at the last turn completion (call right after
 *  Control_PopTurnDone). Angles/rate in deci-units (x10); reason 0=clean,
 *  1=snappy early-release, 2=timeout. Any pointer may be NULL. */
void Control_GetLastTurnDiag(int32_t *cmd_ddeg, int32_t *ach_ddeg,
                             int32_t *peak_ddps, int32_t *ms, int32_t *reason,
                             int32_t *posL, int32_t *posR);

/** Turn-rate slew cap (deg/s) and accel (deg/s^2) for pivots. Pass a value
 *  <= 0 to leave that setting unchanged. */
void Control_SetTurn(float rate_dps, float accel_dps2);
void Control_GetTurn(float *rate_dps, float *accel_dps2);

/** Autonomous-run turn type: 0 = pivot (turn in place), 1 = smooth (arc) turns.
 *  Set by the active profile; read by the solver to choose how it takes corners. */
void Control_SetTurnMode(int mode);
int  Control_GetTurnMode(void);

/** Smooth-turn geometry (mm): arc radius, lead-in straight before the arc, and
 *  lead-out straight after it. For a ~180 mm cell, R=90 and entry=exit=90 land the
 *  mouse centred in the next cell. Pass <0 to leave a field unchanged. */
void Control_SetArcGeom(float radius_mm, float entry_mm, float exit_mm);
void Control_GetArcGeom(float *radius_mm, float *entry_mm, float *exit_mm);

/** Smooth-turn transition length (mm): the floor distance over which the turn
 *  rate ramps 0<->omega at each end of the curve. The angular accel is derived
 *  from it at every arc start (alpha = omega*v/L), which is what makes the drawn
 *  path speed-invariant. Shorter = snappier/closer to a pure arc but harder on
 *  grip; longer = gentler but eats more of the entry/exit legs. Clamped 2..80. */
void  Control_SetArcTrans(float mm);
float Control_GetArcTrans(void);

/** Smooth-arc curvature feed-forward (cps trim per deg/s of turn rate). Raise
 *  until the heading tracks the arc without lag/hunting; 0 = pure PID. SetArcFF
 *  sets BOTH directions; use the L/R setters for the left/right turn asymmetry
 *  (a real mouse rotates more easily one way - left FF is typically lower). */
void  Control_SetArcFF(float ff);
float Control_GetArcFF(void);
void  Control_SetArcFFLeft(float ff);
void  Control_SetArcFFRight(float ff);
float Control_GetArcFFLeft(void);
float Control_GetArcFFRight(void);

/** Arc heading-correction gain (cps trim per deg of error). Gentle: the curvature
 *  feed-forward carries the turn, this only nulls slow drift (no hunt, no buzz). */
void  Control_SetArcKp(float kp);
float Control_GetArcKp(void);

/** Arc heading damping gain (derivative on the filtered error). Raise to remove
 *  turn overshoot/hunt; filtered, so it doesn't bring back the motor buzz. */
void  Control_SetArcKd(float kd);
float Control_GetArcKd(void);

/** Arc gyro-rate damping (motor duty per deg/s of rate error). The fast,
 *  direct-to-motor damping that bypasses the velocity-loop lag - the real buzz fix. */
void  Control_SetArcGyroKd(float k);
float Control_GetArcGyroKd(void);

/** Heading-PID derivative low-pass (EMA coeff 0..1). Lower = smoother (kills the
 *  gyro-noise motor buzz) but laggier D; 1.0 = raw gyro derivative. */
void  Control_SetHeadDFilt(float alpha);
float Control_GetHeadDFilt(void);

/* --- Gyro turn-scale calibration ------------------------------------------
 * The heading is gyro-integrated with a sensitivity constant (LSB per dps).
 * Part-to-part error makes every turn land a fixed % off, skewing the mouse at
 * each pivot. Correct it live: command a known rotation ('turn 360'), measure
 * the real physical angle, and feed it to Control_TurnCalFromActual(). */

/** Current gyro sensitivity (LSB per dps). */
float Control_GetGyroScale(void);

/** Set the gyro sensitivity directly (clamped to a sane 8..33 LSB/dps). */
void  Control_SetGyroScale(float lsb_per_dps);

/** Correct the gyro scale from the last commanded pivot and its measured actual
 *  angle (scale *= commanded/actual). False if there was no prior turn or the
 *  result is implausible. The corrected value is RAM-only - bake it into the
 *  GYRO_LSB_PER_DPS default once found so it survives a power cycle. */
bool  Control_TurnCalFromActual(float actual_deg);

/* --- Front-wall align -----------------------------------------------------
 * Square the mouse to a wall ahead using the two front-diagonal IR sensors:
 * rotate to null (L_F - R_F) and creep to the calibrated stop distance. This
 * re-zeros heading + forward position against an absolute reference, cancelling
 * accumulated turn drift - run it before a pivot when a front wall is present.
 * Non-blocking (runs in the 1 kHz loop); needs 'ircal front'. */

/** Start squaring to the wall ahead. False if battery-gated or front isn't
 *  calibrated. The mouse must already be facing a wall within IR range. */
bool Control_StartFrontAlign(void);

/** True while an align is running. */
bool Control_AlignActive(void);

/** Drains once when an align finishes; ok currently always true (it settles or
 *  times out at "good enough"). */
bool Control_PopAlignDone(bool *ok);

/** Align gains (cps per count): skew = rotate-to-square, dist = creep-to-range.
 *  Tune on the bench with 'acfg'. Pass <= 0 to leave a gain unchanged. */
void Control_SetAlign(float kp_skew, float kp_dist);
void Control_GetAlign(float *kp_skew, float *kp_dist);

/** True while a closed-loop run is active. */
bool Control_IsActive(void);

/** Current mean wheel speed magnitude (encoder counts/s), 0 when idle. For the
 *  speed-adaptive IR sensor timing (faster sweep when moving fast). */
float Control_GetWheelSpeedCps(void);

/** Calibrated odometry scale: encoder counts per millimetre of wheel travel.
 *  For distance<->counts maths outside the controller (e.g. the grip test
 *  converting a braking distance in counts to mm). */
float Control_GetCountsPerMM(void);

/* --- Spin-and-lock rotational grip test -----------------------------------
 * Measures downforce via GRIP, not motor torque (which dominated the braking
 * test): spin the mouse up in place to a set yaw rate, then LOCK the wheels
 * (Motor_Brake) so the spinning body skids the tyres across the floor. The skid
 * deceleration is friction-limited (mu*N), so it rises with downforce - and being
 * a locked skid, motor/gearbox freewheel drag doesn't enter. In place -> no
 * runway. Grip cancels in the ratio: F_down = weight*(decel_on/decel_off - 1). */

/** Spin up to `spin_dps` (<=0 uses a default), lock, and measure the skid decel.
 *  Re-calibrates the gyro bias (keep still until it spins). Battery-gated. */
bool  Control_StartSpinGrip(int spin_dps);

/** True while a spin-grip test is running. */
bool  Control_SpinGripActive(void);

/** Drains once when the test finishes: ok=false if it never reached the target
 *  rate. decel_dps2 = skid deceleration (deg/s per s), omega0_dps = spin rate at
 *  the moment the wheels locked. */
bool  Control_PopSpinGripResult(bool *ok, float *decel_dps2, float *omega0_dps);

/** True once the robot has been lifted/tilted off the floor during a run (the
 *  vertical accel collapsed). Latched until the next Control_Start. The solver
 *  uses it to stop the run and keep the map for dumping. */
bool Control_PickupDetected(void);

/** Snapshot of live odometry/IMU for the run trace logger: wheel positions
 *  (encoder counts), heading (deg), gyro-Z (deg/s), accel-Z (g). Any pointer may
 *  be NULL. Values are the last seen by the 1 kHz loop. */
void Control_GetState(int32_t *posL, int32_t *posR, float *heading_deg,
                      float *gyroz_dps, float *accelz_g);

/** Set PID gains for a loop. CTRL_LOOP_VEL sets both velocity loops. */
void Control_SetGains(CtrlLoop loop, float kp, float ki, float kd);

/** Read back a loop's gains (CTRL_LOOP_VEL reports the left loop). */
void Control_GetGains(CtrlLoop loop, float *kp, float *ki, float *kd);

/** Velocity feedforward gain (motor duty per cps of setpoint). Carries the bulk
 *  of the drive so the velocity PIDs only trim the error. */
void  Control_SetVelFF(float kff);
float Control_GetVelFF(void);

/** Corridor centring (IR wall-following). When enabled, the side IR sensors'
 *  lateral error biases the heading setpoint on forward runs so the mouse holds
 *  the corridor centre (needs 'ircal side'; no walls -> plain heading-hold).
 *  Off by default, sticky across runs. */
void Control_SetFollow(bool on);
bool Control_GetFollow(void);

/** Centring gain: heading-offset degrees per count of IR lateral error. Small
 *  (~0.05) to start. A negative value flips the steer direction; magnitude
 *  clamped to 1.0. */
void  Control_SetCenterGain(float kp);
float Control_GetCenterGain(void);

/** Centring DAMPING: heading-offset degrees per (lateral count / second). The
 *  derivative half of the PD follower that damps the search-run weave. 0 = plain
 *  proportional centring (old behaviour). Clamped to [0, 1]. */
void  Control_SetCenterDamp(float kd);
float Control_GetCenterDamp(void);

/** Setpoint ramp rates in cps/s: how fast the target is allowed to speed up
 *  (accel) and slow down (decel, also used for the move-stop braking). Pass a
 *  value <= 0 to leave that rate unchanged. */
void Control_SetAccel(float accel, float decel);
void Control_GetAccel(float *accel, float *decel);

/** Active reverse brake: while a distance move is braking and still rolling, aim
 *  the velocity setpoint this many cps BELOW zero so the motors reverse-torque for
 *  a real (grip-limited) stop instead of coasting. 0 disables it (default). The
 *  fast run arms it; tune on the bench with the `brake` console command. */
void  Control_SetBrakeReverse(float cps);
float Control_GetBrakeReverse(void);

/** Enable "creep-to-centre" on a distance-move stop: if a hard brake arrests the
 *  mouse short of the target, creep the last few mm to reach it (so a following
 *  pivot is centred). FAST RUN ONLY - leave OFF for search/normal moves, where it
 *  would creep into walls and hang. */
void  Control_SetReachCreep(bool en);

/** Velocity-feedback filter: sliding-window length (2..50 ms) and EMA smoothing
 *  (0..1; smaller = smoother but more lag). Heavier filtering cleans up the
 *  measured-velocity noise the loop reacts to, at the cost of phase lag. Pass
 *  an out-of-range value to leave that setting unchanged. */
void Control_SetVelFilter(int window_ms, float lpf_alpha);
void Control_GetVelFilter(int *window_ms, float *lpf_alpha);

/* --- Relay (Astrom-Hagglund) velocity-loop auto-tune ----------------------
 * Oscillates BOTH wheels around target_cps with a bang-bang relay, measures the
 * loop's ultimate gain (Ku) and period (Tu), and derives PI gains. Runs inside
 * the 1 kHz loop (non-blocking); the robot drives forward in a gentle limit
 * cycle while it measures, so keep it on the floor with room ahead. The result
 * is polled from the main loop - the console applies it to the velocity loop. */

/** Start a relay auto-tune at the given speed (cps; <500 uses a default).
 *  Re-calibrates the gyro, so keep the robot still until it starts moving.
 *  Returns false if blocked by the battery gate. */
bool Control_AutotuneStart(int target_cps);

/** True while a relay auto-tune is running. */
bool Control_AutotuneActive(void);

/** Returns true exactly once after a tune finishes. ok is false if the test
 *  failed (timed out / no clean oscillation); otherwise kp,ki,kd hold the
 *  suggested gains and ku,tu_ms the measured ultimate gain / period (ms).
 *  Gains are NOT applied here - the caller decides. */
bool Control_AutotunePopResult(bool *ok, float *kp, float *ki, float *kd,
                               float *ku, float *tu_ms);

/** Select which loop telemetry follows, and turn streaming on/off. */
void Control_SetTelem(CtrlLoop loop, bool on);

/** Pop one buffered telemetry record (filled by the ISR). Returns false when
 *  empty. Call from the main loop and print as "TLM,t,sp,meas,out". */
bool Control_PopTelem(uint32_t *t_ms, int32_t *setpoint,
                      int32_t *measured, int32_t *output);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
