/**
 ******************************************************************************
 * @file    control.c
 * @brief   1 kHz cascade motion controller - see control.h.
 ******************************************************************************
 */
#include "control.h"
#include "motor.h"
#include "icm42688.h"
#include "battery.h"
#include "sensors.h"
#include "dbg.h"
#include <math.h>

/* ===========================================================================
 *  Tunable wiring / geometry. These are the things to flip first on the bench
 *  if the robot fights itself instead of driving straight.
 * ========================================================================= */
#define CTRL_HZ            1000U          /* loop rate (TIM6)                  */
#define CTRL_DT            (1.0f / (float)CTRL_HZ)
#define CTRL_TELEM_DECIM   10U            /* push telem every Nth tick (~100Hz)*/
#define CTRL_ACCEL_CPS     30000.0f       /* default setpoint slew rate (cps/s) */
/* Velocity estimation. A 1 ms encoder delta is only a few counts, so per-tick
   velocity is quantised in CTRL_HZ-sized steps (~1000 cps) - far too noisy.
   We instead measure over a sliding window of `vel_win` ms (resolution
   CTRL_HZ/vel_win), then apply an EMA to smooth it. Both the window length and
   the EMA are live-tunable (`vfilt`) so smoothness vs lag can be traded per
   speed - at higher speed there's plenty of lag budget to filter harder. */
#define VEL_WIN_MAX        64U             /* ring capacity (max window + slack) */
#define VEL_WIN_DEFAULT    20U             /* default window length (ticks = ms) */
#define VEL_LPF_DEFAULT    0.30f           /* default EMA smoothing              */

/* Wheel pairing: MOTOR_1 <-> left (TIM1), MOTOR_2 <-> right (TIM2).
   If a wheel spins the wrong way or its encoder counts backwards, flip the
   matching sign below (these are the usual "it pivots instead of driving"
   fixes; no re-flash of CubeMX needed). */
#define ENC_L_SIGN         (-1)           /* L counts DOWN on forward -> flip   */
#define ENC_R_SIGN         (+1)           /* R counts UP on forward -> keep     */
#define MOT_L_SIGN         (+1)           /* make +duty drive forward (L)       */
#define MOT_R_SIGN         (+1)           /* make +duty drive forward (R)       */

/* === drivetrain geometry ==================================================
 * FOUR-wheel drive, but only TWO motors: each DRV8874 drives one SIDE's wheel
 * pair (skid / tank steer), so the controller stays 2-channel (left vs right).
 * All four wheels scrub sideways when the robot rotates, so encoder-difference
 * heading is unreliable here - gyro stays the primary heading source.
 *
 * Layout: motor -> middle gear -> front & back wheel gears (13 mm centre-to-
 * centre each side), so the two wheels on a side turn together and one encoder
 * per side represents that side's travel. The AS5047P encoders sit on the FRONT
 * WHEEL shaft, so they read true wheel rotation and the 72:10 reduction does
 * NOT enter counts-per-wheel-rev (ENC_ON_MOTOR = 0).
 *
 * Counts <-> distance: the AS5047P ABI outputs feed TIM1/TIM2 in x4 quadrature.
 *   counts/wheel-rev = ENC_ABI_CPR        (encoder is on the wheel)
 *   counts/mm        = counts-per-rev / (pi * wheel diameter)
 * CALIBRATED 2026-06-25 by rolling the robot exactly 1 m on the `encoder`
 * stream: L moved 24316, R moved 24712 counts -> avg 24514 counts/m =
 * 24.51 counts/mm -> ENC_ABI_CPR = 24.514 * pi * 25 ~= 1925 (a 500-line
 * encoder x4, not the 1000 I first assumed). So 1 m/s = ~24500 cps. */
#define GEAR_RATIO          7.2f     /* 72:10 motor:wheel reduction            */
#define MOTOR_MAX_RPM       51000.0f /* motor shaft no-load max                */
#define ENC_ABI_CPR         1925.0f  /* counts per WHEEL rev - calibrated (1m)  */
#define ENC_ON_MOTOR        0        /* encoder is on the front wheel shaft     */
#define WHEEL_DIAMETER_MM   25.0f    /* wheel diameter                          */

#define COUNTS_PER_WHEELREV (ENC_ABI_CPR * (ENC_ON_MOTOR ? GEAR_RATIO : 1.0f))
/* counts per mm of travel (valid once WHEEL_DIAMETER_MM is set; else 0). */
#define COUNTS_PER_MM       ((WHEEL_DIAMETER_MM > 0.0f) \
        ? (COUNTS_PER_WHEELREV / (3.14159265f * WHEEL_DIAMETER_MM)) : 0.0f)
/* theoretical top wheel speed in counts/s (no-load), for limits / reference. */
#define MAX_WHEEL_CPS       (COUNTS_PER_WHEELREV * (MOTOR_MAX_RPM / GEAR_RATIO) / 60.0f)

/* Heading fusion. Gyro is primary; the encoder-difference estimate slowly
   corrects its drift. HEAD_COMP_A is the gyro weight (1.0 = gyro only).
   CTRL_ENC_COUNTS_PER_DEG converts (R-L) count difference to degrees of yaw
   and MUST be calibrated for the encoder term to mean anything - spin the
   mouse 360 deg by hand, read the count difference, divide by 360. Until then
   leave HEAD_COMP_A at 1.0 so a wrong scale can't bias the heading (this is
   the right default for a scrubbing 4WD anyway). */
/* ICM-42688 +/-2000 dps FS: datasheet nominal is 16.4 LSB/dps. 16.537 is the
   bench-calibrated value for THIS part (2026-06-26, via 'gyroscale cal' against
   physical 360/180/90 turns) so a commanded pivot lands at the true angle.
   Re-cal with 'gyroscale cal <actual_deg>' if the board/IMU changes. */
#define GYRO_LSB_PER_DPS         16.537f  /* bench-calibrated (was 16.4 nominal) */
#define ACCEL_LSB_PER_G          2048.0f  /* ICM-42688 default +/-16 g           */
#define HEAD_COMP_A              1.0f     /* gyro weight (set <1 once cal'd)    */
#define CTRL_ENC_COUNTS_PER_DEG  10.0f    /* PLACEHOLDER - calibrate me         */
#define HEAD_TRIM_SIGN           (+1)     /* flip if heading correction diverges*/
#define HEAD_TRIM_MAX_CPS        6000.0f  /* clamp on the straightness trim     */
/* Heading-PID derivative low-pass (EMA coeff, 0..1). The D-term is effectively
   the raw gyro rate (heading_deg is a pure gyro integral); unfiltered it pumps
   gyro/vibration noise straight into the wheels -> the motor buzz. ~0.08 (fc
   ~13 Hz at 1 kHz) cuts the ~7-10 Hz gizzel while keeping the damping. Smaller =
   smoother but laggier D; 1.0 = old raw behaviour. Live-tunable via 'dfilt'. */
#define HEAD_D_ALPHA_DEFAULT     0.08f
/* Pickup/lift detection (ICM-42688 accel, +/-16 g => 2048 LSB/g). When |accel_z|
   stays below ~0.6 g this long the board is tilted off the floor. */
#define PICKUP_AZ_LSB            1200     /* |az| below this (~0.6 g) = tilted   */
#define PICKUP_TICKS             150U     /* ms of sustained tilt = lifted       */

/* Corridor centring (IR wall-following). The side IR sensors give a signed
   lateral error (Sensors_Lateral: >0 = drifted toward the LEFT wall); we turn
   that into a small bias on the HEADING setpoint, so the heading PID above
   steers the mouse back to the corridor centre. It's the outer-most loop of the
   cascade (lateral -> heading -> velocity) and only acts on forward runs with a
   wall to follow (no walls -> error 0 -> plain heading-hold). Opt-in via
   'follow on'; gain tuned via 'fcfg'. CENTER_SIGN flips if it steers INTO the
   wall instead of away (same idea as HEAD_TRIM_SIGN). */
#define CENTER_SIGN              (-1)     /* -1: lateral>0 (toward left) -> steer right */
#define CENTER_KP_DEFAULT        0.05f    /* heading-offset (deg) per lateral count     */
/* Centring DAMPING (deg of heading-offset per lateral-count/second). A P-only wall
   follower limit-cycles - it reaches centre still angled inward, so it crosses to
   the far wall and back: the small "wobble" on the search run. A derivative term
   (PD) anticipates the approach and eases the steering off early, killing the weave.
   Taken on a low-pass-filtered derivative (same anti-noise trick as the arc tracker)
   so it damps without pumping IR noise. ~kp*30ms of lead; set 0 via 'fcfg' to get
   the exact old proportional-only behaviour back. */
#define CENTER_KD_DEFAULT        0.0015f
#define CENTER_D_ALPHA           0.25f    /* low-pass on the lateral derivative         */
#define CENTER_OFFSET_MAX        20.0f    /* clamp on the centring heading offset (deg) */

/* Pivot (turn-in-place) defaults. rate/accel shape the heading-SETPOINT slew
   (a trapezoidal angle profile); the heading PID tracks it, so the robot turns
   only as fast as the setpoint leads it. tol/settle define a clean finish; the
   creep floor keeps the setpoint inching to the target through the decel so a
   turn can't asymptotically stall just short. All live-tunable via 'turn'. */
#define TURN_RATE_DEFAULT        300.0f   /* deg/s setpoint slew cap            */
#define TURN_ACCEL_DEFAULT       1800.0f  /* deg/s^2 setpoint accel / decel     */
#define TURN_MIN_RATE            30.0f    /* deg/s creep floor near the target  */
#define TURN_TOL_DEG             2.0f     /* "arrived" heading tolerance        */
#define TURN_SETTLE_DPS          20.0f    /* "stopped rotating" gyro threshold  */
/* SNAPPY pivot completion (fast run): finish the pivot as soon as the commanded
   rotation is done and the heading has essentially arrived, WITHOUT waiting for
   the mouse to sit dead-still (grate<20). The next cell's forward flow then starts
   and its heading-hold + wall-follow trim the last degree or two while moving - a
   pivot ends at ~zero forward speed so there's no momentum to fight (unlike the
   arc hand-off). Removes the per-turn "sit and fine-adjust" dead-time. Enabled
   only for fast runs (Control_SetPivotSnappy); search keeps the tight completion. */
#define PIVOT_SNAP_TOL_DEG       4.0f     /* fast: arrived within this (vs 2)      */
/* Fast-run pivot "settled enough to release" gyro-rate threshold. Was 45: the
   pivot finished while still rotating up to 45 deg/s, so the following straight
   started with leftover spin and the flow accel turned it into a ~10-12 deg yaw
   KICK at cell entry (log 20260704-210108) - the wobble/occasional wall clip.
   Lowered to 25 (still above search's 20) so the pivot sheds most of that residual
   before the straight takes over, without sitting to fully settle. Fast run only
   (Control_SetPivotSnappy); search uses TURN_SETTLE_DPS and is unaffected. */
#define PIVOT_SNAP_SETTLE_DPS    25.0f    /* fast: finish while still easing (vs 20)*/
/* Smooth-turn TRANSITION LENGTH (mm): forward distance over which the turn rate
   ramps 0 -> omega entering the curve (and back down leaving it). The angular
   accel is DERIVED from it at every start (alpha = omega * v / L), so the ramp
   always spans the same PHYSICAL length of floor regardless of speed - the
   drawn path (clothoid-arc-clothoid) is then identical at any cruise. This is
   the canonical trapezoidal-omega scaling (micromouseonline "how speed affects
   the smooth turn"); the old fixed ARC_EASE_DEG ease-out was TIME-based, so a
   faster run swept a physically different arc = the under-rotation crashes.
   Live-tunable via 'arctrans'. */
#define ARC_TRANS_MM_DEFAULT     20.0f
/* Distance overrun past the ideal curve length before the sweep setpoint is
   FORCED to the full angle. With the trapezoid, sweep and distance complete
   together by construction; this fires only on real wheel slip (fail-safe),
   never in normal tracking - forcing at exactly the ideal length re-created
   the old setpoint step the robot couldn't follow. */
#define ARC_SLIP_MARGIN_MM       15.0f
#define TURN_TIMEOUT_MS          4000U    /* finish anyway if it stalls         */
/* During a pivot the heading PID gets a HIGHER trim clamp than the straight-run
   straightness trim (the robot can spin its wheels far faster than the gentle
   ±6000 correction trim) - this is what lets a turn actually be quick instead of
   capping out. And a breakaway floor: beyond PIVOT_FLOOR_DEADBAND of the target
   the trim is forced to at least PIVOT_MIN_TRIM so the last stretch punches
   through stiction instead of crawling; inside the deadband it releases for a
   clean stop. (Straight-line driving is untouched - this is all pivot-only.) */
#define PIVOT_TRIM_MAX_CPS       16000.0f /* heading-PID trim clamp while pivoting */
#define PIVOT_MIN_TRIM           2200.0f  /* breakaway floor (cps) beyond deadband */
#define PIVOT_FLOOR_DEADBAND     3.0f     /* deg: release the floor this close in  */
/* Forward-hold: a pure pivot shouldn't translate, so net forward travel
   0.5*(posL+posR) should stay ~0. A gentle P term on that injects a common-mode
   (added equally to both wheels) that cancels forward/back creep, so the robot
   pivots about its centre instead of drifting off its spot. Decoupled from the
   rotation (that's the differential trim). Set KP to 0 to disable. Lateral scrub
   is invisible to the encoders - that's for the IR wall-centring phase. */
#define PIVOT_FWD_KP             4.0f     /* common-mode cps per count of fwd drift */
#define PIVOT_FWD_MAX_CPS        3000.0f  /* clamp on the forward-hold correction   */

/* ===========================================================================
 *  PID
 * ========================================================================= */
typedef struct
{
  float kp, ki, kd;
  float integ;         /* integral accumulator (pre-Ki)                       */
  float prev_meas;     /* for derivative-on-measurement (no setpoint kick)    */
  float dfilt;         /* low-pass-filtered derivative (anti-noise)           */
  float d_alpha;       /* derivative EMA coeff 0..1; 1 = raw (old behaviour)  */
  float out_min, out_max;
} PID;

static void pid_reset(PID *p)
{
  p->integ = 0.0f;
  p->prev_meas = 0.0f;
  p->dfilt = 0.0f;
}

/* PID with an optional feedforward bias `ff` (added before clamping; the
   anti-windup back-calc accounts for it). ff carries the bulk of the drive so
   the PID only trims the residual error -> less overshoot, less hunting. */
static float pid_step(PID *p, float setpoint, float meas, float ff)
{
  float err = setpoint - meas;
  p->integ += err * CTRL_DT;

  float pterm = p->kp * err;
  float iterm = p->ki * p->integ;

  /* Derivative on measurement, LOW-PASS FILTERED. Differentiating the measurement
     and dividing by CTRL_DT (=1ms -> x1000) blows up sensor noise into the output.
     For the heading loop heading_deg is a pure gyro integral, so this derivative
     is LITERALLY the raw gyro rate (incl. its noise + motor-vibration coupling) fed
     straight into the wheel differential at kd -> the ~7-10 Hz motor "gizzel"/buzz
     (a derivative-driven limit cycle). An EMA on the derivative rolls that hash off
     while keeping the low-frequency damping. d_alpha = 1 reproduces the old raw
     derivative exactly (velocity loops use that; their kd is 0 anyway). */
  float draw  = (meas - p->prev_meas) / CTRL_DT;
  p->prev_meas = meas;
  p->dfilt += p->d_alpha * (draw - p->dfilt);
  float dterm = -p->kd * p->dfilt;

  float out = ff + pterm + iterm + dterm;

  /* Clamp with integral back-calculation so we don't wind up at the rail. */
  if (out > p->out_max)
  {
    out = p->out_max;
    if (p->ki != 0.0f) p->integ = (out - ff - pterm - dterm) / p->ki;
  }
  else if (out < p->out_min)
  {
    out = p->out_min;
    if (p->ki != 0.0f) p->integ = (out - ff - pterm - dterm) / p->ki;
  }
  return out;
}

/* ===========================================================================
 *  Module state
 * ========================================================================= */
static ControlCtx       CT;
static TIM_HandleTypeDef htim6;          /* owned here, set up in Control_Init */

static volatile bool    active;
static volatile float   target_cps;       /* commanded target (from drive/speed)*/
static float            ramp_cps;          /* slew-limited setpoint actually used*/
static volatile float   accel_cps;         /* speeding-up slew rate (cps/s)      */
static volatile float   decel_cps;         /* slowing-down slew rate (cps/s)     */
static volatile float   brake_rev_cps;     /* active reverse-brake floor (cps); 0=off.
                                              When a distance move is braking and still
                                              rolling, drive the setpoint this far
                                              NEGATIVE so the motors reverse-torque
                                              (real decel) instead of coasting to 0.
                                              Tune with 'brake'; the fast run enables it
                                              (vacuum grip lets the tyres take it).     */
static volatile bool    move_creep_en;      /* creep-to-centre on move stop (FAST RUN
                                              only). Off for search/normal moves - there
                                              it would creep into walls + hang.         */

/* Distance-limited move (the `move <mm>` command). */
#define MOVE_STOP_CPS      200.0f          /* "stopped" threshold for move end. Was */
                                           /* 400: a move ended (and Control_Stop   */
                                           /* coasted the rest) while still rolling */
                                           /* ~15 mm/s, so a fast front-wall approach*/
                                           /* coasted the last bit INTO the wall.   */
                                           /* Lower = brake nearer to rest first.   */
/* Creep-to-target: a hard (reverse) brake at speed can arrest the mouse SHORT of
   the move target, so a following pivot happens off the cell centre and clips the
   wall. If we've slowed to ~stopped but are still more than REACH_MARGIN short,
   creep gently forward to close the gap so the move ends AT the target centre. */
#define MOVE_REACH_MARGIN  110.0f          /* counts (~4.5 mm) = "at the target"    */
#define MOVE_CREEP_CPS    3000.0f          /* gentle close-the-gap speed (~0.12 m/s)*/
/* Stall guard: if we're commanding the wheels forward (ramp above STALL_MIN_CPS)
   but they're barely turning (measured speed below STALL_SPEED_CPS) for
   STALL_TICKS ms, we're jammed against a wall the sense/front-stop missed - end
   the move as a wall hit instead of grinding. The window is long enough that the
   normal accel-from-rest transient (filter lag) can't trip it. */
#define STALL_MIN_CPS     4000.0f          /* ramp clearly commanding forward     */
#define STALL_SPEED_CPS    800.0f          /* below this = wheels not turning      */
#define STALL_TICKS        150U            /* ms jammed before we call it a wall   */
/* Progress stall (WINDOWED): the speed guard above misses a jam where the mouse is
   SKEWED against a wall and INCHES forward ~1mm at a time - one wheel nudges just
   enough that vmag keeps swinging up AND a best-ever distance tracker keeps resetting,
   so the move hangs stuck 4s+ against the wall (logs 20260705-011018/010846). This
   guard keys on NET progress over a sliding window: a move must cover PROG_WIN_MIN
   (~10mm) every PROG_WIN_MS; if it doesn't, the mouse is wedged - end it as a wall hit
   + stall so the solver backs off. Real motion (even the slow creep, ~300cts/100ms)
   clears PROG_WIN_MIN long before the window elapses, so it CAN'T false-trip; only a
   wedged mouse holds still. Applies to search + fast alike (a jam is a jam). */
#define PROG_WIN_MIN       250.0f          /* counts (~10mm) net travel = still moving */
#define PROG_WIN_MS        500U            /* ms below that = wedged, bail out         */
/* Per-segment speed cap: fraction of the COMMANDED decel assumed actually achievable
   on dry (slipping) wheels, used to cap the fast-run cruise so a short turn-approach
   is only entered as fast as it can be braked. Lower = more braking margin (slower
   short segments); 1.0 = trust the commanded decel fully. See Control_StartAdvance. */
#define VPROF_DECEL_FRAC     0.6f
static volatile bool    move_mode;
static float            move_target_counts;
static volatile bool    move_braking;      /* true once the stop ramp has begun  */
static volatile bool    move_hit_wall;     /* this move was stopped by a front wall */
static volatile bool    move_stalled;      /* ended by the stall guard (jammed)   */
static uint32_t         stall_ticks;       /* consecutive jammed ticks            */
static float            stall_prog_max;    /* best dist reached this move          */
static uint32_t         stall_prog_ticks;  /* ms without new forward progress      */
static volatile bool    front_stop_en = true;  /* solver disables for known-open hops */
static volatile bool    move_done_flag;    /* set on arrival, drained by main    */
static volatile int32_t move_done_mm;

/* Cell-based navigation (the `advance`/`cell` commands). A multi-cell straight
   that does NOT stop at each cell boundary - it cruises the whole run and only
   eases to a stop at the final cell (or when a front wall appears), firing a
   "cell mark" event each time it reaches a cell CENTRE so the main loop can
   latch the walls seen there (the per-cell decision point). Rides on top of the
   move machinery (move_mode does the distance + braking); nav just adds the
   per-cell marks and the front-wall early stop. The maze solver sits above this
   later - this is only the motion primitive. */
#define NAV_CELL_MM_DEFAULT  180.0f        /* classic micromouse cell pitch (mm) */
/* Front-stop is only honoured while APPROACHING a cell boundary (where a real
   blocking wall sits), expressed as a fraction of the cell pitch from the last
   cell centre. The boundary is at 0.5; a wall there must brake us, so we open the
   window a little before it. PAST ~0.58 the mouse has crossed into the next cell,
   where the front-DIAGONAL IR grazes that cell's SIDE walls and falsely reads a
   wall ahead (the phantom that boxed the solver in) - so we stop checking there.
   Below the low bound the mouse just left a centre and any wall is too far to read
   reliably. Bench-tunable if the maze pitch / sensor aim differs. */
#define NAV_FRONTSTOP_FRAC_LO  0.30f
#define NAV_FRONTSTOP_FRAC_HI  0.58f
static volatile float   nav_cell_mm;       /* cell pitch (mm), live-tunable      */
static volatile bool    nav_mode;          /* an advance is in progress          */
static int              nav_cells_total;   /* cells requested for this advance    */
static int              nav_cells_marked;  /* cell centres reached so far         */
static volatile bool    nav_mark_flag;     /* set when a new cell centre reached   */
static volatile int32_t nav_mark_cell;     /* index (1..total) the mark is for     */

/* Nav sequencer (the `path` command / future maze-solver API). A queue of motion
   steps - advance N cells or pivot D degrees - run back-to-back: when one step's
   done flag fires, the next is auto-started from the MAIN loop (Control_NavTask),
   never the ISR, because StartAdvance/StartPivot recalibrate the gyro with a
   blocking delay. Each step reuses the existing primitives, which re-zero heading
   per segment, so the sequence composes cleanly (every straight holds its own
   heading; every turn lands a fresh angle). Turns still stop-pivot-go (pivots
   need v=0); consecutive straight cells within ONE advance step still flow. */
typedef enum { NAV_STEP_ADV = 0, NAV_STEP_TURN = 1 } NavStepType;
typedef struct { uint8_t type; int16_t arg; int16_t spd; } NavStep;  /* arg: cells | deg */
#define NAV_SEQ_MAX        48U
static NavStep  nav_seq[NAV_SEQ_MAX];      /* the queued steps (main-loop only)   */
static uint8_t  nav_seq_len;               /* steps queued                        */
static uint8_t  nav_seq_idx;               /* step currently running / next        */
static bool     nav_seq_active;            /* a sequence is in progress            */
static bool     nav_step_running;          /* the current step's motion is in flight */
/* one-shot events drained by the console to report progress */
static bool     nav_evt_start_flag;        /* a step just started                  */
static uint8_t  nav_evt_start_idx;
static bool     nav_seq_done_flag;         /* the whole sequence ended             */
static bool     nav_seq_done_ok;           /* false = aborted (battery gate)        */

/* Pivot turn-in-place (the `turn`/`pivot` command). Forward speed stays 0 and
   head_setpoint is slewed to head_target; the heading PID does the rotating. */
static volatile bool    pivot_mode;
static bool             pivot_snappy;      /* fast-run: finish pivots early (no settle sit) */
/* Heading carry (hcarry): the pinned drift root cause. Control_Start re-zeros
   heading_deg every primitive, so a pivot that releases a few deg SHORT (snappy
   early-release / PID lag) hands the next straight a skewed heading it then adopts
   as "straight ahead" - the ~8-11 deg under-rotate debt from analyze_drift. When
   ON, a completed mid-run turn stashes its residual (achieved - target); the NEXT
   Control_Start seeds heading_deg with it (setpoint stays 0), so that segment
   rotates the debt back out while moving instead of banking it. Reversible toggle
   (default ON) so the bench can A/B the old zero-reset behaviour. */
static bool             head_carry_en = true;
static float            head_residual;        /* pending carry, deg (achieved-target) */
static bool             head_residual_pending;/* a completed turn left a debt to carry */
static float            head_setpoint;     /* heading PID setpoint (deg); 0 = straight */
static float            head_target;       /* final commanded angle (deg)        */
static float            turn_sgn;          /* +/-1: sign of head_target          */
static float            turn_sp_mag;       /* |head_setpoint| progress, 0..|tgt| */
static float            turn_rate_mag;     /* current slew rate magnitude (dps)  */
static volatile float   turn_rate_dps;     /* slew cap (tunable)                 */
static volatile float   turn_accel_dps2;   /* slew accel/decel (tunable)         */
static volatile bool    turn_done_flag;    /* set on arrival, drained by main    */
static volatile int32_t turn_done_deg;

/* --- Per-turn DIAGNOSTICS (instrumentation only, no effect on motion) -------
   Latched at each turn completion so the console can emit a structured TURNDIAG
   line for the drift bench test (commanded vs actually-achieved rotation, peak
   rate, duration, why it released). All angles in DECI-degrees (0.1 deg) for the
   precision the ~8 deg under-rotation needs; peak rate in deci-dps. reason:
   0 = arrived clean, 1 = snappy early-release (still rotating), 2 = timeout. */
static volatile float   turn_peak_dps;     /* running peak |gyro| this turn      */
static volatile int32_t td_cmd_ddeg;       /* commanded angle * 10               */
static volatile int32_t td_ach_ddeg;       /* achieved heading at release * 10   */
static volatile int32_t td_peak_ddps;      /* peak rotation rate * 10            */
static volatile int32_t td_ms;             /* turn duration (ms)                 */
static volatile int32_t td_reason;         /* 0 clean / 1 snappy-early / 2 timeout */
static volatile int32_t td_posL, td_posR;  /* per-wheel counts this turn (balance) */

/* Smooth (arc) turn: drive FORWARD at speed while the heading setpoint advances
   in proportion to forward distance (dtheta = ds / R) -> a true circular arc of
   radius arc_radius_mm, independent of the speed ramp. Unlike a pivot the common
   wheel speed stays at cruise, so the mouse curves through the corner instead of
   stopping to spin. Stage 1: stops on completion (bench test); the flow
   integration will hand off to the next straight. */
static volatile bool    arc_mode;
static float            arc_radius_mm;
static float            arc_exit_counts;   /* >0: after the arc, drive on this far
                                              in the NEW heading without stopping
                                              (the no-stop hand-off the flow needs);
                                              0 = stop on completion (bench Stage 1) */
/* Curvature feed-forward (cps of wheel-differential trim per deg/s of turn rate).
   An arc needs a steady differential to rotate at the commanded rate; supplying it
   open-loop stops the heading PID lagging then hunting (the buzz). Tune with
   'arcff'; 0 = pure PID (the old laggy behaviour). */
/* PER-DIRECTION feed-forward: a real 4WD mouse rotates more easily one way than the
   other (motor/encoder asymmetry), so the same FF over-turns LEFT and under-turns
   RIGHT - confirmed in the 2026-06-29 flow-turn logs (~7 deg spread at one FF). The
   reference firmware likewise keeps separate left/right turn params. arc_ff_l drives
   LEFT turns (turn_sgn>=0, +deg), arc_ff_r drives RIGHT (-deg). Now SAFE to raise
   above the old 18: completion is DISTANCE-gated, so a strong FF can't run away (the
   old runaway came from waiting on the setpoint/robot to reach target). Bench-tuned
   defaults from the flow-turn logs. Tune with 'arcff [l|r] <v>'. */
/* Back to the bench-proven values (2026-07-03): the fast-log "under-rotation"
   that briefly justified raising these turned out to be the inline-start velocity
   -window poisoning (see pos_ref) braking the mouse mid-curve - NOT an FF
   shortfall. These are the only uncontaminated data. If the first clean run's
   FLOW trace still shows the measured heading lagging the sweep, raise via
   'arcff' with that clean evidence. */
#define ARC_FF_DEFAULT_L  25.5f
#define ARC_FF_DEFAULT_R  27.5f
static volatile float   arc_ff_l = ARC_FF_DEFAULT_L;
static volatile float   arc_ff_r = ARC_FF_DEFAULT_R;

/* Arc heading-correction gain (cps of trim per deg of heading error). During an
   arc the curvature feed-forward (arc_ff) carries the turn, so this only needs to
   GENTLY null the slow residual drift - deliberately FAR lower than the pivot/
   straight-hold kp (200) and with NO derivative term, so it can't hunt and there's
   no gyro-derivative buzz path. Tune with 'arckp'. */
#define ARC_HEAD_KP_DEFAULT  80.0f
static volatile float   arc_kp = ARC_HEAD_KP_DEFAULT;

/* Arc heading DAMPING gain (cps of trim per deg/s of heading-ERROR change). This
   is the reference-firmware (UKMARSBOT) smooth-turn trick: a derivative term for
   damping, but taken on the LOW-PASS-FILTERED error (ARC_ERR_ALPHA below), not on
   the raw gyro - so it damps overshoot/hunt WITHOUT amplifying gyro noise (no
   buzz). Tune with 'arckd'. */
/* 0 by default: the CSV logs proved that through the 35 ms velocity-loop lag this
   D-term does NOT damp - it amplifies tiny heading wiggles into +/-1000 cps motor
   swings (the ~10 Hz buzz). arckd=0 made the arc output smooth. Raise only if the
   velocity loop is later sped up so D can actually damp. Live-tunable via 'arckd'. */
#define ARC_HEAD_KD_DEFAULT  0.0f
static volatile float   arc_kd = ARC_HEAD_KD_DEFAULT;

/* Arc gyro-rate damping (motor DUTY per deg/s of rotation-rate error). This is the
   real fix for the buzz/limit-cycle: the steering trim is applied straight to the
   motors (see the mixer), bypassing the 35 ms velocity-loop lag, and damped by the
   RAW gyro rate - which is fast and unfiltered, so it damps instead of buzzing the
   way arc_kd did through the cascade. Also breaks stiction (pushes harder when the
   robot under-rotates). Tune with 'arcgkd'. */
/* BENCH-PROVEN VALUE - do not raise casually. A 4->10 experiment (2026-07-03)
   made the inline turn SLAM the inner wheel into reverse at the launch: the
   commanded rate leads the chassis by ~a full omega for the first tens of ms, so
   at 10 the rate-error term (~2000 duty) dwarfed the ~400-duty cruise common ->
   inner wheel reversed (correct arc kinematics keep it rolling FORWARD), the
   mouse skidded and bogged (the run2_fastest 02:39 trace: posR -1978 counts in
   the first 100 ms). At 4 the same transient is ~800 duty and the arc tracks. */
#define ARC_GYRO_KD_DEFAULT  4.0f
static volatile float   arc_gyro_kd = ARC_GYRO_KD_DEFAULT;
#define ARC_ERR_ALPHA        0.2f          /* heading-error LPF (ref uses 0.2)   */
static float            arc_err_filt;      /* low-pass-filtered heading error    */
static float            arc_err_prev;      /* previous filtered error (for D)    */

/* The arc overshoots a touch and oscillates as it lands, so the gyro rate dips
   below the settle threshold for a single tick MID-swing. Completing then re-zeros
   the heading while the chassis still carries turn momentum -> it rotates ~10 deg
   into the exit straight (the "kick" in the logs). Require the settle condition to
   hold for this many CONSECUTIVE ticks (ms) so the turn has genuinely stopped
   rotating before the hand-off / stop. */
#define ARC_SETTLE_TICKS     15U
static uint16_t         arc_settle_ticks;  /* consecutive settled ticks          */
static uint32_t         arc_ticks;         /* ms since this arc/flow-turn started -
                                              its OWN timeout (tick_ms isn't reset on
                                              an inline flow-turn start)            */

/* An arc is geometrically EXACTLY R*theta long, so completion is anchored to the
   DISTANCE travelled, not to the gyro going quiet. The old code held cruise speed
   until the gyro settled; when the heading hunted (e.g. a non-zero arckd) it never
   settled, so the robot rolled forward at full speed for seconds -> it "stopped at
   a random cell" and buzzed the whole way (confirmed in the arc CSV logs). Once the
   sweep is geometrically done we brake the forward speed to a halt (settle ~in
   place) and force-complete after at most this much slack past the ideal length. */
#define ARC_DIST_BACKSTOP_MM    40.0f
#define NAV_MAX_RUN_CELLS       32         /* cap on a continued flow run (cells)   */
static float            arc_dist_counts;   /* curve length R*theta + trans, counts */
static float            arc_entry_counts;  /* lead-in straight before the curve     */
static volatile bool    arc_continue;      /* flow-turn: resume forward, don't stop */
/* Trapezoidal omega profile for THIS arc (computed at start from speed + radius +
   transition length). Kept separate from turn_rate_dps on purpose: the old code
   wrote the arc omega INTO turn_rate_dps, so one smooth turn silently slowed every
   later pivot from the profile's rate (550) to the arc's (~230) for the whole run. */
static float            arc_omega_dps;     /* omega = v/R, the trapezoid's hold rate */
static float            arc_alpha_dps2;    /* omega ramp rate = omega*v/arc_trans_mm */
static volatile float   arc_trans_mm = ARC_TRANS_MM_DEFAULT; /* 'arctrans' tunable   */
/* PIVOT-FINISH handoff: if the measured heading is still short when the flow turn
   has braked to its stop at E (FF couldn't keep up - e.g. vacuum-on scrub), the
   arc completion flips to the PIVOT machinery to actively spin the residual out.
   The arc trim alone can't: with turn_rate_mag zeroed and the mouse stopped, the
   gentle arc_kp yields ~1% duty - the 20260703 fast log shows it frozen 3.5 s at
   20 deg short until the timeout, then driving off skewed (the crash chain). The
   pivot loop has the breakaway floor + full trim authority for exactly this. */
static bool             arc_pivot_finish;  /* pivot block is finishing a flow turn  */
static int32_t          arc_finish_mm;     /* distance to report in move_done then  */
/* Forward-position REFERENCE for the arc distance + pivot forward-hold, instead of
   zeroing pos_l/pos_r mid-run. ROOT-CAUSE FIX (2026-07-03, all three fast-run crash
   sessions): the windowed velocity estimator computes v from (pos - vel_hist[w ago]);
   the old inline-start `pos_l = pos_r = 0` left the history ring holding the big
   pre-turn totals, so for the next vel_win ms the measured velocity was a huge
   garbage negative -> the velocity loop slammed the motors (the audible "jizzle" =
   wheel-spin at every corner), the spinning wheel inflated the distance counts (80mm
   entry leg "covered" in tens of ms -> rotate-before-the-corner), the astronomic
   v^2/2a brake tripped target_cps=0 at the corner entry, and the slip fail-safe /
   pivot-finish fired from the fake readings. Measuring from pos_ref leaves the
   estimator's frame intact; from-rest starts (Control_Start et al) flush the ring
   properly and set pos_ref = 0. */
static float            pos_ref;

/* Autonomous-run turn type + smooth-turn geometry (set by the active profile via
   Settings_Apply; read by the solver). turn_mode_smooth=false => pivot turns. */
static volatile bool    turn_mode_smooth;          /* false = pivot, true = arc    */
static float            arc_geom_radius = 90.0f;    /* smooth-turn radius (mm)      */
static float            arc_geom_entry  = 90.0f;    /* lead-in straight (mm)        */
static float            arc_geom_exit   = 90.0f;    /* lead-out straight (mm)       */

static float            gyro_dps;          /* last gyro-Z rate (pivot settle chk)*/

static PID  pid_lvel, pid_rvel, pid_head;
static volatile float vel_ff;            /* velocity feedforward (duty per cps) */

/* Corridor centring (the `follow`/`fcfg` commands). Sticky across runs. */
static volatile bool  follow_enabled;
static volatile float center_kp;         /* heading-offset deg per lateral count */
static volatile float center_kd;         /* centring damping: deg per lateral count/s */
static float          center_lat_prev;   /* prev lateral (for the D term)          */
static float          center_dlat_filt;  /* low-pass-filtered lateral derivative   */

/* Front-wall align (the `align`/`acfg` commands). Squares the mouse to a wall
   ahead using the two front-diagonal IR sensors: rotate to null (L_F - R_F) and
   creep to the calibrated stop distance. Re-zeros heading + forward position
   against an absolute reference, killing accumulated turn drift before a pivot.
   Runs in the 1 kHz loop like a pivot; reads the cached IR values (ISR-safe). */
#define ALIGN_TIMEOUT_MS     700U        /* give up if it can't settle           */
#define ALIGN_SETTLE_TICKS    60U        /* ms within tolerance to call it done   */
#define ALIGN_SKEW_TOL        6.0f       /* |L_F - R_F| counts = "squared"        */
#define ALIGN_DIST_TOL        8.0f       /* |dist error| counts = "at distance"   */
#define ALIGN_ROT_MAX_CPS  3000.0f       /* clamp on the rotate command           */
#define ALIGN_FWD_MAX_CPS  2500.0f       /* clamp on the creep command            */
#define ALIGN_KP_SKEW_DEF     90.0f      /* cps per count of L_F-R_F skew         */
#define ALIGN_KP_DIST_DEF     60.0f      /* cps per count of distance error       */
static volatile bool  align_mode;
static volatile float align_kp_skew, align_kp_dist;   /* tunable via 'acfg'       */
static int            align_lref, align_rref;         /* front cal refs (targets) */
static uint32_t       align_settle;                   /* ticks inside tolerance   */
static volatile bool  align_done_flag;
static volatile bool  align_done_ok;

/* Recenter (reverse a known distance). After a front-stop the mouse is jammed
   against the wall ahead; we back up by exactly the distance the blocked advance
   travelled, returning it to the cell centre so the next turn starts square. */
#define RECENTER_CPS       4500.0f       /* gentle reverse speed (cps)            */
#define RECENTER_MIN_CPS   1500.0f       /* creep speed in the slow-down zone     */
#define RECENTER_SLOW_MM     20.0f       /* decelerate over the last this-many mm */
static volatile bool  recenter_mode;
static float          recenter_target_counts;
static volatile bool  recenter_done_flag;

/* Contact probe: creep forward (front-stop OFF) until the front IR says the wall
   is right in front (reading well above its threshold), reporting how far it
   drove. Used by 'frontcal' to learn the centre->wall distance. */
#define CONTACT_CREEP_CPS  3000.0f       /* slow creep toward the wall            */
#define CONTACT_CAP_MM        130        /* safety: give up if no wall by here     */
static volatile bool  contact_mode;
static int            contact_level;     /* L_F/R_F reading that means "at wall"  */
static volatile bool  contact_done_flag;
static int32_t        contact_mm;        /* distance driven when contact was seen  */

static uint16_t last_cnt_l, last_cnt_r;
static int32_t  pos_l, pos_r;            /* accumulated wheel position (counts)  */
static int32_t  vel_hist_l[VEL_WIN_MAX], vel_hist_r[VEL_WIN_MAX]; /* position ring */
static uint8_t  vel_idx;
static volatile uint8_t vel_win;          /* live window length (ms), 2..50      */
static volatile float   vel_lpf_alpha;    /* live EMA smoothing, 0..1            */
static float    vL_filt, vR_filt;        /* windowed + smoothed velocities (cps) */
static float    gyro_bias_dps;
/* When a 'path'/'solve' sequence is running we calibrate the gyro bias ONCE at the
   start (robot stationary) and skip the ~250 ms per-step recal - that dead sit
   before every turn AND every advance was the bulk of a fast run's lost time (the
   "stops and waits to correct" between cells). Bias drift over a few-second run is
   negligible and the heading is re-zeroed each segment anyway, so once is enough. */
static volatile bool seq_skip_gyro_cal;
static float    gyro_scale;              /* live LSB-per-dps; corrected by turncal */
static float    last_turn_cmd_deg;       /* last commanded pivot, for turncal ratio */
static float    heading_deg;
static volatile bool picked_up;          /* latched when lifted off the floor    */
static uint32_t      pickup_ticks;       /* consecutive tilted ticks             */
static float         last_accel_z;       /* last accel-Z (g), for the run trace  */
static int32_t  enc_diff_counts;         /* running (R - L) for encoder heading*/
static uint32_t tick_ms;                 /* ms since the run started           */
static uint32_t telem_decim;

/* Telemetry ring (ISR producer, main-loop consumer). */
typedef struct { uint32_t t; int32_t sp, meas, out; } Telem;
#define TELEM_SZ 64U
static volatile Telem    telem_buf[TELEM_SZ];
static volatile uint16_t telem_head, telem_tail;
static volatile bool     telem_on;
static volatile CtrlLoop telem_loop = CTRL_LOOP_LVEL;

static void telem_push(int32_t sp, int32_t meas, int32_t out)
{
  uint16_t nh = (uint16_t)((telem_head + 1U) % TELEM_SZ);
  if (nh == telem_tail) return;          /* full: drop (consumer too slow)     */
  telem_buf[telem_head].t    = tick_ms;
  telem_buf[telem_head].sp   = sp;
  telem_buf[telem_head].meas = meas;
  telem_buf[telem_head].out  = out;
  telem_head = nh;
}

/* ===========================================================================
 *  Relay (Astrom-Hagglund) auto-tune of the velocity loop
 *
 *  Instead of a PID, drive both wheels with a bang-bang relay that flips
 *  between two duty levels around the operating point: push harder when the
 *  wheel is below the target speed, softer when above. The loop's own lag
 *  (motor + the velocity filter) makes the speed settle into a steady
 *  oscillation - a "limit cycle". From its amplitude `a` and period `Tu` we get
 *  the ultimate gain Ku = 4h / (pi * a) and then standard PI gains. This runs
 *  inside the 1 kHz loop, so the robot keeps moving forward in a gentle wobble
 *  while it measures. Nothing here disturbs the manual `pid`/`ff` tuning paths.
 * ========================================================================= */
#define AT_WARMUP_SW    3U      /* relay switches to discard (startup transient) */
#define AT_MEAS_SW     12U      /* half-cycles to average over (~6 periods)      */
#define AT_TIMEOUT_MS  8000U    /* give up if no clean limit cycle by here       */

static volatile bool at_active;
static float    at_sp;             /* target velocity the relay centres on (cps)*/
static float    at_base;           /* feedforward duty bias (centres the relay) */
static float    at_h;              /* relay half-amplitude (duty)               */
static int      at_relay;          /* current relay sign (+1 / -1)              */
static uint32_t at_sw_count;       /* relay switches seen so far                */
static uint32_t at_t_last_sw;      /* tick_ms of the last switch                */
static float    at_vmax, at_vmin;  /* velocity peaks within this half-cycle     */
static float    at_per_acc; static uint32_t at_per_n;  /* half-period sum,count  */
static float    at_amp_acc; static uint32_t at_amp_n;  /* peak-to-peak sum,count */

static bool     at_have_result;    /* one-shot flag for the main loop to read   */
static bool     at_ok;
static float    at_r_kp, at_r_ki, at_r_kd, at_r_ku, at_r_tu;  /* tu in seconds   */

static void autotune_finish(bool ok)
{
  at_active = false;
  Control_Stop();                       /* coast, clear active flags            */

  if (ok && at_amp_n > 0U && at_per_n > 0U)
  {
    float a  = 0.5f * (at_amp_acc / (float)at_amp_n);            /* osc amplitude */
    float Tu = 2.0f * (at_per_acc / (float)at_per_n) / 1000.0f;  /* period (s)    */
    if (a > 1.0f && Tu > 0.0f)
    {
      float Ku = (4.0f * at_h) / (3.14159265f * a);
      /* Ziegler-Nichols PI. We deliberately skip Kd: the windowed velocity is
         still a bit noisy and a derivative term tends to make it buzz. */
      at_r_ku = Ku;
      at_r_tu = Tu;
      at_r_kp = 0.45f * Ku;
      at_r_ki = 0.54f * Ku / Tu;
      at_r_kd = 0.0f;
      at_ok = true;
      at_have_result = true;
      return;
    }
  }
  at_ok = false;
  at_have_result = true;
}

static void autotune_tick(float v)
{
  if (tick_ms > AT_TIMEOUT_MS) { autotune_finish(false); return; }

  if (v > at_vmax) at_vmax = v;          /* track this half-cycle's peaks        */
  if (v < at_vmin) at_vmin = v;

  int prev = at_relay;
  at_relay = (at_sp - v > 0.0f) ? +1 : -1;   /* below target -> push harder      */

  if (at_relay != prev)                  /* a switch closes one half-cycle       */
  {
    uint32_t now = tick_ms;
    at_sw_count++;
    if (at_sw_count > AT_WARMUP_SW)      /* let the startup transient pass first  */
    {
      at_per_acc += (float)(now - at_t_last_sw);
      at_per_n++;
      at_amp_acc += (at_vmax - at_vmin);
      at_amp_n++;
    }
    at_t_last_sw = now;
    at_vmax = at_vmin = v;               /* restart peak tracking                */

    if (at_per_n >= AT_MEAS_SW) { autotune_finish(true); return; }
  }

  float out = at_base + (float)at_relay * at_h;
  if (out < 0.0f) out = 0.0f;            /* keep the wheels driving forward       */
  Motor_SetDuty(MOTOR_1, MOT_L_SIGN * (int)out);
  Motor_SetDuty(MOTOR_2, MOT_R_SIGN * (int)out);

  if (telem_on && ++telem_decim >= CTRL_TELEM_DECIM)
  {
    telem_decim = 0;
    telem_push((int32_t)at_sp, (int32_t)v, (int32_t)out);   /* graph the wobble   */
  }
}

/* ===========================================================================
 *  Front-wall align (square + set distance against the wall ahead)
 * ========================================================================= */
static void align_finish(bool ok)
{
  align_mode     = false;
  align_done_ok  = ok;
  align_done_flag = true;
  Control_Stop();                          /* coast, clear active flags          */
}

static void align_tick(void)
{
  /* Cached IR values (read in the main loop) - safe to read from the ISR. */
  int lf = Sensors_Get(SENS_L_F);
  int rf = Sensors_Get(SENS_R_F);

  /* skew: >0 means the LEFT front is closer -> rotate to even them out.
     dist: target is the calibrated stop-distance reading; if the live average
     is below it we're too far and creep forward (+). */
  float skew   = (float)(lf - rf);
  float target = 0.5f * (float)(align_lref + align_rref);
  float derr   = target - 0.5f * (float)(lf + rf);

  float askew = skew < 0.0f ? -skew : skew;
  float aderr = derr < 0.0f ? -derr : derr;
  if (askew < ALIGN_SKEW_TOL && aderr < ALIGN_DIST_TOL)
  {
    if (++align_settle >= ALIGN_SETTLE_TICKS) { align_finish(true); return; }
  }
  else align_settle = 0;

  if (tick_ms > ALIGN_TIMEOUT_MS) { align_finish(true); return; }   /* good enough */

  float rot = align_kp_skew * skew;
  if      (rot >  ALIGN_ROT_MAX_CPS) rot =  ALIGN_ROT_MAX_CPS;
  else if (rot < -ALIGN_ROT_MAX_CPS) rot = -ALIGN_ROT_MAX_CPS;
  float fwd = align_kp_dist * derr;
  if      (fwd >  ALIGN_FWD_MAX_CPS) fwd =  ALIGN_FWD_MAX_CPS;
  else if (fwd < -ALIGN_FWD_MAX_CPS) fwd = -ALIGN_FWD_MAX_CPS;

  /* rotate = differential wheel speed; the sign mirrors the pivot convention
     (HEAD_TRIM_SIGN) so a positive skew turns the right way. */
  float spL = fwd - HEAD_TRIM_SIGN * rot;
  float spR = fwd + HEAD_TRIM_SIGN * rot;
  float outL = pid_step(&pid_lvel, spL, vL_filt, vel_ff * spL);
  float outR = pid_step(&pid_rvel, spR, vR_filt, vel_ff * spR);
  Motor_SetDuty(MOTOR_1, MOT_L_SIGN * (int)outL);
  Motor_SetDuty(MOTOR_2, MOT_R_SIGN * (int)outR);
}

/* ===========================================================================
 *  Recenter (reverse a fixed distance back to the cell centre)
 * ========================================================================= */
static void recenter_tick(void)
{
  /* Reversing makes the wheel counts go negative; travelled-back is positive. */
  float trav = -0.5f * (float)(pos_l + pos_r);
  float vmag = 0.5f * (vL_filt + vR_filt);
  if (vmag < 0.0f) vmag = -vmag;

  if (trav >= recenter_target_counts || tick_ms > 1500U)
  {
    recenter_mode      = false;
    recenter_done_flag = true;
    Control_Stop();
    return;
  }

  /* Both wheels straight back; decelerate over the last RECENTER_SLOW_MM so we
     land on the target instead of coasting past it (the old "backs up too much").
     Equal setpoints keep it straight enough for a short hop. */
  float remain_mm = (recenter_target_counts - trav) / COUNTS_PER_MM;
  float sp = -RECENTER_CPS;
  if (remain_mm < RECENTER_SLOW_MM)
  {
    sp = -RECENTER_CPS * (remain_mm / RECENTER_SLOW_MM);
    if (sp > -RECENTER_MIN_CPS) sp = -RECENTER_MIN_CPS;
  }
  float outL = pid_step(&pid_lvel, sp, vL_filt, vel_ff * sp);
  float outR = pid_step(&pid_rvel, sp, vR_filt, vel_ff * sp);
  Motor_SetDuty(MOTOR_1, MOT_L_SIGN * (int)outL);
  Motor_SetDuty(MOTOR_2, MOT_R_SIGN * (int)outR);
}

/* ===========================================================================
 *  Contact probe (creep forward to the wall, report distance)
 * ========================================================================= */
static void contact_tick(void)
{
  float dist = 0.5f * (float)(pos_l + pos_r);
  float dmm  = (COUNTS_PER_MM > 0.0f) ? dist / COUNTS_PER_MM : 0.0f;
  int lf = Sensors_Get(SENS_L_F);
  int rf = Sensors_Get(SENS_R_F);

  if (lf > contact_level || rf > contact_level || dmm >= (float)CONTACT_CAP_MM ||
      tick_ms > 2500U)
  {
    contact_mm        = (int32_t)dmm;
    contact_mode      = false;
    contact_done_flag = true;
    Control_Stop();
    return;
  }

  float sp = CONTACT_CREEP_CPS;
  float outL = pid_step(&pid_lvel, sp, vL_filt, vel_ff * sp);
  float outR = pid_step(&pid_rvel, sp, vR_filt, vel_ff * sp);
  Motor_SetDuty(MOTOR_1, MOT_L_SIGN * (int)outL);
  Motor_SetDuty(MOTOR_2, MOT_R_SIGN * (int)outR);
}

/* ===========================================================================
 *  Spin-and-lock rotational grip test (ISR mode; see control.h)
 * ========================================================================= */
#define SG_SPIN_DUTY_MAX   6000     /* open-loop differential spin-up duty      */
#define SG_SPIN_DUTY_STEP  60       /* ramp per 1 ms tick -> full in ~100 ms    */
#define SG_SPINUP_TMO_MS   2500U    /* fail if the target rate isn't reached    */
#define SG_STOP_DPS        30.0f    /* skid finished below this yaw rate        */
#define SG_SKID_MIN_MS     12U      /* ignore the lock transient before this    */
#define SG_SKID_TMO_MS     900U     /* safety cap on the skid                   */

typedef enum { SG_SPINUP = 0, SG_SKID } SgPhase;
static volatile bool sg_active;
static SgPhase  sg_phase;
static float    sg_target_dps;
static int      sg_duty;
static float    sg_omega0;          /* |yaw rate| captured at the lock (dps)    */
static uint32_t sg_lock_ms;         /* tick_ms when the wheels locked           */
static bool     sg_have_result, sg_ok;
static float    sg_r_decel, sg_r_omega0;

/* Runs in place of the normal tick while a spin-grip test is active. Reads the
   yaw rate itself (this branch pre-empts the loop's own gyro read). */
static void spingrip_tick(void)
{
  float rate = 0.0f;
  ICM42688_Raw raw;
  if (ICM42688_ReadData(CT.hspi_imu, &raw))
    rate = ((float)raw.gyro[2] / gyro_scale) - gyro_bias_dps;
  float mag = (rate < 0.0f) ? -rate : rate;

  if (sg_phase == SG_SPINUP)
  {
    if (sg_duty < SG_SPIN_DUTY_MAX) sg_duty += SG_SPIN_DUTY_STEP;
    Motor_SetDuty(MOTOR_1, MOT_L_SIGN * (+sg_duty));   /* differential = rotate  */
    Motor_SetDuty(MOTOR_2, MOT_R_SIGN * (-sg_duty));
    if (mag >= sg_target_dps)
    {
      sg_omega0  = mag;
      sg_lock_ms = tick_ms;
      Motor_Brake(MOTOR_1);
      Motor_Brake(MOTOR_2);            /* lock -> the spinning body skids        */
      sg_phase = SG_SKID;
    }
    else if (tick_ms > SG_SPINUP_TMO_MS)
    {                                   /* couldn't reach the rate (torque/grip) */
      sg_ok = false; sg_r_decel = 0.0f; sg_r_omega0 = mag;
      sg_have_result = true; sg_active = false; Control_Stop();
    }
  }
  else /* SG_SKID: wheels locked, measure how fast the skid bleeds the spin off */
  {
    Motor_Brake(MOTOR_1);
    Motor_Brake(MOTOR_2);
    uint32_t dt = tick_ms - sg_lock_ms;
    bool done = (mag < SG_STOP_DPS && dt >= SG_SKID_MIN_MS) || (dt > SG_SKID_TMO_MS);
    if (done)
    {
      float secs = (float)dt / 1000.0f;
      sg_ok       = true;
      sg_r_omega0 = sg_omega0;
      /* decel over the skid; if it timed out still spinning, use the drop so far */
      sg_r_decel  = (secs > 0.0f) ? (sg_omega0 - (mag < SG_STOP_DPS ? 0.0f : mag)) / secs
                                  : 0.0f;
      sg_have_result = true; sg_active = false; Control_Stop();
    }
  }
}

/* ===========================================================================
 *  The control loop (TIM6 ISR context)
 * ========================================================================= */
static void control_tick(void)
{
  if (!active) return;                  /* idle: don't touch motors or the bus */

  tick_ms++;

  /* Safety: if the pack drops critical, kill drive immediately. */
  if (!Battery_AllowHighCurrent())
  {
    Control_Stop();
    return;
  }

  /* --- wheel velocities from the quadrature counters (no bus access) ------ */
  uint16_t cl = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encL);
  uint16_t cr = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encR);
  int16_t  dl = (int16_t)(cl - last_cnt_l);   /* wrap-safe 16-bit delta        */
  int16_t  dr = (int16_t)(cr - last_cnt_r);
  last_cnt_l = cl;
  last_cnt_r = cr;

  int32_t dcl = ENC_L_SIGN * dl;
  int32_t dcr = ENC_R_SIGN * dr;
  pos_l += dcl;
  pos_r += dcr;

  /* velocity over a vel_win-ms sliding window (look back vel_win slots in a
     fixed ring), then a live EMA. Window + alpha are tunable via 'vfilt'. */
  uint8_t w = vel_win;
  vel_hist_l[vel_idx] = pos_l;
  vel_hist_r[vel_idx] = pos_r;
  int oi = (int)vel_idx - (int)w;
  if (oi < 0) oi += (int)VEL_WIN_MAX;
  float vL = (float)(pos_l - vel_hist_l[oi]) * (float)CTRL_HZ / (float)w;
  float vR = (float)(pos_r - vel_hist_r[oi]) * (float)CTRL_HZ / (float)w;
  vel_idx = (uint8_t)((vel_idx + 1U) % VEL_WIN_MAX);
  vL_filt += vel_lpf_alpha * (vL - vL_filt);
  vR_filt += vel_lpf_alpha * (vR - vR_filt);

  /* --- auto-tune owns the wheels while it runs (relay, no PID) ------------ */
  if (at_active) { autotune_tick(0.5f * (vL_filt + vR_filt)); return; }

  /* --- spin-grip test owns the wheels: spin up in place, then lock + skid - */
  if (sg_active) { spingrip_tick(); return; }

  /* --- front-wall align owns the wheels while it squares to the wall ------ */
  if (align_mode) { align_tick(); return; }

  /* --- recenter owns the wheels while it reverses back to the cell centre - */
  if (recenter_mode) { recenter_tick(); return; }

  /* --- contact probe creeps forward to the wall to measure the distance --- */
  if (contact_mode) { contact_tick(); return; }

  /* --- heading: gyro integration fused with encoder difference ----------- */
  enc_diff_counts += (dcr - dcl);
  float head_enc = (float)enc_diff_counts / CTRL_ENC_COUNTS_PER_DEG;

  ICM42688_Raw raw;
  if (ICM42688_ReadData(CT.hspi_imu, &raw))
  {
    float gz = ((float)raw.gyro[2] / gyro_scale) - gyro_bias_dps;
    gyro_dps = gz;                          /* exposed for the pivot settle check */
    heading_deg += gz * CTRL_DT;

    /* Pickup/lift guard: flat on the floor the vertical (Z) accel reads ~1 g;
       lift/tilt the robot and it collapses. Driving + braking accels are
       HORIZONTAL (and a wall crash is a brief horizontal jolt), so they leave Z
       near 1 g - which cleanly separates "lifted" from normal motion. Sustained
       low |az| -> lifted: kill the motors and latch the flag so the run stops and
       the map is kept for dumping. (Assumes the IMU Z axis is vertical when flat.) */
    int16_t az    = raw.accel[2];
    last_accel_z  = (float)az / ACCEL_LSB_PER_G;   /* g, exposed for the trace    */
    int     azmag = (az < 0) ? -az : az;
    if (azmag < PICKUP_AZ_LSB)
    {
      if (++pickup_ticks >= PICKUP_TICKS) { picked_up = true; Control_Stop(); return; }
    }
    else pickup_ticks = 0;
  }
  heading_deg = HEAD_COMP_A * heading_deg + (1.0f - HEAD_COMP_A) * head_enc;

  /* --- distance-limited move: ease the target down as we approach -------- */
  if (move_mode)
  {
    float dist = 0.5f * (float)(pos_l + pos_r);            /* counts travelled  */
    float vmag = 0.5f * (vL_filt + vR_filt);
    if (vmag < 0.0f) vmag = -vmag;

    /* Stall guard: commanding forward but the wheels aren't turning -> jammed
       against a wall the sense/front-stop missed. Stop, flag a wall hit + stall
       so the solver records the wall and backs off, instead of grinding. */
    if (ramp_cps > STALL_MIN_CPS && vmag < STALL_SPEED_CPS)
    {
      if (++stall_ticks >= STALL_TICKS)
      {
        move_done_mm   = (int32_t)(COUNTS_PER_MM > 0.0f ? dist / COUNTS_PER_MM : 0.0f);
        move_hit_wall  = true;
        move_stalled   = true;
        move_done_flag = true;
        Control_Stop();
        return;
      }
    }
    else stall_ticks = 0;

    /* Progress stall (WINDOWED - see PROG_WIN_* notes): a move must cover PROG_WIN_MIN
       every PROG_WIN_MS; an inching/skewed jam that never does is caught in ~0.5s and
       ended (vs the best-ever tracker it used to beat, hanging 4s+). */
    if (dist > stall_prog_max + PROG_WIN_MIN)
    {
      stall_prog_max   = dist;             /* covered a real step -> restart the window */
      stall_prog_ticks = 0;
    }
    else if (++stall_prog_ticks >= PROG_WIN_MS)
    {
      move_done_mm   = (int32_t)(COUNTS_PER_MM > 0.0f ? dist / COUNTS_PER_MM : 0.0f);
      move_hit_wall  = true;
      move_stalled   = true;
      move_done_flag = true;
      Control_Stop();
      return;
    }


    /* Cell marks: each time we pass a cell centre (k * cell pitch from the
       start cell centre), flag it so the main loop can latch the walls there.
       Cheap integer count; the main loop does the sensor read, not the ISR.

       FRONT-WALL POSITION CORRECTION (fast run): the intermittent "rams a wall and
       gets stuck" is a 1-cell position LAG. When a fast flow drives into a wall the
       wheels SLIP-SPIN against it - the encoders keep counting distance the mouse
       isn't travelling, which fires a PHANTOM cell mark. That mark steps the believed
       position one cell PAST the wall (believed (2,3) while physically jammed at
       (2,2)); since that phantom cell reads 'open', the solver keeps driving into the
       same wall forever. The front wall is ground truth for position: if the nose is
       at CONTACT with a wall, the mouse is NOT crossing a cell centre - it's stopped
       against a boundary - so any 'distance' the encoders show now is slip, not
       travel. Suppress the mark while contact holds: believed position stays at the
       TRUE cell, so when the move ends the solver senses the real wall in the right
       place and reroutes (confirmed_open is false for a genuine wall, so it records
       it). Gated move_creep_en => FAST RUN ONLY; search (move_creep_en=false) is
       byte-for-byte unchanged - this is the search-safety the earlier always-on
       version lacked. Contact grade (not detection) so a normal mid-cell flow still
       marks; only a nose jammed on a wall is blocked. */
    if (nav_mode && !(move_creep_en && Walls_FrontContact()))
    {
      float cell_counts = nav_cell_mm * COUNTS_PER_MM;
      int   next = nav_cells_marked + 1;
      if (cell_counts > 0.0f && next <= nav_cells_total &&
          dist >= (float)next * cell_counts)
      {
        nav_cells_marked = next;
        nav_mark_cell    = next;
        nav_mark_flag    = true;
      }
    }

    /* Front-wall stop: ramp the setpoint to 0 (active brake) the instant a real
       wall is seen ahead, then let the stop-when-slow finish below catch it - we
       must NOT collapse the distance target or the loop would cut the motors and
       COAST into the wall instead. Needs 'ircal front'; uncalibrated ->
       Walls_Front() false -> never trips.

       Gated to the boundary-approach band of the current cell: a real blocking
       wall sits at a cell boundary (frac 0.5 of the pitch from the last centre),
       so we only honour the stop near there. Once the mouse crosses into a cell
       the front-diagonal IR grazes that cell's SIDE walls and reads a phantom
       wall ahead - exactly what boxed the solver in - so outside the band we
       ignore it and flow on to the cell centre. */
    bool in_boundary_band = true;
    {
      float cellc = nav_cell_mm * COUNTS_PER_MM;
      if (cellc > 0.0f)
      {
        int   k    = (int)(dist / cellc);
        float frac = (dist - (float)k * cellc) / cellc;   /* 0=centre .5=boundary */
        in_boundary_band = (frac > NAV_FRONTSTOP_FRAC_LO &&
                            frac < NAV_FRONTSTOP_FRAC_HI);
      }
    }
    /* SEARCH: sensitive single-diagonal unfiltered read, gated to the boundary band so
       a mid-cell side-graze phantom is ignored (unchanged). FAST run: the CONFIDENT
       read (BOTH diagonals) - a single-diagonal graze can't trip it, so it needs no
       band gate and works even when odometry has DRIFTED (the band is encoder-based and
       would mis-gate a drifted mouse). This is what lets the fast run STOP at real
       front walls instead of driving into them, without the phantom that forced the
       old confirmed_open disable. */
    /* ...but a SKEWED hit puts one diagonal hard on the wall and the other in the
       clear (log: LF599/RF48), so Confident (needs BOTH) misses it and the mouse
       drives in. OR in Walls_FrontContact (EITHER diagonal at contact grade): a real
       wall - even skewed - drives one diagonal to contact, while a mid-cell side-graze
       phantom stays moderate and never reaches it. So: square walls stop on Confident,
       skewed walls stop on Contact, phantoms trip neither. */
    bool front_hit = move_creep_en
                       ? (Walls_FrontConfident() || Walls_FrontContact())
                       : (in_boundary_band && Walls_FrontRaw());
    if (front_stop_en && nav_mode && !move_braking && front_hit)
    {
      target_cps    = 0.0f;
      move_braking  = true;
      move_hit_wall = true;            /* tell the solver this cell has a front wall */
      /* FAST run: fire the final cell mark the instant the front wall confirms we're at
         the turn cell, so the believed position steps WITH the stop (no lag). The lag -
         front-stop braking BEFORE the distance mark - is exactly what left the position
         a cell behind so confirmed_open blocked the wall and re-rammed it. */
      if (move_creep_en && nav_mode && nav_cells_marked < nav_cells_total)
      {
        nav_cells_marked = nav_cells_total;
        nav_mark_cell    = nav_cells_total;
        nav_mark_flag    = true;
      }
    }

    /* distance it'll coast while the ramp brings speed to 0 (v^2 / 2a) */
    float brake = (vmag * vmag) / (2.0f * decel_cps);
    if ((move_target_counts - dist) <= brake)
    {
      target_cps   = 0.0f;                                 /* start braking     */
      move_braking = true;
    }
    /* Active reverse brake: once braking, while still rolling faster than the stop
       threshold, aim the setpoint BELOW zero so the velocity PID commands reverse
       torque (real decel, not a coast). Prevents overshooting the turn cell at
       speed. 0 = disabled. */
    if (move_braking && brake_rev_cps > 0.0f && vmag > MOVE_STOP_CPS)
      target_cps = -brake_rev_cps;

    /* Creep-to-centre (FAST RUN ONLY - move_creep_en): a hard brake at speed can
       arrest the mouse SHORT of the target, landing a following pivot off the cell
       centre -> wall clip. If we've slowed to ~rest but are still >REACH_MARGIN
       short, creep gently forward to close the gap so the move ends AT the centre.
       NOT for the search (it hangs/creeps into walls there) and NOT when a wall is
       ahead - stopping short is correct, never creep INTO it. We must check the
       front sensor DIRECTLY here, not just move_hit_wall: on a fast run the
       front-stop is disabled for confirmed-open passages, so move_hit_wall never
       latches and the old guard let the creep drive the nose straight into a real
       wall (the "creeps to centre then slams the front wall" bug). Walls_Front()
       is the calibrated latched front read, independent of the disabled front-stop. */
    float remain  = move_target_counts - dist;
    bool  reached = (remain <= MOVE_REACH_MARGIN);
    bool  wall_ahead = move_hit_wall || Walls_Front();       /* detection ~90mm       */
    /* Block the creep at a wall so it never drives the nose in - but block at CLOSE
       range (~45mm, Walls_FrontClose), NOT at plain detection (~90mm, Walls_Front).
       Detection trips right at the cell CENTRE, so blocking there stalls the creep
       ~100 counts SHORT of the cell mark; the mark never fires, the believed position
       lags a cell, and the NEXT move rams the wall it doesn't know about (the
       intermittent "creep + collide" on the from-memory fast run). Blocking at CLOSE
       range lets the creep reach the mark first, then stop before the wall.
       Walls_FrontClose keys on the ircal threshold (always calibrated), so it can't
       silently no-op the way a frontcal check can. */
    bool  wall_block = move_hit_wall || Walls_FrontClose();
    if (move_braking && move_creep_en && !wall_block && !reached &&
        vmag < MOVE_CREEP_CPS)
      target_cps = MOVE_CREEP_CPS;

    /* Overshoot guard (FAST RUN): the distance-limited move normally finishes the
       instant dist>=target and then Control_Stop COASTS (motors off). If a wall is
       right ahead (a turn/stop cell) and we're still rolling fast, that coast
       carries the nose into the wall (the "kisses the front wall at speed" bug -
       move ends at v>>STOP, then drifts ~80mm into the wall). Instead, hold the
       finish and keep ACTIVE-braking until we're actually slow: active decel stops
       us in a mm or two, so we halt with clearance instead of coasting in. The
       target cell mark has already fired (dist>=target), so believed position is
       correct - holding past it can't desync. Fast run only (move_creep_en); the
       search finishes on distance exactly as before. */
    bool brake_to_stop = (move_creep_en && wall_ahead && vmag >= MOVE_STOP_CPS);
    if (brake_to_stop) { target_cps = 0.0f; move_braking = true; }

    /* Finish: reached the target distance, OR braking brought us to a stop. When
       creep is ON, only finish once we're WITHIN the reach margin (or a wall stopped
       us short) so we don't declare done mid-creep; when OFF (search / normal moves)
       finish on ANY stop, exactly as before. Uses wall_BLOCK (close range), not mere
       detection, so a wall seen ~90mm out at the cell centre doesn't finish us short
       of the mark - the creep above closes that gap first. (Search: move_creep_en is
       false, so this branch is unchanged.) */
    if ((dist >= move_target_counts && !brake_to_stop) ||
        (move_braking && vmag < MOVE_STOP_CPS &&
         (!move_creep_en || reached || wall_block)))
    {
      move_done_mm   = (int32_t)(COUNTS_PER_MM > 0.0f ? dist / COUNTS_PER_MM : 0.0f);
      move_done_flag = true;
      Control_Stop();                                      /* arrived: coast    */
      return;
    }
  }

  /* --- smooth arc turn: trapezoidal-omega heading sweep at constant speed ----
     arc_omega_dps was set from R + speed (omega = v/R) in StartSmoothTurn, so
     slewing the heading at that rate while driving forward traces radius R; the
     ramps to/from omega run at arc_alpha_dps2 (= omega*v/arc_trans_mm), i.e. over
     a fixed FLOOR DISTANCE, so the whole drawn path is speed-invariant and the
     swept angle integrates to exactly the target. Rotation eases to a creep as
     the heading arrives - no leftover spin at the hand-off. */
  if (arc_mode)
  {
    arc_ticks++;                                             /* arc-local timeout    */
    float tgt      = (turn_sgn >= 0.0f) ? head_target : -head_target;  /* |target| */
    float arc_dist = 0.5f * (float)(pos_l + pos_r) - pos_ref; /* counts since start */
    float arc_vmag = 0.5f * (vL_filt + vR_filt); if (arc_vmag < 0.0f) arc_vmag = -arc_vmag;

    /* ENTRY lead-in: a smooth maze turn must BEGIN about a radius before the corner
       cell centre, so the curve drops the mouse onto the next corridor centred. We
       drive straight (heading held) for arc_entry_counts first, THEN start the
       heading sweep. The bench 'arc' command uses entry=0, so this is a no-op there
       and that proven path is unchanged. Distances for the sweep/exit are measured
       from the curve start (arc_dist - arc_entry_counts). */
    if (arc_dist < arc_entry_counts)
    {
      head_setpoint = 0.0f;                       /* hold straight into the corner  */
    }
    else
    {
    float curve_dist = arc_dist - arc_entry_counts;          /* counts since curve  */
    /* TRAPEZOIDAL omega profile in ANGLE space (the canonical UKMARS method, cf.
       the reference Profile class): ramp the rate up at arc_alpha, hold at the
       arc omega, and ramp DOWN once the remaining angle equals the braking angle
       rate^2/(2*alpha) - so the swept setpoint integrates to EXACTLY the target
       angle, always, at ANY speed (alpha was derived from speed + arc_trans_mm at
       start, so the ramps also span the same floor distance at any speed). The
       creep floor keeps it from asymptotically stalling short - same proven shape
       as the pivot slew. Replaces the old fixed ARC_EASE_DEG time-based ease-out
       whose sweep finished LATER than the arc distance at speed -> the setpoint
       got force-stepped to the target -> the ~17-20 deg measured lag + crashes. */
    float rem = tgt - turn_sp_mag;                                 /* deg left   */
    float brk = (arc_alpha_dps2 > 0.0f)
              ? (turn_rate_mag * turn_rate_mag) / (2.0f * arc_alpha_dps2) : 0.0f;
    if (rem <= brk)                                /* close in: decelerate the slew */
    {
      turn_rate_mag -= arc_alpha_dps2 * CTRL_DT;
      if (turn_rate_mag < TURN_MIN_RATE) turn_rate_mag = TURN_MIN_RATE; /* creep   */
    }
    else if (turn_rate_mag < arc_omega_dps)        /* ramp the rate up to omega     */
    {
      turn_rate_mag += arc_alpha_dps2 * CTRL_DT;
      if (turn_rate_mag > arc_omega_dps) turn_rate_mag = arc_omega_dps;
    }
    turn_sp_mag += turn_rate_mag * CTRL_DT;
    if (turn_sp_mag >= tgt)
    {
      /* Sweep complete: clamp AND zero the rate so the curvature feed-forward
         (proportional to turn_rate_mag) STOPS pushing. Leaving it at the creep
         rate kept driving rotation past the target -> overshoot, and held the gyro
         above the settle threshold so it never settled and timed out mid-spin. */
      turn_sp_mag   = tgt;
      turn_rate_mag = 0.0f;
    }
    head_setpoint = turn_sgn * turn_sp_mag;       /* sign = turn direction         */

    bool  swept    = (turn_sp_mag >= tgt);

    if (arc_exit_counts > 0.0f)
    {
      /* A flow turn (bench 'flowturn' OR the fast-run smooth turn) ends STOPPED and
         ALIGNED at the next-cell centre, then the solver resumes the flow. Hard rules
         from the fast-run crash logs (the mouse handed off under-rotated at ~56 deg
         and the re-zero-while-rotating spun it out to -190 deg):
          - once the arc length is covered, target the FULL turn angle (the ease-out
            can leave the swept setpoint a few deg short);
          - brake to a stop AT the cell centre;
          - finish on the MEASURED heading actually ARRIVING (+ rotation settled), not
            on distance - if the heading lags, finish the last bit of rotation IN
            PLACE at the centre rather than driving on skewed into a wall. */
      float total = arc_dist_counts + arc_exit_counts;        /* from the curve start */
      /* Fail-safe only: with the trapezoid, the sweep completes WITH the distance
         by construction, so in normal tracking this never fires. It catches real
         wheel slip (distance ran ahead of the profile) so a slipping mouse still
         targets the full angle instead of sweeping into the next corridor. */
      if (curve_dist >= arc_dist_counts + ARC_SLIP_MARGIN_MM * COUNTS_PER_MM)
      {
        turn_sp_mag   = tgt;
        turn_rate_mag = 0.0f;
        head_setpoint = turn_sgn * tgt;     /* hold the FULL target angle           */
      }
      float herr  = head_target - heading_deg; if (herr  < 0.0f) herr  = -herr;
      float grate = gyro_dps;                  if (grate < 0.0f) grate = -grate;
      bool  arrived = (herr < TURN_TOL_DEG && grate < TURN_SETTLE_DPS);

      float brake = (arc_vmag * arc_vmag) / (2.0f * decel_cps);
      if ((total - curve_dist) <= brake) target_cps = 0.0f;   /* ease to a stop at E  */

      bool at_end  = (curve_dist >= total) ||
                     (target_cps == 0.0f && arc_vmag < MOVE_STOP_CPS);
      bool stopped = (arc_vmag < MOVE_STOP_CPS);
      if (at_end && arrived)
      {
        turn_done_deg  = (int32_t)heading_deg;
        turn_done_flag = true;
        move_done_mm   = (int32_t)(COUNTS_PER_MM > 0.0f ? arc_dist / COUNTS_PER_MM : 0.0f);
        move_done_flag = true;
        Control_Stop();                      /* stopped + aligned at the next cell   */
        return;
      }
      /* Stopped at E (or timed out mid-curve) with the heading still SHORT: hand
         the residual rotation to the PIVOT machinery this same tick. Its breakaway
         floor + per-wheel velocity loops deliver real duty from rest (the arc trim
         at ~1% duty just sat frozen against stiction - see the 20260703 fast log),
         so the mouse leaves this corner ALIGNED instead of 20-50 deg skewed. */
      if ((at_end && stopped) || arc_ticks > TURN_TIMEOUT_MS)
      {
        arc_finish_mm    = (int32_t)(COUNTS_PER_MM > 0.0f ? arc_dist / COUNTS_PER_MM : 0.0f);
        arc_pivot_finish = true;
        arc_ticks        = 0;                /* fresh timeout budget for the finish */
        arc_mode         = false;
        turn_sp_mag      = tgt;              /* setpoint parked at the full target  */
        turn_rate_mag    = 0.0f;
        head_setpoint    = turn_sgn * tgt;
        target_cps       = 0.0f;             /* pivot in place at E                 */
        ramp_cps         = 0.0f;
        pos_ref          = 0.5f * (float)(pos_l + pos_r); /* fwd-hold about HERE -
                                                do NOT zero pos (poisons vel window) */
        pid_reset(&pid_lvel);                /* both were serving the arc's         */
        pid_reset(&pid_rvel);                /*   decoupled duty path - start clean */
        pid_reset(&pid_head);
        pivot_mode       = true;             /* falls into the pivot block below    */
      }
    }
    else
    {
      /* BENCH arc (no exit leg): the instant the sweep is geometrically complete,
         brake the forward speed and stop ~in place. Distance - not gyro-quiet - is
         the reliable bound (a hunting heading can't carry the mouse across cells). */
      if (swept) target_cps = 0.0f;
      float herr  = head_target - heading_deg; if (herr  < 0.0f) herr  = -herr;
      float grate = gyro_dps;                  if (grate < 0.0f) grate = -grate;
      if (swept && herr < TURN_TOL_DEG && grate < TURN_SETTLE_DPS)
      {
        if (arc_settle_ticks < ARC_SETTLE_TICKS) arc_settle_ticks++;
      }
      else arc_settle_ticks = 0;
      bool dist_cap = (arc_dist_counts > 0.0f &&
                       curve_dist >= arc_dist_counts + ARC_DIST_BACKSTOP_MM * COUNTS_PER_MM);
      bool stopped  = (swept && arc_vmag < MOVE_STOP_CPS);
      if (arc_settle_ticks >= ARC_SETTLE_TICKS || stopped || dist_cap ||
          tick_ms > TURN_TIMEOUT_MS)
      {
        turn_done_deg  = (int32_t)heading_deg;
        turn_done_flag = true;
        Control_Stop();                  /* arrived: stop after the arc           */
        return;
      }
    }
    }
  }

  /* --- pivot turn-in-place: slew the HEADING setpoint to the target ------- */
  if (pivot_mode)
  {
    float tgt   = (turn_sgn >= 0.0f) ? head_target : -head_target;  /* |target| */
    { float gpk = gyro_dps < 0.0f ? -gyro_dps : gyro_dps;          /* diag: peak rate */
      if (gpk > turn_peak_dps) turn_peak_dps = gpk; }
    float brake = (turn_rate_mag * turn_rate_mag) / (2.0f * turn_accel_dps2);
    if ((tgt - turn_sp_mag) <= brake)            /* close in: decelerate the slew */
    {
      turn_rate_mag -= turn_accel_dps2 * CTRL_DT;
      if (turn_rate_mag < TURN_MIN_RATE) turn_rate_mag = TURN_MIN_RATE; /* creep  */
    }
    else if (turn_rate_mag < turn_rate_dps)      /* ramp the slew up to the cap   */
    {
      turn_rate_mag += turn_accel_dps2 * CTRL_DT;
      if (turn_rate_mag > turn_rate_dps) turn_rate_mag = turn_rate_dps;
    }
    turn_sp_mag += turn_rate_mag * CTRL_DT;
    if (turn_sp_mag > tgt) turn_sp_mag = tgt;
    head_setpoint = turn_sgn * turn_sp_mag;

    /* Done when the setpoint has reached the target AND the robot has arrived -
       stopped rotating for a normal turn, or just "close enough while still easing"
       for a SNAPPY fast-run turn (the forward flow finishes the trim) - or timeout.
       When this pivot is FINISHING a flow turn (arc_pivot_finish), the timeout runs
       on the arc's own tick counter: tick_ms counts from Control_Start, which on an
       inline flow turn is the whole RUN - it would "time out" instantly. */
    float herr = head_target - heading_deg; if (herr < 0.0f) herr = -herr;
    float grate = gyro_dps;                 if (grate < 0.0f) grate = -grate;
    float tol  = pivot_snappy ? PIVOT_SNAP_TOL_DEG    : TURN_TOL_DEG;
    float sett = pivot_snappy ? PIVOT_SNAP_SETTLE_DPS : TURN_SETTLE_DPS;
    if (arc_pivot_finish) arc_ticks++;
    uint32_t piv_t = arc_pivot_finish ? arc_ticks : tick_ms;
    if ((turn_sp_mag >= tgt && herr < tol && grate < sett) ||
        piv_t > TURN_TIMEOUT_MS)
    {
      turn_done_deg  = (int32_t)heading_deg;
      turn_done_flag = true;
      /* hcarry: stash the under-rotate debt for the next segment (mid-run only, so
         single-shot console pivots are unaffected). Snappy/timeout release SHORT of
         head_target; a clean release makes this ~0 (harmless). */
      if (seq_skip_gyro_cal)
      {
        head_residual         = heading_deg - head_target;   /* signed; sign-symmetric */
        head_residual_pending = true;
      }
      /* latch per-turn diagnostics (see TURNDIAG). angles/rate in deci-units. */
      td_cmd_ddeg  = (int32_t)(head_target   * 10.0f);
      td_ach_ddeg  = (int32_t)(heading_deg   * 10.0f);
      td_peak_ddps = (int32_t)(turn_peak_dps * 10.0f);
      td_ms        = (int32_t)piv_t;
      td_reason    = (piv_t > TURN_TIMEOUT_MS) ? 2
                   : ((pivot_snappy && grate >= TURN_SETTLE_DPS) ? 1 : 0);
      td_posL      = pos_l;              /* per-wheel counts: a pivot spins them   */
      td_posR      = pos_r;              /* opposite; |L| vs |R| = wheel balance   */
      if (arc_pivot_finish)
      {
        /* a flow turn ends here: report the move too (solver/app wait on it) */
        arc_pivot_finish = false;
        move_done_mm     = arc_finish_mm;
        move_done_flag   = true;
      }
      Control_Stop();                                      /* arrived: coast    */
      return;
    }
  }

  /* --- slew the setpoint toward the target: accel rate when speeding up,
         decel rate when slowing down (incl. the move-stop ramp above) ------- */
  if      (ramp_cps < target_cps) { ramp_cps += accel_cps * CTRL_DT; if (ramp_cps > target_cps) ramp_cps = target_cps; }
  else if (ramp_cps > target_cps) { ramp_cps -= decel_cps * CTRL_DT; if (ramp_cps < target_cps) ramp_cps = target_cps; }

  /* --- corridor centring: bias the heading setpoint to null the IR lateral
     error. Only on forward runs, never during a pivot; with no walls present
     Sensors_Lateral returns 0 so this is a no-op (plain heading-hold). ------ */
  float hsp = head_setpoint;
  if (follow_enabled && !pivot_mode && !arc_mode && ramp_cps > 0.0f)
  {
    float lat = (float)Sensors_Lateral();        /* signed; 0 when no wall to follow */
    /* PD centring: P nulls the offset, D damps the cross-corridor WEAVE (the search
       wobble). Only differentiate when BOTH this tick and last had a wall (lat!=0),
       so a wall appearing/clearing (lat stepping to/from 0) can't spike the steering;
       the derivative is low-pass-filtered to keep IR noise out of the motors. */
    float dlat = 0.0f;
    if (lat != 0.0f && center_lat_prev != 0.0f)
      dlat = (lat - center_lat_prev) * (float)CTRL_HZ;             /* counts/s        */
    center_lat_prev  = lat;
    center_dlat_filt += CENTER_D_ALPHA * (dlat - center_dlat_filt);
    float off = (float)CENTER_SIGN * (center_kp * lat + center_kd * center_dlat_filt);
    if      (off >  CENTER_OFFSET_MAX) off =  CENTER_OFFSET_MAX;
    else if (off < -CENTER_OFFSET_MAX) off = -CENTER_OFFSET_MAX;
    hsp += off;
  }
  else { center_lat_prev = 0.0f; center_dlat_filt = 0.0f; }  /* fresh D on re-engage */

  /* --- outer heading PID -> straightness trim / pivot drive (no FF) ------
     setpoint is 0 for straight runs (hold heading, plus any centring bias) and
     the slewed angle target during a pivot; with forward speed 0 the +/-trim
     becomes pure rotation. */
  /* Heading-setpoint slew rate (deg/s): non-zero only while a turn is slewing.
     The PID uses derivative-on-measurement (= -kd*heading_rate), which is right
     for HOLDING a heading but during an arc/pivot reads the whole COMMANDED turn
     rate as something to damp - a constant ~kd*rate term fighting the turn, plus
     gyro noise amplified by 1/CTRL_DT into the trim => the ~7 Hz buzz and the
     steady ~8 deg tracking lag. Adding kd*sp_rate back makes the derivative act
     on the rate ERROR (deviation from the commanded rotation) instead, so it
     still damps real wobble but no longer fights the turn itself. Expressed in
     the PID's own measurement frame, so the sign is automatic; it eases to 0 as
     the arc eases out and is exactly 0 on straight runs (heading-hold unchanged). */
  float trim;
  if (arc_mode)
  {
    /* --- ARC: feed-forward trajectory + well-damped tracker (UKMARSBOT style) -
       The reference firmware makes turns smooth NOT by a high-gain position loop
       but by: (1) a feed-forward that draws the arc open-loop, and (2) a tracker
       whose DAMPING comes from a derivative taken on the LOW-PASS-FILTERED error -
       big enough to kill overshoot/hunt, but on a doubly-smoothed signal so it
       can't pump gyro noise into the motors (the buzz). We mirror that here:
         trim = arc_ff*rate   (FF, the bulk)
              + arc_kp*err_f   (gentle P, nulls slow drift)
              + arc_kd*d(err_f)/dt   (damping, on the FILTERED error => no buzz)
       heading_deg is already a clean gyro integral; we filter the error on top,
       so arc_kd can be sizeable without the noise blow-up that wrecked the old
       raw-gyro D term. */
    float head_err = head_setpoint - heading_deg;          /* deg, signed       */
    arc_err_filt += ARC_ERR_ALPHA * (head_err - arc_err_filt);   /* LPF the error */
    float err_rate = (arc_err_filt - arc_err_prev) * (float)CTRL_HZ; /* d/dt    */
    arc_err_prev = arc_err_filt;
    float arc_ff = (turn_sgn >= 0.0f) ? arc_ff_l : arc_ff_r;  /* per-direction FF */
    trim = HEAD_TRIM_SIGN * (turn_sgn * arc_ff * turn_rate_mag   /* curvature FF */
                             + arc_kp * arc_err_filt             /* gentle P     */
                             + arc_kd * err_rate);               /* damping (D)  */
    if      (trim >  PIVOT_TRIM_MAX_CPS) trim =  PIVOT_TRIM_MAX_CPS;
    else if (trim < -PIVOT_TRIM_MAX_CPS) trim = -PIVOT_TRIM_MAX_CPS;
  }
  else
  {
    /* --- straight-hold / pivot: outer heading PID -> straightness or spin trim
       setpoint is 0 for straight runs (hold heading, plus any centring bias) and
       the slewed angle target during a pivot. The derivative-on-measurement here
       acts on the rate ERROR (kd*sp_rate added back) so it damps real wobble
       without fighting a pivot's commanded rotation. */
    float head_sp_rate = pivot_mode ? (turn_sgn * turn_rate_mag) : 0.0f;
    trim = HEAD_TRIM_SIGN * (pid_step(&pid_head, hsp, heading_deg, 0.0f)
                             + pid_head.kd * head_sp_rate);
  }

  /* Pivot breakaway floor: while still more than a few degrees from the target,
     force the trim to at least PIVOT_MIN_TRIM in the push direction so the turn
     drives through stiction instead of crawling. Released inside the deadband so
     it eases to a clean stop. (The PID already turns the correct way, so the
     floor's sign just follows the heading error.) */
  if (pivot_mode)
  {
    float e = head_target - heading_deg;
    if ((e < 0.0f ? -e : e) > PIVOT_FLOOR_DEADBAND)
    {
      float floor = HEAD_TRIM_SIGN * (e >= 0.0f ? 1.0f : -1.0f) * PIVOT_MIN_TRIM;
      if (floor >= 0.0f) { if (trim < floor) trim = floor; }
      else               { if (trim > floor) trim = floor; }
    }
  }

  /* Common-mode wheel speed: the slewed drive setpoint normally, but during a
     pivot it's the forward-hold correction that keeps net translation at zero so
     the turn doesn't drift off its spot. */
  float common = ramp_cps;
  if (pivot_mode)
  {
    float fwd = 0.5f * (float)(pos_l + pos_r) - pos_ref; /* net forward counts    */
    common = -PIVOT_FWD_KP * fwd;
    if      (common >  PIVOT_FWD_MAX_CPS) common =  PIVOT_FWD_MAX_CPS;
    else if (common < -PIVOT_FWD_MAX_CPS) common = -PIVOT_FWD_MAX_CPS;
  }

  /* --- mix forward speed + steering onto the two motors ------------------- */
  float spL = common - trim;            /* nominal per-wheel setpoints (telem)  */
  float spR = common + trim;
  float outL, outR;

  if (arc_mode)
  {
    /* DECOUPLED arc steering - the architectural fix for the buzz/limit-cycle.
       PROVEN by the logs: routing the heading trim through the per-wheel velocity
       loop (35 ms filter ~ 20 ms lag) made the steering oscillate at ~10 Hz for
       ANY gain - arc_kd>0 buzzed, arc_kd=0 limit-cycled and never settled. So here
       the steering does NOT go through that loop:
         - forward speed is regulated on the AVERAGE wheel velocity (one PID), and
         - steering is applied straight to motor DUTY = (open-loop trim FF/position)
           + (FAST gyro-rate damping). gyro_dps is unfiltered, so the damping is
           prompt -> it actually damps (settles the end, breaks stiction) instead
           of pumping the cascade. Per-wheel feedback is dropped here because it
           would only FIGHT an open-loop differential. Arc-only; straight/pivot use
           the proven per-wheel path below. */
    float v_avg       = 0.5f * (vL_filt + vR_filt);
    float common_duty = pid_step(&pid_lvel, common, v_avg, vel_ff * common);
    float rate_err    = (turn_sgn * turn_rate_mag) - gyro_dps;   /* cmd - meas dps */
    float steer_duty  = vel_ff * trim                            /* FF + position  */
                      + HEAD_TRIM_SIGN * arc_gyro_kd * rate_err; /* fast damping   */
    /* STEERING PRIORITY: at high cruise, common + |steer| can exceed the PWM
       ceiling; the old per-wheel clamp then ate the DIFFERENTIAL, so the turn
       under-rotated while the speed held (exactly backwards for a maze corner -
       the 12000 cps crash). Give steering its duty first and let the forward
       speed sag the little that's left over instead. */
    {
      float sd = (steer_duty < 0.0f) ? -steer_duty : steer_duty;
      if (sd > (float)MOTOR_PWM_MAX) sd = (float)MOTOR_PWM_MAX;
      float cmax = (float)MOTOR_PWM_MAX - sd;
      if      (common_duty >  cmax) common_duty =  cmax;
      else if (common_duty < -cmax) common_duty = -cmax;
    }
    outL = common_duty - steer_duty;
    outR = common_duty + steer_duty;
    if      (outL >  (float)MOTOR_PWM_MAX) outL =  (float)MOTOR_PWM_MAX;
    else if (outL < -(float)MOTOR_PWM_MAX) outL = -(float)MOTOR_PWM_MAX;
    if      (outR >  (float)MOTOR_PWM_MAX) outR =  (float)MOTOR_PWM_MAX;
    else if (outR < -(float)MOTOR_PWM_MAX) outR = -(float)MOTOR_PWM_MAX;
  }
  else
  {
    /* inner velocity PIDs (feedforward = vel_ff * setpoint) - per wheel */
    outL = pid_step(&pid_lvel, spL, vL_filt, vel_ff * spL);
    outR = pid_step(&pid_rvel, spR, vR_filt, vel_ff * spR);
  }

  Motor_SetDuty(MOTOR_1, MOT_L_SIGN * (int)outL);
  Motor_SetDuty(MOTOR_2, MOT_R_SIGN * (int)outR);

  /* --- telemetry (decimated) --------------------------------------------- */
  if (telem_on && ++telem_decim >= CTRL_TELEM_DECIM)
  {
    telem_decim = 0;
    switch (telem_loop)
    {
      case CTRL_LOOP_RVEL:
        telem_push((int32_t)spR, (int32_t)vR_filt, (int32_t)outR); break;
      case CTRL_LOOP_HEAD:
        telem_push((int32_t)(hsp * 100.0f),
                   (int32_t)(heading_deg * 100.0f), (int32_t)trim); break;
      case CTRL_LOOP_LVEL:
      case CTRL_LOOP_VEL:
      default:
        telem_push((int32_t)spL, (int32_t)vL_filt, (int32_t)outL); break;
    }
  }
}

/* TIM6 update interrupt -> control tick. */
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
    control_tick();
}

/* ===========================================================================
 *  Setup
 * ========================================================================= */
static void tim6_init(void)
{
  __HAL_RCC_TIM6_CLK_ENABLE();

  /* TIM6 runs off the APB1 timer clock (= PCLK1, doubled when the APB1
     prescaler isn't /1). Derive it so 1 kHz is right whatever the tree. */
  RCC_ClkInitTypeDef clk; uint32_t flash_lat;
  HAL_RCC_GetClockConfig(&clk, &flash_lat);
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t timclk = (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : pclk1 * 2U;

  htim6.Instance               = TIM6;
  htim6.Init.Prescaler         = (timclk / 1000000U) - 1U;   /* 1 MHz tick     */
  htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim6.Init.Period            = (1000000U / CTRL_HZ) - 1U;  /* -> CTRL_HZ      */
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    LOG("CONTROL: TIM6 init FAILED\r\n");

  /* Above SysTick/USART so the loop is jitter-free; below nothing critical. */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);
}

void Control_Init(const ControlCtx *ctx)
{
  CT = *ctx;
  active   = false;
  telem_on = false;
  target_cps = 0.0f;

  Motor_PWM_Init(CT.htim_mot);

  /* Power-on defaults = the confirmed-good 1 m/s (~24500 cps) "search run"
     tune, baked in 2026-06-25 after a clean 1450 mm move (1 mm distance error,
     calm output, smooth braking). These survive a power cycle; still fully
     live-tunable from the app. Re-tune for the fast run via the (upcoming)
     flash profiles, not by editing these. */
  pid_lvel.out_min = pid_rvel.out_min = -(float)MOTOR_PWM_MAX;
  pid_lvel.out_max = pid_rvel.out_max =  (float)MOTOR_PWM_MAX;
  pid_lvel.d_alpha = pid_rvel.d_alpha = 1.0f;   /* velocity loops: raw D (kd=0)  */
  Control_SetGains(CTRL_LOOP_VEL, 0.05f, 0.2f, 0.0f);

  pid_head.out_min = -HEAD_TRIM_MAX_CPS;
  pid_head.out_max =  HEAD_TRIM_MAX_CPS;
  pid_head.d_alpha = HEAD_D_ALPHA_DEFAULT;      /* filter the gyro-rate D-term    */
  Control_SetGains(CTRL_LOOP_HEAD, 200.0f, 0.0f, 8.0f);

  /* Gyro sensitivity (LSB per dps). Starts at the datasheet nominal; correct the
     part-to-part error live with 'gyroscale'/turncal so a commanded 90 deg pivot
     lands at a true 90 deg. Once you find the value, bake it into the
     GYRO_LSB_PER_DPS default so it survives a power cycle. */
  gyro_scale        = GYRO_LSB_PER_DPS;
  last_turn_cmd_deg = 0.0f;

  /* Velocity feedforward (duty per cps); tune with 'ff'. */
  vel_ff = 0.046f;

  /* Setpoint ramp rates (cps/s); tune with 'accel'. ~3.3 m/s^2 at 24.5 c/mm.
     decel 80k->130k helped the front-stop overshoot; 160k was WORSE (wheels skid
     under too-hard a brake -> doesn't stop better AND the slip desyncs odometry,
     seen as a phantom-wall cluster in run 044), reverted to 130k. We're at the
     tyre-grip limit, so the remaining bumps are fixed by detecting the wall
     EARLIER (raw front read for the brake), not by braking harder. */
  accel_cps = 80000.0f;
  decel_cps = 130000.0f;
  brake_rev_cps = 0.0f;         /* active reverse brake off by default; fast run arms it */
  move_creep_en = false;        /* creep-to-centre off; fast run enables it              */

  /* Velocity feedback filter (window ms + EMA); tune with 'vfilt'. The heavier
     35 ms / 0.20 filter cut cruise ripple from ~+/-18% to ~+/-5%. */
  vel_win       = 35U;
  vel_lpf_alpha = 0.20f;

  /* Pivot turn slew defaults; tune with 'turn'. The heading PID (set above)
     does the actual rotating - these just shape how the setpoint leads it. */
  head_setpoint   = 0.0f;
  pivot_mode      = false;
  turn_rate_dps   = TURN_RATE_DEFAULT;
  turn_accel_dps2 = TURN_ACCEL_DEFAULT;

  /* Corridor centring off by default (opt-in via 'follow on'); tune with 'fcfg'. */
  follow_enabled  = false;
  center_kp       = CENTER_KP_DEFAULT;
  center_kd       = CENTER_KD_DEFAULT;
  center_lat_prev = 0.0f;
  center_dlat_filt = 0.0f;

  /* Front-wall align gains; tune with 'acfg'. */
  align_mode      = false;
  align_kp_skew   = ALIGN_KP_SKEW_DEF;
  align_kp_dist   = ALIGN_KP_DIST_DEF;

  /* Cell pitch for 'advance' (mm); tune with 'cell'. Classic maze = 180 mm. */
  nav_cell_mm     = NAV_CELL_MM_DEFAULT;

  /* Nav sequencer (the 'path' queue) starts empty. */
  Control_NavReset();

  tim6_init();
  LOG("CONTROL: 1 kHz loop armed (idle)\r\n");
  LOG("CONTROL: geom %ld counts/mm (x100), max %ld cps ~ %ld mm/s\r\n",
      (long)(COUNTS_PER_MM * 100.0f),
      (long)MAX_WHEEL_CPS,
      (long)(COUNTS_PER_MM > 0.0f ? MAX_WHEEL_CPS / COUNTS_PER_MM : 0.0f));
}

static void calibrate_gyro_bias(void)
{
  /* Wait for the robot to PHYSICALLY stop before sampling bias. A pivot (and the
     pre-turn align) ends by COASTING, so the wheels are still turning when this
     runs; averaging the bias during that motion poisons it and makes the next
     turn over-rotate or run away ("keeps turning"). We detect "stopped" from the
     ENCODERS, not the gyro: a slow constant coast-rotation reads as steady on the
     gyro (so a gyro-stability test wrongly passes) but the wheels are plainly
     still moving. Bounded by a timeout so it can't hang the start. */
  uint16_t pl = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encL);
  uint16_t pr = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encR);
  uint32_t t0 = HAL_GetTick();
  int quiet = 0;
  for (;;)
  {
    HAL_Delay(5);
    uint16_t cl = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encL);
    uint16_t cr = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encR);
    int16_t  dl = (int16_t)(cl - pl);
    int16_t  dr = (int16_t)(cr - pr);
    pl = cl; pr = cr;
    int moved = (dl < 0 ? -dl : dl) + (dr < 0 ? -dr : dr);
    if (moved <= 2) { if (++quiet >= 10) break; }   /* ~50 ms of no wheel motion   */
    else quiet = 0;
    if (HAL_GetTick() - t0 > 700U) break;            /* give up, sample anyway      */
  }

  float acc = 0.0f; int n = 0;
  for (int i = 0; i < 200; i++)
  {
    ICM42688_Raw raw;
    if (ICM42688_ReadData(CT.hspi_imu, &raw)) { acc += (float)raw.gyro[2]; n++; }
    HAL_Delay(1);
  }
  gyro_bias_dps = (n > 0) ? (acc / (float)n) / gyro_scale : 0.0f;
}

/* Mark the start / end of an autonomous MULTI-STEP run (a 'path' sequence or a
   'solve'/fast run). RunBegin samples the gyro bias ONCE here (robot stationary at
   the run start) and tells every step's Control_Start to skip its own ~250 ms recal
   - that dead sit before each turn/advance was the bulk of a fast run's lost time.
   RunEnd re-arms per-start recal so ordinary single-shot commands behave as before.
   Bias drift over a few-second run is negligible and heading is re-zeroed each
   segment, so once is enough. Always pair them (every run-end path calls RunEnd). */
void Control_RunBegin(void)
{
  calibrate_gyro_bias();               /* robot must be stationary here          */
  seq_skip_gyro_cal = true;
  head_residual_pending = false;       /* no carry into the run's first primitive */
}
void Control_RunEnd(void)
{
  seq_skip_gyro_cal = false;
}

bool Control_Start(int target)
{
  if (!Battery_AllowHighCurrent())
    return false;

  Motor_SetDuty(MOTOR_1, 0);
  Motor_SetDuty(MOTOR_2, 0);

  if (!seq_skip_gyro_cal)             /* mid auto-run: bias already sampled once */
    calibrate_gyro_bias();             /* robot must be still here            */

  pid_reset(&pid_lvel);
  pid_reset(&pid_rvel);
  pid_reset(&pid_head);
  /* hcarry: mid-run, seed the residual from the previous turn instead of zeroing so
     this segment drives the under-rotate debt back to 0 (head_setpoint stays 0). */
  heading_deg     = (head_carry_en && seq_skip_gyro_cal && head_residual_pending)
                    ? head_residual : 0.0f;
  head_residual_pending = false;
  enc_diff_counts = 0;
  picked_up       = false;             /* fresh run clears any prior lift latch */
  pickup_ticks    = 0;
  tick_ms         = 0;
  telem_decim     = 0;
  ramp_cps        = 0.0f;
  vL_filt = vR_filt = 0.0f;
  pos_l = pos_r = 0;
  vel_idx = 0;
  for (unsigned i = 0; i < VEL_WIN_MAX; i++) { vel_hist_l[i] = 0; vel_hist_r[i] = 0; }
  last_cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encL);
  last_cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encR);

  target_cps    = (float)target;
  move_mode     = false;                /* plain drive = no distance limit      */
  nav_mode      = false;                /* and not a cell-counted advance       */
  pivot_mode    = false;                /* and not a turn                       */
  arc_mode      = false;                /* and not a smooth arc                 */
  head_setpoint = 0.0f;                 /* hold heading straight                */
  active = true;                        /* ISR starts driving next tick        */
  return true;
}

bool Control_StartPivot(int degrees, int rate_dps)
{
  if (!Control_Start(0)) return false;  /* battery gate + full reset, fwd spd 0  */

  if (rate_dps > 0) turn_rate_dps = (float)rate_dps;
  head_target   = (float)degrees;
  last_turn_cmd_deg = (float)degrees;        /* remembered for a turncal afterwards */
  turn_sgn      = (degrees >= 0) ? 1.0f : -1.0f;
  turn_sp_mag   = 0.0f;
  turn_rate_mag = 0.0f;
  head_setpoint = 0.0f;
  turn_done_flag = false;
  turn_peak_dps  = 0.0f;                /* fresh per-turn diagnostic capture     */
  arc_pivot_finish = false;             /* plain pivot, not a flow-turn finish   */
  pos_ref = 0.0f;                       /* Control_Start zeroed pos_l/pos_r      */
  pid_head.out_min = -PIVOT_TRIM_MAX_CPS;  /* give the turn room to spin quickly  */
  pid_head.out_max =  PIVOT_TRIM_MAX_CPS;
  pivot_mode = true;                    /* latch AFTER Control_Start cleared it  */
  return true;
}

bool Control_StartSmoothTurn(int degrees, int target_cps, int radius_mm, int exit_mm)
{
  if (radius_mm <= 0 || target_cps <= 0) return false;
  if (!Control_Start(target_cps)) return false; /* battery gate + reset; fwd=target */

  head_target       = (float)degrees;
  last_turn_cmd_deg = (float)degrees;          /* lets a turncal use the arc too   */
  turn_sgn          = (degrees >= 0) ? 1.0f : -1.0f;
  arc_radius_mm     = (float)radius_mm;
  /* exit_mm means "straight after the IDEAL arc end". The omega ramp-down eats
     arc_trans_mm/2 of floor past that ideal end (it's inside arc_dist_counts),
     so shorten the exit leg to match - total distance from the curve start stays
     entry + R*theta + exit, exactly the pure-arc geometry rule the profiles seed
     (entry = exit = pitch - R). Floored at 1 count so a flow turn's exit branch
     (selected by arc_exit_counts > 0) can't collapse to the bench branch. */
  arc_exit_counts   = 0.0f;
  if (exit_mm > 0)
  {
    arc_exit_counts = ((float)exit_mm - 0.5f * arc_trans_mm) * COUNTS_PER_MM;
    if (arc_exit_counts < 1.0f) arc_exit_counts = 1.0f;
  }

  /* Trapezoidal-omega profile for this arc, parameterized by THE ACTUAL SPEED:
       omega = v / R                 (deg/s - the hold rate that traces radius R)
       alpha = omega * v / trans_mm  (deg/s^2 - so the 0->omega ramp always spans
                                      arc_trans_mm of floor at any speed: the
                                      drawn path is speed-invariant)
     Stored in their OWN fields - writing turn_rate_dps here silently slowed all
     later pivots to the arc omega (found in the 2026-07 rework). */
  {
    float v_mmps    = (float)target_cps / COUNTS_PER_MM;
    float omega_dps = (v_mmps / (float)radius_mm) * (180.0f / 3.14159265f);
    arc_omega_dps   = omega_dps;
    arc_alpha_dps2  = (arc_trans_mm > 1.0f) ? (omega_dps * v_mmps / arc_trans_mm)
                                            : (omega_dps * v_mmps);
  }
  /* Forward distance the curve covers. Each omega ramp spans arc_trans_mm but
     only sweeps HALF the angle a constant-omega arc would over that length, so
     the full curve = ideal arc length R*theta PLUS one transition length total.
     Used as the completion bound so a hunting heading can't overshoot the cell. */
  {
    int   adeg      = (degrees < 0) ? -degrees : degrees;
    float theta_rad = (float)adeg * (3.14159265f / 180.0f);
    arc_dist_counts = ((float)radius_mm * theta_rad + arc_trans_mm) * COUNTS_PER_MM;
  }
  arc_entry_counts  = 0.0f;                     /* bench arc: no lead-in            */
  arc_continue      = false;                    /* bench arc: stop at the end       */
  arc_ticks         = 0;                         /* fresh arc timeout                */
  turn_sp_mag       = 0.0f;
  turn_rate_mag     = 0.0f;
  head_setpoint     = 0.0f;
  turn_done_flag    = false;
  arc_pivot_finish  = false;                    /* fresh arc, no finish pending     */
  pos_ref           = 0.0f;                     /* Control_Start zeroed pos_l/pos_r */
  arc_err_filt      = 0.0f;                     /* fresh tracker state for the arc  */
  arc_err_prev      = 0.0f;
  arc_settle_ticks  = 0;                        /* require a fresh settle window    */
  pid_head.out_min  = -PIVOT_TRIM_MAX_CPS;      /* room for the arc differential    */
  pid_head.out_max  =  PIVOT_TRIM_MAX_CPS;
  arc_mode = true;                              /* latch AFTER Control_Start clears  */
  return true;
}

/* Flow (maze) smooth turn: lead-in straight (entry_mm) -> arc through <degrees> at
   radius_mm -> lead-out straight (exit_mm). The lead-in is what lets the curve start
   a radius BEFORE the corner-cell centre so the mouse lands centred on the next
   corridor. cont=false stops at the end (bench geometry test); cont=true re-zeros
   the heading and resumes a normal forward flow at the end (the solver fast run, so
   it flows on without stopping). Like the bench 'arc' it ramps up from rest here;
   the solver positions the mouse a cell before the corner and lets the lead-in carry
   it to the curve. */
bool Control_StartFlowTurn(int degrees, int target_cps, int radius_mm,
                           int entry_mm, int exit_mm, bool cont)
{
  if (radius_mm <= 0 || target_cps <= 0 || exit_mm <= 0) return false;
  if (!Control_StartSmoothTurn(degrees, target_cps, radius_mm, exit_mm)) return false;
  /* StartSmoothTurn set the arc up (incl. arc_exit_counts) + zeroed entry/continue;
     layer the flow extras on top. A flow turn always has an exit leg so 'cont' has a
     straight to settle the rotation + re-zero the heading on. entry_mm means
     "straight to the IDEAL arc start": the omega ramp-up begins arc_trans_mm/2
     BEFORE that point (transition straddles it), so shorten the lead-in to match. */
  arc_entry_counts = 0.0f;
  if (entry_mm > 0)
  {
    arc_entry_counts = ((float)entry_mm - 0.5f * arc_trans_mm) * COUNTS_PER_MM;
    if (arc_entry_counts < 0.0f) arc_entry_counts = 0.0f;
  }
  arc_continue     = cont;
  return true;
}

/* INLINE flow turn - start a flow turn FROM THE CURRENT MOTION, without stopping or
   resetting the forward speed/gyro. This is what lets the fast run curve through a
   corner non-stop: the solver calls it mid-flow (the mouse already cruising and
   holding a straight heading), the lead-in carries the curve start to a radius
   before the corner-cell centre, then arc + exit, then (cont) resumes the forward
   flow. Heading is re-zeroed to 0 here: the mouse has been driving straight so it's
   already ~0, making this a clean frame shift with no kick. Must be called while a
   run is active. */
bool Control_StartFlowTurnInline(int degrees, int cps, int radius_mm,
                                 int entry_mm, int exit_mm, bool cont)
{
  if (!active) return false;                       /* must already be running       */
  if (radius_mm <= 0 || cps <= 0 || exit_mm <= 0) return false;

  /* Compute the speed-dependent values FIRST (outside the critical section).
     Same trapezoidal-omega parameterization as StartSmoothTurn: omega = v/R,
     alpha spans arc_trans_mm of floor, curve length = R*theta + trans, and the
     entry/exit legs give back trans/2 each so E lands where the pure-arc
     geometry rule (entry = exit = pitch - R) says it should. */
  float v_mmps    = (float)cps / COUNTS_PER_MM;
  float omega_dps = (v_mmps / (float)radius_mm) * (180.0f / 3.14159265f);
  float alpha_dps2= (arc_trans_mm > 1.0f) ? (omega_dps * v_mmps / arc_trans_mm)
                                          : (omega_dps * v_mmps);
  int   adeg      = (degrees < 0) ? -degrees : degrees;
  float theta_rad = (float)adeg * (3.14159265f / 180.0f);
  float dist_cnt  = ((float)radius_mm * theta_rad + arc_trans_mm) * COUNTS_PER_MM;
  float entry_cnt = 0.0f;
  if (entry_mm > 0)
  {
    entry_cnt = ((float)entry_mm - 0.5f * arc_trans_mm) * COUNTS_PER_MM;
    if (entry_cnt < 0.0f) entry_cnt = 0.0f;
  }
  float exit_cnt  = ((float)exit_mm - 0.5f * arc_trans_mm) * COUNTS_PER_MM;
  if (exit_cnt < 1.0f) exit_cnt = 1.0f;

  /* Commit ALL the loop state in one shot with the 1 kHz control ISR masked: this is
     started mid-flow while the ISR is actively driving, so a torn read (e.g. heading
     zeroed but arc_mode still false) would jerk the motors for a tick. Mask, swap
     the whole mode atomically, unmask. */
  __disable_irq();
  head_target       = (float)degrees;
  last_turn_cmd_deg = (float)degrees;
  turn_sgn          = (degrees >= 0) ? 1.0f : -1.0f;
  arc_radius_mm     = (float)radius_mm;
  arc_entry_counts  = entry_cnt;
  arc_exit_counts   = exit_cnt;
  arc_continue      = cont;
  arc_omega_dps     = omega_dps;
  arc_alpha_dps2    = alpha_dps2;
  arc_dist_counts   = dist_cnt;
  turn_sp_mag       = 0.0f;
  turn_rate_mag     = 0.0f;
  arc_err_filt      = 0.0f;
  arc_err_prev      = 0.0f;
  arc_settle_ticks  = 0;
  arc_ticks         = 0;               /* fresh arc-local timeout                    */
  /* INLINE: keep ramp_cps (current cruise) - do NOT reset it (no Control_Start). */
  heading_deg       = 0.0f;            /* clean arc reference (we were going straight)*/
  enc_diff_counts   = 0;
  /* Measure the flow turn from HERE via the reference - NEVER zero pos_l/pos_r
     mid-run: the velocity window still holds the pre-turn totals and would read a
     huge garbage velocity for vel_win ms (motor slam + fake brake + wheel-spin =
     every fast-run corner crash; see pos_ref above). */
  pos_ref           = 0.5f * (float)(pos_l + pos_r);
  target_cps        = (float)cps;      /* hold cruise through the turn               */
  head_setpoint     = 0.0f;
  pid_reset(&pid_head);
  pid_head.out_min  = -PIVOT_TRIM_MAX_CPS;
  pid_head.out_max  =  PIVOT_TRIM_MAX_CPS;
  move_mode  = false;
  nav_mode   = false;
  pivot_mode = false;
  turn_done_flag = false;
  arc_pivot_finish = false;            /* fresh arc, no finish pending               */
  arc_mode = true;                     /* latch last                                 */
  __enable_irq();
  return true;
}

bool Control_PopTurnDone(int32_t *deg_turned)
{
  if (!turn_done_flag) return false;
  turn_done_flag = false;
  if (deg_turned) *deg_turned = turn_done_deg;
  return true;
}

/* Read the diagnostics latched at the LAST turn completion (non-draining; call
   right after Control_PopTurnDone returns true). All angles/rate in deci-units
   (x10). reason: 0 arrived clean, 1 snappy early-release, 2 timeout. */
void Control_GetLastTurnDiag(int32_t *cmd_ddeg, int32_t *ach_ddeg,
                             int32_t *peak_ddps, int32_t *ms, int32_t *reason,
                             int32_t *posL, int32_t *posR)
{
  if (cmd_ddeg)  *cmd_ddeg  = td_cmd_ddeg;
  if (ach_ddeg)  *ach_ddeg  = td_ach_ddeg;
  if (peak_ddps) *peak_ddps = td_peak_ddps;
  if (ms)        *ms        = td_ms;
  if (reason)    *reason    = td_reason;
  if (posL)      *posL      = td_posL;
  if (posR)      *posR      = td_posR;
}

void Control_SetTurn(float rate_dps, float accel_dps2)
{
  if (rate_dps   > 0.0f) turn_rate_dps   = rate_dps;
  if (accel_dps2 > 0.0f) turn_accel_dps2 = accel_dps2;
}

void Control_GetTurn(float *rate_dps, float *accel_dps2)
{
  if (rate_dps)   *rate_dps   = turn_rate_dps;
  if (accel_dps2) *accel_dps2 = turn_accel_dps2;
}

/* Turn mode + smooth-turn geometry for the autonomous run. The active profile sets
   these (Settings_Apply); the solver reads them to pick pivot vs smooth turns and
   how to shape a smooth turn. Live state only - persisted via the flash profiles. */
void Control_SetTurnMode(int mode) { turn_mode_smooth = (mode != 0); }
int  Control_GetTurnMode(void)     { return turn_mode_smooth ? 1 : 0; }

void Control_SetArcGeom(float radius_mm, float entry_mm, float exit_mm)
{
  if (radius_mm > 0.0f) arc_geom_radius = radius_mm;
  if (entry_mm  >= 0.0f) arc_geom_entry = entry_mm;
  if (exit_mm   >= 0.0f) arc_geom_exit  = exit_mm;
}
void Control_GetArcGeom(float *radius_mm, float *entry_mm, float *exit_mm)
{
  if (radius_mm) *radius_mm = arc_geom_radius;
  if (entry_mm)  *entry_mm  = arc_geom_entry;
  if (exit_mm)   *exit_mm   = arc_geom_exit;
}

/* Smooth-turn transition length (mm) - see control.h. Applied at the NEXT arc
   start (each start derives alpha from it), so it's safe to tune live. */
void Control_SetArcTrans(float mm)
{
  if (mm >= 2.0f && mm <= 80.0f) arc_trans_mm = mm;
}
float Control_GetArcTrans(void) { return arc_trans_mm; }

/* Per-direction arc feed-forward. SetArcFF sets BOTH (back-compat / "same both
   ways"); the L/R setters tune each side for the turn asymmetry. */
void  Control_SetArcFF(float ff)      { if (ff >= 0.0f) { arc_ff_l = ff; arc_ff_r = ff; } }
void  Control_SetArcFFLeft(float ff)  { if (ff >= 0.0f) arc_ff_l = ff; }
void  Control_SetArcFFRight(float ff) { if (ff >= 0.0f) arc_ff_r = ff; }
float Control_GetArcFF(void)          { return arc_ff_l; }   /* representative      */
float Control_GetArcFFLeft(void)      { return arc_ff_l; }
float Control_GetArcFFRight(void)     { return arc_ff_r; }

/* Arc heading-correction gain (cps trim per deg of heading error). Gentle by
   design - the curvature feed-forward carries the turn; this only nulls drift. */
void  Control_SetArcKp(float kp) { if (kp >= 0.0f) arc_kp = kp; }
float Control_GetArcKp(void)     { return arc_kp; }

/* Arc heading damping gain (D on the filtered error). Raise to kill overshoot/
   hunt in the turn; on the filtered error so it doesn't reintroduce the buzz. */
void  Control_SetArcKd(float kd) { if (kd >= 0.0f) arc_kd = kd; }
float Control_GetArcKd(void)     { return arc_kd; }

/* Arc gyro-rate damping (duty per deg/s of rate error) - the fast, direct-to-motor
   damping that replaces the cascade-laggy arc_kd. Raise to settle the turn / break
   stiction; too high will oscillate (but fast, gyro-limited). */
void  Control_SetArcGyroKd(float k) { if (k >= 0.0f) arc_gyro_kd = k; }
float Control_GetArcGyroKd(void)    { return arc_gyro_kd; }

/* Heading-PID derivative low-pass coefficient (0..1). Smaller = smoother (less
   motor buzz) but laggier damping; 1.0 = raw gyro D (the old buzzy behaviour). */
void  Control_SetHeadDFilt(float alpha)
{
  if (alpha > 0.0f && alpha <= 1.0f) pid_head.d_alpha = alpha;
}
float Control_GetHeadDFilt(void) { return pid_head.d_alpha; }

/* --- gyro turn-scale calibration ------------------------------------------
 * Heading is gyro-integrated with gyro_scale (LSB per dps). A part-to-part
 * sensitivity error shows up as every turn landing a fixed % short or long, so
 * the mouse ends each pivot skewed. Correct it from a known turn: command a
 * rotation (e.g. 'turn 360'), measure the ACTUAL physical angle, then
 * 'gyroscale cal <actual>'. We scale by commanded/actual: the loop integrates to
 * the commanded value, so if it physically moved less, the sensitivity is higher
 * than assumed and the scale goes up (and vice-versa). */
float Control_GetGyroScale(void) { return gyro_scale; }

void Control_SetGyroScale(float lsb_per_dps)
{
  if (lsb_per_dps >= 8.0f && lsb_per_dps <= 33.0f) gyro_scale = lsb_per_dps;
}

bool Control_TurnCalFromActual(float actual_deg)
{
  float a = actual_deg  < 0.0f ? -actual_deg  : actual_deg;
  float c = last_turn_cmd_deg < 0.0f ? -last_turn_cmd_deg : last_turn_cmd_deg;
  if (a < 1.0f || c < 1.0f) return false;          /* need a real prior turn       */
  float ns = gyro_scale * (c / a);
  if (ns < 8.0f || ns > 33.0f) return false;        /* implausible - reject         */
  gyro_scale = ns;
  return true;
}

/* --- front-wall align ------------------------------------------------------ */
bool Control_StartFrontAlign(void)
{
  if (!Battery_AllowHighCurrent()) return false;

  /* Need the front reference (the reading at stop distance) as our target. */
  int ref[SENSOR_COUNT]; uint16_t flags = 0;
  Sensors_GetCal(NULL, ref, NULL, &flags);
  if (!(flags & 0x4)) return false;            /* no front cal -> can't square  */
  align_lref = ref[SENS_L_F];
  align_rref = ref[SENS_R_F];

  /* Light reset (no gyro cal needed - align uses the IR, not the gyro). */
  Motor_SetDuty(MOTOR_1, 0);
  Motor_SetDuty(MOTOR_2, 0);
  pid_reset(&pid_lvel);
  pid_reset(&pid_rvel);
  ramp_cps   = 0.0f;
  vL_filt = vR_filt = 0.0f;
  pos_l = pos_r = 0;
  vel_idx = 0;
  for (unsigned i = 0; i < VEL_WIN_MAX; i++) { vel_hist_l[i] = 0; vel_hist_r[i] = 0; }
  last_cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encL);
  last_cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encR);
  tick_ms      = 0;
  align_settle = 0;

  move_mode = nav_mode = pivot_mode = false;
  align_done_flag = false;
  align_mode = true;
  active     = true;                            /* ISR starts squaring next tick */
  return true;
}

bool Control_AlignActive(void) { return align_mode; }

bool Control_PopAlignDone(bool *ok)
{
  if (!align_done_flag) return false;
  align_done_flag = false;
  if (ok) *ok = align_done_ok;
  return true;
}

void Control_SetAlign(float kp_skew, float kp_dist)
{
  /* 0 = leave unchanged. A NEGATIVE skew gain is allowed: it flips the rotate
     direction, the fix if 'align' squares the wrong way on your geometry. */
  if (kp_skew != 0.0f) align_kp_skew = kp_skew;
  if (kp_dist != 0.0f) align_kp_dist = kp_dist;
}

/* --- recenter (reverse a fixed distance) ----------------------------------- */
bool Control_StartRecenter(int mm)
{
  if (mm <= 0) return false;
  if (!Battery_AllowHighCurrent()) return false;

  Motor_SetDuty(MOTOR_1, 0);
  Motor_SetDuty(MOTOR_2, 0);
  pid_reset(&pid_lvel);
  pid_reset(&pid_rvel);
  ramp_cps = 0.0f;
  vL_filt = vR_filt = 0.0f;
  pos_l = pos_r = 0;
  vel_idx = 0;
  for (unsigned i = 0; i < VEL_WIN_MAX; i++) { vel_hist_l[i] = 0; vel_hist_r[i] = 0; }
  last_cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encL);
  last_cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encR);
  tick_ms = 0;

  recenter_target_counts = (float)mm * COUNTS_PER_MM;
  move_mode = nav_mode = pivot_mode = align_mode = false;
  recenter_done_flag = false;
  recenter_mode = true;
  active        = true;
  return true;
}

bool Control_RecenterActive(void) { return recenter_mode; }

bool Control_PopRecenterDone(void)
{
  if (!recenter_done_flag) return false;
  recenter_done_flag = false;
  return true;
}

/* --- contact probe --------------------------------------------------------- */
bool Control_StartContactProbe(void)
{
  if (!Battery_AllowHighCurrent()) return false;

  /* "At the wall" = front reading well above the calibrated stop-distance ref. */
  int ref[SENSOR_COUNT]; uint16_t flags = 0;
  Sensors_GetCal(NULL, ref, NULL, &flags);
  if (!(flags & 0x4)) return false;
  int hi = (ref[SENS_L_F] > ref[SENS_R_F]) ? ref[SENS_L_F] : ref[SENS_R_F];
  contact_level = 2 * hi;
  if (contact_level < 200) contact_level = 200;

  Motor_SetDuty(MOTOR_1, 0);
  Motor_SetDuty(MOTOR_2, 0);
  pid_reset(&pid_lvel);
  pid_reset(&pid_rvel);
  ramp_cps = 0.0f;
  vL_filt = vR_filt = 0.0f;
  pos_l = pos_r = 0;
  vel_idx = 0;
  for (unsigned i = 0; i < VEL_WIN_MAX; i++) { vel_hist_l[i] = 0; vel_hist_r[i] = 0; }
  last_cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encL);
  last_cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encR);
  tick_ms = 0;

  move_mode = nav_mode = pivot_mode = align_mode = recenter_mode = false;
  contact_done_flag = false;
  contact_mm = 0;
  contact_mode = true;
  active       = true;
  return true;
}

bool Control_ContactActive(void) { return contact_mode; }

bool Control_PopContactDone(int32_t *mm)
{
  if (!contact_done_flag) return false;
  contact_done_flag = false;
  if (mm) *mm = contact_mm;
  return true;
}

void Control_GetAlign(float *kp_skew, float *kp_dist)
{
  if (kp_skew) *kp_skew = align_kp_skew;
  if (kp_dist) *kp_dist = align_kp_dist;
}

bool Control_StartMove(int distance_mm, int target_cps)
{
  if (distance_mm <= 0) return false;
  if (!Control_Start(target_cps)) return false;  /* battery gate + full reset   */

  /* Control_Start zeroed pos_l/pos_r, so distance is measured from here. The
     one tick before move_mode latches is at rest, so it travels nothing. */
  move_target_counts = (float)distance_mm * COUNTS_PER_MM;
  move_braking   = false;
  move_hit_wall  = false;
  move_stalled   = false;
  stall_ticks    = 0;
  stall_prog_max   = 0.0f;
  stall_prog_ticks = 0;
  move_done_flag = false;
  move_mode = true;
  return true;
}

bool Control_PopMoveDone(int32_t *mm_travelled)
{
  if (!move_done_flag) return false;
  move_done_flag = false;
  if (mm_travelled) *mm_travelled = move_done_mm;
  return true;
}

bool Control_LastMoveHitWall(void) { return move_hit_wall; }

bool Control_LastMoveStalled(void) { return move_stalled; }

bool Control_PickupDetected(void) { return picked_up; }

void Control_GetState(int32_t *posL, int32_t *posR, float *heading_deg_out,
                      float *gyroz_dps, float *accelz_g)
{
  if (posL)            *posL            = pos_l;
  if (posR)            *posR            = pos_r;
  if (heading_deg_out) *heading_deg_out = heading_deg;
  if (gyroz_dps)       *gyroz_dps       = gyro_dps;
  if (accelz_g)        *accelz_g        = last_accel_z;
}

void Control_SetFrontStop(bool enable) { front_stop_en = enable; }

/* Snappy pivots for the fast run: finish a pivot as soon as it's commanded-done and
   the heading has essentially arrived, instead of sitting still to fine-adjust - the
   next cell's forward flow trims the residual. Off = the tight search completion. */
void Control_SetPivotSnappy(bool on) { pivot_snappy = on; }
bool Control_GetPivotSnappy(void)    { return pivot_snappy; }

/* Heading carry (hcarry): carry a turn's under-rotate residual into the next
   mid-run segment instead of re-zeroing the heading reference. Default ON = the
   drift fix; OFF restores the old per-segment zero-reset for an A/B bench compare. */
void Control_SetHeadingCarry(bool on) { head_carry_en = on; if (!on) head_residual_pending = false; }
bool Control_GetHeadingCarry(void)    { return head_carry_en; }

bool Control_StartAdvance(int cells, int target_cps)
{
  if (cells <= 0) return false;
  /* The advance is a single continuous move of cells*pitch - it never brakes at
     the intermediate boundaries, so the mouse flows through the corridor and
     only eases to a stop at the last cell. StartMove does the distance/braking;
     we just layer the per-cell marks + front-wall stop on top. */
  int mm = (int)((float)cells * nav_cell_mm);
  if (mm <= 0) return false;

  /* Per-segment velocity cap (FAST RUN, move_creep_en): never cruise faster than we
     can brake to a stop within THIS advance. A short 1-cell flow into a turn cell was
     accelerating all the way to run_cps and then couldn't brake in the distance left,
     so it arrived nose-to-wall and sometimes clipped. For a start->stop trapezoid over
     distance D with accel a, decel d the peak reachable-AND-stoppable speed is
     sqrt(2 D a d/(a+d)); we use a FRACTION of the commanded decel because the dry
     wheels slip (real decel < commanded), so the cap leaves genuine braking margin.
     Long straights compute a cap ABOVE run_cps, so they still cruise at full speed -
     only short turn-approaches are slowed, and only as much as physics demands.
     Search (move_creep_en=false) is never capped. */
  if (move_creep_en && accel_cps > 0.0f && decel_cps > 0.0f && COUNTS_PER_MM > 0.0f)
  {
    float D     = (float)mm * COUNTS_PER_MM;
    float d     = decel_cps * VPROF_DECEL_FRAC;
    float vpeak = sqrtf(2.0f * D * accel_cps * d / (accel_cps + d));
    if ((float)target_cps > vpeak) target_cps = (int)vpeak;
  }

  /* The advance is a single continuous move of cells*pitch - it never brakes at
     the intermediate boundaries, so the mouse flows through the corridor and
     only eases to a stop at the last cell. StartMove does the distance/braking;
     we just layer the per-cell marks + front-wall stop on top. */
  if (!Control_StartMove(mm, target_cps)) return false;  /* battery gate + reset */

  nav_cells_total  = cells;
  nav_cells_marked = 0;
  nav_mark_flag    = false;
  nav_mode         = true;               /* latch AFTER StartMove cleared it     */
  return true;
}

bool Control_PopCellMark(int32_t *cell_index)
{
  if (!nav_mark_flag) return false;
  nav_mark_flag = false;
  if (cell_index) *cell_index = nav_mark_cell;
  return true;
}

void Control_AdvanceBrakeNow(void)
{
  /* Make the in-flight advance brake to a stop now - used by the solver to halt
     at a decision cell while flowing through straights. Same active-brake path as
     the front-wall stop: ramp the setpoint to 0 and let the stop-when-slow finish
     fire move_done. No effect if no distance-limited move is running. */
  if (move_mode) { target_cps = 0.0f; move_braking = true; }
}

void Control_SetCellMM(float mm) { if (mm > 0.0f) nav_cell_mm = mm; }
float Control_GetCellMM(void)    { return nav_cell_mm; }

/* --- nav sequencer (path / solver) --------------------------------------- */
void Control_NavReset(void)
{
  nav_seq_len      = 0;
  nav_seq_idx      = 0;
  nav_seq_active   = false;
  nav_step_running = false;
}

bool Control_NavAddAdvance(int cells, int cps)
{
  if (cells <= 0 || nav_seq_len >= NAV_SEQ_MAX) return false;
  nav_seq[nav_seq_len].type = NAV_STEP_ADV;
  nav_seq[nav_seq_len].arg  = (int16_t)cells;
  nav_seq[nav_seq_len].spd  = (int16_t)cps;
  nav_seq_len++;
  return true;
}

bool Control_NavAddTurn(int degrees, int rate_dps)
{
  if (degrees == 0 || nav_seq_len >= NAV_SEQ_MAX) return false;
  nav_seq[nav_seq_len].type = NAV_STEP_TURN;
  nav_seq[nav_seq_len].arg  = (int16_t)degrees;
  nav_seq[nav_seq_len].spd  = (int16_t)rate_dps;
  nav_seq_len++;
  return true;
}

bool Control_NavStart(void)
{
  if (nav_seq_len == 0) return false;
  if (!Battery_AllowHighCurrent()) return false;
  Control_RunBegin();                /* one gyro cal now; steps skip per-start recal */
  nav_seq_idx       = 0;
  nav_step_running  = false;
  nav_seq_done_flag = false;
  nav_seq_active    = true;          /* Control_NavTask starts step 0 next pump */
  return true;
}

bool Control_NavActive(void) { return nav_seq_active; }

void Control_NavAbort(void)
{
  nav_seq_active    = false;
  nav_step_running  = false;
  Control_RunEnd();                   /* re-arm per-start recal for normal cmds   */
  Control_Stop();
}

/* Pump from the main loop: advance the queue as each step's done flag fires, and
   start the next. MUST run in main-loop context (each start blocks on gyro cal).
   While a sequence is active this is the sole consumer of Move/Turn done, so the
   console's own "MOVE/TURN done" prints stay quiet and we report at path level. */
void Control_NavTask(void)
{
  if (!nav_seq_active) return;

  /* Wait for the in-flight step to report done (drain its one-shot flag). */
  if (nav_step_running)
  {
    bool done;
    if (nav_seq[nav_seq_idx].type == NAV_STEP_ADV)
    {
      int32_t mm;  done = Control_PopMoveDone(&mm);
    }
    else
    {
      int32_t deg; done = Control_PopTurnDone(&deg);
    }
    if (!done) return;                       /* still moving                      */
    nav_step_running = false;
    nav_seq_idx++;
  }

  /* Sequence finished? */
  if (nav_seq_idx >= nav_seq_len)
  {
    nav_seq_active    = false;
    nav_seq_done_ok   = true;
    nav_seq_done_flag = true;
    Control_RunEnd();                 /* re-arm per-start recal for normal cmds   */
    return;
  }

  /* Start the next step (blocks ~200 ms on gyro cal - robot is stopped here). */
  NavStep *s = &nav_seq[nav_seq_idx];
  bool ok = (s->type == NAV_STEP_ADV)
          ? Control_StartAdvance((int)s->arg, (int)s->spd)
          : Control_StartPivot((int)s->arg, (int)s->spd);
  if (!ok)                                   /* battery gate etc.: abort cleanly  */
  {
    nav_seq_active    = false;
    nav_seq_done_ok   = false;
    nav_seq_done_flag = true;
    Control_RunEnd();                       /* re-arm per-start recal             */
    return;
  }
  nav_step_running   = true;
  nav_evt_start_idx  = nav_seq_idx;
  nav_evt_start_flag = true;
}

bool Control_NavPopStepStart(int *idx, int *total, int *type, int *arg)
{
  if (!nav_evt_start_flag) return false;
  nav_evt_start_flag = false;
  uint8_t i = nav_evt_start_idx;
  if (idx)   *idx   = (int)i;
  if (total) *total = (int)nav_seq_len;
  if (type)  *type  = (int)nav_seq[i].type;
  if (arg)   *arg   = (int)nav_seq[i].arg;
  return true;
}

bool Control_NavPopDone(bool *ok)
{
  if (!nav_seq_done_flag) return false;
  nav_seq_done_flag = false;
  if (ok) *ok = nav_seq_done_ok;
  return true;
}

void Control_Stop(void)
{
  active = false;
  at_active = false;                    /* also aborts a running auto-tune     */
  sg_active = false;                    /* and a spin-grip test                */
  move_mode = false;                    /* and a distance-limited move         */
  nav_mode = false;                     /* and a cell-counted advance          */
  pivot_mode = false;                   /* and a pivot turn                    */
  arc_mode = false;                     /* and a smooth arc turn               */
  head_setpoint = 0.0f;                 /* back to straight-hold for next run  */
  pid_head.out_min = -HEAD_TRIM_MAX_CPS; /* restore the gentle straightness clamp */
  pid_head.out_max =  HEAD_TRIM_MAX_CPS;
  target_cps = 0.0f;
  Motor_SetDuty(MOTOR_1, 0);
  Motor_SetDuty(MOTOR_2, 0);
}

void Control_SetSpeed(int target)
{
  if (active) target_cps = (float)target;
}

bool Control_IsActive(void) { return active; }

float Control_GetWheelSpeedCps(void)
{
  if (!active) return 0.0f;                /* idle -> report 0 (clean sensor timing) */
  float v = 0.5f * (vL_filt + vR_filt);
  return (v < 0.0f) ? -v : v;
}

float Control_GetCountsPerMM(void)
{
  return COUNTS_PER_MM;
}

void Control_SetGains(CtrlLoop loop, float kp, float ki, float kd)
{
  switch (loop)
  {
    case CTRL_LOOP_LVEL: pid_lvel.kp = kp; pid_lvel.ki = ki; pid_lvel.kd = kd; break;
    case CTRL_LOOP_RVEL: pid_rvel.kp = kp; pid_rvel.ki = ki; pid_rvel.kd = kd; break;
    case CTRL_LOOP_HEAD: pid_head.kp = kp; pid_head.ki = ki; pid_head.kd = kd; break;
    case CTRL_LOOP_VEL:
    default:
      pid_lvel.kp = pid_rvel.kp = kp;
      pid_lvel.ki = pid_rvel.ki = ki;
      pid_lvel.kd = pid_rvel.kd = kd;
      break;
  }
}

void Control_GetGains(CtrlLoop loop, float *kp, float *ki, float *kd)
{
  PID *p = (loop == CTRL_LOOP_RVEL) ? &pid_rvel
         : (loop == CTRL_LOOP_HEAD) ? &pid_head
         : &pid_lvel;
  if (kp) *kp = p->kp;
  if (ki) *ki = p->ki;
  if (kd) *kd = p->kd;
}

void Control_SetVelFF(float kff) { vel_ff = kff; }
float Control_GetVelFF(void)      { return vel_ff; }

void Control_SetFollow(bool on)   { follow_enabled = on; }
bool Control_GetFollow(void)      { return follow_enabled; }

void Control_SetCenterGain(float kp)
{
  if (kp >= -1.0f && kp <= 1.0f) center_kp = kp;  /* negative flips steer direction */
}
float Control_GetCenterGain(void) { return center_kp; }

void Control_SetCenterDamp(float kd)
{
  if (kd >= 0.0f && kd <= 1.0f) center_kd = kd;   /* 0 = plain proportional centring */
}
float Control_GetCenterDamp(void) { return center_kd; }

void Control_SetAccel(float accel, float decel)
{
  if (accel > 0.0f) accel_cps = accel;
  if (decel > 0.0f) decel_cps = decel;
}
void Control_GetAccel(float *accel, float *decel)
{
  if (accel) *accel = accel_cps;
  if (decel) *decel = decel_cps;
}

void  Control_SetBrakeReverse(float cps) { brake_rev_cps = (cps > 0.0f) ? cps : 0.0f; }
float Control_GetBrakeReverse(void)      { return brake_rev_cps; }

void  Control_SetReachCreep(bool en)     { move_creep_en = en; }

void Control_SetVelFilter(int window_ms, float lpf_alpha)
{
  if (window_ms >= 2 && window_ms <= 50)        vel_win = (uint8_t)window_ms;
  if (lpf_alpha > 0.0f && lpf_alpha <= 1.0f)    vel_lpf_alpha = lpf_alpha;
}
void Control_GetVelFilter(int *window_ms, float *lpf_alpha)
{
  if (window_ms) *window_ms = (int)vel_win;
  if (lpf_alpha) *lpf_alpha = vel_lpf_alpha;
}

bool Control_AutotuneStart(int target_cps)
{
  if (!Battery_AllowHighCurrent())
    return false;

  int sp = target_cps;
  if (sp < 500) sp = 3000;              /* sane default if unset / too low      */

  Motor_SetDuty(MOTOR_1, 0);
  Motor_SetDuty(MOTOR_2, 0);
  calibrate_gyro_bias();                /* keep the robot still here            */

  /* same measurement bring-up as Control_Start, minus arming the PIDs */
  heading_deg = 0.0f; enc_diff_counts = 0;
  tick_ms = 0; telem_decim = 0; ramp_cps = 0.0f;
  vL_filt = vR_filt = 0.0f; pos_l = pos_r = 0; vel_idx = 0;
  for (unsigned i = 0; i < VEL_WIN_MAX; i++) { vel_hist_l[i] = 0; vel_hist_r[i] = 0; }
  last_cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encL);
  last_cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(CT.htim_encR);

  /* centre the relay on the feedforward operating point, swing +/- around it */
  at_sp   = (float)sp;
  at_base = vel_ff * at_sp;
  at_h    = 0.45f * at_base;
  if (at_h < 250.0f) at_h = 250.0f;     /* guarantee a clear limit cycle        */
  at_relay    = +1;
  at_sw_count = 0; at_t_last_sw = 0;
  at_vmax = at_vmin = 0.0f;
  at_per_acc = 0.0f; at_per_n = 0;
  at_amp_acc = 0.0f; at_amp_n = 0;
  at_have_result = false;

  active    = true;
  at_active = true;                     /* ISR runs the relay next tick         */
  return true;
}

bool Control_AutotuneActive(void) { return at_active; }

bool Control_StartSpinGrip(int spin_dps)
{
  if (!Battery_AllowHighCurrent())
    return false;

  float tgt = (float)spin_dps;
  if (tgt < 120.0f) tgt = 400.0f;       /* sane default spin rate               */

  Motor_SetDuty(MOTOR_1, 0);
  Motor_SetDuty(MOTOR_2, 0);
  calibrate_gyro_bias();                /* keep the robot still here            */

  tick_ms       = 0;
  sg_target_dps = tgt;
  sg_duty       = 0;
  sg_omega0     = 0.0f;
  sg_phase      = SG_SPINUP;
  sg_have_result = false;

  active    = true;
  sg_active = true;                     /* ISR runs the spin+skid next tick     */
  return true;
}

bool Control_SpinGripActive(void) { return sg_active; }

bool Control_PopSpinGripResult(bool *ok, float *decel_dps2, float *omega0_dps)
{
  if (!sg_have_result) return false;
  sg_have_result = false;
  if (ok)         *ok         = sg_ok;
  if (decel_dps2) *decel_dps2 = sg_r_decel;
  if (omega0_dps) *omega0_dps = sg_r_omega0;
  return true;
}

bool Control_AutotunePopResult(bool *ok, float *kp, float *ki, float *kd,
                               float *ku, float *tu_ms)
{
  if (!at_have_result) return false;
  at_have_result = false;
  if (ok)    *ok    = at_ok;
  if (kp)    *kp    = at_r_kp;
  if (ki)    *ki    = at_r_ki;
  if (kd)    *kd    = at_r_kd;
  if (ku)    *ku    = at_r_ku;
  if (tu_ms) *tu_ms = at_r_tu * 1000.0f;
  return true;
}

void Control_SetTelem(CtrlLoop loop, bool on)
{
  telem_loop  = loop;
  telem_decim = 0;
  telem_head  = telem_tail = 0;         /* flush stale samples on (re)select   */
  telem_on    = on;
}

bool Control_PopTelem(uint32_t *t, int32_t *sp, int32_t *meas, int32_t *out)
{
  if (telem_tail == telem_head) return false;
  Telem r;
  r.t    = telem_buf[telem_tail].t;
  r.sp   = telem_buf[telem_tail].sp;
  r.meas = telem_buf[telem_tail].meas;
  r.out  = telem_buf[telem_tail].out;
  telem_tail = (uint16_t)((telem_tail + 1U) % TELEM_SZ);
  if (t)    *t    = r.t;
  if (sp)   *sp   = r.sp;
  if (meas) *meas = r.meas;
  if (out)  *out  = r.out;
  return true;
}
