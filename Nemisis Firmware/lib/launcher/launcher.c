/**
 ******************************************************************************
 * @file    launcher.c
 * @brief   Offline race-start sequencer - see launcher.h.
 ******************************************************************************
 */
#include "launcher.h"
#include "main.h"
#include "ws2812.h"
#include "buzzer.h"
#include "sensors.h"
#include "solver.h"
#include "console.h"
#include "control.h"
#include "mazestore.h"

/* --- tuning -------------------------------------------------------------- */
#define BTN_DEBOUNCE_MS   25U     /* button settle time                         */
#define BTN_LONG_MS      800U     /* hold this long = "long press" (cancel)     */
/* Button calibration (offline, no console): from the armed state a short press
   snapshots the side+front references off the wall the mouse is placed facing,
   then it pivots this far to end facing the open run direction (sampling the
   dark floor as it sweeps the opening). 180 deg flips a standard start cell from
   facing the back wall to facing the exit. */
#define CAL_PIVOT_DEG    180      /* turn to face the run direction after capture */
#define CAL_PIVOT_DPS      0      /* 0 = tuned default turn rate                  */
/* A hand reflects far less IR than a white wall, so the cover threshold is low.
   False triggers from a real front wall are prevented by the low->high EDGE (we
   only fire if the front was CLEAR first), not by a high level - so keep this low
   enough that a hand reliably crosses it. Raise if it fires too easily. */
#define GESTURE_HIGH      70      /* both front IR above this = hand over them    */
#define GESTURE_LOW       30      /* both below this = front clear (re-arm edge)  */
#define GESTURE_HOLD_MS  300U     /* sustain the cover this long to commit        */
#define COUNTDOWN_S        3U     /* go this many seconds after the gesture       */

/* === Offline run SPEED (the knobs to go faster) =============================
 * RUN_CPS is wheel speed in encoder counts/s. ~24500 cps = 1 m/s, so:
 *   8000 ~ 0.33 m/s (the proven baseline) | 10000 ~ 0.41 | 12000 ~ 0.49
 * Raise it gradually and re-test. THE CATCH: braking distance grows with speed^2
 * (brake_mm ~= RUN_CPS^2 / (2*RUN_DECEL) / 24.5). If you only raise speed, the
 * mouse overshoots each turn/decision cell and clips - so raise RUN_DECEL too,
 * roughly with speed^2, to keep the brake distance ~15-25 mm:
 *   12000 cps -> DECEL ~150000   |   14000 -> ~200000   (at 8000: 80000 is fine)
 * RUN_ACCEL just needs to be high enough to reach speed within a cell; matching
 * DECEL is fine. Beyond ~0.5-0.6 m/s the 5 ms IR sweep starts lagging the
 * centring/wall-sense - that's the next ceiling, fixed by the DMA sensor swap. */
/* SEARCH speed: must be slow enough that the mouse stays centred and reads every
   wall right - at 12000 (~0.49 m/s) it skews and the front-diagonal IR catches
   SIDE walls -> phantom walls box it in (flood 999 abort) before it reaches the
   goal. 8000 (~0.33 m/s) was the proven-clean value; 9000 is a small step up.
   Save the speed for the FAST run on a known map, not the search. */
#define RUN_CPS        8000      /* ~0.33 m/s proven-clean search (24500 cps = 1 m/s) */
#define RUN_ACCEL  100000.0f     /* setpoint accel (cps/s)                        */
#define RUN_DECEL  160000.0f     /* brake rate - raised from 100k: front-stop was */
                                 /* coasting the last bit into walls after long   */
                                 /* straights (nose-bump on decel). Harder brake  */
                                 /* sheds cruise speed in less distance.          */
/* Corridor centring strength: heading-offset degrees per count of side-wall
   lateral error. Default is 0.05; raise to pull back to centre harder if it
   drifts/oversteps in straights. Too high -> it weaves. Negative flips the steer
   direction (only if it ever steers INTO a wall). */
#define RUN_CENTER_KP   0.10f

/* === FAST (speed) run ======================================================
 * Triggered automatically after a search comes home: the launcher mutes comms,
 * plays a "map saved / path ready" jingle, then waits for the SAME hand gesture
 * to launch a fast run that replays the least-cost path on the learned map.
 * Brake distance grows with speed^2, so DECEL is scaled up to keep the
 * per-decision-cell brake ~15-25 mm. The fast run brakes PREDICTIVELY at every
 * turn from the known map (not off the late front-IR), so it doesn't have the
 * search-time nose-bump.
 *
 * VACUUM: DECOUPLED 2026-07-02 - the pivot fast run now runs DRY (vacuum off, see
 * RUN_FAST_USE_VACUUM in solver.c). Plan: tune the dry pivot run as fast as it goes
 * first, THEN bring the vacuum back for the smooth-turn run. So cruise is back to
 * the proven 12000 dry baseline; bench-tune higher live with `fast <cps>` on the
 * console before raising this constant (watch for pivot over/undershoot + wall-smack
 * on the brake-in). Solver_StartFast owns the fast decel + reverse brake now. */
#define RUN_CPS_FAST    21000     /* ~0.86 m/s (24500 cps = 1 m/s) offline fast run.
                                     Lowered from 24000 (2026-07-03); ACCEL/DECEL
                                     rescaled by (21/24)^2 so the accel ramp and the
                                     brake DISTANCE keep the same geometry.          */
#define RUN_FAST_ACCEL  145000.0f      /* was 190000 at 24000 cps                    */
/* Braking for the OFFLINE fast run (the launcher sets these before Solver_StartFast;
   the console fast run uses live `accel`/`brake` instead). Brake distance = v^2/
   (2*decel), so LOWER decel brakes EARLIER - too-high a decel brakes late and
   overshoots. RUN_FAST_BRAKE_REV adds active reverse
   torque for the last bit. Bake in whatever the bench `accel`/`brake` tuning wins. */
#define RUN_FAST_DECEL       80000.0f  /* was 105000: same ~112mm brake from cruise   */
/* 2026-07-05: halved 6000->3000 to cut WHEEL SLIP at the final stop. A too-hard
   reverse brake breaks traction just as the mouse comes to rest, so the encoders
   under-count (wheels skid) and the odometry 'centre' drifts ~20-30mm off physical
   centre -> the pivot fires off-centre and grazes a wall. A softer last-bit brake
   keeps the wheels gripping = truer odometry = lands nearer the real centre. Single
   reversible knob; if turns start OVER-shooting again, step it back up toward 6000. */
#define RUN_FAST_BRAKE_REV    3000.0f  /* active reverse-brake floor (cps); 0 = off  */
#define RUN_FAST_CENTER  0.10f

/* === Three-run session (gesture-sequenced) =================================
 * One button press starts a hands-off session that runs, each launched by its OWN
 * hand gesture and staying fully offline (comms muted) the whole time:
 *   run 0 = SEARCH  (explore + learn the map, gentle pivots)
 *   run 1 = FAST    (replay the least-cost path, snappy PIVOTS)
 *   run 2 = FASTEST (replay it again with SMOOTH/arc turns)
 * All three runs' logs are buffered per-run in RAM (Solver_LogSession*); after the
 * last run (or a pickup) the link comes back up so you can 'dump' all three from
 * the app. The FASTEST run needs smooth turns on, so we set the arc geometry here
 * (matches the Fastest profile's R90/entry90/exit90 for a 180 mm cell). */
#define RUN_ARC_R        90       /* smooth-turn radius (mm)                       */
#define RUN_ARC_ENTRY    90       /* lead-in straight before the arc (mm)          */
#define RUN_ARC_EXIT     90       /* lead-out straight after the arc (mm)          */
#define SESSION_LAST_RUN  2       /* run index of the final (fastest) run          */
/* FASTEST cruise is SLOWER than the pivot fast run. Data (run2_fastest 2026-07-01)
   showed the smooth turns UNDER-ROTATED at 12000 cps: the measured heading lagged
   the arc setpoint ~17 deg (73 of 90), the completion then hung to the 4 s timeout
   and the mouse drove on mis-headed -> drifted into walls -> a later turn spun out
   ("jizzel" = gyro oscillating +-300 deg/s then an accelZ impact). The rotation is
   a lag effect: at lower cruise the sweep spans more control ticks so the heading
   keeps up and the turn actually reaches 90. Start conservative and raise it once
   the arc FF (arcff) is tuned for the flowing run. */
#define RUN_CPS_FASTEST  9000     /* ~0.37 m/s - slower so smooth turns complete    */

/* === Offline run target - BOOT FALLBACK ONLY ================================
 * These seed the maze size + goal ONCE at boot (Launcher_Init), so a mouse that
 * never connects to the app still has a sensible target. The app's
 * 'maze'/'start'/'goal' commands (sent while connected) OVERRIDE these and
 * persist all the way into the offline run - the launcher no longer re-applies
 * them at GO. So the normal flow is: connect, set start + goal from the app,
 * disconnect, then trigger the offline run; it targets exactly what you set.
 * Size is the maze edge in cells; the goal is one cell (x,y) with x=East,
 * y=North. Set RUN_GOAL_X to -1 to default to the centre 2x2 block. The start
 * cell defaults to SW (0,0) facing N until the app's 'start' command changes it. */
#define RUN_MAZE_N         6      /* maze is RUN_MAZE_N x RUN_MAZE_N cells         */
#define RUN_GOAL_X        -1      /* goal cell X (East);  -1 = use centre 2x2      */
#define RUN_GOAL_Y         0      /* goal cell Y (North)                           */

/* Button is active-LOW with an internal pull-up (button shorts PD2 to GND). If
   yours pulls the other way, switch the pull below to GPIO_PULLDOWN and the
   released level auto-detects regardless. */ 
typedef enum { LA_IDLE = 0, LA_ARMED, LA_CAL_TURN, LA_COUNT, LA_RUN } LaState;

static LaState       st = LA_IDLE;

static GPIO_PinState btn_released;   /* auto-detected un-pressed level           */
static GPIO_PinState btn_stable;     /* debounced level                          */
static GPIO_PinState btn_last_raw;
static uint32_t      btn_t;
static bool          btn_press_evt;  /* one-shot: a fresh SHORT press (on release) */
static bool          btn_long_evt;   /* one-shot: held past BTN_LONG_MS            */
static uint32_t      btn_down_t;     /* tick the current press started            */
static bool          btn_long_sent;  /* long event already emitted this press      */

static int           cal_ref[SENSOR_COUNT];      /* side+front refs grabbed at the wall */
static int           cal_dark_min[SENSOR_COUNT]; /* running-min dark over the cal turn  */

static bool          front_clear;    /* saw the front go clear since arming       */
static uint32_t      gesture_t0;
static uint32_t      count_t0;
static uint8_t       count_shown;
static bool          fast_mode;       /* armed run is a FAST run, not a search     */
static bool          session;         /* in the Search->Fast->Fastest sequence     */
static uint8_t       run_idx;         /* current run of the session (0/1/2)        */

/* --- button: debounce + short (on release) / long (while held) events ----- */
static void btn_poll(void)
{
  btn_press_evt = false;
  btn_long_evt  = false;

  GPIO_PinState raw = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
  if (raw != btn_last_raw)
  {
    btn_last_raw = raw;
    btn_t = HAL_GetTick();
  }
  else if ((HAL_GetTick() - btn_t) >= BTN_DEBOUNCE_MS && raw != btn_stable)
  {
    GPIO_PinState prev = btn_stable;
    btn_stable = raw;
    if (prev == btn_released && btn_stable != btn_released)
    {
      btn_down_t    = HAL_GetTick();   /* pressed down - start timing the hold     */
      btn_long_sent = false;
    }
    else if (prev != btn_released && btn_stable == btn_released)
    {
      if (!btn_long_sent) btn_press_evt = true;   /* released a short tap          */
    }
  }

  /* Long press fires once, while the button is still held down. */
  if (btn_stable != btn_released && !btn_long_sent &&
      (HAL_GetTick() - btn_down_t) >= BTN_LONG_MS)
  {
    btn_long_sent = true;
    btn_long_evt  = true;
  }
}

static bool front_covered(void)
{
  return Sensors_Get(SENS_L_F) > GESTURE_HIGH && Sensors_Get(SENS_R_F) > GESTURE_HIGH;
}
static bool front_open(void)
{
  return Sensors_Get(SENS_L_F) < GESTURE_LOW && Sensors_Get(SENS_R_F) < GESTURE_LOW;
}

/* --- LED helpers (the launcher owns the LEDs while active) ---------------- */
static void leds(uint8_t r, uint8_t g, uint8_t b)
{
  WS2812_SetAll(r, g, b);
  WS2812_Show();
}

static void go_idle(void)
{
  Console_SetMuted(false);           /* wireless link back on                    */
  WS2812_Clear(); WS2812_Show();
  st = LA_IDLE;
}

/* Enter the armed (offline, waiting-for-gesture) state with a clean gesture
   latch, used both from idle and after a calibration finishes. */
static void enter_armed(void)
{
  front_clear = false;
  gesture_t0  = 0;
  leds(20, 8, 0);                    /* amber = offline / armed                  */
  st = LA_ARMED;
}

/* End the 3-run session: stop tagging further runs into the shared log buffers,
   and make sure IDLE won't auto-re-arm a stray fast run off the last one's
   fast_ready latch. The buffers themselves are KEPT (fetch them from the app).
   Comms are restored by the go_idle() the caller runs next. */
static void session_end(void)
{
  if (!session) return;
  session = false;
  Solver_LogSessionEnd();
  Solver_ClearFastReady();           /* don't let LA_IDLE re-arm after the session */
}

void Launcher_Init(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin   = BUTTON_Pin;
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = GPIO_PULLUP;             /* see note above if your button pulls high  */
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &g);

  /* Whatever the pin reads now (boot, button not held) is the released level. */
  btn_released  = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
  btn_stable    = btn_released;
  btn_last_raw  = btn_released;
  btn_t         = HAL_GetTick();
  btn_down_t    = HAL_GetTick();
  btn_long_sent = false;
  st            = LA_IDLE;

  /* Seed a boot-default target so a never-connected (pure offline) mouse still
     has somewhere to go. The app's 'maze'/'start'/'goal' override these and
     persist into the offline run (the launcher does NOT re-apply them at GO). */
  Solver_SetSize(RUN_MAZE_N);
#if (RUN_GOAL_X) >= 0
  Solver_SetGoalCell(RUN_GOAL_X, RUN_GOAL_Y);
#else
  Solver_SetGoalCenter();
#endif
}

bool Launcher_Active(void) { return st != LA_IDLE; }

void Launcher_Task(void)
{
  btn_poll();

  switch (st)
  {
  /* ---- online/normal: watch for the button to go offline ---------------- */
  case LA_IDLE:
    /* LONG press = FAST run from the SAVED map (survives a power cycle). Loads
       the last search's learned map out of flash, arms a fast run, and waits for
       the hand gesture - exactly like the post-search fast arm, but sourced from
       flash so it works after a battery swap. Requires a saved run present (a
       search that came home and wasn't wiped by a warm reset). */
    if (btn_long_evt && !Solver_Active())
    {
      MazeMap m;
      if (MazeStore_Valid() && MazeStore_Get(&m) && Solver_RestoreMap(&m))
      {
        if (Solver_Active()) Solver_Abort();
        Control_Stop();
        Console_SetMuted(true);        /* cut the WiFi/BLE link traffic          */
        session   = false;            /* standalone fast run, not the 3-run seq  */
        run_idx   = 0;
        fast_mode = true;
        Buzzer_Tone(NOTE_C6, 110);     /* ascending "saved path ready"            */
        Buzzer_Tone(NOTE_E6, 110);
        Buzzer_Tone(NOTE_G6, 160);
        leds(0, 0, 25); WS2812_Show(); HAL_Delay(120);   /* blue flash = fast arm */
        enter_armed();                 /* gesture latch reset; meter tints blue    */
      }
      else                             /* nothing saved to replay                 */
      {
        Buzzer_Tone(NOTE_C4, 250);     /* refused: low tone + red blink            */
        leds(30, 0, 0); WS2812_Show(); HAL_Delay(300);
        WS2812_Clear(); WS2812_Show();
      }
      break;
    }
    if (btn_press_evt)
    {
      if (Solver_Active()) Solver_Abort();
      Control_Stop();                /* clean slate                              */
      Console_SetMuted(true);        /* cut the WiFi/BLE link traffic            */
      Buzzer_Tone(NOTE_A5, 80);      /* "armed" chirp                            */
      /* Start a 3-run session: SEARCH now, then FAST, then FASTEST, each launched
         by its own gesture and logged separately for one fetch at the end. */
      Solver_LogSessionBegin();
      session   = true;
      run_idx   = 0;                 /* run 0 = search                           */
      fast_mode = false;             /* button arms an offline SEARCH            */
      enter_armed();                 /* amber, gesture latch reset               */
    }
    /* Not in a session but a search came home -> offer a single FAST run off the
       learned map (legacy standalone path). During a session the run-to-run advance
       is driven from LA_RUN, so don't double-arm here.
       GATE: only when the link is MUTED (genuinely offline). Over a LIVE console the
       user drives the fast run by TYPING `fast <cps>` - auto-arming would mute the
       link and hand the front sensors to the gesture watcher, which then hijacks the
       console run (the "front sensors trigger a gesture during my run" bug). */
    else if (!session && Solver_FastReady() && !Solver_Active() && Console_IsMuted())
    {
      Console_SetMuted(true);
      fast_mode = true;
      Buzzer_Tone(NOTE_C6, 110);     /* ascending "path saved / fast ready"      */
      Buzzer_Tone(NOTE_E6, 110);
      Buzzer_Tone(NOTE_G6, 160);
      leds(0, 0, 25); WS2812_Show(); HAL_Delay(120);   /* blue flash = fast armed */
      enter_armed();                 /* gesture latch reset; meter tints blue     */
    }
    break;

  /* ---- offline, waiting for the hand-over-front-sensors gesture ---------- */
  case LA_ARMED:
  {
    /* A run started from somewhere else (the console `fast`/`solve`, or the app)
       while we sat armed -> stand down. Otherwise the front-IR gesture watcher
       reads the moving run's wall reflections as a hand and can fire mid-run. Hand
       control back to IDLE (unmutes); the console owns the run now. */
    if (Solver_Active()) { go_idle(); break; }

    if (btn_long_evt)                              /* hold = cancel               */
    {
      Buzzer_Tone(NOTE_A4, 80);
      if (fast_mode) Solver_ClearFastReady();      /* so IDLE doesn't re-arm fast  */
      session_end();                               /* leave the session (keep logs) */
      go_idle();
      break;
    }

    /* Short press = button calibration. Place the mouse facing a wall that fills
       the front + both side sensors (a standard start cell faced backwards), then
       press: grab the references here, pivot 180 to the run direction sampling
       the dark floor on the way, then finalise + save (see LA_CAL_TURN). */
    if (btn_press_evt)
    {
      Control_Stop();
      Buzzer_Tone(NOTE_E5, 90);                    /* "capturing" chirp           */
      leds(20, 0, 20);                             /* magenta = grabbing refs     */
      Sensors_CalSnapshot(cal_ref);                /* side+front refs at the wall  */
      for (int i = 0; i < SENSOR_COUNT; i++) cal_dark_min[i] = 100000;
      Buzzer_Tone(NOTE_A5, 90);                    /* refs captured               */

      if (Control_StartPivot(CAL_PIVOT_DEG, CAL_PIVOT_DPS))
      {
        st = LA_CAL_TURN;
      }
      else                                         /* battery-gated: can't turn   */
      {
        Buzzer_Tone(NOTE_C4, 250);
        leds(30, 0, 0);
        HAL_Delay(400);
        enter_armed();
      }
      break;
    }

    if (front_open()) front_clear = true;          /* arm the low->high edge      */

    if (front_clear && front_covered())
    {
      leds(0, 20, 20);             /* cyan = hand detected, holding             */
      if (gesture_t0 == 0) gesture_t0 = HAL_GetTick();
      else if ((HAL_GetTick() - gesture_t0) >= GESTURE_HOLD_MS)
      {
        Buzzer_Tone(NOTE_C6, 120);   /* gesture accepted                         */
        count_t0    = HAL_GetTick();
        count_shown = 0xFF;          /* force the first countdown redraw         */
        st = LA_COUNT;
      }
    }
    else
    {
      gesture_t0 = 0;
      /* LIVE METER: the green channel grows with the front IR level, so you can
         SEE the sensors respond to your hand and how close it gets to the cyan
         trigger. If it never greens up when you wave, the hand isn't being seen
         (distance / battery-dimmed IR) - note the brightest you reach and I'll
         drop GESTURE_HIGH to match. Base tint amber, brighter once 'clear'. */
      int lf = Sensors_Get(SENS_L_F), rf = Sensors_Get(SENS_R_F);
      int fmin = (lf < rf) ? lf : rf;              /* the weaker front sensor      */
      int g = (fmin >= GESTURE_HIGH) ? 24 : (fmin * 24) / GESTURE_HIGH;
      /* Base tint marks the armed run type: amber = search, blue = fast,
         purple = fastest (smooth). */
      uint8_t base = front_clear ? 14 : 8;
      if (!fast_mode)
        leds(base, (uint8_t)g, 0);                 /* search: green meter on amber */
      else if (session && run_idx >= SESSION_LAST_RUN)
        leds(base, (uint8_t)g, base);              /* fastest: green meter on purple */
      else
        leds(0, (uint8_t)g, base);                 /* fast: green meter on blue    */
    }
    break;
  }

  /* ---- pivoting 180 after a button cal; sample dark, then finalise ------- */
  case LA_CAL_TURN:
  {
    /* As the mouse sweeps past the open run direction, the front/side sensors
       briefly see no wall - track the per-sensor minimum as the dark floor. */
    const int *raw = Sensors_Raw();
    for (int i = 0; i < SENSOR_COUNT; i++)
      if (raw[i] < cal_dark_min[i]) cal_dark_min[i] = raw[i];

    leds(0, 12, 16);                 /* cyan = working                          */

    int32_t deg;
    if (Control_PopTurnDone(&deg))
    {
      Control_Stop();                /* motors off before the flash erase        */
      if (Sensors_CalApplyButton(cal_dark_min, cal_ref) && Sensors_CalSave())
      {
        Buzzer_Tone(NOTE_C6, 90);    /* confirmed: ascending jingle + green      */
        Buzzer_Tone(NOTE_E6, 90);
        Buzzer_Tone(NOTE_G6, 140);
        leds(0, 25, 0);
        HAL_Delay(300);
      }
      else                           /* bad placement / flash error: keep old cal */
      {
        Buzzer_Tone(NOTE_C4, 200);
        Buzzer_Tone(NOTE_C4, 200);
        leds(30, 0, 0);
        HAL_Delay(500);
      }
      enter_armed();                 /* facing the run direction, await gesture   */
    }
    break;
  }

  /* ---- 3..2..1 countdown, then launch ----------------------------------- */
  case LA_COUNT:
  {
    /* A console/app run started during our countdown -> abort it; don't double-launch. */
    if (Solver_Active()) { session_end(); go_idle(); break; }
    if (btn_press_evt || btn_long_evt) { Buzzer_Tone(NOTE_A4, 80); session_end(); go_idle(); break; }  /* cancel */

    uint32_t elapsed = (HAL_GetTick() - count_t0) / 1000U;
    uint8_t  left    = (elapsed < COUNTDOWN_S) ? (uint8_t)(COUNTDOWN_S - elapsed) : 0;

    if (left != count_shown)         /* one tick boundary: redraw + beep         */
    {
      count_shown = left;
      WS2812_Clear();
      for (uint8_t i = 0; i < left && i < WS2812_NUM_LEDS; i++)
        WS2812_SetPixel(i, 0, 0, 30);            /* blue bars count down          */
      WS2812_Show();
      if (left > 0) Buzzer_Tone(NOTE_E5, 60);
    }

    if (elapsed >= COUNTDOWN_S)
    {
      Buzzer_Tone(NOTE_C6, 200);     /* GO                                       */

      /* Use the maze size + start + goal that are CURRENTLY set. The app's
         'maze'/'start'/'goal' commands (sent while connected) persist to here, so
         the offline run targets exactly what you configured; if no app ever
         connected, the boot defaults from Launcher_Init are still in effect. We
         deliberately do NOT re-apply RUN_* here - a hardcoded goal that lands on
         the start cell makes the solver think it's already done and never move. */

      /* Speed profile for this run - keep accel/decel scaled with speed so the
         flow reaches cruise and still brakes cleanly at each decision cell. The
         fast run uses the faster profile + replays the learned path to the goal. */
      bool ok;
      if (fast_mode)
      {
        Control_SetAccel(RUN_FAST_ACCEL, RUN_FAST_DECEL);
        Control_SetBrakeReverse(RUN_FAST_BRAKE_REV);   /* active reverse brake on   */
        Control_SetCenterGain(RUN_FAST_CENTER);
        /* FASTEST (session run 2) = SMOOTH arc turns; FAST = snappy pivots. The
           solver only flow-turns when the turn mode is smooth (Control_GetTurnMode
           == 1), so set it per run. */
        int fast_cps = RUN_CPS_FAST;
        if (session && run_idx >= SESSION_LAST_RUN)
        {
          Control_SetTurnMode(1);
          Control_SetArcGeom((float)RUN_ARC_R, (float)RUN_ARC_ENTRY, (float)RUN_ARC_EXIT);
          fast_cps = RUN_CPS_FASTEST;           /* slower so the arcs don't lag     */
        }
        else
        {
          Control_SetTurnMode(0);               /* pivots                          */
        }
        ok = Solver_StartFast(fast_cps);
      }
      else
      {
        Control_SetAccel(RUN_ACCEL, RUN_DECEL);
        Control_SetBrakeReverse(0.0f);          /* search: no active reverse brake  */
        Control_SetCenterGain(RUN_CENTER_KP);   /* stronger side-wall centring     */
        Control_SetTurnMode(0);                 /* search always pivots            */
        ok = Solver_Start(RUN_CPS);
      }

      if (ok)
      {
        leds(0, 25, 0);              /* green = running                          */
        st = LA_RUN;
      }
      else
      {
        /* couldn't start (needs 'ircal side'+'front', battery gate, or - for a
           fast run - no usable learned map / goal unreachable). */
        if (fast_mode) Solver_ClearFastReady();   /* don't loop on a bad arm       */
        session_end();
        Buzzer_Tone(NOTE_C4, 300);
        leds(30, 0, 0);              /* red = refused                            */
        HAL_Delay(500);
        go_idle();
      }
    }
    break;
  }

  /* ---- running fully offline; button = abort ---------------------------- */
  case LA_RUN:
    /* Mid-run milestone: the search just reached the goal. The mouse is stopped
       in its sense state here, so a short chirp can't disturb motion. */
    if (Solver_PopGoalReached())
    {
      Buzzer_Tone(NOTE_G5, 60);
      Buzzer_Tone(NOTE_C6, 90);        /* quick "goal found" rising blip           */
    }
    /* The search auto-saved its learned map to flash at run end (motors idle).
       Confirm it, or clearly warn if the flash write failed - a failed save means
       no fast run after a power cycle, which is competition-critical to know. */
    {
      bool saved;
      if (Solver_PopMapSaved(&saved))
      {
        if (saved)
        {
          Buzzer_Tone(NOTE_E6, 70);    /* subtle "map saved to memory"             */
          Buzzer_Tone(NOTE_G6, 70);
        }
        else
        {
          Buzzer_Tone(NOTE_C4, 400);   /* SAVE FAILED - long low warning + red     */
          leds(30, 0, 0); WS2812_Show(); HAL_Delay(500);
        }
      }
    }
    if (btn_press_evt || btn_long_evt)
    {
      Solver_Abort();
      Buzzer_Tone(NOTE_A4, 120);
      session_end();                 /* logs kept; link back up to fetch them     */
      go_idle();
      break;
    }
    if (!Solver_Active())            /* run ended on its own (or a pickup)        */
    {
      /* In a session: if that run came home with a usable map and there are more
         runs to go, advance to the NEXT run and re-arm the gesture WITHOUT going
         idle (stay muted/offline the whole session). Otherwise the session is done
         (all runs finished, or a pickup/box-in left no fast map) - restore the link
         so the app can fetch all the buffered runs. */
      if (session && Solver_FastReady() && run_idx < SESSION_LAST_RUN)
      {
        run_idx++;
        fast_mode = true;            /* runs 1 (fast) and 2 (fastest) are fast runs */
        Buzzer_Tone(NOTE_C6, 110);   /* ascending "run saved / next armed" jingle  */
        Buzzer_Tone(NOTE_E6, 110);
        Buzzer_Tone(NOTE_G6, 160);
        /* tint the next-run meter: blue for FAST, magenta-ish for FASTEST (smooth) */
        if (run_idx >= SESSION_LAST_RUN) { leds(20, 0, 25); }   /* fastest: purple  */
        else                             { leds(0, 0, 25);  }   /* fast: blue       */
        WS2812_Show(); HAL_Delay(150);
        enter_armed();               /* await this run's gesture (still muted)     */
        break;
      }

      /* Session complete (or a standalone run finished). */
      session_end();
      Buzzer_Tone(NOTE_C6, 100);
      Buzzer_Tone(NOTE_E6, 100);
      Buzzer_Tone(NOTE_G6, 160);     /* little finish jingle                     */
      leds(0, 25, 0);
      HAL_Delay(300);
      go_idle();                     /* link back up: 'dump' fetches all runs      */
    }
    break;
  }
}
