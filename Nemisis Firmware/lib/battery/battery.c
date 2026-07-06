/**
 ******************************************************************************
 * @file    battery.c
 * @brief   Battery safety monitor - see battery.h.
 ******************************************************************************
 */
#include "battery.h"

#include "max17049.h"
#include "ws2812.h"
#include "buzzer.h"
#include "motor.h"

/* ============================ state ====================================== */

static I2C_HandleTypeDef *FUEL;
static TIM_HandleTypeDef *FAN;
static uint32_t           FAN_CH;

static BatteryState  state;
static BatterySnapshot snap;

static uint32_t last_poll;          /* gauge read scheduler */
static BatteryState pending;        /* candidate next state being debounced */
static uint8_t      pending_cnt;

/* indication timers (independent of the poll rate) */
static uint32_t last_blink;
static uint32_t last_beep;
static bool     blink_on;

/* ============================ helpers ==================================== */

const char *Battery_StateName(BatteryState s)
{
  switch (s)
  {
    case BATT_OK:       return "OK";
    case BATT_LOW:      return "LOW";
    case BATT_CRITICAL: return "CRITICAL";
    default:            return "?";
  }
}

/* Cut the high-current loads NOW. Called the moment we trip CRITICAL so the
   robot stops even if no console command is in flight. */
static void cut_high_current(void)
{
  if (FAN) { __HAL_TIM_SET_COMPARE(FAN, FAN_CH, 0); }   /* vacuum off */
  Motor_Coast(MOTOR_1);
  Motor_Coast(MOTOR_2);
}

/* Raw classification with NO hysteresis - used only to seed the initial state
   at boot so an already-flat pack is protected immediately. */
static BatteryState classify_raw(uint16_t mv)
{
  if (mv < BATTERY_CRIT_MV) return BATT_CRITICAL;
  if (mv < BATTERY_LOW_MV)  return BATT_LOW;
  return BATT_OK;
}

/* Desired next state given the CURRENT state + voltage, applying hysteresis:
   trips use the low thresholds, recoveries use the higher CLEAR thresholds. */
static BatteryState next_state(BatteryState cur, uint16_t mv)
{
  switch (cur)
  {
    case BATT_OK:
      return (mv < BATTERY_LOW_MV) ? BATT_LOW : BATT_OK;

    case BATT_LOW:
      if (mv < BATTERY_CRIT_MV)        return BATT_CRITICAL;
      if (mv > BATTERY_LOW_CLEAR_MV)   return BATT_OK;
      return BATT_LOW;

    case BATT_CRITICAL:                /* latched until a real recovery */
      return (mv > BATTERY_CRIT_CLEAR_MV) ? BATT_LOW : BATT_CRITICAL;

    default:
      return cur;
  }
}

/* One-time actions when a new state is entered. */
static void enter_state(BatteryState s)
{
  state = s;
  /* force the indication to refresh immediately on the next task tick */
  last_blink = 0;
  last_beep  = 0;
  blink_on   = false;

  switch (s)
  {
    case BATT_CRITICAL:
      cut_high_current();                 /* safety: kill loads right away */
      WS2812_SetAll(40, 0, 0);            /* solid red until the flasher takes over */
      WS2812_Show();
      Buzzer_Tone(NOTE_A5, 120);         /* immediate alarm chirp */
      break;

    case BATT_LOW:
      WS2812_SetAll(30, 14, 0);          /* amber warning */
      WS2812_Show();
      Buzzer_Tone(NOTE_E5, 80);          /* single heads-up chirp */
      break;

    case BATT_OK:
    default:
      /* hand the LED back to the main-loop heartbeat */
      break;
  }
}

/* ============================ poll + FSM ================================= */

static void poll_and_update(void)
{
  MAX17049_Status s;
  if (!MAX17049_ReadStatus(FUEL, &s))
    return;                              /* read failed: keep last state */

  uint16_t mv = (uint16_t)(s.pack_v * 1000.0f);
  snap.pack_mv = mv;
  snap.cell_mv = (uint16_t)(s.cell_v * 1000.0f);
  snap.soc_pct = (int16_t)s.soc;
  snap.valid   = true;

  BatteryState want = next_state(state, mv);
  if (want == state)
  {
    pending_cnt = 0;                     /* nothing pending */
  }
  else if (want == pending)
  {
    if (++pending_cnt >= BATTERY_DEBOUNCE)
    {
      pending_cnt = 0;
      enter_state(want);
    }
  }
  else
  {
    pending     = want;                  /* new candidate, restart debounce */
    pending_cnt = 1;
  }

  snap.state = state;
}

/* ============================ indication ================================= */

static void run_indication(uint32_t now)
{
  switch (state)
  {
    case BATT_CRITICAL:
      /* fast red flash ~3 Hz */
      if ((now - last_blink) >= 160U)
      {
        last_blink = now;
        blink_on = !blink_on;
        if (blink_on) WS2812_SetAll(60, 0, 0);
        else          WS2812_SetAll(0, 0, 0);
        WS2812_Show();
      }
      /* repeating alarm beep once a second (short, so the loop barely stalls) */
      if ((now - last_beep) >= 1000U)
      {
        last_beep = now;
        Buzzer_Tone(NOTE_A5, 120);
      }
      break;

    case BATT_LOW:
      /* steady amber; refresh occasionally in case something else touched it */
      if ((now - last_blink) >= 500U)
      {
        last_blink = now;
        WS2812_SetAll(30, 14, 0);
        WS2812_Show();
      }
      break;

    case BATT_OK:
    default:
      /* LED owned by the heartbeat in main.c while healthy */
      break;
  }
}

/* ============================ public API ================================= */

void Battery_Init(I2C_HandleTypeDef *hi2c_fuel,
                  TIM_HandleTypeDef *htim_fan, uint32_t fan_channel)
{
  FUEL   = hi2c_fuel;
  FAN    = htim_fan;
  FAN_CH = fan_channel;

  state       = BATT_OK;
  pending     = BATT_OK;
  pending_cnt = 0;
  last_poll   = HAL_GetTick();
  last_blink  = 0;
  last_beep   = 0;
  blink_on    = false;

  snap = (BatterySnapshot){ .state = BATT_OK, .valid = false };

  /* Seed the real state from one reading so a flat pack is protected at boot. */
  MAX17049_Status s;
  if (MAX17049_ReadStatus(FUEL, &s))
  {
    uint16_t mv  = (uint16_t)(s.pack_v * 1000.0f);
    snap.pack_mv = mv;
    snap.cell_mv = (uint16_t)(s.cell_v * 1000.0f);
    snap.soc_pct = (int16_t)s.soc;
    snap.valid   = true;
    snap.state   = classify_raw(mv);
    if (snap.state != BATT_OK)
      enter_state(snap.state);           /* fires the cut + indication */
  }
}

void Battery_Task(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - last_poll) >= BATTERY_POLL_MS)
  {
    last_poll = now;
    poll_and_update();
  }

  run_indication(now);
}

BatteryState Battery_State(void)
{
  return state;
}

bool Battery_AllowHighCurrent(void)
{
  return (state != BATT_CRITICAL);
}

/* Set the vacuum fan PWM to a 0..100 % duty. Returns the duty actually applied,
   or -1 if a >0 request was refused by the battery gate (the fan is the biggest
   current draw on the board, so it's blocked while the pack is critical). This is
   the single source of truth for the fan; the console `vacuum` cmd, the grip test
   and the fast-run auto-vacuum all funnel through here. FAN full-scale = 65535
   (htim8 ARR), matching the emergency-off above. */
int Battery_SetVacuum(int pct)
{
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  if (pct > 0 && !Battery_AllowHighCurrent())
    return -1;
  if (FAN) __HAL_TIM_SET_COMPARE(FAN, FAN_CH, ((uint32_t)pct * 65535U) / 100U);
  return pct;
}

void Battery_GetSnapshot(BatterySnapshot *out)
{
  if (out) *out = snap;
}
