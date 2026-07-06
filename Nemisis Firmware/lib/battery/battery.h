/**
 ******************************************************************************
 * @file    battery.h
 * @brief   Battery safety monitor for the Nemisis micromouse (2S LiPo via the
 *          MAX17049 fuel gauge on I2C3).
 *
 * WHY THIS EXISTS
 *   Motors + vacuum fan pull heavy current. On a sagging 2S pack that can brown
 *   out the STM32 mid-run, and discharging LiPo below ~3.0 V/cell permanently
 *   damages the cells. This module watches the pack voltage and:
 *     - blocks the high-current subsystems (motors, vacuum) when CRITICAL,
 *     - actively cuts them the instant it trips (doesn't wait for a command),
 *     - signals state on the RGB LED + buzzer.
 *
 * WHY VOLTAGE, NOT SOC %
 *   The MAX17049's ModelGauge SOC needs charge/discharge cycles to converge and
 *   is not trustworthy out of the box (it reads ~86 % on a genuinely full pack
 *   here). Pack VOLTAGE is independent of the model and is the real predictor of
 *   brownout, so every threshold below is in millivolts. SOC is display-only.
 *   See [[max17049-fuelgauge-i2c3]].
 *
 * Thresholds are deliberately measured UNDER LOAD (the gauge reads the sagged
 * voltage while motors/fan run), which is the conservative, correct number.
 *
 * Usage (main.c):
 *     Battery_Init(&hi2c3, &htim8, TIM_CHANNEL_3);   // after WS2812/Buzzer init
 *     while (1) { Battery_Task(); Console_Task(); }  // both non-blocking
 ******************************************************************************
 */
#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* --- Pack-voltage thresholds, millivolts (2S = 2 cells) ------------------- *
 * Tune these to your cells. Defaults: CRIT 3.30 V/cell, LOW 3.50 V/cell, with
 * hysteresis so the state can't chatter at the edge. CLEAR points are higher
 * than the trip points and must be reached (post-recovery) to re-arm.        */
#ifndef BATTERY_CRIT_MV
#define BATTERY_CRIT_MV         6600U   /* < this  -> CRITICAL (cut everything) */
#endif
#ifndef BATTERY_CRIT_CLEAR_MV
#define BATTERY_CRIT_CLEAR_MV   7200U   /* must rise above this to leave CRIT   */
#endif
#ifndef BATTERY_LOW_MV
#define BATTERY_LOW_MV          7000U   /* < this  -> LOW (warn, still allowed) */
#endif
#ifndef BATTERY_LOW_CLEAR_MV
#define BATTERY_LOW_CLEAR_MV    7300U   /* must rise above this to return to OK */
#endif

/* How often to read the gauge, and how many consecutive confirming reads are
 * needed before a state change commits (rejects motor-inrush voltage dips). */
#ifndef BATTERY_POLL_MS
#define BATTERY_POLL_MS         200U
#endif
#ifndef BATTERY_DEBOUNCE
#define BATTERY_DEBOUNCE        3U      /* 3 * 200 ms = ~0.6 s to trip          */
#endif

typedef enum
{
  BATT_OK = 0,      /* healthy: high-current loads allowed, LED left to heartbeat */
  BATT_LOW,         /* warning: still allowed, solid amber + one-shot chirp        */
  BATT_CRITICAL     /* cut off: loads blocked + killed, red flash + alarm beep     */
} BatteryState;

typedef struct
{
  BatteryState state;
  uint16_t     pack_mv;    /* last good pack voltage [mV]      */
  uint16_t     cell_mv;    /* per-cell voltage [mV]            */
  int16_t      soc_pct;    /* gauge SOC [%] (display only)     */
  bool         valid;      /* false until first successful read */
} BatterySnapshot;

/** Bind handles, take one reading, set the initial state. Call once after the
 *  WS2812 + buzzer are initialised. fan_channel is the vacuum PWM channel on
 *  htim_fan (TIM_CHANNEL_3 on this board). */
void Battery_Init(I2C_HandleTypeDef *hi2c_fuel,
                  TIM_HandleTypeDef *htim_fan, uint32_t fan_channel);

/** Non-blocking pump: polls the gauge on its interval, runs the state machine,
 *  and drives the LED/buzzer indication. Call every superloop pass. */
void Battery_Task(void);

/** Current battery state. */
BatteryState Battery_State(void);

/** False when CRITICAL (latched until the pack recovers above the clear point).
 *  The motor/vacuum console commands gate on this. */
bool Battery_AllowHighCurrent(void);

/** Set the vacuum fan PWM duty (0..100 %). Returns the duty applied, or -1 if a
 *  >0 request was refused by the battery gate. Single source of truth for the fan:
 *  the `vacuum` command, the grip test and the fast-run auto-vacuum all call this. */
int Battery_SetVacuum(int pct);

/** Copy the last reading (voltage/soc/state) for display. */
void Battery_GetSnapshot(BatterySnapshot *out);

/** Human-readable state label ("OK" / "LOW" / "CRITICAL"). */
const char *Battery_StateName(BatteryState s);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
