/**
 ******************************************************************************
 * @file    sensors.h
 * @brief   IR reflectance wall-sensing front-end for the Nemisis micromouse.
 *
 * Six emitter/receiver pairs around the nose. Each read pulses one emitter,
 * samples its receiver lit then dark (ambient), and subtracts - so daylight /
 * desk-lamp DC drops out and only the reflected IR remains. Higher value =
 * closer / more reflective (a wall is near).
 *
 * Layout (left to right across the front):
 *
 *     L_LM   L_M   L_F   R_F   R_M   R_RM
 *      \      \     \     /     /     /
 *    side    mid  front-diag  mid   side
 *
 *   - L_LM / R_RM : side-facing  -> left/right wall distance + corridor centring
 *   - L_F  / R_F  : front-diag   -> front wall presence + approach distance
 *   - L_M  / R_M  : mid-angle    -> diagonal / cell-opening detection (maze)
 *
 * Acquisition runs in the MAIN loop at ~100-200 Hz (NOT the control ISR): the
 * 1 kHz controller reads the latest values with a zero-order hold. This is the
 * polling-first stage; the read path is structured to drop onto ADC+DMA later
 * behind this same API without touching anything built on top.
 ******************************************************************************
 */
#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define SENSOR_COUNT 6

/* Physical sensor order, left to right around the front of the mouse. Use these
   to index Sensors_Raw() / Sensors_Get(). */
typedef enum
{
  SENS_L_LM = 0,   /* left,  side-facing  - left-wall distance / centring   */
  SENS_L_M,        /* left,  mid-angle    - diagonal / opening detect        */
  SENS_L_F,        /* left,  front-diag   - front wall                       */
  SENS_R_F,        /* right, front-diag   - front wall                       */
  SENS_R_M,        /* right, mid-angle    - diagonal / opening detect        */
  SENS_R_RM        /* right, side-facing  - right-wall distance / centring   */
} SensorId;

/** Bind the two ADC handles (emitter GPIOs are wired internally from main.h)
 *  and start the DWT cycle counter used for the short emitter settle. Call once
 *  after the MX_ADC*_Init + ADC calibration (i.e. after Console_Init). */
void Sensors_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc4);

/** One full 6-sensor sweep: pulse each emitter, read ambient-subtracted
 *  reflectance, fold into a light EMA. Call at a steady rate (~100-200 Hz) from
 *  the main loop. Blocks ~3 ms (polling) - the 1 kHz control loop preempts it,
 *  so its timing is unaffected; only the main loop's spare time is consumed
 *  (that's what the later DMA path reclaims). */
void Sensors_Update(void);

/** Pump the NON-BLOCKING background sweep - call every main-loop pass. When async
 *  mode is on (Sensors_SetAsync) this drives the whole 6-sensor sweep as a state
 *  machine that returns immediately while the emitter settles / the ADC converts,
 *  so the ~3.4 ms the blocking Sensors_Update() spins is freed for the rest of the
 *  loop. Same readings + calibration. No-op when async is off. */
void Sensors_Pump(void);

/** Enable/disable the non-blocking background sweep (console 'irasync'). OFF (the
 *  default) = the proven blocking Sensors_Update() runs from the main loop as
 *  before; ON = Sensors_Pump() owns the sweep and Sensors_Update() no-ops. Safe to
 *  flip live: it aborts any in-flight conversion on the transition. */
void Sensors_SetAsync(bool on);
bool Sensors_GetAsync(void);

/** Latest filtered ambient-subtracted reflectance for one sensor (>= 0; higher
 *  = closer / brighter). */
int Sensors_Get(SensorId id);

/** Pointer to all SENSOR_COUNT filtered values, indexed by SensorId. */
const int *Sensors_Raw(void);

/** Latest RAW ambient (emitter off) and lit (emitter on) ADC readings per
 *  sensor, before subtraction. For the 'irraw' bench diagnostic - tells us
 *  whether the receiver node is saturated (amb near 4095), starved (near 0), or
 *  well-biased. Pass NULL for either array to skip it. */
void Sensors_GetRaw(int *amb, int *lit);

/** Emitter settle time (us) and ADC oversampling count per phase. Live-tunable
 *  from the console ('irset') to sweep the weak IR signal on the bench. Pass an
 *  out-of-range value (settle 0, samples not in 1..64) to leave it unchanged. */
void Sensors_SetTiming(uint32_t settle_us, uint32_t samples);
void Sensors_GetTiming(uint32_t *settle_us, uint32_t *samples);

/* --- Speed-adaptive sweep timing ------------------------------------------
 * Drops the emitter settle + ADC oversampling as the mouse speeds up so the
 * 6-sensor sweep finishes quicker (less position lag at speed), and averages
 * hard when slow/stationary (clean readings + matches calibration). ON by
 * default. Call Sensors_AdaptTiming() each loop with the current wheel speed
 * (Control_GetWheelSpeedCps); it overrides Sensors_SetTiming while enabled. */
void Sensors_SetAdaptive(bool on);
bool Sensors_GetAdaptive(void);
void Sensors_AdaptTiming(float cps);

/* ===========================================================================
 *  Calibration  (Increment 1)
 *
 * Each channel gets three numbers, captured on the bench and stored in their
 * OWN flash page (separate from the tuning profiles, so saving calibration
 * never touches your Search/Fast/Fastest tune):
 *
 *   dark    - reading with no wall near (the floor; folds in L_F's crosstalk
 *             pedestal so it's subtracted back out).
 *   ref     - reading at the canonical reference position: for the side
 *             sensors (L_LM/R_RM) the centred-corridor value centring nulls
 *             against; for the fronts (L_F/R_F) the value at the stop distance.
 *   thresh  - wall-present trip point, auto-set partway from dark to ref.
 *
 * Capture flow (mouse held still on the bench):
 *   Sensors_CalDark()  - in an OPEN cell, nothing near.
 *   Sensors_CalSide()  - CENTRED between a left and right wall.
 *   Sensors_CalFront() - at the stop point facing a FRONT wall.
 *   Sensors_CalSave()  - commit to flash (run stopped; an erase stalls the CPU).
 * Sensors_CalLoad() runs once at boot (after Sensors_Init).
 * ========================================================================= */

/** Capture the no-wall dark baseline for all six sensors. Do this first. */
void Sensors_CalDark(void);

/** Capture the centred side reference (L_LM, R_RM) and set their thresholds. */
void Sensors_CalSide(void);

/** Capture the front reference (L_F, R_F) and set their thresholds. */
void Sensors_CalFront(void);

/** Commit the current calibration to its flash page. Returns false on flash
 *  error. Caller must ensure no control run is active (erase stalls the loop). */
bool Sensors_CalSave(void);

/** Load calibration from flash, or zero it if none/invalid. Call once at boot
 *  after Sensors_Init. */
void Sensors_CalLoad(void);

/* --- Button / competition self-cal (no console) ----------------------------
 * For the offline launcher: the mouse is placed by hand facing a 3-walled end
 * (a front wall AND both side walls in view), the side+front references are
 * grabbed there, the dark floor is sampled as the per-sensor running minimum
 * while the mouse pivots 180deg to the open run direction, and the two are then
 * combined into the SAME thresholds 'ircal dark/side/front' would produce. Split
 * into snapshot + apply so the launcher can run the turn in between. */

/** One settled, averaged snapshot of all six sensors (EMA-bypassed - the exact
 *  capture the manual cal uses). Grab the reference readings with this while the
 *  mouse faces the wall. */
void Sensors_CalSnapshot(int out[SENSOR_COUNT]);

/** Commit a button cal from a dark floor + a reference snapshot: writes dark for
 *  every sensor and ref+thresh for the side (L_LM/R_RM) and front (L_F/R_F)
 *  channels, identical to running 'ircal dark/side/front'. Sanity-checks each of
 *  those four references (a wall must read clearly above dark and not be railed)
 *  and returns false WITHOUT modifying the stored calibration if any check fails,
 *  so a bad placement can never poison a good cal. On success the caller persists
 *  it with Sensors_CalSave(). */
bool Sensors_CalApplyButton(const int dark[SENSOR_COUNT], const int ref[SENSOR_COUNT]);

/** Copy the stored calibration table for display ('ircshow'). Any pointer may
 *  be NULL. flags bit0=dark, bit1=side, bit2=front captured. */
void Sensors_GetCal(int *dark, int *ref, int *thresh, uint16_t *flags);

/** Wall-presence booleans, true when the channel is above its calibrated
 *  threshold. Return false for an uncalibrated channel (no false positives). */
bool Walls_Left(void);    /* L_LM side wall            */
bool Walls_Right(void);   /* R_RM side wall            */
bool Walls_Front(void);   /* L_F OR  R_F: sensitive, for the front-stop  */

/** High-confidence front wall: L_F AND R_F. Use when committing a wall to the
 *  permanent maze model, where a phantom would falsely sever a passage. */
bool Walls_FrontConfident(void);

/** Early/sensitive front wall from the UNFILTERED reading, for the front-stop
 *  brake only (trips ~one EMA tau sooner). Too noisy to record a wall from. */
bool Walls_FrontRaw(void);

/** Nose AT a wall (CONTACT grade): filtered front >= ~2x the frontcal stop-distance
 *  ref, far above detection. True when the mouse has driven up to a wall (wheels
 *  slip-spinning). Used to suppress the phantom slip-spin cell-mark on a fast run.
 *  Needs 'frontcal'; never trips uncalibrated (callers degrade to no-op). */
bool Walls_FrontContact(void);

/** Front wall CLOSE (~40-50mm, between detection and contact): filtered front > ~2.5x
 *  the ircal threshold. Blocks the fast-run creep LATE so it reaches the cell mark
 *  before stopping (detection-range blocking stalls it short -> position lag). Keyed
 *  on ircal (always calibrated), so it never no-ops the way a frontcal check can. */
bool Walls_FrontClose(void);

/** Signed lateral error from the side sensors for corridor centring (Inc 2):
 *  0 = centred, >0 = drifted toward the LEFT wall (steer right), <0 = toward
 *  the right wall. Returns 0 until the side reference is calibrated. */
int Sensors_Lateral(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
