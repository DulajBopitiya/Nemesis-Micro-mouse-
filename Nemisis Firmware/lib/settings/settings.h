/**
 ******************************************************************************
 * @file    settings.h
 * @brief   Persistent tuning profiles in on-chip flash (survive power cycle).
 *
 * Stores SETTINGS_NUM_PROFILES independent copies of the controller's tuning
 * (feedforward, velocity + heading PID, accel/decel, velocity filter) in the
 * last flash page, so the dialed-in values survive a power cycle and you can
 * keep one set per speed regime:
 *
 *     0  Search   gentle, ~1 m/s maze search run
 *     1  Fast     ~2 m/s straight run
 *     2  Fastest  vacuum-assisted top speed
 *
 * Flow:
 *   - Control_Init() loads the baked-in defaults (the known-good Search tune).
 *   - Settings_Init() then reads flash: if a valid blob is present it applies
 *     the saved *active* profile over those defaults; if flash is blank/corrupt
 *     it seeds all profiles in RAM from the baked defaults (nothing is written
 *     until the user saves).
 *   - Console `profile <n>` switches the active profile, `save <n>` snapshots
 *     the live params into a profile and commits ALL profiles to flash, `load
 *     <n>` pushes a stored profile back into the live controller.
 *
 * A flash erase stalls the CPU (code runs from flash), so Settings_Save refuses
 * while a control run is active - save on the bench, stopped.
 ******************************************************************************
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define SETTINGS_NUM_PROFILES 3

/** Read flash and either apply the saved active profile or seed RAM profiles
 *  from the controller's current (baked-default) params. Call once, right after
 *  Control_Init(). */
void Settings_Init(void);

/** Index (0..N-1) of the active profile. */
int Settings_Active(void);

/** Short name of a profile ("Search"/"Fast"/"Fastest"); "?" if out of range. */
const char *Settings_ProfileName(int idx);

/** Snapshot the controller's current live params into profile <idx> (RAM only;
 *  call Settings_Save to persist). False if idx out of range. */
bool Settings_Capture(int idx);

/** Push profile <idx>'s params into the live controller. False if out of range. */
bool Settings_Apply(int idx);

/** Make <idx> the active profile and apply it live. False if out of range. */
bool Settings_SetActive(int idx);

/** Commit the whole RAM blob (all profiles + active index) to flash. Refuses
 *  while a control run is active. False on busy / flash error. */
bool Settings_Save(void);

/** Map a profile token - a name ("search"/"fast"/"fastest", any case) or an
 *  index "0".."2" - to 0..N-1. Returns -1 if unrecognised. */
int Settings_Parse(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_H */
