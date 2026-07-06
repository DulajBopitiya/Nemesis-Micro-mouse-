/**
 ******************************************************************************
 * @file    mazestore.h
 * @brief   Persistent maze / speed-run memory in on-chip flash.
 *
 * A micromouse competition run must survive a BATTERY SWAP: the search learns
 * the maze, then the pack is changed, then a fast run replays the learned path.
 * The learned map lives in RAM (solver.c), so it's normally lost on power-off.
 * This module snapshots the learned wall map + config into the STM32's internal
 * flash so a fast run can be launched after a power cycle.
 *
 * FLASH LAYOUT: one blob in the THIRD-to-last flash page. The last page is the
 * settings/tuning blob, the second-to-last is the IR wall-sensor calibration
 * (see settings.c / sensors.c); this takes the next page down. Code uses ~12% of
 * flash, so these top pages are free without touching the linker script. The
 * page address/bank/number are computed at runtime (DBANK-aware), exactly as the
 * other two blobs do.
 *
 * ERASE SEMANTICS (competition rule the user asked for):
 *   - A cold POWER-ON / battery swap  -> KEEP the saved map (so a fast run works
 *     after changing the pack).
 *   - A warm NRST BUTTON press (power maintained) -> ERASE the saved map (start a
 *     fresh maze for the next search).
 * These are distinguished by the RCC reset-cause flags: a power event sets BOR;
 * a button-only reset sets just the PIN flag. The logic is DEFAULT-SAFE: it
 * erases ONLY on the exact "pin reset with no power/software/watchdog flag"
 * signature, so any ambiguity keeps the map (a battery swap can never wipe a
 * solved maze). See MazeStore_Init.
 ******************************************************************************
 */
#ifndef MAZESTORE_H
#define MAZESTORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Array capacity - MUST match the solver's MAZE_MAX (compile-checked there). */
#define MAZE_STORE_DIM  16

/* The learned maze, packed for flash. Wall bytes are the solver's 4-bit masks
   (N=1,E=2,S=4,W=8); `open` packs confirmed_open[x][y][d] as bit d. Sized to a
   multiple of 8 so the wrapping blob programs cleanly in 64-bit doublewords. */
typedef struct
{
  uint8_t n;                 /* maze edge length (cells)                     */
  uint8_t startX, startY;    /* start cell                                   */
  uint8_t startFacing;       /* 0..3 = N,E,S,W                               */
  uint8_t goalCount;         /* number of goal cells (1..4)                  */
  uint8_t goalX[4];
  uint8_t goalY[4];
  uint8_t reserved0[3];      /* pad the header to 16 bytes                   */
  uint8_t walls[MAZE_STORE_DIM][MAZE_STORE_DIM];      /* sensed wall map      */
  uint8_t wall_phys[MAZE_STORE_DIM][MAZE_STORE_DIM];  /* physically-confirmed */
  uint8_t open[MAZE_STORE_DIM][MAZE_STORE_DIM];       /* confirmed-open edges */
} MazeMap;                                            /* 784 bytes           */

/** Read the RCC reset-cause flags, apply the erase-or-keep rule (see file
 *  header), then load the flash blob into the RAM cache. Call ONCE, early in
 *  boot, BEFORE anything else clears the reset flags. Safe with motors idle. */
void MazeStore_Init(void);

/** True if a valid saved map is present (loaded at Init / after a Save, cleared
 *  by an Erase or a warm-reset wipe). */
bool MazeStore_Valid(void);

/** Copy the saved map into *out. False if none is valid. */
bool MazeStore_Get(MazeMap *out);

/** Write a map to flash (erases + programs the page). Refuses while the control
 *  loop is active (the erase stalls the CPU) - call only with motors stopped.
 *  Returns true iff the readback verifies. Updates the RAM cache on success. */
bool MazeStore_Save(const MazeMap *m);

/** Erase the saved map (page erase) and clear the cache. */
void MazeStore_Erase(void);

/** Diagnostics for verifying the erase scheme on the bench: the reset-cause
 *  flags captured at Init, a short human name for the dominant cause, and
 *  whether Init treated this boot as a warm (map-erasing) reset. */
uint32_t    MazeStore_LastResetFlags(void);
const char *MazeStore_LastResetName(void);
bool        MazeStore_WasWarmReset(void);

#ifdef __cplusplus
}
#endif

#endif /* MAZESTORE_H */
