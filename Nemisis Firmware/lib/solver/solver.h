/**
 ******************************************************************************
 * @file    solver.h
 * @brief   Autonomous maze solver (flood-fill search) for the Nemisis micromouse.
 *
 * This is the on-robot port of the MMS-simulator-proven algorithm in
 * SIMULATOR/flood_fill_Algo/Main.c. The maze logic (BFS flood-fill, the explore
 * heuristic, wall recording) is carried over unchanged; only the hardware layer
 * differs: where the simulator port spoke a stdin/stdout text protocol
 * (API.c -> "moveForward"/"wallFront"/...), here we drive the real control loop
 * (Control_StartAdvance / Control_StartPivot) and read the IR wall booleans
 * (Walls_Front/Left/Right).
 *
 * SCOPE: search phase only - flood-fill explore to the centre 2x2 goal, then a
 * direct flood back to the start cell. Whole-cell, stop-and-go motion (move one
 * cell, stop, sense, decide). The 8-direction diagonal Dijkstra speed run from
 * Main.c is deliberately NOT ported yet (needs diagonal/half-cell motion in the
 * control loop and a memory trim first).
 *
 * Non-blocking: Solver_Start() kicks it off and Solver_Task() is pumped every
 * main-loop pass (like Control_NavTask). The console stays responsive while it
 * runs, so 'stop' aborts cleanly and the battery-safety FSM keeps running.
 *
 * Convention (inherited from the proven algo): cells are (x,y) with x=East,
 * y=North; the mouse starts at (0,0) facing NORTH; the goal is the centre 2x2
 * {(7,7),(7,8),(8,7),(8,8)}. "NORTH" is simply the robot's initial heading -
 * the maze coordinates are abstract, so only forward/left/right consistency
 * matters (handled by the wall-sense and turn mapping in solver.c).
 *
 * REQUIRES wall-sensor calibration ('ircal side' + 'ircal front'); without it
 * Walls_* read false (everything looks open) and Solver_Start() refuses.
 ******************************************************************************
 */
#ifndef SOLVER_H
#define SOLVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "mazestore.h"      /* MazeMap - persistent map snapshot type */

/** Reset the maze model + mouse state to the start cell. Called by
 *  Solver_Start; also safe to call at boot. */
void Solver_Reset(void);

/* --- Persistent map (save/restore across a power cycle) --------------------
 * The learned wall map + config IS the speed-run "path": a fast run replays the
 * least-cost route over it. These snapshot it to / from a MazeMap so mazestore
 * can persist it in flash, letting a fast run survive a battery swap. */

/** Snapshot the current learned map + config into *m (for MazeStore_Save). */
void Solver_CaptureMap(MazeMap *m);

/** Load a saved map + config back into the solver and arm a fast run
 *  (FastReady becomes true). Does NOT touch the motors. Returns false if the
 *  map is malformed. Use before Solver_StartFast after a power cycle. */
bool Solver_RestoreMap(const MazeMap *m);

/** Drains once when a completed SEARCH auto-saves its map to flash (at the end
 *  of the run, motors idle). ok = whether the flash write verified. */
bool Solver_PopMapSaved(bool *ok);

/** Drains once when the search first reaches the goal (phase 0 -> 1), so the
 *  caller can indicate "goal found" mid-run. */
bool Solver_PopGoalReached(void);

/* --- Maze configuration (settable so the app can match the real maze) ------ */

/** Set the maze edge length (cells, 2..16). Re-centres the goal and resets the
 *  map. Default is 6 (the home practice maze). */
void Solver_SetSize(int n);

/** Goal = the centre 2x2 block, computed from the current size. */
void Solver_SetGoalCenter(void);

/** Goal = a single custom cell (x,y). Out-of-range is ignored. */
void Solver_SetGoalCell(int x, int y);

/* --- Start cell / corner (where you physically place the mouse) ------------
 * The mouse no longer has to start at the SW corner (0,0) facing NORTH. Set the
 * start cell + initial heading to match whichever corner you put it in; the
 * search floods back to THIS cell at the end, and wall-sensing maps correctly
 * because the heading is recorded. */

/** Set the start cell (x,y, clamped into the maze) and initial facing
 *  (0..3 = N,E,S,W). Resets the map so the new start takes effect. Default is
 *  (0,0) facing NORTH = the SW corner. */
void Solver_SetStart(int x, int y, int facing);

/** Read back the start cell + facing. Any pointer may be NULL. */
void Solver_GetStart(int *x, int *y, int *facing);

/** Read back the config: edge length and goal cell list (up to 4). Any pointer
 *  may be NULL; `gcount` returns how many goal cells are filled. */
void Solver_GetConfig(int *n, int *goalX, int *goalY, int *gcount);

/** Begin an autonomous search run: flood-fill explore to the centre, then a
 *  direct flood back to the start cell, at `cps` counts/s (<=0 uses a sane
 *  default). Returns false if the wall sensors aren't calibrated, a run is
 *  already active, or the battery gate blocks the first move. */
bool Solver_Start(int cps);

/** Begin a FAST (speed) run: replay the least-cost path on the map the last
 *  search learned - start -> goal -> back to start (then the home 180) - at `cps`
 *  (typically higher than search). The learned walls are kept; only the pose resets.
 *  Returns false if no search has produced a usable map (see Solver_FastReady),
 *  the goal isn't reachable on that map, the sensors aren't calibrated, or a run
 *  is already active. */
bool Solver_StartFast(int cps);

/** True once a search has come home successfully, so a fast run is possible.
 *  Cleared by a new Solver_Start/Reset, or when Solver_StartFast consumes it. */
bool Solver_FastReady(void);
void Solver_ClearFastReady(void);

/** True while a search run is in progress. */
bool Solver_Active(void);

/** Enable/disable front-wall square+centre before each pivot (needs 'ircal
 *  front'). ON by default: it lands the mouse at the true cell centre, squared
 *  to the maze, before every turn so it can't pivot off-centre into a post. When
 *  there's no front wall to reference it falls back to a plain pivot. Turn it off
 *  with 'align' only if the 'acfg' gains misbehave. */
void Solver_SetAlign(bool on);
bool Solver_GetAlign(void);

/* --- Path turn cost (flood planner) ---------------------------------------
 * The flood plans by MOVEMENT cost, not just cell count: each 90 pivot on the
 * route is charged `turn_cost` extra units (one unit = one cell driven). Higher
 * => the mouse prefers longer-but-straighter paths and bunches its turns (also
 * the shape a later diagonal run wants); 0 => the old cell-count-only behaviour.
 * Clamped 0..50, default 2. Persists across runs (a tuning, not run state). */
void Solver_SetTurnCost(int c);
int  Solver_GetTurnCost(void);

/* --- Front-wall distance calibration --------------------------------------
 * Learns how far the centre of a cell is from where the front-stop halts the
 * mouse, so a front-stop can be undone by reversing that exact distance back to
 * centre. Place the mouse at a cell CENTRE facing a wall, then run it. */

/** Drive from cell centre to the front-stop and back, recording the distance as
 *  the centre->wall-stop value. Needs 'ircal front'. False if a run is active or
 *  front isn't calibrated. */
bool Solver_StartFrontCal(int cps);

/** Drains once when the calibration finishes. ok=false means no wall was hit
 *  (or an implausible distance); mm is the measured/attempted distance. */
bool Solver_PopCalDone(bool *ok, int *mm);

/** Get/set the centre->wall-stop distance (mm) used to recenter after a
 *  front-stop. Set is clamped to 10..140. */
int  Solver_GetFrontWallMM(void);
void Solver_SetFrontWallMM(int mm);

/** Abort the run and stop the motors (what 'stop' calls). */
void Solver_Abort(void);

/** Pump the solver state machine - call every main-loop pass. */
void Solver_Task(void);

/** Drains once per cell decision point, for console reporting:
 *  x,y = cell, facing = 0..3 (N,E,S,W), flood = distance-to-goal at this cell,
 *  phase = 0 (search) | 1 (return), wallbits = walls known at this cell as a
 *  bitmask (N=1,E=2,S=4,W=8). Returns false when nothing new. */
bool Solver_PopStep(int *x, int *y, int *facing, int *flood, int *phase,
                    int *wallbits);

/** The 6 raw IR values (SensorId order) read at the last sense point, plus the
 *  two front-sensor thresholds. For the SIR diagnostic line - shows exactly what
 *  the sensors saw when a wall was missed/hallucinated. */
void Solver_GetSenseIR(int ir[6], int *thr_lf, int *thr_rf);

/** Drains once when the whole run ends. ok=false means aborted or boxed in. */
bool Solver_PopDone(bool *ok);

/** Drains once each time a fast-run SMOOTH (flow) turn launches: the turn angle
 *  (deg), radius (mm) and speed (cps). Lets the app mark flow-turns in a telemetry
 *  log. Any pointer may be NULL; false if none pending. */
bool Solver_PopFlowStart(int *deg, int *r, int *cps);

/* --- Run log (offline data capture) ---------------------------------------
 * Every decision/sense point of the last run is buffered in RAM as it happens
 * (no live streaming, so the loop isn't jittered). Fetch it afterwards with the
 * 'dump' console command, which replays it as SOLVE,/SIR, lines the app already
 * parses. RAM-only: lost on power cycle, so dump before powering off. */

/* --- Multi-run session logging --------------------------------------------
 * The launcher runs Search -> Fast -> Fastest hands-off. Bracket that sequence
 * with Solver_LogSessionBegin()/End() so the trace + run-log buffers are NOT
 * cleared between the three runs; every buffered sample is then tagged with a run
 * id (0=search, 1=fast, 2=fastest) and 'dump' hands all three back separately.
 * Outside a session each run clears its buffers as before. */
void Solver_LogSessionBegin(void);
void Solver_LogSessionEnd(void);

/** True when the buffered log is a single STANDALONE fast run (long-press from-memory
 *  replay) - its run 0 should be dumped as "fast", not "search". */
bool Solver_StandaloneFastLogged(void);

/** Highest run id present in the buffers (-1 if empty); 'dump' walks 0..this. */
int  Solver_MaxRun(void);
/** Run id tagged on log entry i / trace sample i (-1 if out of range). */
int  Solver_GetLogRun(int i);
int  Solver_GetTraceRun(int i);

/** Number of logged points from the last/current run. */
int  Solver_LogCount(void);

/** Read logged point i (0..count-1). x,y,facing,flood,phase,wallbits as in
 *  Solver_PopStep; ir[6] = raw IR (SensorId order) seen there; thr_lf/thr_rf =
 *  front thresholds. Any pointer may be NULL. False if i is out of range. */
bool Solver_GetLogEntry(int i, int *x, int *y, int *facing, int *flood, int *phase,
                        int *wallbits, int ir[6], int *thr_lf, int *thr_rf);

/* --- Continuous motion/sensor trace (sampled through the whole run) --------
 * A time-series of encoders, heading, gyro-Z, accel-Z, the 6 IR sensors, and the
 * current cell + flood. Replayed by 'dump' as TRACE, lines for the app to plot. */

/** Number of trace samples from the last/current run. */
int  Solver_TraceCount(void);

/** Read trace sample i. t_ms since run start; posL/posR encoder counts;
 *  head_ddeg (0.1 deg); gyroz_ddps (0.1 deg/s); accelz_mg (milli-g); s[6] IR;
 *  x,y cell; fld flood; vbat_mv pack voltage (mV). Any pointer may be NULL.
 *  False if i out of range. */
bool Solver_GetTraceEntry(int i, uint32_t *t_ms, int32_t *posL, int32_t *posR,
                          int *head_ddeg, int *gyroz_ddps, int *accelz_mg,
                          int s[6], int *x, int *y, int *fld, int *vbat_mv);

/* --- On-MCU headless simulation -------------------------------------------
 * Runs the REAL flood-fill algorithm against a virtual maze on the MCU CPU (no
 * motors/IR), to validate 16x16 behaviour, memory and per-decision CPU time, and
 * - with fault injection - to stress the phantom-wall recovery the way real
 * sensor errors / position desyncs would. Set the size + goal (Solver_SetSize /
 * goal) first, generate a maze, then run. The run populates the same run-log the
 * 'dump' command replays, so the app draws the simulated solve. */

/** Fault-injection rates for the sim, in percent (0 = perfect). miss = chance a
 *  real wall is NOT sensed; false = chance an open side reads as a wall; desync =
 *  chance, per cell driven, that the TRUE position slips an extra cell (models a
 *  ram-induced odometry desync). */
void Solver_SimSetFaults(int miss_pct, int false_pct, int desync_pct);

/** Generate a random, solvable virtual maze (randomized DFS) at the current size
 *  into the sim's ground-truth grid. Same seed -> same maze. */
void Solver_SimGenMaze(uint32_t seed);

/** Reset the sim's virtual maze to outer-border-only. Call before loading a real
 *  (e.g. MMS) maze cell-by-cell. */
void Solver_SimClear(void);

/** Load one ground-truth cell into the sim maze (bits = N1 E2 S4 W8). Use to push
 *  an MMS maze in from the app, then 'sim run' it. Out-of-range ignored. */
void Solver_SimSetCell(int x, int y, int bits);

/** Search strategy for the sim's phase-0 explore:
 *   0 = current explore-biased pick (prefers unvisited, with slack)
 *   1 = optimistic shortest-path descent (chase the shortest possible route,
 *       sensing as it goes - explores far less of the maze).
 *  This is what we're A/B testing in the sim before porting to the robot. */
void Solver_SetSearchMode(int mode);
int  Solver_GetSearchMode(void);

/** Result of one headless sim run. */
typedef struct
{
  bool reached;       /* reached the goal AND flooded back to start            */
  int  mode;          /* search mode used (0/1)                                */
  int  steps;         /* total decision steps                                  */
  int  path_len;      /* moves taken to FIRST reach the goal (search journey)   */
  int  learned;       /* shortest path through the DISCOVERED map (what a speed */
                      /* run would drive) - this is "the quickest path it found"*/
  int  optimal;       /* true shortest start->goal on the full maze (-1 = n/a) */
  int  explored;      /* unique cells the mouse visited                        */
  int  cells;         /* total cells (n*n) - for explored%                     */
  int  recoveries;    /* phantom-wall recoveries used                          */
  int  worst_us;      /* worst per-decision reflood time (us)                  */
  int  avg_us;        /* average per-decision reflood time (us)                */
} SimResult;

/** Run one headless solve over the current virtual maze. Returns true if it
 *  reached the goal and flooded back to start; fills *r (may be NULL). */
bool Solver_RunSim(SimResult *r);

#ifdef __cplusplus
}
#endif

#endif /* SOLVER_H */
