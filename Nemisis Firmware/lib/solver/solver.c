/**
 ******************************************************************************
 * @file    solver.c
 * @brief   On-robot flood-fill maze solver - see solver.h.
 *
 * Port of SIMULATOR/flood_fill_Algo/Main.c (search phase). The maze logic is
 * the proven simulator code; the simulator's blocking API (move/turn return an
 * "ack", wall reads return "true"/"false") is replaced by a non-blocking state
 * machine over the real control loop:
 *
 *   sim API_moveForward()  -> Control_StartAdvance(1) then wait for idle
 *   sim API_turnLeft/Right -> Control_StartPivot(+/-90) then wait for idle
 *   sim API_wallFront/L/R   -> Walls_Front()/Left()/Right()
 *
 * Each cell is stop-and-go: arrive, settle the IR EMA, sense, flood, pick, turn,
 * advance one cell, repeat. The maze size and goal are settable at runtime so
 * the app can match the real maze (default 6x6 home maze, goal = centre 2x2).
 ******************************************************************************
 */
#include "solver.h"
#include "control.h"
#include "sensors.h"
#include "battery.h"         /* pack voltage for the trace */
#include "main.h"            /* HAL_GetTick */
#include <string.h>

/* ===========================================================================
 *  Maze model  (logic verbatim from the simulator; size now runtime-settable)
 * ========================================================================= */
#define MAZE_MAX  16        /* array capacity; the live size `maze_n` is <= this */
#define INF       999

/* The persistent-map snapshot (mazestore.h) is sized to MAZE_MAX; keep them in
   lock-step so a save/restore can't overrun. */
typedef char solver_dim_matches_mazestore[(MAZE_MAX == MAZE_STORE_DIM) ? 1 : -1];

#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

#define WALL_N 0x1
#define WALL_E 0x2
#define WALL_S 0x4
#define WALL_W 0x8

static int walls[MAZE_MAX][MAZE_MAX];
/* Walls we TRUST absolutely: borders + ones we physically drove into (front-stop
   / stall). The rest of `walls` is sensed-only and may be a phantom. When the
   goal looks unreachable we distrust a sensed-only wall and re-verify it rather
   than give up - one bad IR read shouldn't kill the whole search. */
static int wall_phys[MAZE_MAX][MAZE_MAX];
static int flood[MAZE_MAX][MAZE_MAX];
static int visited[MAZE_MAX][MAZE_MAX];
/* confirmed_open[x][y][d] = physically confirmed no wall in direction d. Kept
   even though the search-phase flood-fill only needs walls[][]: it's the input
   the (deferred) diagonal speed-run planner will consume, so we build it now. */
static int confirmed_open[MAZE_MAX][MAZE_MAX][4];

/* --- Turn-aware path cost (heading-state Dijkstra) -------------------------
   flood[][] above counts CELLS only: a 5-cell straight and a 5-cell zig-zag
   score identically, yet the zig-zag is far slower (every turn = brake, pivot,
   re-accelerate). costg[x][y][h] is instead the MOVEMENT cost to reach the goal
   from cell (x,y) while facing heading h, charging `turn_cost` per 90 pivot on
   top of one unit per cell driven. The direction pickers below order their
   choices by this cost, so the planned route hugs long straights and bunches
   its turns - which is also exactly the shape a future diagonal speed-run
   collapses into smooth 45s, so this state graph is the foundation for that.
   flood[][] (pure cell distance) is still the source of truth for reachability,
   the explore "rush" threshold, phantom recovery and the app's map overlay;
   costg only re-orders WHICH of the equally-reachable moves we take. Setting
   turn_cost = 0 makes costg == cell distance, reproducing the old behaviour. */
typedef int16_t cost_t;
#define COST_INF  ((cost_t)30000)
#define MOVE_COST 1
#define GSTATES   (MAZE_MAX * MAZE_MAX * 4)
static cost_t  costg[MAZE_MAX][MAZE_MAX][4];
static int     turn_cost = 2;           /* movement-cost units per 90 pivot (0..50) */
static int16_t g_queue[GSTATES + 1];    /* ring buffer of encoded (x,y,h) states  */
static uint8_t g_inq[MAZE_MAX][MAZE_MAX][4];

static int maze_n = 6;                  /* edge length (cells); home maze = 6   */
static int goalX[4], goalY[4], goalCount;

/* Start cell + heading (the corner you place the mouse in). Default SW corner
   (0,0) facing NORTH - the classic micromouse start. Settable so the mouse can
   be launched from any corner; the return-phase flood targets this cell. */
static int startX = 0, startY = 0, startFacing = NORTH;

static int mouseX, mouseY, mouseFacing;
static int phase;                       /* 0 = search to centre, 1 = return     */

static const int DX[4]        = {  0,  1,  0, -1 };
static const int DY[4]        = {  1,  0, -1,  0 };
static const int WALL_BIT[4]  = { WALL_N, WALL_E, WALL_S, WALL_W };
static const int OPPOSITE[4]  = { SOUTH, WEST, NORTH, EAST };

static int queueX[MAZE_MAX * MAZE_MAX];
static int queueY[MAZE_MAX * MAZE_MAX];

/* Exploration heuristic tuning (from the simulator). */
#define RUSH_THRESHOLD  8       /* cells-from-goal below which we stop exploring */
#define EXPLORE_SLACK   2       /* extra flood steps we'll accept to visit new   */

/* ===========================================================================
 *  Hardware-layer config
 * ========================================================================= */
/* Firmware pivot sign: control.c uses +90 = LEFT, -90 = RIGHT (same mapping the
   'path'/'turn' commands use). Flip these two if a bench test shows the mouse
   turns the wrong way. */
#define TURN_LEFT_DEG    (+90)
#define TURN_RIGHT_DEG   (-90)
#define TURN_AROUND_DEG  (180)

#define DEFAULT_CPS      8000   /* search speed (counts/s); ~matches 'path'      */
/* Vacuum on the FAST run? DECOUPLED 2026-07-02: OFF for now. Plan = tune the PIVOT
   fast run as fast as it'll go DRY first (one variable), THEN bring the vacuum back
   for the SMOOTH-turn run. With vacuum ON the aggressive brake below skidded and
   spun the mouse out (asymmetric wheel slip); dry grip is lower still, so we run a
   gentler dry-safe brake here and let the bench tune it up. Flip to 1 to re-enable
   (keeps the measured 75% sweet spot). See [[speed-run-velocity-profile]]. */
#define RUN_FAST_USE_VACUUM   0
#define RUN_VACUUM_PCT       75   /* duty when RUN_FAST_USE_VACUUM==1 (smooth-turn era) */

/* NOTE: fast-run braking (accel/decel + active reverse brake) is NO LONGER forced
   here - Solver_StartFast leaves it to the caller so it's live-tunable. Console:
   set `accel <a> <d>` + `brake <cps>` then `fast <cps>`. Offline: the launcher sets
   RUN_FAST_ACCEL/DECEL + RUN_FAST_BRAKE_REV before the run. Overriding them here
   silently ate the user's console tuning (the "brake/accel does nothing" bug). */
#define SETTLE_MS        80U    /* let the IR EMA settle after stopping in a cell */
#define STALL_BACKOFF_MM   45   /* reverse this far after stalling into a missed wall */

/* ===========================================================================
 *  Run state
 * ========================================================================= */
typedef enum { S_IDLE = 0, S_SETTLE, S_SENSE, S_ALIGN, S_TURN, S_FLOW,
               S_RECENTER, S_CAL_DRIVE, S_CAL_BACK, S_FINISH_TURN } SolveState;

static SolveState state = S_IDLE;
static bool       active_flag;
static bool       cal_mode;             /* running the front-wall distance cal   */
/* Optional front-wall square+centre before a pivot. OFF by default: it leans on
   the front-IR align gains, and when those aren't dialled in it rotates the wrong
   amount before the pivot - the mouse "turns ~45 and does weird stuff" instead of
   a clean 90 (regression seen 2026-06-27 when this was briefly defaulted on). The
   solver pivots directly on odometry+gyro, exactly like the manual 'path'/'turn'
   runs that drive smoothly. Re-enable with 'align on' ONLY after tuning the align
   gains ('acfg') on the bench - then it can cancel post-turn skew. */
static bool       align_enabled = false; /* off: pivot directly like manual path */
/* Corridor wall-following: continuously re-centre off the SIDE sensors while
   driving each cell, so small per-cell errors can't accumulate into a drift that
   eventually clips a wall. ON for the duration of a run (restored after). The
   reliable side IR does the work; gain is the global 'fcfg'. */
static bool       prev_follow;            /* saved global follow state to restore  */
static int        run_cps;
/* Fast (speed) run: replays the least-cost path on the ALREADY-LEARNED map from
   start to goal at higher speed, no exploring, stops at the goal. is_fast picks
   the cost-optimal direction + makes the goal terminal; fast_ready latches once a
   search has come home successfully so the launcher knows a fast run is possible. */
static bool       is_fast;                /* the current run is a fast run         */
static bool       fast_ready;             /* a search finished OK -> map usable     */
static int        pending_dir;          /* cardinal we will advance along next  */
static int        pending_deg;          /* the pivot we'll do after any align   */
static int        pending_facing;       /* facing once that pivot completes     */
static bool       pending_is_finish;    /* the pending pivot is the home 180 -
                                           complete the run after it, don't flow */
static uint32_t   settle_t0;
/* Post-pivot rotation guard: a FOULED pivot (grinding a wall -> completes on
   timeout) can hand off while the chassis is still visibly spinning; starting the
   next advance then corrupts its heading frame from tick one (20260703 02:55 fast
   run: advance began at +82 dps -> entered the final cell 37 deg skewed -> ground
   the wall short of centre -> the home 180 crashed). A clean pivot exits well
   under this (snappy hand-off is <=45 dps by design), so waiting costs nothing. */
#define TURN_EXIT_GYRO_DPS   70.0f      /* still spinning faster = wait          */
#define TURN_EXIT_SETTLE_MS  500U       /* give up waiting after this            */
static uint32_t   turn_exit_t0;         /* 0 = not currently waiting             */
/* Stuck guards: a search shouldn't take many more cells than the maze has, and it
   shouldn't stall repeatedly in one place. Either = lost (drift/phantom walls), so
   we stop cleanly (keeping the log) instead of grinding/looping forever. */
static int        step_count;           /* sense/decision points this run        */
static int        stall_run;            /* consecutive stalls without progress   */
static int        recoveries;           /* phantom-wall re-verifies used this run */
#define MAX_RECOVERIES  16              /* cap so a contradictory map still ends   */

/* Calibrated centre->front-wall-stop distance (mm). After a front-stop the mouse
   is reversed this far to return to the cell centre. Learned by 'frontcal':
   from the centre, drive to the front-stop, that travelled distance IS this. */
static int        front_wall_mm = 70;
static bool       cal_done_flag;
static bool       cal_ok;
static int        cal_value;

/* one-shot events drained by the console */
static bool       step_flag;
static int        step_x, step_y, step_facing, step_flood, step_phase, step_walls;
static int        sense_ir[6];          /* the 6 IR values read at this sense    */
static int        sense_thr_lf, sense_thr_rf;  /* front thresholds for context   */
static bool       done_flag, done_ok;
/* One-shot events drained by the launcher/console for indications. */
static bool       goal_evt;              /* search first reached the goal          */
static bool       map_saved_evt;         /* a completed search auto-saved to flash */
static bool       map_saved_ok;          /* ...and whether the flash write verified */

/* === Run log (RAM) ========================================================
 * Every decision/sense point + the raw IR seen there, captured AS the run goes
 * so we never need to stream live (which jittered the loop). Fetched afterwards
 * with the 'dump' console command, which replays it as the same SOLVE,/SIR, lines
 * the app already parses -> the app rebuilds the discovered map and overlays it
 * on the maze you drew. RAM-only: lost on power cycle, so dump before powering
 * off. Cleared at the start of each run (Solver_Reset). */
#define RUNLOG_MAX  256
typedef struct
{
  uint8_t  x, y, facing, phase, wallbits;
  uint8_t  run;                         /* which run of the session (0=search..)   */
  int16_t  flood;
  int16_t  ir[6];                       /* raw IR (SensorId order) at this point  */
  int16_t  thr_lf, thr_rf;              /* front thresholds for context           */
} RunLogEntry;
static RunLogEntry runlog[RUNLOG_MAX];
static int         runlog_n;

/* === Multi-run session logging ============================================
 * A "session" is the hands-off launcher sequence Search -> Fast -> Fastest. So a
 * fetch afterwards can hand back all three runs separately, the trace + run-log
 * buffers are NOT cleared between the runs of a session; every sample is tagged
 * with `log_run_id` (0=search, 1=fast, 2=fastest) and the fresh-buffer reset
 * happens only at the session's first run (the search's Solver_Reset). Outside a
 * session (a standalone console 'solve'/'fast') each run clears the buffers as
 * before, so nothing changes for the bench. */
static int  log_run_id;                 /* current run index within the session   */
static bool log_session;                /* accumulate runs instead of clearing    */
static bool trace_new_run;              /* re-zero the trace clock at a run start  */
static bool standalone_fast_log;        /* the lone logged run (run 0) was a FAST run */

void Solver_LogSessionBegin(void) { log_session = true;  log_run_id = 0; }
void Solver_LogSessionEnd(void)   { log_session = false; }

/* True when the buffered log holds a single STANDALONE fast run (e.g. the long-press
   from-memory replay). Its run index is 0, which the dump would otherwise label
   "search" - callers use this to relabel run 0 "fast". A session (search+fast+
   fastest) leaves this false, so its run 0 stays "search". */
bool Solver_StandaloneFastLogged(void) { return standalone_fast_log; }

/* Highest run id present in either buffer (-1 if both empty) - the dump walks
   runs 0..MaxRun so it can group each run's lines for the app. */
int Solver_MaxRun(void)
{
  int m = -1;
  for (int i = 0; i < runlog_n; i++) if (runlog[i].run > m) m = runlog[i].run;
  return m;
}
int Solver_GetLogRun(int i)   { return (i >= 0 && i < runlog_n) ? runlog[i].run : -1; }

static void log_append(void)
{
  if (runlog_n >= RUNLOG_MAX) return;
  RunLogEntry *e = &runlog[runlog_n++];
  e->run     = (uint8_t)log_run_id;
  e->x       = (uint8_t)mouseX;     e->y     = (uint8_t)mouseY;
  e->facing  = (uint8_t)mouseFacing; e->phase = (uint8_t)phase;
  e->wallbits = (uint8_t)walls[mouseX][mouseY];
  e->flood   = (int16_t)flood[mouseX][mouseY];
  const int *raw = Sensors_Raw();
  for (int i = 0; i < 6; i++) e->ir[i] = (int16_t)raw[i];
  int thr[SENSOR_COUNT]; Sensors_GetCal(NULL, NULL, thr, NULL);
  e->thr_lf = (int16_t)thr[SENS_L_F];
  e->thr_rf = (int16_t)thr[SENS_R_F];
}

/* === Continuous run trace (RAM) ===========================================
 * A periodic time-series of everything moving - wheel encoders, heading, gyro-Z,
 * accel-Z, the 6 IR sensors, and the current cell + flood - sampled at TRACE_HZ
 * through the whole run. Lets the app plot the full motion afterwards (drift,
 * stalls, sensor behaviour) without live streaming. Dumped as TRACE, lines.
 * RAM-only; capped at TRACE_MAX samples (then it just stops appending). */
#define TRACE_HZ        10U
#define TRACE_PERIOD_MS (1000U / TRACE_HZ)
/* One shared buffer holds all three runs of a session. 1024 was too small: a long
   search (713 samples) + the fast run (311) filled it exactly, so the FASTEST run
   - the one we most want to trace (its smooth turns) - got ZERO samples (seen
   2026-07-01). Two fixes: bump the buffer, AND sample the slow, turn-uncritical
   search at half rate (below) so it can't hog the buffer. */
#define TRACE_MAX       1200            /* ~fits search@5Hz + fast + fastest @10Hz */
typedef struct
{
  uint32_t t_ms;
  int32_t  posL, posR;                 /* encoder counts                         */
  int16_t  head_ddeg;                  /* heading, 0.1 deg                        */
  int16_t  gyroz_ddps;                 /* gyro-Z, 0.1 deg/s                       */
  int16_t  accelz_mg;                  /* accel-Z, milli-g                        */
  int16_t  s[6];                       /* 6 IR sensors (SensorId order)          */
  uint8_t  x, y;
  uint8_t  run;                        /* which run of the session (0=search..)   */
  int16_t  flood;
  uint16_t vbat_mv;                    /* pack voltage [mV] - watch sag over a run */
} TraceEntry;
static TraceEntry trace[TRACE_MAX];
static int        trace_n;
static uint32_t   trace_t0, trace_last;

int Solver_GetTraceRun(int i) { return (i >= 0 && i < trace_n) ? trace[i].run : -1; }

static void trace_sample(void)
{
  /* Per-run period: in a session the search (run 0) is slow and the least
     turn-critical, so it samples at HALF rate to leave the shared buffer for the
     fast + FASTEST runs (whose smooth turns we most want to trace). Standalone runs
     (no session) always use the full rate. */
  uint32_t period = (log_session && log_run_id == 0) ? (TRACE_PERIOD_MS * 2U)
                                                     : TRACE_PERIOD_MS;
  uint32_t now = HAL_GetTick();
  if (trace_n == 0 || trace_new_run)
  { trace_t0 = now; trace_last = now - period; trace_new_run = false; }
  if ((now - trace_last) < period) return;
  if (trace_n >= TRACE_MAX) return;
  trace_last = now;

  TraceEntry *e = &trace[trace_n++];
  e->run  = (uint8_t)log_run_id;
  e->t_ms = now - trace_t0;
  int32_t pl = 0, pr = 0; float hd = 0, gz = 0, az = 0;
  Control_GetState(&pl, &pr, &hd, &gz, &az);
  e->posL = pl; e->posR = pr;
  e->head_ddeg  = (int16_t)(hd * 10.0f);
  e->gyroz_ddps = (int16_t)(gz * 10.0f);
  e->accelz_mg  = (int16_t)(az * 1000.0f);
  const int *raw = Sensors_Raw();
  for (int i = 0; i < 6; i++) e->s[i] = (int16_t)raw[i];
  e->x = (uint8_t)mouseX; e->y = (uint8_t)mouseY;
  e->flood = (int16_t)flood[mouseX][mouseY];
  BatterySnapshot bs; Battery_GetSnapshot(&bs);
  e->vbat_mv = bs.valid ? bs.pack_mv : 0;
}

/* ===========================================================================
 *  Maze algorithm  (verbatim from the simulator, simulator I/O removed)
 * ========================================================================= */
static int isGoal(int x, int y)
{
  for (int i = 0; i < goalCount; i++)
    if (goalX[i] == x && goalY[i] == y) return 1;
  return 0;
}
static int isStart(int x, int y) { return x == startX && y == startY; }

/* Flood the distance-to-goal into flood[][] using the given wall grid (so we can
   flood the real map `walls`, or the "optimistic" map `wall_phys` that ignores
   sensed-only walls). */
static void floodFromGrid(const int wg[MAZE_MAX][MAZE_MAX],
                          const int gx[], const int gy[], int gcount)
{
  for (int x = 0; x < maze_n; x++)
    for (int y = 0; y < maze_n; y++)
      flood[x][y] = INF;

  int head = 0, tail = 0;
  for (int i = 0; i < gcount; i++)
  {
    flood[gx[i]][gy[i]] = 0;
    queueX[tail] = gx[i]; queueY[tail++] = gy[i];
  }
  while (head < tail)
  {
    int cx = queueX[head], cy = queueY[head++];
    for (int d = 0; d < 4; d++)
    {
      if (wg[cx][cy] & WALL_BIT[d]) continue;
      int nx = cx + DX[d], ny = cy + DY[d];
      if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
      if (flood[nx][ny] != INF) continue;
      flood[nx][ny] = flood[cx][cy] + 1;
      queueX[tail] = nx; queueY[tail++] = ny;
    }
  }
}

/* Normal flood: distance to goal using the REAL (sensed) map. */
static void floodFill(const int gx[], const int gy[], int gcount)
{
  floodFromGrid(walls, gx, gy, gcount);
}

/* 90-degree pivots needed to rotate from heading `from` to heading `to` (0/1/2). */
static inline int turnsBetween(int from, int to)
{
  int d = (to - from + 4) & 3;
  return (d == 0) ? 0 : (d == 2) ? 2 : 1;
}

/* Turn-aware cost-to-go over the REAL (sensed) map, into costg[x][y][h]. SPFA on
   the reversed heading-state graph: every goal cell costs 0 in all four headings;
   the predecessors of a state (x,y,h) are
     - the cell BEHIND it (one MOVE_COST, same heading h) iff that edge is open, and
     - the two headings one 90 pivot away (turn_cost each; a 180 falls out as two).
   Re-orders moves only; topology (hence reachability) is identical to flood[][],
   so unreachable states simply stay COST_INF. Off the 1kHz path (decision points
   only), and at most GSTATES states each enqueued while not already queued. */
static void costFill(const int gx[], const int gy[], int gcount)
{
  for (int x = 0; x < maze_n; x++)
    for (int y = 0; y < maze_n; y++)
      for (int h = 0; h < 4; h++) { costg[x][y][h] = COST_INF; g_inq[x][y][h] = 0; }

  int head = 0, tail = 0;
  for (int i = 0; i < gcount; i++)
    for (int h = 0; h < 4; h++)
    {
      costg[gx[i]][gy[i]][h] = 0;
      g_inq[gx[i]][gy[i]][h] = 1;
      g_queue[tail] = (int16_t)(((gx[i] * MAZE_MAX) + gy[i]) * 4 + h);
      tail = (tail + 1) % (GSTATES + 1);
    }

  while (head != tail)
  {
    int s = g_queue[head]; head = (head + 1) % (GSTATES + 1);
    int h = s & 3, y = (s >> 2) % MAZE_MAX, x = (s >> 2) / MAZE_MAX;
    g_inq[x][y][h] = 0;
    int cs = costg[x][y][h];

    /* predecessor that drives forward INTO (x,y) facing h = the cell behind us,
       valid iff the edge between them is open (no wall on our `back` side). */
    if (!(walls[x][y] & WALL_BIT[OPPOSITE[h]]))
    {
      int px = x - DX[h], py = y - DY[h];
      if (px >= 0 && px < maze_n && py >= 0 && py < maze_n)
      {
        int nc = cs + MOVE_COST;
        if (nc < costg[px][py][h])
        {
          costg[px][py][h] = (cost_t)nc;
          if (!g_inq[px][py][h])
          {
            g_inq[px][py][h] = 1;
            g_queue[tail] = (int16_t)(((px * MAZE_MAX) + py) * 4 + h);
            tail = (tail + 1) % (GSTATES + 1);
          }
        }
      }
    }

    /* predecessors that pivot INTO heading h (from h+/-1), same cell. */
    for (int t = 0; t < 2; t++)
    {
      int ph = (h + (t ? 1 : 3)) & 3;
      int nc = cs + turn_cost;
      if (nc < costg[x][y][ph])
      {
        costg[x][y][ph] = (cost_t)nc;
        if (!g_inq[x][y][ph])
        {
          g_inq[x][y][ph] = 1;
          g_queue[tail] = (int16_t)(((x * MAZE_MAX) + y) * 4 + ph);
          tail = (tail + 1) % (GSTATES + 1);
        }
      }
    }
  }
}

/* Re-flood from the current goal/start set (phase-aware): both the pure
   cell-distance map (flood[][]) and the turn-aware cost map (costg[][][]) so
   every decision below sees a consistent, current pair. */
static void reflood(void)
{
  if (phase == 0) { floodFill(goalX, goalY, goalCount); costFill(goalX, goalY, goalCount); }
  else            { int sx[1] = {startX}, sy[1] = {startY};
                    floodFill(sx, sy, 1);  costFill(sx, sy, 1); }
}

/* Recover from a "boxed in" flood (goal unreachable through the SENSED map) that
   is actually caused by a phantom wall. We flood the OPTIMISTIC map (only borders
   + physically-confirmed walls block), and if THAT reaches the goal we know one or
   more sensed-only walls are lying. We walk the optimistic path from here toward
   the goal, clear the first sensed-only wall it crosses (the phantom), re-flood
   the real map, and carry on - the mouse will drive to that passage and physically
   re-verify it (a real wall just gets re-recorded; a phantom is now gone). Returns
   true if the goal is reachable again; false if the map is genuinely walled off. */
static bool try_recover(void)
{
  if (recoveries >= MAX_RECOVERIES) return false;

  /* optimistic flood into flood[][] (the real-map flood here was all INF). */
  if (phase == 0) floodFromGrid(wall_phys, goalX, goalY, goalCount);
  else            { int sx[1] = {startX}, sy[1] = {startY}; floodFromGrid(wall_phys, sx, sy, 1); }

  if (flood[mouseX][mouseY] >= INF) { reflood(); return false; }  /* truly walled */

  int cx = mouseX, cy = mouseY, cleared = 0;
  for (int g = 0; g < maze_n * maze_n; g++)
  {
    if (flood[cx][cy] == 0) break;                 /* reached the goal             */
    int bestd = -1;
    for (int d = 0; d < 4; d++)
    {
      if (wall_phys[cx][cy] & WALL_BIT[d]) continue;
      int nx = cx + DX[d], ny = cy + DY[d];
      if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
      if (flood[nx][ny] == flood[cx][cy] - 1) { bestd = d; break; }
    }
    if (bestd < 0) break;
    /* first edge that's a sensed-only wall = the phantom to distrust */
    if ((walls[cx][cy] & WALL_BIT[bestd]) && !(wall_phys[cx][cy] & WALL_BIT[bestd]))
    {
      walls[cx][cy] &= ~WALL_BIT[bestd];
      int nx = cx + DX[bestd], ny = cy + DY[bestd];
      if (nx >= 0 && nx < maze_n && ny >= 0 && ny < maze_n)
        walls[nx][ny] &= ~WALL_BIT[OPPOSITE[bestd]];
      cleared = 1;
      break;
    }
    cx += DX[bestd]; cy += DY[bestd];
  }

  reflood();                                        /* back to the real map         */
  if (!cleared) return false;
  recoveries++;
  return (flood[mouseX][mouseY] < INF);
}

/* Read the three wall sensors (relative to the mouse's facing) and record them
   into the absolute-direction maze model. The firmware Walls_* booleans are
   already front/left/right relative to the robot, so this maps 1:1. */
/* Record three front/right/left wall booleans (sensed at the mouse's heading)
   into the absolute-direction maze model at the BELIEVED cell. Split out so the
   on-MCU sim can feed walls from a virtual maze through the exact same path. */
static void record_walls(bool front, bool right, bool left)
{
  int dirs[3]    = { mouseFacing, (mouseFacing + 1) % 4, (mouseFacing + 3) % 4 };
  bool sensed[3] = { front, right, left };

  for (int i = 0; i < 3; i++)
  {
    /* No wall seen: record NOTHING. The front-diagonal IR can't see a wall from
       cell centre (reads below threshold), so "open" here is unreliable - we
       must NOT mark the passage open, or we'd suppress the front-stop and drive
       straight into a wall the sense missed. `confirmed_open` is set ONLY by
       actually driving a passage (markPassage), never by a sensor read. */
    if (!sensed[i]) continue;
    int d  = dirs[i];
    /* Never wall a passage we've physically driven through (a stray front read
       on a skewed return must not sever a known-open route). */
    if (confirmed_open[mouseX][mouseY][d]) continue;
    int nx = mouseX + DX[d], ny = mouseY + DY[d];
    walls[mouseX][mouseY] |= WALL_BIT[d];
    if (nx >= 0 && nx < maze_n && ny >= 0 && ny < maze_n)
      walls[nx][ny] |= WALL_BIT[OPPOSITE[d]];
  }
}

/* The real sense: the firmware Walls_* booleans are already front/left/right
   relative to the robot, so they map 1:1 into record_walls. */
static void senseAndRecordWalls(void)
{
  /* Commit walls with the high-confidence front read (both diagonals): a phantom
     front wall here permanently severs a passage, and record_walls() never
     clears one. The sensitive OR-based Walls_Front() stays for the front-stop. */
  record_walls(Walls_FrontConfident(), Walls_Right(), Walls_Left());
}

/* Return-phase direction: least-COST descent toward the goal (turn-aware), not
   just least cell distance - so the route prefers driving straight on and pays a
   `turn_cost` penalty for each pivot it would take. Ties prefer straight ahead. */
/* Least-cost outgoing direction from an ARBITRARY cell, entered facing `facing`.
   Same turn-aware cost as pickBestDirection, but parameterised so the fast-run
   smooth-turn look-ahead can ask "which way does the path go at the NEXT cell?"
   one cell early (the flow-turn must start a radius before that cell's centre). */
static int bestDirAt(int x, int y, int facing)
{
  int bestDir = -1, bestCost = COST_INF;
  for (int d = 0; d < 4; d++)
  {
    if (walls[x][y] & WALL_BIT[d]) continue;
    int nx = x + DX[d], ny = y + DY[d];
    if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
    if (costg[nx][ny][d] >= COST_INF) continue;            /* dead end / unreachable */
    int c = turnsBetween(facing, d) * turn_cost + MOVE_COST + costg[nx][ny][d];
    if (c < bestCost)                        { bestCost = c; bestDir = d; }
    else if (c == bestCost && d == facing)     bestDir = d;
  }
  return bestDir;
}

static int pickBestDirection(void)
{
  return bestDirAt(mouseX, mouseY, mouseFacing);
}

/* Search-phase direction: when far from goal, allow a little slack and prefer
   UNVISITED neighbours (explore); when close, rush straight in. */
static int pickExploreDirection(void)
{
  int minVal = INF + 1;
  for (int d = 0; d < 4; d++)
  {
    if (walls[mouseX][mouseY] & WALL_BIT[d]) continue;
    int nx = mouseX + DX[d], ny = mouseY + DY[d];
    if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
    if (flood[nx][ny] < minVal) minVal = flood[nx][ny];
  }
  if (minVal == INF + 1) return -1;

  int slack = (flood[mouseX][mouseY] > RUSH_THRESHOLD) ? EXPLORE_SLACK : 0;

  /* Candidate set is unchanged (flood within slack of the best, unvisited first)
     so the proven exploration behaviour stands; the only change is the tie-break:
     among equally-good candidates take the one reachable with the fewest pivots
     (turn-aware cost), so exploring still favours straights. */
  int bestDir = -1, bestCost = COST_INF, bestUnvis = 0;
  for (int d = 0; d < 4; d++)
  {
    if (walls[mouseX][mouseY] & WALL_BIT[d]) continue;
    int nx = mouseX + DX[d], ny = mouseY + DY[d];
    if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
    int val   = flood[nx][ny];
    int unvis = !visited[nx][ny];
    if (val > minVal + slack) continue;

    int cg   = (costg[nx][ny][d] >= COST_INF) ? (val * MOVE_COST) : costg[nx][ny][d];
    int cost = turnsBetween(mouseFacing, d) * turn_cost + MOVE_COST + cg;
    if ((unvis && !bestUnvis) ||
        (unvis == bestUnvis && cost < bestCost) ||
        (unvis == bestUnvis && cost == bestCost && d == mouseFacing))
    {
      bestDir = d; bestCost = cost; bestUnvis = unvis;
    }
  }
  return bestDir;
}

/* Mark a passage between the current cell and its neighbour in `dir` as either
   confirmed-open (we drove it) or a wall (the advance hit it). */
static void markPassage(int dir, bool open)
{
  int nx = mouseX + DX[dir], ny = mouseY + DY[dir];
  if (open)
  {
    confirmed_open[mouseX][mouseY][dir] = 1;
    if (nx >= 0 && nx < maze_n && ny >= 0 && ny < maze_n)
      confirmed_open[nx][ny][OPPOSITE[dir]] = 1;
  }
  else
  {
    /* markPassage(.,false) is only ever called from a PHYSICAL hit (front-stop /
       stall), so this wall is trusted - record it in wall_phys too so the phantom
       re-verify never clears it. (Sensed walls go straight into `walls` in
       senseAndRecordWalls and stay distrust-able.) */
    walls[mouseX][mouseY]     |= WALL_BIT[dir];
    wall_phys[mouseX][mouseY] |= WALL_BIT[dir];
    if (nx >= 0 && nx < maze_n && ny >= 0 && ny < maze_n)
    {
      walls[nx][ny]     |= WALL_BIT[OPPOSITE[dir]];
      wall_phys[nx][ny] |= WALL_BIT[OPPOSITE[dir]];
    }
  }
}

/* ===========================================================================
 *  Configuration
 * ========================================================================= */
void Solver_SetGoalCenter(void)
{
  int lo = maze_n / 2 - 1;        /* even n -> centre 2x2; clamps for tiny n   */
  int hi = maze_n / 2;
  if (lo < 0) lo = 0;
  if (hi >= maze_n) hi = maze_n - 1;
  goalCount = 0;
  for (int x = lo; x <= hi; x++)
    for (int y = lo; y <= hi; y++)
    {
      goalX[goalCount] = x; goalY[goalCount] = y; goalCount++;
      if (goalCount >= 4) return;
    }
}

void Solver_SetGoalCell(int x, int y)
{
  if (x < 0 || x >= maze_n || y < 0 || y >= maze_n) return;
  goalCount = 1;
  goalX[0] = x; goalY[0] = y;
}

void Solver_SetSize(int n)
{
  if (n < 2)        n = 2;
  if (n > MAZE_MAX) n = MAZE_MAX;
  maze_n = n;
  /* keep the start corner inside the resized maze (e.g. an NE start shrinks). */
  if (startX >= maze_n) startX = maze_n - 1;
  if (startY >= maze_n) startY = maze_n - 1;
  Solver_SetGoalCenter();
  Solver_Reset();
}

void Solver_SetStart(int x, int y, int facing)
{
  if (x < 0)        x = 0;
  if (x >= maze_n)  x = maze_n - 1;
  if (y < 0)        y = 0;
  if (y >= maze_n)  y = maze_n - 1;
  startX = x;
  startY = y;
  startFacing = ((facing % 4) + 4) % 4;
  Solver_Reset();                 /* re-init the mouse pose to the new start */
}

void Solver_GetStart(int *x, int *y, int *facing)
{
  if (x)      *x      = startX;
  if (y)      *y      = startY;
  if (facing) *facing = startFacing;
}

void Solver_GetConfig(int *n, int *gx, int *gy, int *gcount)
{
  if (n)      *n = maze_n;
  if (gcount) *gcount = goalCount;
  for (int i = 0; i < goalCount; i++)
  {
    if (gx) gx[i] = goalX[i];
    if (gy) gy[i] = goalY[i];
  }
}

/* ===========================================================================
 *  Public API
 * ========================================================================= */
void Solver_Reset(void)
{
  memset(walls,          0, sizeof(walls));
  memset(wall_phys,      0, sizeof(wall_phys));
  memset(visited,        0, sizeof(visited));
  memset(confirmed_open, 0, sizeof(confirmed_open));

  if (goalCount == 0) Solver_SetGoalCenter();

  /* Outer border walls are always known - and TRUSTED (in wall_phys), so the
     phantom re-verify never tries to drive through the maze edge. */
  for (int i = 0; i < maze_n; i++)
  {
    walls[i][0]          |= WALL_S;  wall_phys[i][0]          |= WALL_S;
    walls[i][maze_n - 1] |= WALL_N;  wall_phys[i][maze_n - 1] |= WALL_N;
    walls[0][i]          |= WALL_W;  wall_phys[0][i]          |= WALL_W;
    walls[maze_n - 1][i] |= WALL_E;  wall_phys[maze_n - 1][i] |= WALL_E;
  }

  mouseX = startX; mouseY = startY; mouseFacing = startFacing;
  phase  = 0;
  is_fast    = false;      /* a full reset is for a search                        */
  fast_ready = false;      /* the old learned map is gone, so no fast run yet     */
  state  = S_IDLE;
  step_flag = false;
  done_flag = false;
  runlog_n  = 0;            /* fresh run log */
  trace_n   = 0;            /* fresh motion trace */
  standalone_fast_log = false;   /* this run is a SEARCH -> dump labels run 0 "search" */
  step_count = 0;
  stall_run  = 0;
  recoveries = 0;
  goal_evt      = false;    /* clear stale one-shot indication events */
  map_saved_evt = false;
}

/* --- Persistent map: snapshot / restore (see solver.h) -------------------- */
void Solver_CaptureMap(MazeMap *m)
{
  if (!m) return;
  memset(m, 0, sizeof(*m));
  m->n           = (uint8_t)maze_n;
  m->startX      = (uint8_t)startX;
  m->startY      = (uint8_t)startY;
  m->startFacing = (uint8_t)startFacing;
  m->goalCount   = (uint8_t)goalCount;
  for (int i = 0; i < goalCount && i < 4; i++)
  {
    m->goalX[i] = (uint8_t)goalX[i];
    m->goalY[i] = (uint8_t)goalY[i];
  }
  for (int x = 0; x < maze_n; x++)
    for (int y = 0; y < maze_n; y++)
    {
      m->walls[x][y]     = (uint8_t)(walls[x][y]     & 0x0F);
      m->wall_phys[x][y] = (uint8_t)(wall_phys[x][y] & 0x0F);
      uint8_t o = 0;
      for (int d = 0; d < 4; d++)
        if (confirmed_open[x][y][d]) o |= (uint8_t)(1u << d);
      m->open[x][y] = o;
    }
}

bool Solver_RestoreMap(const MazeMap *m)
{
  if (!m || m->n < 2 || m->n > MAZE_MAX) return false;

  maze_n      = m->n;
  startX      = (m->startX < maze_n) ? m->startX : 0;
  startY      = (m->startY < maze_n) ? m->startY : 0;
  startFacing = m->startFacing & 3;

  goalCount = (m->goalCount > 4) ? 4 : m->goalCount;
  if (goalCount < 1) goalCount = 1;
  for (int i = 0; i < goalCount; i++)
  {
    goalX[i] = (m->goalX[i] < maze_n) ? m->goalX[i] : maze_n - 1;
    goalY[i] = (m->goalY[i] < maze_n) ? m->goalY[i] : maze_n - 1;
  }

  memset(walls,          0, sizeof(walls));
  memset(wall_phys,      0, sizeof(wall_phys));
  memset(confirmed_open, 0, sizeof(confirmed_open));
  memset(visited,        0, sizeof(visited));

  for (int x = 0; x < maze_n; x++)
    for (int y = 0; y < maze_n; y++)
    {
      walls[x][y]     = m->walls[x][y]     & 0x0F;
      wall_phys[x][y] = m->wall_phys[x][y] & 0x0F;
      for (int d = 0; d < 4; d++)
        confirmed_open[x][y][d] = (m->open[x][y] >> d) & 1;
    }

  /* Re-assert the outer border in case an old/truncated blob under-set it. */
  for (int i = 0; i < maze_n; i++)
  {
    walls[i][0]          |= WALL_S;  wall_phys[i][0]          |= WALL_S;
    walls[i][maze_n - 1] |= WALL_N;  wall_phys[i][maze_n - 1] |= WALL_N;
    walls[0][i]          |= WALL_W;  wall_phys[0][i]          |= WALL_W;
    walls[maze_n - 1][i] |= WALL_E;  wall_phys[maze_n - 1][i] |= WALL_E;
  }

  mouseX = startX; mouseY = startY; mouseFacing = startFacing;
  phase      = 0;
  is_fast    = false;
  state      = S_IDLE;
  step_flag  = false;
  done_flag  = false;
  fast_ready = true;      /* a saved map is exactly what a fast run needs      */
  return true;
}

bool Solver_PopMapSaved(bool *ok)
{
  if (!map_saved_evt) return false;
  map_saved_evt = false;
  if (ok) *ok = map_saved_ok;
  return true;
}

bool Solver_PopGoalReached(void)
{
  if (!goal_evt) return false;
  goal_evt = false;
  return true;
}

bool Solver_Active(void) { return active_flag; }

bool Solver_Start(int cps)
{
  if (active_flag) return false;

  /* Need side + front wall calibration, or every cell looks open and the mouse
     would drive straight into walls. (flags: bit1=side, bit2=front.) */
  uint16_t flags = 0;
  Sensors_GetCal(NULL, NULL, NULL, &flags);
  if (!(flags & 0x2) || !(flags & 0x4)) return false;

  Solver_Reset();
  /* The search is run 0 of a session (or the sole run of a standalone solve);
     either way its Solver_Reset above cleared the buffers. Re-zero the per-run
     trace clock so t_ms starts near 0 for this run. */
  trace_new_run = true;
  run_cps     = (cps > 0) ? cps : DEFAULT_CPS;

  /* Centre off the side walls for the whole run so drift can't build up. Only
     acts while driving forward with a wall to reference; pivots are unaffected. */
  prev_follow = Control_GetFollow();
  Control_SetFollow(true);

  /* One gyro-bias cal now (mouse is stationary at the start); every per-cell turn
     and advance then skips its own ~250 ms recal - that dead sit between cells was
     the bulk of a run's lost time, worst on the fast run. */
  Control_RunBegin();
  Control_SetPivotSnappy(false);   /* search pivots stay tight/accurate for mapping */
  Control_SetReachCreep(false);    /* NO creep-to-centre in search (would creep into walls) */

  active_flag = true;
  settle_t0   = HAL_GetTick();
  turn_exit_t0 = 0;
  state       = S_SETTLE;          /* settle, then sense at the start cell */
  return true;
}

void Solver_Abort(void)
{
  if (!active_flag) return;
  active_flag = false;
  cal_mode    = false;
  state       = S_IDLE;
  Battery_SetVacuum(0);                /* fan off on abort (no-op if search)   */
  Control_SetReachCreep(false);        /* creep-to-centre off for normal commands */
  Control_RunEnd();                    /* re-arm per-start gyro recal          */
  Control_SetPivotSnappy(false);       /* restore tight pivots for other commands */
  Control_SetFrontStop(true);          /* restore default for other commands */
  Control_SetFollow(prev_follow);      /* restore the global follow setting    */
  Control_Stop();
  done_flag = true;
  done_ok   = false;
}

bool Solver_FastReady(void)      { return fast_ready; }
void Solver_ClearFastReady(void) { fast_ready = false; }

/* Fast (speed) run: drive the least-cost path on the map the last search LEARNED -
   start -> goal -> back to start (then the home 180), at `cps` (higher than the
   search). Keeps walls[]/confirmed_open[]/visited[] - only the pose + run reset.
   Refuses if no usable map (no search came home) or the goal isn't reachable on
   the learned walls, or IR cal is missing. */
bool Solver_StartFast(int cps)
{
  if (active_flag)  return false;
  if (!fast_ready)  return false;            /* no search has produced a map yet  */

  uint16_t flags = 0;                        /* same cal gate as Solver_Start     */
  Sensors_GetCal(NULL, NULL, NULL, &flags);
  if (!(flags & 0x2) || !(flags & 0x4)) return false;

  /* LIGHT reset: keep the learned map, refresh only pose + per-run bookkeeping. */
  mouseX = startX; mouseY = startY; mouseFacing = startFacing;
  phase  = 0;                                /* flood targets the GOAL            */
  is_fast    = true;
  state      = S_IDLE;
  step_flag  = false;
  done_flag  = false;
  /* In a launcher session (Search->Fast->Fastest) KEEP the buffers and start a new
     tagged run, so all three runs can be fetched afterwards; a standalone 'fast'
     from the console clears them as before. */
  if (log_session) { log_run_id++; }
  else             { runlog_n = 0; trace_n = 0; log_run_id = 0; }
  standalone_fast_log = !log_session;         /* a lone fast run -> dump labels it "fast" */
  trace_new_run = true;                       /* per-run trace clock              */
  step_count = 0;  stall_run = 0;  recoveries = 0;

  reflood();                                 /* cost/flood on the learned walls   */
  if (flood[startX][startY] >= INF) return false;   /* goal unreachable on the map */

  run_cps     = (cps > 0) ? cps : DEFAULT_CPS;
  prev_follow = Control_GetFollow();
  Control_SetFollow(true);

  /* Braking (accel/decel + active reverse brake) is NOT set here - it uses whatever
     the caller left in place, so it's LIVE-TUNABLE: the console `accel`/`brake`
     commands apply directly to `fast <cps>`, and the offline launcher sets its own
     RUN_FAST_* before calling us. (We used to override them here, which silently
     ignored the user's console tuning - that was the "brake/accel does nothing" bug.
     Reminder: brake distance = v^2/(2*decel), so LOWER decel brakes EARLIER.) */

  fast_ready  = false;                       /* consume the arm                   */
  Control_RunBegin();                        /* cal once; skip per-cell recal     */
#if RUN_FAST_USE_VACUUM
  /* Vacuum ON for the whole fast run (can't toggle per-cell - fan spool is too
     slow). Started AFTER RunBegin so the one-shot gyro-bias sample above is taken
     fan-off (no vibration); the fan then spools during the SETTLE before we move.
     Grip data says the vacuum buys ~+56% traction -> more braking + pivot grip. */
  Battery_SetVacuum(RUN_VACUUM_PCT);
#endif
  Control_SetPivotSnappy(true);              /* fast run: no per-turn settle sit  */
  Control_SetReachCreep(true);               /* creep to the turn-cell centre on a short stop */
  active_flag = true;
  settle_t0   = HAL_GetTick();
  turn_exit_t0 = 0;
  state       = S_SETTLE;
  return true;
}

static void finish(bool ok)
{
  active_flag = false;
  state       = S_IDLE;
  Battery_SetVacuum(0);                /* fan off on any run end (no-op if search) */
  Control_RunEnd();                    /* re-arm per-start gyro recal          */
  Control_SetPivotSnappy(false);       /* restore tight pivots for other commands */
  Control_SetReachCreep(false);        /* creep-to-centre off for normal commands */
  Control_SetFrontStop(true);          /* restore default for other commands */
  Control_SetFollow(prev_follow);      /* restore the global follow setting    */
  Control_Stop();
  done_flag = true;
  done_ok   = ok;
  /* Any run that comes home OK (search OR fast) ends back at the start with a
     usable map -> arm a (next) fast run. Fast runs now do the full there-and-back,
     so re-arming is safe: the mouse really is at the start, ready to go again. */
  if (ok) fast_ready = true;

  /* Persist the learned map so a fast run survives a power cycle / battery swap.
     Only a SEARCH that came home is saved (a fast run replays this same map, and
     the mouse is genuinely stopped here so the flash erase can't glitch the loop).
     The "quickest path" is recomputed from these walls at Solver_StartFast, so
     the map IS the saved path - nothing else to store. */
  if (ok && !is_fast)
  {
    MazeMap m;
    Solver_CaptureMap(&m);
    map_saved_ok  = MazeStore_Save(&m);
    map_saved_evt = true;
  }
}

/* Issue the queued pivot and commit the new facing. False on battery gate. */
static bool start_pending_turn(void)
{
  if (!Control_StartPivot(pending_deg, 0)) return false;
  mouseFacing = pending_facing;
  return true;
}

/* Front-stop trust for the passage ahead of the current cell. The front-stop
   fires on the noisy front-DIAGONAL IR (Walls_FrontRaw), which grazes a cell's
   SIDE walls mid-cell and reads a PHANTOM wall ahead. On a SEARCH we honour it
   anyway (an unexplored passage might really be walled). On a FAST run over the
   ALREADY-LEARNED map, though, a passage we physically drove during the search
   (confirmed_open) is known clear - and a phantom front-stop there is poison: it
   brakes the advance at ~half a cell, BEFORE the cell mark that would step our
   position (mark @180mm vs band @54-104mm), so the position never advances, the
   solver re-senses in place, picks the same open passage, and front-stops on the
   same phantom again - an in-place stall that eventually drifts off the map and
   spins out (root cause of the 2026-07-01 stuck-at-(2,4) crash, no FLOW ever
   fired). So on a fast run we trust confirmed_open and disable the front-stop for
   that passage; the physical stall guard still catches a genuinely-walled one. */
static void set_frontstop_for_ahead(void)
{
  /* Front-stop now stays ON for fast runs too. The reason it used to be disabled -
     a single-diagonal PHANTOM front-stop mid-cell that braked before the cell mark ->
     position lag -> re-ram - is gone: the fast-run front-stop (control.c) uses the
     CONFIDENT (both-diagonal) read (no single-side phantom) and FIRES the cell mark on
     trip (position steps with the stop, no lag). Keeping it on is what stops the mouse
     driving THROUGH walls when drift misplaces it. (Search unchanged - always on.) */
  Control_SetFrontStop(true);
}

/* FAST run only: how many cells straight ahead until the path turns? Walks the
   KNOWN least-cost path forward from the current cell/facing (read-only, via
   bestDirAt on the current flood) and counts consecutive straight cells until the
   direction changes, a terminal cell, or a wall/edge. Lets the flow advance stop
   PREDICTIVELY at the turn cell (the move's own v^2/2decel brake) instead of the
   reactive Control_AdvanceBrakeNow, which fires AT the cell centre = too late to
   stop in it at speed (the overshoot-into-the-turn bug). >=1. */
static int fast_cells_to_turn(void)
{
  int x = mouseX, y = mouseY, f = mouseFacing, n = 0;
  for (int guard = 0; guard < maze_n * maze_n; guard++)
  {
    if (walls[x][y] & WALL_BIT[f]) break;                 /* wall dead ahead      */
    int nx = x + DX[f], ny = y + DY[f];
    if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) break;   /* maze edge   */
    n++; x = nx; y = ny;                                  /* step into next cell  */
    if ((phase == 0 && isGoal(x, y)) || (phase == 1 && isStart(x, y))) break;
    if (bestDirAt(x, y, f) != f) break;                   /* path turns here      */
  }
  return (n > 0) ? n : 1;
}

/* Begin a continuous forward flow: one advance that coasts through consecutive
   open cells (centring active) instead of stopping at each. On a FAST run it caps
   the advance at the next turn cell so the distance-brake stops PREDICTIVELY there;
   on a search (path unknown) it runs up to maze_n and relies on the reactive brake
   / front-stop. Either way it also ends at a wall ahead or the S_FLOW decision. */
static bool start_flow(void)
{
  set_frontstop_for_ahead();
  int cells = is_fast ? fast_cells_to_turn() : maze_n;
  return Control_StartAdvance(cells, run_cps);
}

/* Fast-run SMOOTH turn (Fastest profile). Called at a cell mark while flowing
   straight, BEFORE the turn cell (tx,ty = the next cell we're heading into). If that
   cell is a clean 90 deg turn, launch an inline flow-turn (keeps cruise speed) whose
   lead-in carries the curve start to a radius before its centre, curves through it,
   and flows out into the cell beyond - no stop. Advances the map position past the
   corner. Returns true if launched (caller stops draining marks). Falls back (false)
   for U-turns, stop cells, consecutive turns (diagonals, not handled yet) or a
   blocked passage, so those take the normal brake+pivot. */
/* one-shot event: a fast-run flow-turn just launched (deg/radius/cps), for logging */
static volatile bool flow_evt_flag;
static int flow_evt_deg, flow_evt_r, flow_evt_cps;

bool Solver_PopFlowStart(int *deg, int *r, int *cps)
{
  if (!flow_evt_flag) return false;
  flow_evt_flag = false;
  if (deg) *deg = flow_evt_deg;
  if (r)   *r   = flow_evt_r;
  if (cps) *cps = flow_evt_cps;
  return true;
}

static bool try_smooth_turn(int tx, int ty)
{
  if (tx < 0 || tx >= maze_n || ty < 0 || ty >= maze_n) return false;
  if (walls[mouseX][mouseY] & WALL_BIT[mouseFacing]) return false;   /* not open  */
  /* never smooth INTO a cell we must stop at (goal in search-back / start) */
  if ((phase == 0 && isGoal(tx, ty)) || (phase == 1 && isStart(tx, ty))) return false;

  int dirT = bestDirAt(tx, ty, mouseFacing);
  if (dirT < 0) return false;
  int diff = (dirT - mouseFacing + 4) % 4;
  if (diff == 0 || diff == 2) return false;            /* straight or U-turn        */

  int ex = tx + DX[dirT], ey = ty + DY[dirT];          /* the exit cell             */
  if (ex < 0 || ex >= maze_n || ey < 0 || ey >= maze_n) return false;
  bool e_term = (phase == 0 && isGoal(ex, ey)) || (phase == 1 && isStart(ex, ey));
  if (!e_term && bestDirAt(ex, ey, dirT) != dirT) return false;  /* exit turns again */

  float rad, ent, exo; Control_GetArcGeom(&rad, &ent, &exo);
  int deg = (diff == 1) ? TURN_RIGHT_DEG : TURN_LEFT_DEG;
  if (!Control_StartFlowTurnInline(deg, run_cps, (int)rad, (int)ent, (int)exo, true))
    return false;

  /* mark this flow-turn in the telemetry stream so a log can locate it */
  flow_evt_deg = deg; flow_evt_r = (int)rad; flow_evt_cps = run_cps;
  flow_evt_flag = true;

  /* bookkeeping: the flow-turn drives current->T (the corner) then T->E (the exit) */
  markPassage(mouseFacing, true);                       /* current -> T open         */
  mouseX = tx; mouseY = ty;
  markPassage(dirT, true);                               /* T -> E open               */
  mouseX = ex; mouseY = ey;
  mouseFacing = dirT;
  visited[mouseX][mouseY] = 1;
  reflood();
  step_x = mouseX; step_y = mouseY; step_facing = mouseFacing;
  step_flood = flood[mouseX][mouseY]; step_phase = phase;
  step_walls = walls[mouseX][mouseY]; step_flag = true;
  log_append();
  return true;
}

void Solver_SetAlign(bool on) { align_enabled = on; }
bool Solver_GetAlign(void)    { return align_enabled; }

void Solver_SetTurnCost(int c) { if (c < 0) c = 0; if (c > 50) c = 50; turn_cost = c; }
int  Solver_GetTurnCost(void)  { return turn_cost; }

/* --- front-wall distance calibration -------------------------------------- */
bool Solver_StartFrontCal(int cps)
{
  if (active_flag) return false;

  uint16_t flags = 0;                        /* need front cal for the front-stop */
  Sensors_GetCal(NULL, NULL, NULL, &flags);
  if (!(flags & 0x4)) return false;

  run_cps     = (cps > 0) ? cps : 8000;
  cal_mode    = true;
  active_flag = true;
  cal_done_flag = false;
  /* Creep to the wall (front IR contact), not the early front-stop trip. */
  if (!Control_StartContactProbe())
  {
    active_flag = false; cal_mode = false;
    return false;
  }
  state = S_CAL_DRIVE;
  return true;
}

bool Solver_PopCalDone(bool *ok, int *mm)
{
  if (!cal_done_flag) return false;
  cal_done_flag = false;
  if (ok) *ok = cal_ok;
  if (mm) *mm = cal_value;
  return true;
}

int  Solver_GetFrontWallMM(void)        { return front_wall_mm; }
void Solver_SetFrontWallMM(int mm)      { if (mm >= 10 && mm <= 140) front_wall_mm = mm; }

/* ===========================================================================
 *  On-MCU headless simulation
 *
 *  Runs the REAL maze algorithm (reflood / pickExplore|BestDirection / markPassage
 *  / try_recover - the same functions the live FSM calls) against a VIRTUAL maze
 *  held in sim_truth[][], with the motors + IR replaced by instant virtual moves
 *  and a ground-truth wall lookup. Lets us prove the 16x16 brain (correctness,
 *  memory, CPU timing) and - via fault injection - stress the phantom-wall
 *  recovery the way real sensor errors would, all without a physical maze.
 *
 *  Believed vs true pose: the algorithm only ever knows mouseX/mouseY (believed).
 *  We keep a separate truthX/truthY (where the robot REALLY is) so that, under
 *  desync injection, walls get SENSED at the true cell but RECORDED at the
 *  believed cell - exactly the "missed wall -> ram -> ghost walls" cascade.
 * ========================================================================= */
static int      sim_truth[MAZE_MAX][MAZE_MAX];   /* virtual ground-truth walls    */
static int      truthX, truthY;                  /* REAL pose (believed = mouseX/Y) */
static uint32_t sim_rng = 1u;
static int      sim_miss_pct, sim_false_pct, sim_desync_pct;
static uint32_t sim_dec_us_max, sim_dec_us_sum;  /* per-decision reflood timing     */
static int      sim_dec_n;
static int      sim_search_mode;                 /* 0=explore (current), 1=shortest  */

void Solver_SetSearchMode(int m) { sim_search_mode = (m < 0) ? 0 : (m > 2) ? 2 : m; }
int  Solver_GetSearchMode(void)  { return sim_search_mode; }

static uint32_t sim_rand(void)
{
  sim_rng = sim_rng * 1664525u + 1013904223u;    /* Numerical Recipes LCG          */
  return sim_rng;
}
static bool sim_chance(int pct)
{
  return pct > 0 && (int)(sim_rand() % 100u) < pct;
}

void Solver_SimSetFaults(int miss_pct, int false_pct, int desync_pct)
{
  sim_miss_pct   = (miss_pct   < 0) ? 0 : (miss_pct   > 100 ? 100 : miss_pct);
  sim_false_pct  = (false_pct  < 0) ? 0 : (false_pct  > 100 ? 100 : false_pct);
  sim_desync_pct = (desync_pct < 0) ? 0 : (desync_pct > 100 ? 100 : desync_pct);
}

/* Build a random, fully-connected (solvable) maze into sim_truth via iterative
   randomized-DFS (spanning tree). Outer borders stay; the goal is reachable. */
void Solver_SimGenMaze(uint32_t seed)
{
  static uint8_t vis[MAZE_MAX][MAZE_MAX];
  int  stack[MAZE_MAX * MAZE_MAX], sp = 0;

  sim_rng = seed ? seed : 1u;
  for (int x = 0; x < maze_n; x++)
    for (int y = 0; y < maze_n; y++)
    { sim_truth[x][y] = WALL_N | WALL_E | WALL_S | WALL_W; vis[x][y] = 0; }

  vis[0][0] = 1; stack[sp++] = 0;                /* start carving from (0,0)        */
  while (sp > 0)
  {
    int cur = stack[sp - 1], cx = cur / MAZE_MAX, cy = cur % MAZE_MAX;
    int nd[4], nn = 0;
    for (int d = 0; d < 4; d++)
    {
      int nx = cx + DX[d], ny = cy + DY[d];
      if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
      if (!vis[nx][ny]) nd[nn++] = d;
    }
    if (nn == 0) { sp--; continue; }             /* dead end: backtrack             */
    int d  = nd[sim_rand() % (uint32_t)nn];
    int nx = cx + DX[d], ny = cy + DY[d];
    sim_truth[cx][cy] &= ~WALL_BIT[d];           /* knock the shared wall down       */
    sim_truth[nx][ny] &= ~WALL_BIT[OPPOSITE[d]];
    vis[nx][ny] = 1; stack[sp++] = nx * MAZE_MAX + ny;
  }
}

/* Reset the sim maze to outer-border-only - call before loading a real maze. */
void Solver_SimClear(void)
{
  for (int x = 0; x < maze_n; x++)
    for (int y = 0; y < maze_n; y++)
    {
      int b = 0;
      if (y == maze_n - 1) b |= WALL_N;
      if (x == maze_n - 1) b |= WALL_E;
      if (y == 0)          b |= WALL_S;
      if (x == 0)          b |= WALL_W;
      sim_truth[x][y] = b;
    }
}

/* Load one ground-truth cell (bits = N1 E2 S4 W8) - lets the app push an MMS maze
   in and 'sim run' it instead of a generated one. Out-of-range ignored. */
void Solver_SimSetCell(int x, int y, int bits)
{
  if (x < 0 || x >= maze_n || y < 0 || y >= maze_n) return;
  sim_truth[x][y] = bits & (WALL_N | WALL_E | WALL_S | WALL_W);
}

/* Sense the three walls from sim_truth at the TRUE pose (heading still synced),
   apply miss/false fault injection, and record at the believed cell. */
static void sim_sense(void)
{
  int dirs[3] = { mouseFacing, (mouseFacing + 1) % 4, (mouseFacing + 3) % 4 };
  bool s[3];
  for (int i = 0; i < 3; i++)
  {
    bool real = (sim_truth[truthX][truthY] & WALL_BIT[dirs[i]]) != 0;
    if      ( real && sim_chance(sim_miss_pct))  real = false;   /* missed a wall   */
    else if (!real && sim_chance(sim_false_pct)) real = true;    /* phantom wall    */
    s[i] = real;
  }
  record_walls(s[0], s[1], s[2]);
}

/* Shortest cell-path length start->goal over a wall grid (BFS; a set wall bit
   blocks, unknown = open). Used for both the true optimal (on sim_truth) and the
   best path the mouse LEARNED (on the discovered `walls`). -1 if unreachable. */
static int bfs_to_goal(int grid[MAZE_MAX][MAZE_MAX])
{
  static int dist[MAZE_MAX][MAZE_MAX];
  static int qx[MAZE_MAX * MAZE_MAX], qy[MAZE_MAX * MAZE_MAX];
  for (int x = 0; x < maze_n; x++)
    for (int y = 0; y < maze_n; y++) dist[x][y] = -1;

  int h = 0, t = 0;
  dist[startX][startY] = 0; qx[t] = startX; qy[t] = startY; t++;
  while (h < t)
  {
    int x = qx[h], y = qy[h]; h++;
    if (isGoal(x, y)) return dist[x][y];
    for (int d = 0; d < 4; d++)
    {
      if (grid[x][y] & WALL_BIT[d]) continue;
      int nx = x + DX[d], ny = y + DY[d];
      if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
      if (dist[nx][ny] >= 0) continue;
      dist[nx][ny] = dist[x][y] + 1; qx[t] = nx; qy[t] = ny; t++;
    }
  }
  return -1;
}

/* --- Mode 2: "confirm-the-optimal-path" explorer (SIM ONLY) ----------------
   The catch with modes 0/1 is they stop the moment they touch the goal, so the
   shortest OPTIMISTIC path (bfs over sensed walls, treating unknowns as open -
   what `learned` measures) usually still runs through cells the mouse never
   sensed => learned < optimal (optimistic phantom shortcut). Mode 2 keeps
   exploring until that optimistic best-path is FULLY sensed: then it has no
   unknown walls left, so it's a real drivable path, and since optimistic <=
   true-optimal, a driven path of that length IS the optimal => found-optimal.

   Per phase-0 step it: floods optimistically to the goal, traces the shortest
   path from the START, and finds the first cell on it the mouse hasn't visited.
   If none -> the optimal path is confirmed (return -2, caller switches to the
   home phase). Otherwise it navigates one step toward that unvisited cell.
   Uses only the existing flood helpers + visited[] - no new maze state, and it
   never touches the live FSM (real `solve` still uses pickExploreDirection). */
static int sim_confirm_optimal_dir(void)
{
  visited[mouseX][mouseY] = 1;              /* here + already sensed = explored, so   */
                                           /* the trace won't target the cell we're on */
  floodFill(goalX, goalY, goalCount);      /* flood[][] = optimistic dist to goal */

  /* trace the optimistic shortest path from start; first unvisited cell on it */
  int px = startX, py = startY, tx = -1, ty = -1;
  for (int g = 0; g < maze_n * maze_n; g++)
  {
    if (flood[px][py] == 0) break;                 /* reached the goal on the trace */
    int nd = -1;
    for (int d = 0; d < 4; d++)
    {
      if (walls[px][py] & WALL_BIT[d]) continue;
      int nx = px + DX[d], ny = py + DY[d];
      if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
      if (flood[nx][ny] == flood[px][py] - 1) { nd = d; break; }
    }
    if (nd < 0) break;
    px += DX[nd]; py += DY[nd];
    if (!visited[px][py]) { tx = px; ty = py; break; }
  }

  if (tx < 0) return -2;                    /* whole optimistic path sensed = optimal */

  /* navigate the mouse toward (tx,ty): flood to it, descend (turn-aware tie-break) */
  int gtx[1] = { tx }, gty[1] = { ty };
  floodFromGrid(walls, gtx, gty, 1);
  if (flood[mouseX][mouseY] >= INF) return -1;      /* unreachable (shouldn't happen) */
  int bestd = -1, bestcost = 99;
  for (int d = 0; d < 4; d++)
  {
    if (walls[mouseX][mouseY] & WALL_BIT[d]) continue;
    int nx = mouseX + DX[d], ny = mouseY + DY[d];
    if (nx < 0 || nx >= maze_n || ny < 0 || ny >= maze_n) continue;
    if (flood[nx][ny] == flood[mouseX][mouseY] - 1)
    {
      int c = turnsBetween(mouseFacing, d);
      if (bestd < 0 || c < bestcost || (c == bestcost && d == mouseFacing))
      { bestd = d; bestcost = c; }
    }
  }
  return bestd;
}

/* One headless solve over the current sim_truth. Fills the result struct (NULL
   ok). reached = got to the goal and flooded back to start. The phase-0 search
   strategy is sim_search_mode: 0 = the current explore-biased pick, 1 = the
   optimistic shortest-path descent, 2 = confirm-the-optimal-path explorer
   (keeps sensing until the optimistic best-path is fully known => learned==optimal). */
bool Solver_RunSim(SimResult *r)
{
  Solver_Reset();                                /* model -> borders + start pose   */
  truthX = mouseX; truthY = mouseY;              /* true pose starts == believed    */
  sim_dec_us_max = sim_dec_us_sum = 0; sim_dec_n = 0;

  uint32_t cyc_per_us = SystemCoreClock / 1000000u; if (!cyc_per_us) cyc_per_us = 1;
  int  guard = 6 * maze_n * maze_n + 40;
  int  stall_run_local = 0, steps = 0, moves = 0, path_len = -1;
  bool reached = false;

  for (;;)
  {
    if (++steps > guard) break;                  /* thrashing -> give up            */
    if (phase == 0 && isGoal(mouseX, mouseY))
    {
      if (path_len < 0) path_len = moves;        /* moves taken to first reach goal */
      /* modes 0/1 stop exploring at the goal; mode 2 keeps going until the
         optimistic best-path is confirmed (it owns its own phase transition). */
      if (sim_search_mode != 2) phase = 1;
    }
    if (phase == 1 && isStart(mouseX, mouseY)) { reached = true; break; }

    sim_sense();

    uint32_t c0 = DWT->CYCCNT;
    reflood();
    uint32_t us = (DWT->CYCCNT - c0) / cyc_per_us;
    if (us > sim_dec_us_max) sim_dec_us_max = us;
    sim_dec_us_sum += us; sim_dec_n++;

    if (flood[mouseX][mouseY] >= INF && !try_recover()) break;   /* boxed in        */
    log_append();                                /* so 'dump' can render the map    */

    /* phase-0 picker is the strategy under test; return phase always least-cost */
    int dir;
    if (phase == 0 && sim_search_mode == 2)
    {
      dir = sim_confirm_optimal_dir();
      if (dir == -2) { phase = 1; continue; }    /* optimal confirmed -> go home     */
    }
    else
    {
      dir = (phase == 0)
          ? (sim_search_mode ? pickBestDirection() : pickExploreDirection())
          : pickBestDirection();
    }
    if (dir < 0) break;                          /* no open neighbour               */
    visited[mouseX][mouseY] = 1;

    mouseFacing = dir;                            /* virtual pivot (exact)           */

    if (sim_truth[truthX][truthY] & WALL_BIT[dir])   /* a wall the sense missed     */
    {
      markPassage(dir, false);                   /* front-stop/stall records it     */
      if (++stall_run_local >= 4) break;         /* repeatedly jammed = lost        */
    }
    else
    {
      markPassage(dir, true);
      mouseX += DX[dir]; mouseY += DY[dir];       /* believed move                   */
      truthX += DX[dir]; truthY += DY[dir];       /* true move                       */
      stall_run_local = 0; moves++;

      if (sim_chance(sim_desync_pct))            /* slip an extra cell (ram-like)   */
        for (int t = 0; t < 4; t++)
        {
          int dd = (int)(sim_rand() % 4u);
          if (sim_truth[truthX][truthY] & WALL_BIT[dd]) continue;
          int tx = truthX + DX[dd], ty = truthY + DY[dd];
          if (tx >= 0 && tx < maze_n && ty >= 0 && ty < maze_n)
          { truthX = tx; truthY = ty; break; }
        }
    }
  }

  if (r)
  {
    int explored = 0;
    for (int x = 0; x < maze_n; x++)
      for (int y = 0; y < maze_n; y++) if (visited[x][y]) explored++;
    r->reached     = reached;
    r->mode        = sim_search_mode;
    r->steps       = steps;
    r->path_len    = path_len;                   /* -1 if goal never reached         */
    r->learned     = bfs_to_goal(walls);         /* best path through what it learned */
    r->optimal     = bfs_to_goal(sim_truth);     /* true shortest on the full maze   */
    r->explored    = explored;
    r->cells       = maze_n * maze_n;
    r->recoveries  = recoveries;
    r->worst_us    = (int)sim_dec_us_max;
    r->avg_us      = sim_dec_n ? (int)(sim_dec_us_sum / (uint32_t)sim_dec_n) : 0;
  }
  return reached;
}

void Solver_Task(void)
{
  if (!active_flag) return;

  /* Lifted off the floor mid-run: stop here and keep the map/log for dumping.
     The control loop already killed the motors when it latched the pickup. */
  if (Control_PickupDetected())
  {
    finish(false);                  /* incomplete, but the run log is retained   */
    return;
  }

  trace_sample();                   /* periodic motion/sensor trace (rate-limited) */

  switch (state)
  {
  case S_SETTLE:
    if ((HAL_GetTick() - settle_t0) >= SETTLE_MS)
      state = S_SENSE;
    break;

  case S_SENSE:
  {
    /* Stuck guard: a search that runs many more cells than the maze holds is
       thrashing (drift + phantom walls re-routing it endlessly) - stop cleanly,
       keeping the log, rather than wandering forever. */
    if (++step_count > 6 * maze_n * maze_n + 40) { finish(false); return; }

    /* Phase transition / terminal check. The fast run does the full there-and-back
       too (goal then home), just on the least-cost path at speed - so it ends back
       at the start, ready to be re-armed for another speed run hands-off. */
    if (phase == 0 && isGoal(mouseX, mouseY))
    {
      phase = 1;                       /* reached centre - now flood back home */
      if (!is_fast) goal_evt = true;   /* let the launcher chirp "goal found"  */
    }
    if (phase == 1 && isStart(mouseX, mouseY))
    {
      /* Back home: spin 180 to face into the maze (the fast-run launch heading),
         then stop, so the next run can be triggered without touching the mouse.
         (Front-align here is 'align on' opt-in only - forcing it on fast runs
         drove the mouse into the wall, see the S_SENSE pivot note.)
         If the pivot can't start (battery gate) just finish where we are. */
      pending_deg       = TURN_AROUND_DEG;
      pending_facing    = (mouseFacing + 2) % 4;
      pending_is_finish = true;
      if (align_enabled && Walls_Front() && Control_StartFrontAlign())
      { state = S_ALIGN; break; }
      if (start_pending_turn()) { state = S_FINISH_TURN; break; }
      finish(true); return;
    }

    senseAndRecordWalls();

    /* Snapshot the raw IR + front thresholds at this decision point so the
       console can stream them - this is how we see WHY a front wall was missed
       (front value below threshold) or hallucinated. */
    {
      const int *raw = Sensors_Raw();
      for (int i = 0; i < 6; i++) sense_ir[i] = raw[i];
      int thr[SENSOR_COUNT];
      Sensors_GetCal(NULL, NULL, thr, NULL);
      sense_thr_lf = thr[SENS_L_F];
      sense_thr_rf = thr[SENS_R_F];
    }

    reflood();          /* cell-distance + turn-aware cost for this cell           */

    /* Boxed in (goal unreachable through the SENSED walls)? A single phantom wall
       can falsely do that. Distrust a sensed-only wall blocking the goal and
       re-verify it by driving there, instead of giving up. Only a genuinely
       walled-off map (or hitting the re-verify cap) ends the run. */
    if (flood[mouseX][mouseY] >= INF && !try_recover()) { finish(false); return; }

    /* report this decision point (incl. the walls now known at this cell) */
    step_x = mouseX; step_y = mouseY; step_facing = mouseFacing;
    step_flood = flood[mouseX][mouseY]; step_phase = phase;
    step_walls = walls[mouseX][mouseY];
    step_flag = true;
    log_append();                          /* record this point for later 'dump'  */

    /* Fast run + return phase both descend the least-COST route; only the search
       (phase 0) explores. */
    int dir = (is_fast || phase == 1) ? pickBestDirection() : pickExploreDirection();
    if (dir < 0) { finish(false); return; }   /* boxed in / no open neighbour */

    visited[mouseX][mouseY] = 1;
    pending_dir = dir;   /* passage marked open/wall in S_ADVANCE per the outcome */

    int diff = (dir - mouseFacing + 4) % 4;
    if (diff == 0)
    {
      if (!start_flow()) { finish(false); return; }
      state = S_FLOW;
    }
    else
    {
      if      (diff == 1) { pending_deg = TURN_RIGHT_DEG;  pending_facing = (mouseFacing + 1) % 4; }
      else if (diff == 3) { pending_deg = TURN_LEFT_DEG;   pending_facing = (mouseFacing + 3) % 4; }
      else                { pending_deg = TURN_AROUND_DEG; pending_facing = (mouseFacing + 2) % 4; }

      /* Pivot directly - exactly like the manual 'path'/'turn' that drives
         smoothly. The optional front-wall align (creep + square) is OFF by
         default: it leans on the flaky front-IR calibration and was mis-
         positioning the mouse - RE-CONFIRMED 2026-07-03 (03:11 fast run): forced
         on for fast runs, the align crept the mouse INTO the front wall and its
         skew servo ground one wheel against it until the timeout, then the run
         crashed. Do NOT force it on; 'align on' only for bench experiments. The
         under-travel/skew problem it was meant to fix is handled by the S_TURN
         rotation-settle guard (the real cause was a fouled pivot handing off
         mid-spin) + the odometry creep-to-centre. */
      pending_is_finish = false;
      if (align_enabled && Walls_Front() && Control_StartFrontAlign())
        state = S_ALIGN;
      else if (start_pending_turn())
        state = S_TURN;
      else { finish(false); return; }
    }
    break;
  }

  case S_ALIGN:
    if (Control_IsActive()) break;            /* still squaring to the wall */
    { bool ok; Control_PopAlignDone(&ok); }   /* drain */
    if (!start_pending_turn()) { finish(pending_is_finish); return; }
    state = pending_is_finish ? S_FINISH_TURN : S_TURN;
    break;

  case S_TURN:
    if (Control_IsActive()) break;            /* still pivoting */
    /* Rotation-settle guard: if the pivot handed off still spinning fast (a
       fouled/timed-out turn), WAIT for the chassis to stop before launching the
       advance - its heading frame re-zeroes at start, so launching mid-spin bakes
       the residual rotation in as skew. A clean pivot (snappy <=45 dps) passes
       instantly; cap the wait so a gyro fault can't hang the run. */
    {
      float gz; Control_GetState(NULL, NULL, NULL, &gz, NULL);
      if (gz < 0.0f) gz = -gz;
      if (gz > TURN_EXIT_GYRO_DPS)
      {
        if (turn_exit_t0 == 0) turn_exit_t0 = HAL_GetTick();
        if ((HAL_GetTick() - turn_exit_t0) < TURN_EXIT_SETTLE_MS) break;
      }
    }
    turn_exit_t0 = 0;
    { int32_t deg; Control_PopTurnDone(&deg); }   /* drain, don't depend on it */
    if (!start_flow()) { finish(false); return; }
    state = S_FLOW;
    break;

  case S_FINISH_TURN:
    if (Control_IsActive()) break;            /* still spinning the home 180     */
    { int32_t deg; Control_PopTurnDone(&deg); }   /* drain                       */
    finish(true);                             /* search complete, faced for run  */
    return;

  case S_FLOW:
  {
    /* Flowing forward continuously. Each cell centre reached (a cell mark) = one
       cell stepped: record the passage just driven as open, advance our position,
       sense the new cell on the fly and re-flood. Keep flowing while the route
       simply continues straight; brake to a stop at a DECISION cell - a turn, the
       goal/start, or boxed in - so the pivot that follows happens in that cell.
       Walls map as we pass, so the live console map keeps updating without the
       mouse ever stopping in a straight run. */
    int32_t cidx;
    while (Control_PopCellMark(&cidx))
    {
      markPassage(mouseFacing, true);          /* the passage we just drove is open */
      mouseX += DX[mouseFacing];
      mouseY += DY[mouseFacing];

      senseAndRecordWalls();
      visited[mouseX][mouseY] = 1;
      reflood();        /* cell-distance + turn-aware cost for this flowed cell    */

      /* live report for this cell (same shape as the S_SENSE decision points) */
      step_x = mouseX; step_y = mouseY; step_facing = mouseFacing;
      step_flood = flood[mouseX][mouseY]; step_phase = phase;
      step_walls = walls[mouseX][mouseY]; step_flag = true;
      log_append();                        /* record this flowed cell for 'dump'  */

      bool terminal = (phase == 0 && isGoal(mouseX, mouseY)) ||
                      (phase == 1 && isStart(mouseX, mouseY));
      int nd = terminal ? -1
             : ((is_fast || phase == 1) ? pickBestDirection() : pickExploreDirection());
      if (terminal || nd != mouseFacing)
      {
        Control_AdvanceBrakeNow();             /* decision cell -> stop here        */
        break;                                  /* stop draining; wait for move end  */
      }
      /* Straight through this cell. Fast run on the SMOOTH (Fastest) profile: peek
         one cell ahead and, if it turns, curve through it now without stopping
         (else fall through and keep flowing; turns it can't smooth get braked +
         pivoted at their own cell). */
      if (is_fast && Control_GetTurnMode() == 1 &&
          try_smooth_turn(mouseX + DX[mouseFacing], mouseY + DY[mouseFacing]))
        break;                                  /* flow-turn launched; wait for it   */
      /* else pure straight-through: keep flowing. Re-arm the front-stop for the
         NEXT passage (trust confirmed_open on a fast run, so a phantom can't stall
         us mid-corridor - see set_frontstop_for_ahead). */
      set_frontstop_for_ahead();
    }

    if (Control_IsActive()) break;              /* still flowing or braking          */

    /* Flow ended: braked at a decision cell, a wall ahead stopped us, or we ran
       the full run. If the front-stop caught a wall the on-the-fly sense missed,
       record it; then decide while stopped in S_SENSE. */
    {
      int32_t mm; Control_PopMoveDone(&mm); (void)mm;
      /* The front-stop now brakes on the sensitive UNFILTERED read (Walls_FrontRaw)
         so it can fire early and not bump - but that read is too noisy to trust as
         a wall. Re-confirm with the clean latched Walls_FrontConfident() now that
         we're stopped before committing the wall, so a false early brake costs a
         brief stop + re-sense, never a phantom wall in the map. */
      /* Record the wall on a wall-hit. Accept a SKEWED hit too: Walls_FrontConfident
         (BOTH diagonals) reads false when the mouse hits at an ANGLE (one diagonal
         high, one low), but the front-stop already fired on Confident OR Contact - so
         honour Walls_FrontContact() here as well (a contact-grade read, ~2x the ref,
         is a DEFINITE wall, not a phantom). Without this the skewed wall goes
         unrecorded and the solver re-picks forward and re-rams it. */
      if (Control_LastMoveHitWall() &&
          (Walls_FrontConfident() || Walls_FrontContact()) &&
          !confirmed_open[mouseX][mouseY][mouseFacing])
        markPassage(mouseFacing, false);

      /* Drove INTO a missed wall and stalled (not a clean front-stop): the mouse
         is jammed forward against it, so a pivot from here would clip. Back off a
         little to clear the wall before deciding. Only on a genuine stall. But if
         it stalls repeatedly without progress, the position has drifted off the
         map (records don't match the real walls) - give up cleanly. */
      if (Control_LastMoveStalled())
      {
        /* THE STALL *IS* THE WALL: a fast-run stall means the nose jammed on a wall
           the skewed front diagonals couldn't read confidently. Record it so we
           REROUTE, instead of re-picking forward and re-ramming - the slam -> back-up
           -> slam loop. Guarded by confirmed_open so a search-proven-open passage is
           never severed. This is the core fix for the fast-run wall-slam loop. */
        if (is_fast && !confirmed_open[mouseX][mouseY][mouseFacing])
          markPassage(mouseFacing, false);
        if (++stall_run >= 4) { finish(false); return; }     /* lost: stop */
        if (Control_StartRecenter(STALL_BACKOFF_MM)) { state = S_RECENTER; break; }
      }
      else stall_run = 0;                                     /* clean move = progress */
    }
    settle_t0 = HAL_GetTick();
    state = S_SETTLE;                           /* re-sense + decide while stopped   */
    break;
  }

  case S_RECENTER:
    if (Control_IsActive()) break;            /* still reversing to centre */
    Control_PopRecenterDone();
    settle_t0 = HAL_GetTick();
    state = S_SETTLE;
    break;

  /* --- front-wall distance calibration -------------------------------------
     From cell centre facing a wall: drive to the front-stop (S_CAL_DRIVE), the
     travelled distance IS centre->stop = front_wall_mm; then reverse it to land
     back at centre (S_CAL_BACK). */
  case S_CAL_DRIVE:
    if (Control_IsActive()) break;
    {
      int32_t mm; Control_PopContactDone(&mm);
      if (mm < 20 || mm >= 125)
      {
        /* no wall reached within range - reject, keep the old value */
        cal_ok = false; cal_value = (int)mm;
        active_flag = false; cal_mode = false; state = S_IDLE;
        Control_Stop();
        cal_done_flag = true;
        return;
      }
      front_wall_mm = (int)mm;
      cal_value = (int)mm;
      if (!Control_StartRecenter(front_wall_mm))   /* drive back to where we started */
      {
        cal_ok = true; active_flag = false; cal_mode = false; state = S_IDLE;
        cal_done_flag = true; return;
      }
      state = S_CAL_BACK;
    }
    break;

  case S_CAL_BACK:
    if (Control_IsActive()) break;
    Control_PopRecenterDone();
    cal_ok = true;
    active_flag = false; cal_mode = false; state = S_IDLE;
    cal_done_flag = true;
    break;

  case S_IDLE:
  default:
    break;
  }
}

bool Solver_PopStep(int *x, int *y, int *facing, int *fld, int *ph, int *wallbits)
{
  if (!step_flag) return false;
  step_flag = false;
  if (x)        *x        = step_x;
  if (y)        *y        = step_y;
  if (facing)   *facing   = step_facing;
  if (fld)      *fld      = step_flood;
  if (ph)       *ph       = step_phase;
  if (wallbits) *wallbits = step_walls;
  return true;
}

void Solver_GetSenseIR(int ir[6], int *thr_lf, int *thr_rf)
{
  for (int i = 0; i < 6; i++) ir[i] = sense_ir[i];
  if (thr_lf) *thr_lf = sense_thr_lf;
  if (thr_rf) *thr_rf = sense_thr_rf;
}

/* --- run-log read-back (the 'dump' command replays these) ------------------ */
int Solver_LogCount(void) { return runlog_n; }

bool Solver_GetLogEntry(int i, int *x, int *y, int *facing, int *fld, int *ph,
                        int *wallbits, int ir[6], int *thr_lf, int *thr_rf)
{
  if (i < 0 || i >= runlog_n) return false;
  RunLogEntry *e = &runlog[i];
  if (x)        *x        = e->x;
  if (y)        *y        = e->y;
  if (facing)   *facing   = e->facing;
  if (fld)      *fld      = e->flood;
  if (ph)       *ph       = e->phase;
  if (wallbits) *wallbits = e->wallbits;
  if (ir) for (int k = 0; k < 6; k++) ir[k] = e->ir[k];
  if (thr_lf)   *thr_lf   = e->thr_lf;
  if (thr_rf)   *thr_rf   = e->thr_rf;
  return true;
}

int Solver_TraceCount(void) { return trace_n; }

bool Solver_GetTraceEntry(int i, uint32_t *t_ms, int32_t *posL, int32_t *posR,
                          int *head_ddeg, int *gyroz_ddps, int *accelz_mg,
                          int s[6], int *x, int *y, int *fld, int *vbat_mv)
{
  if (i < 0 || i >= trace_n) return false;
  TraceEntry *e = &trace[i];
  if (t_ms)       *t_ms       = e->t_ms;
  if (posL)       *posL       = e->posL;
  if (posR)       *posR       = e->posR;
  if (head_ddeg)  *head_ddeg  = e->head_ddeg;
  if (gyroz_ddps) *gyroz_ddps = e->gyroz_ddps;
  if (accelz_mg)  *accelz_mg  = e->accelz_mg;
  if (s) for (int k = 0; k < 6; k++) s[k] = e->s[k];
  if (x)          *x          = e->x;
  if (y)          *y          = e->y;
  if (fld)        *fld        = e->flood;
  if (vbat_mv)    *vbat_mv    = e->vbat_mv;
  return true;
}

bool Solver_PopDone(bool *ok)
{
  if (!done_flag) return false;
  done_flag = false;
  if (ok) *ok = done_ok;
  return true;
}
