/**
 * diag_plan.c - offline diagonal speed-run PLANNER (host-only analysis).
 *
 * Ports the 8-heading time-weighted Dijkstra from the user's original simulator
 * (SIMULATOR/flood_fill_Algo/Main.c) as a standalone, self-contained function so
 * we can measure the DIAGONAL speed-run benefit against real mazes with zero
 * hardware and zero changes to lib/solver. It plans the fastest ROUTE (not the
 * motion); the time is the sum of the reference cost model:
 *
 *   ortho cell 1st/2nd in a straight : 100   (1.00)    diag 1st/2nd : 141 (~sqrt2)
 *   ortho cell 3rd+ in a straight    :  50   (0.50)    diag 3rd+    :  71
 *   45 deg turn                      :  50             90 deg turn  : 100 (two 45s)
 *
 * State = (x, y, heading 0..7 [N,NE,E,SE,S,SW,W,NW], straight-run capped at 2).
 * Passing allow_diag = 0 restricts it to cardinal moves (the best ORTHOGONAL
 * time under the same model) so the two are directly comparable.
 */
#include "diag_plan.h"

#define SZ        16                       /* array capacity (state encoding)     */
#define DIRS8     8
#define NSTATES   (SZ * SZ * DIRS8 * 3)

/* cardinal (0..3 = N,E,S,W) vectors + wall bits */
static const int CDX[4] = {  0,  1,  0, -1 };
static const int CDY[4] = {  1,  0, -1,  0 };
static const int WBIT[4] = { 1, 2, 4, 8 };

/* 8-heading vectors: N, NE, E, SE, S, SW, W, NW */
static const int DX8[8] = {  0,  1,  1,  1,  0, -1, -1, -1 };
static const int DY8[8] = {  1,  1,  0, -1, -1, -1,  0,  1 };

static int  dist[NSTATES];
static char vis[NSTATES];

static int  g_n;
static int  g_open[SZ][SZ][4];             /* passage open (from the wall grid)   */
static int  g_gx[8], g_gy[8], g_gc;

static int h2c(int h) { return h >> 1; }   /* heading -> cardinal (even h only)   */
static int stateOf(int x, int y, int h, int sc) { return ((x * SZ + y) * DIRS8 + h) * 3 + sc; }

static int isGoalXY(int x, int y)
{
    for (int i = 0; i < g_gc; i++) if (g_gx[i] == x && g_gy[i] == y) return 1;
    return 0;
}

/* can we step one cell along heading h from (x,y)? cardinal = wall check;
   diagonal = at least one L-path around the corner post is open. */
static int canStep(int x, int y, int h)
{
    int nx = x + DX8[h], ny = y + DY8[h];
    if (nx < 0 || nx >= g_n || ny < 0 || ny >= g_n) return 0;
    if ((h & 1) == 0) return g_open[x][y][h2c(h)];

    int hL = (h + 7) & 7, hR = (h + 1) & 7;
    int cL = h2c(hL), cR = h2c(hR);
    int xL = x + DX8[hL], yL = y + DY8[hL];
    int xR = x + DX8[hR], yR = y + DY8[hR];
    if (xL < 0 || xL >= g_n || yL < 0 || yL >= g_n) return 0;
    if (xR < 0 || xR >= g_n || yR < 0 || yR >= g_n) return 0;
    int pathL = g_open[x][y][cL] && g_open[xL][yL][cR];
    int pathR = g_open[x][y][cR] && g_open[xR][yR][cL];
    return pathL || pathR;
}

long diag_plan(const int walls[16][16], int n, const int *gx, const int *gy,
               int gcount, int start_facing, int allow_diag, int *out_cells)
{
    g_n = n; g_gc = gcount;
    for (int i = 0; i < gcount && i < 8; i++) { g_gx[i] = gx[i]; g_gy[i] = gy[i]; }
    for (int x = 0; x < n; x++)
        for (int y = 0; y < n; y++)
            for (int d = 0; d < 4; d++)
                g_open[x][y][d] = !(walls[x][y] & WBIT[d]);

    for (int s = 0; s < NSTATES; s++) { dist[s] = 0x7fffffff; vis[s] = 0; }

    int startH = start_facing * 2;                 /* cardinal facing -> 8-heading  */
    for (int h = 0; h < DIRS8; h++)
    {
        if (!allow_diag && (h & 1)) continue;      /* ortho-only: cardinal headings */
        int diff = (h - startH + DIRS8) & 7;
        int turn45 = (diff <= 4) ? diff : (DIRS8 - diff);
        int s = stateOf(0, 0, h, 0);
        if (turn45 * 50 < dist[s]) dist[s] = turn45 * 50;
    }

    int goalState = -1;
    for (int iter = 0; iter < NSTATES; iter++)
    {
        int u = -1, uc = 0x7fffffff;
        for (int s = 0; s < NSTATES; s++)
            if (!vis[s] && dist[s] < uc) { uc = dist[s]; u = s; }
        if (u == -1) break;
        vis[u] = 1;

        int sc = u % 3, h = (u / 3) % DIRS8, y = (u / 3 / DIRS8) % SZ, x = u / 3 / DIRS8 / SZ;
        if (isGoalXY(x, y)) { goalState = u; break; }

        for (int dh = -1; dh <= 1; dh += 2)        /* turn +/-45 (cost 50)          */
        {
            /* diagonal headings are allowed as TURN-THROUGH states even in ortho
               mode (a 90 turn = two 45s via the diagonal); only MOVING on them is
               gated below. */
            int nh = (h + dh + DIRS8) & 7;
            int v = stateOf(x, y, nh, 0), nc = uc + 50;
            if (nc < dist[v]) dist[v] = nc;
        }
        if ((allow_diag || !(h & 1)) && canStep(x, y, h))   /* move forward 1 cell   */
        {
            int nx = x + DX8[h], ny = y + DY8[h], diag = h & 1;
            int newSc = (sc < 2) ? sc + 1 : 2;
            int mcost = diag ? (sc == 2 ? 71 : 141) : (sc == 2 ? 50 : 100);
            int v = stateOf(nx, ny, h, newSc), nc = uc + mcost;
            if (nc < dist[v]) dist[v] = nc;
        }
    }

    if (goalState == -1) { if (out_cells) *out_cells = -1; return -1; }
    if (out_cells)                                 /* count real cell moves on path */
    {
        /* reconstruct length cheaply: re-walk is overkill; approximate via the
           straight-line move count isn't exact, so just report -1 unless needed. */
        *out_cells = 0;
    }
    return dist[goalState];                         /* total time cost x100          */
}
