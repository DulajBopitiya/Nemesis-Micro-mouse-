/**
 * run_sim.c - offline solver regression runner.
 *
 * Compiles lib/solver natively and runs the REAL flood-fill search (the exact
 * code the robot runs) against real MMS competition mazes, with no hardware and
 * no physical maze. For each maze it reports whether the search reached the goal
 * and came home, and - the real question - whether the path it LEARNED equals
 * the true optimal (learned == optimal => it found the quickest route).
 *
 * MMS .maz/.num format: one line per cell "x y N E S W" (y up, 0 = bottom),
 * mapping 1:1 onto the solver's wall bits (N1 E2 S4 W8) - same as mms_maze.py.
 *
 * Usage:  run_sim [--mode 0|1] [--faults miss false desync] <maze> [maze...]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "solver.h"

#define MAXN 16

/* Load one MMS maze file into the sim's ground-truth grid. Returns edge length,
   or -1 on error. */
static int load_maze(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "  cannot open %s\n", path); return -1; }

    static int cx[MAXN * MAXN], cy[MAXN * MAXN], cbits[MAXN * MAXN];
    int cnt = 0, maxx = 0, maxy = 0;
    char line[256];
    while (fgets(line, sizeof line, f))
    {
        int x, y, n, e, s, w;
        if (sscanf(line, "%d %d %d %d %d %d", &x, &y, &n, &e, &s, &w) != 6) continue;
        if (x < 0 || x >= MAXN || y < 0 || y >= MAXN) continue;
        if (cnt < MAXN * MAXN)
        {
            cx[cnt] = x; cy[cnt] = y;
            cbits[cnt] = (n & 1) | ((e & 1) << 1) | ((s & 1) << 2) | ((w & 1) << 3);
            cnt++;
        }
        if (x > maxx) maxx = x;
        if (y > maxy) maxy = y;
    }
    fclose(f);
    if (cnt == 0) { fprintf(stderr, "  no cells in %s\n", path); return -1; }

    int size = (maxx > maxy ? maxx : maxy) + 1;
    Solver_SetSize(size);
    Solver_SimClear();
    for (int i = 0; i < cnt; i++) Solver_SimSetCell(cx[i], cy[i], cbits[i]);
    Solver_SetGoalCenter();
    return size;
}

static const char *basename2(const char *p)
{
    const char *b = p;
    for (const char *q = p; *q; q++) if (*q == '/' || *q == '\\') b = q + 1;
    return b;
}

int main(int argc, char **argv)
{
    int mode = 0, miss = 0, fpct = 0, desync = 0, first = 1;

    /* crude flag parse */
    while (first < argc && argv[first][0] == '-')
    {
        if (!strcmp(argv[first], "--mode") && first + 1 < argc)
        { mode = atoi(argv[first + 1]); first += 2; }
        else if (!strcmp(argv[first], "--faults") && first + 3 < argc)
        { miss = atoi(argv[first+1]); fpct = atoi(argv[first+2]); desync = atoi(argv[first+3]); first += 4; }
        else break;
    }
    if (first >= argc)
    {
        printf("usage: run_sim [--mode 0|1] [--faults miss false desync] <maze> [maze...]\n");
        return 2;
    }

    int clean = (miss == 0 && fpct == 0 && desync == 0);

    printf("mode=%d (0=explore,1=shortest)  faults miss/false/desync=%d/%d/%d\n\n",
           mode, miss, fpct, desync);
    printf("%-18s %3s %6s %8s %8s %8s %7s  %s\n",
           "maze", "n", "reach", "learned", "optimal", "journey", "explor", "result");
    printf("--------------------------------------------------------------------------------\n");

    /* PASS = the correctness invariant we must never regress: the search reaches
       the goal AND floods home. Learning the TRUE-optimal route is a separate
       efficiency metric (found-optimal), tracked but NOT a hard failure - the
       explorer legitimately doesn't fully explore every maze.
       Invariant with 0 faults: learned <= optimal (the discovered map is a subset
       of the true walls, so its shortest path can only be optimistic). A
       learned > optimal with no faults would mean phantom walls = a real BUG. */
    int fails = 0, total = 0, reached_n = 0, opt_n = 0, invariant_bug = 0;
    for (int a = first; a < argc; a++)
    {
        total++;
        int n = load_maze(argv[a]);
        if (n < 0) { fails++; continue; }

        Solver_SetSearchMode(mode);
        Solver_SimSetFaults(miss, fpct, desync);

        SimResult r;
        Solver_RunSim(&r);

        int found_opt = (r.learned > 0 && r.learned == r.optimal);
        int bad_invariant = (clean && r.learned > 0 && r.optimal > 0 && r.learned > r.optimal);
        int pass = r.reached && !bad_invariant;

        if (r.reached) reached_n++;
        if (found_opt) opt_n++;
        if (bad_invariant) invariant_bug++;

        printf("%-18s %3d %6s %8d %8d %8d %6d%%  %s%s\n",
               basename2(argv[a]), n, r.reached ? "yes" : "NO",
               r.learned, r.optimal, r.path_len,
               r.cells ? r.explored * 100 / r.cells : 0,
               pass ? (found_opt ? "PASS+opt" : "PASS") : "FAIL",
               bad_invariant ? " (learned>optimal BUG!)" : "");
        if (!pass) fails++;
    }

    printf("--------------------------------------------------------------------------------\n");
    printf("reached: %d/%d   found-optimal: %d/%d   %s\n",
           reached_n, total, opt_n, total,
           clean ? "(0 faults)" : "(faults injected - reach is the bar)");
    if (invariant_bug)
        printf("*** %d maze(s) violated learned<=optimal with 0 faults = phantom-wall BUG ***\n",
               invariant_bug);
    printf("%d/%d passed\n", total - fails, total);
    return fails ? 1 : 0;
}
