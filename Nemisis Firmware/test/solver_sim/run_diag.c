/**
 * run_diag.c - measure the DIAGONAL speed-run benefit offline.
 *
 * For each MMS maze it plans the fastest route start->centre both ORTHOGONAL-only
 * and with DIAGONALS (same time-cost model), and prints the time saved. This
 * quantifies "how much do diagonals actually buy us" against real competition
 * mazes, with no hardware. Route planning only - no motion.
 *
 * Usage:  run_diag <maze.maz> [maze...]
 */
#include <stdio.h>
#include <string.h>
#include "diag_plan.h"

#define MAXN 16

static int load(const char *path, int walls[16][16], int *gx, int *gy, int *gc)
{
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "  cannot open %s\n", path); return -1; }
    for (int x = 0; x < 16; x++) for (int y = 0; y < 16; y++) walls[x][y] = 0;
    int maxx = 0, maxy = 0;
    char line[256];
    while (fgets(line, sizeof line, f))
    {
        int x, y, n, e, s, w;
        if (sscanf(line, "%d %d %d %d %d %d", &x, &y, &n, &e, &s, &w) != 6) continue;
        if (x < 0 || x >= 16 || y < 0 || y >= 16) continue;
        walls[x][y] = (n & 1) | ((e & 1) << 1) | ((s & 1) << 2) | ((w & 1) << 3);
        if (x > maxx) maxx = x;
        if (y > maxy) maxy = y;
    }
    fclose(f);
    int nn = (maxx > maxy ? maxx : maxy) + 1;
    int c = (nn / 2);                       /* centre 2x2 goal */
    gx[0] = c - 1; gy[0] = c - 1; gx[1] = c - 1; gy[1] = c;
    gx[2] = c;     gy[2] = c - 1; gx[3] = c;     gy[3] = c;
    *gc = 4;
    return nn;
}

static const char *base(const char *p)
{
    const char *b = p;
    for (const char *q = p; *q; q++) if (*q == '/' || *q == '\\') b = q + 1;
    return b;
}

int main(int argc, char **argv)
{
    if (argc < 2) { printf("usage: run_diag <maze.maz> [maze...]\n"); return 2; }

    printf("%-18s %3s  %10s  %10s  %8s\n",
           "maze", "n", "ortho(t)", "diag(t)", "faster");
    printf("------------------------------------------------------------\n");
    double sum_ortho = 0, sum_diag = 0;
    int count = 0;
    for (int a = 1; a < argc; a++)
    {
        int walls[16][16], gx[8], gy[8], gc;
        int n = load(argv[a], walls, gx, gy, &gc);
        if (n < 0) continue;

        long ortho = diag_plan(walls, n, gx, gy, gc, 0, 0, NULL);
        long diag  = diag_plan(walls, n, gx, gy, gc, 0, 1, NULL);
        if (ortho <= 0 || diag <= 0) { printf("%-18s  no route\n", base(argv[a])); continue; }

        double pct = 100.0 * (double)(ortho - diag) / (double)ortho;
        printf("%-18s %3d  %10.2f  %10.2f  %6.1f%%\n",
               base(argv[a]), n, ortho / 100.0, diag / 100.0, pct);
        sum_ortho += ortho; sum_diag += diag; count++;
    }
    printf("------------------------------------------------------------\n");
    if (count)
        printf("mean speed-run time: ortho %.1f, diagonal %.1f -> diagonals are "
               "%.1f%% faster on average\n",
               sum_ortho / count / 100.0, sum_diag / count / 100.0,
               100.0 * (sum_ortho - sum_diag) / sum_ortho);
    return 0;
}
