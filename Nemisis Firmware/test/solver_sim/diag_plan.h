#ifndef DIAG_PLAN_H
#define DIAG_PLAN_H

/* Plan the fastest route start(0,0)->goal over a fully-known wall grid and return
   its TIME cost (x100, reference cost model). walls[x][y] bits: N1 E2 S4 W8.
   start_facing 0..3 = N,E,S,W. allow_diag 0 = orthogonal-only, 1 = 8-way diagonal.
   Returns -1 if no route. out_cells (may be NULL) is reserved. */
long diag_plan(const int walls[16][16], int n, const int *gx, const int *gy,
               int gcount, int start_facing, int allow_diag, int *out_cells);

#endif
