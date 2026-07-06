"""
planner.py - app-side mirror of the firmware's turn-weighted flood planner.

Lets the app PREVIEW how the robot's flood will route a maze for a given turn
penalty (`tcost`) WITHOUT the robot - it runs the same heading-state Dijkstra as
lib/solver/solver.c (costFill + pickBestDirection), so the path you see here is
the path the firmware would pick from the same map.

Why this exists: the plain flood counts CELLS only, so a straight and a zig-zag of
equal length tie even though the zig-zag is far slower (every turn = brake, pivot,
re-accelerate). Charging `turn_cost` per 90 pivot on top of one unit per cell makes
the planner prefer longer-but-straighter routes that bunch their turns - the shape
a later diagonal speed-run collapses into smooth 45s. Drop `tcost` to 0 and you get
the old cell-count-only behaviour back, which is exactly the comparison the maze
view draws so you can SEE what the penalty buys you.

Pure state; no Qt, so it can be unit-tested headless. Wall grids are the same
[x][y] list-of-bitmask layout MazeModel uses (bits N=1,E=2,S=4,W=8).
"""

from __future__ import annotations

from collections import deque
from typing import List, Optional, Sequence, Tuple

DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
BIT = [1, 2, 4, 8]
OPP = [2, 3, 0, 1]          # opposite direction index
INF = 10 ** 9

Cell = Tuple[int, int]


def turns_between(a: int, b: int) -> int:
    """Number of 90-degree pivots to rotate from heading a to heading b (0/1/2)."""
    d = (b - a) % 4
    return 0 if d == 0 else (2 if d == 2 else 1)


def cost_to_go(grid: Sequence[Sequence[int]], n: int,
               goals: Sequence[Cell], turn_cost: int) -> List[List[List[int]]]:
    """Heading-aware cost-to-go costg[x][y][h]: the movement cost to reach the
    nearest goal cell FROM cell (x,y) facing heading h, charging one unit per cell
    driven and `turn_cost` per 90 pivot. Mirrors solver.c costFill - an SPFA on the
    reversed (cell, heading) graph where the predecessors of (x,y,h) are the cell
    behind it (same heading, if that edge is open) and the two headings a pivot
    away. Unreachable states stay INF."""
    costg = [[[INF] * 4 for _ in range(n)] for _ in range(n)]
    inq = [[[False] * 4 for _ in range(n)] for _ in range(n)]
    q: deque = deque()

    for (gx, gy) in goals:
        if 0 <= gx < n and 0 <= gy < n:
            for h in range(4):
                if costg[gx][gy][h] != 0:
                    costg[gx][gy][h] = 0
                    inq[gx][gy][h] = True
                    q.append((gx, gy, h))

    while q:
        x, y, h = q.popleft()
        inq[x][y][h] = False
        cs = costg[x][y][h]

        # predecessor that drives forward INTO (x,y) facing h = the cell behind us,
        # valid iff the edge between them is open (no wall on our 'back' side).
        if not (grid[x][y] & BIT[OPP[h]]):
            px, py = x - DX[h], y - DY[h]
            if 0 <= px < n and 0 <= py < n:
                nc = cs + 1
                if nc < costg[px][py][h]:
                    costg[px][py][h] = nc
                    if not inq[px][py][h]:
                        inq[px][py][h] = True
                        q.append((px, py, h))

        # predecessors that pivot INTO heading h (from h+/-1), same cell.
        for ph in ((h + 1) % 4, (h + 3) % 4):
            nc = cs + turn_cost
            if nc < costg[x][y][ph]:
                costg[x][y][ph] = nc
                if not inq[x][y][ph]:
                    inq[x][y][ph] = True
                    q.append((x, y, ph))

    return costg


class Plan:
    """Result of planning a route: the cells walked, how many pivots it takes, and
    the total movement cost. `reached` is False if the goal is walled off."""

    def __init__(self, cells: List[Cell], turns: int, cost: int, reached: bool):
        self.cells = cells
        self.turns = turns
        self.cost = cost
        self.reached = reached

    @property
    def length(self) -> int:
        """Cells driven (path segments) = cell count - 1."""
        return max(0, len(self.cells) - 1)

    def __repr__(self) -> str:
        tag = "" if self.reached else " UNREACHABLE"
        return (f"Plan({self.length} cells, {self.turns} turns, "
                f"cost {self.cost}{tag})")


def plan_path(grid: Sequence[Sequence[int]], n: int, start: Cell,
              start_facing: int, goals: Sequence[Cell], turn_cost: int) -> Plan:
    """Greedy least-cost descent from start -> nearest goal, picking each move the
    same way solver.c pickBestDirection does (turn-aware cost, ties prefer going
    straight). Because costg is a consistent optimal cost-to-go, following its
    argmin can't loop - every step strictly lowers the remaining cost - so this is
    the exact route the firmware would drive. Returns a Plan."""
    sx, sy = start
    costg = cost_to_go(grid, n, goals, turn_cost)
    goalset = {(gx, gy) for (gx, gy) in goals}

    if not (0 <= sx < n and 0 <= sy < n) or costg[sx][sy][start_facing] >= INF:
        return Plan([start], 0, INF, False)          # boxed in / off the map

    total = costg[sx][sy][start_facing]
    cells: List[Cell] = [(sx, sy)]
    x, y, facing, turns = sx, sy, start_facing, 0
    cap = n * n * 4                                   # safety net; policy can't loop

    while (x, y) not in goalset and cap > 0:
        cap -= 1
        best_d, best_c = -1, INF
        for d in range(4):
            if grid[x][y] & BIT[d]:
                continue
            nx, ny = x + DX[d], y + DY[d]
            if not (0 <= nx < n and 0 <= ny < n):
                continue
            if costg[nx][ny][d] >= INF:
                continue
            c = turns_between(facing, d) * turn_cost + 1 + costg[nx][ny][d]
            if c < best_c or (c == best_c and d == facing):
                best_c, best_d = c, d
        if best_d < 0:
            break
        turns += turns_between(facing, best_d)
        facing = best_d
        x, y = x + DX[best_d], y + DY[best_d]
        cells.append((x, y))

    reached = (x, y) in goalset
    return Plan(cells, turns, total if reached else INF, reached)
