"""
mms_maze.py - load Micromouse-Maze-Simulator (MMS) maze files.

MMS `.maz` / `.num` files list one cell per line:

    x y N E S W

where x is the column (East), y the row (North, 0 = bottom), and N/E/S/W are
0/1 flags for a wall on that side. This matches the firmware's convention
exactly (y up, bits N=1 E=2 S=4 W=8), so a file maps 1:1 onto the solver's
wall grid - which is why the same maze can be run in MMS on the PC and in the
on-MCU `sim`, and the two compared.

`load_mms` returns (n, grid) with grid[x][y] = wall bitmask (N1 E2 S4 W8).
"""

from __future__ import annotations

from typing import Dict, List, Tuple


def load_mms(path: str) -> Tuple[int, List[List[int]]]:
    cells: Dict[Tuple[int, int], int] = {}
    max_x = max_y = 0

    with open(path, "r") as f:
        for raw in f:
            parts = raw.split()
            if len(parts) < 6:
                continue
            try:
                x, y, n, e, s, w = (int(v) for v in parts[:6])
            except ValueError:
                continue
            cells[(x, y)] = (n & 1) | ((e & 1) << 1) | ((s & 1) << 2) | ((w & 1) << 3)
            max_x = max(max_x, x)
            max_y = max(max_y, y)

    if not cells:
        raise ValueError("no maze cells found (expected lines: x y N E S W)")

    n = max(max_x, max_y) + 1
    grid = [[0] * n for _ in range(n)]
    for (x, y), mask in cells.items():
        if 0 <= x < n and 0 <= y < n:
            grid[x][y] = mask
    return n, grid
