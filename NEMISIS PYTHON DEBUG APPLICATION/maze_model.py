"""
maze_model.py - the app-side maze state for the Nemisis micromouse.

Holds everything needed to draw the live map and to diagnose wall sensing:

  * size + goal/start (kept in sync with the robot via MAZE,/GOAL, lines)
  * DISCOVERED walls - what the robot reports as it solves (SOLVE,... lines)
  * ACTUAL walls     - what you paint by hand (the real maze), used to spot
                       sensing mistakes: discovered-only = phantom wall,
                       actual-only = a wall the mouse missed.
  * robot pose, the path it has driven, and the per-cell flood values.

Pure state + parsing; no Qt here so it can be unit-tested / reused headless.
Directions are N,E,S,W = 0,1,2,3 with bits N=1,E=2,S=4,W=8 (matches firmware).
"""

from __future__ import annotations

from typing import List, Tuple

import protocol

DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
BIT = [1, 2, 4, 8]
OPP = [2, 3, 0, 1]          # opposite direction index
DIR_NAMES = ["N", "E", "S", "W"]

MAX_SIZE = 16


class MazeModel:
    def __init__(self, size: int = 6):
        self.set_size(size)

    # -- configuration -----------------------------------------------------
    def set_size(self, n: int) -> None:
        n = max(2, min(MAX_SIZE, int(n)))
        self.n = n
        # flood turn penalty used by the app-side planner preview (mirrors the
        # robot's `tcost`); a config, so keep it across a resize/reset.
        self.turn_cost = getattr(self, "turn_cost", 2)
        self.disc = [[0] * n for _ in range(n)]     # discovered wall bitmasks
        self.act = [[0] * n for _ in range(n)]      # hand-painted "actual" walls
        self.visited = [[False] * n for _ in range(n)]
        self.flood = [[None] * n for _ in range(n)]
        self.start = (0, 0)
        self.start_facing = 0                        # heading you place it in (N)
        self.goal: List[Tuple[int, int]] = self._center_goal(n)
        self.pose = (0, 0, 0)                        # x, y, facing
        self.path: List[Tuple[int, int]] = []
        self.phase = 0
        self.running = False
        self.dirty = True
        self._add_border(self.act)                  # border walls are ground truth

    @staticmethod
    def _center_goal(n: int) -> List[Tuple[int, int]]:
        lo, hi = n // 2 - 1, n // 2
        lo = max(0, lo)
        hi = min(n - 1, hi)
        return [(x, y) for x in (lo, hi) for y in (lo, hi)]

    def set_goal(self, cells: List[Tuple[int, int]]) -> bool:
        """Set the goal to the given in-range cells. Returns False (and leaves the
        goal unchanged) if none are valid, so a typo can't blank the goal."""
        valid = [(x, y) for (x, y) in cells
                 if 0 <= x < self.n and 0 <= y < self.n]
        if not valid:
            return False
        self.goal = valid
        self.dirty = True
        return True

    # corner -> (x, y, facing) for the current size. Bottom corners face N (up),
    # top corners face S (down) - the natural "nose away from your back wall".
    def corner_cell(self, corner: str) -> Tuple[int, int, int]:
        n = self.n
        return {
            "sw": (0, 0, 0),
            "se": (n - 1, 0, 0),
            "nw": (0, n - 1, 2),
            "ne": (n - 1, n - 1, 2),
        }[corner]

    def corner_name(self) -> str:
        """Which corner the current start cell sits in ('sw'/'se'/'nw'/'ne'),
        or '' if it isn't a corner."""
        n = self.n
        return {
            (0, 0): "sw", (n - 1, 0): "se",
            (0, n - 1): "nw", (n - 1, n - 1): "ne",
        }.get(self.start, "")

    def set_start(self, x: int, y: int, facing: int = 0) -> bool:
        """Set the start cell + heading (the corner you place the mouse in).
        Returns False (unchanged) if the cell is out of range."""
        if not (0 <= x < self.n and 0 <= y < self.n):
            return False
        self.start = (x, y)
        self.start_facing = facing % 4
        self.pose = (x, y, self.start_facing)        # show it parked at the start
        self.dirty = True
        return True

    def _add_border(self, grid) -> None:
        n = self.n
        for i in range(n):
            grid[i][0] |= BIT[2]            # south edge of bottom row
            grid[i][n - 1] |= BIT[0]        # north edge of top row
            grid[0][i] |= BIT[3]            # west edge of left column
            grid[n - 1][i] |= BIT[1]        # east edge of right column

    # -- wall helpers ------------------------------------------------------
    def _set_wall(self, grid, x: int, y: int, d: int, present: bool) -> None:
        if not (0 <= x < self.n and 0 <= y < self.n):
            return
        if present:
            grid[x][y] |= BIT[d]
        else:
            grid[x][y] &= ~BIT[d]
        nx, ny = x + DX[d], y + DY[d]
        if 0 <= nx < self.n and 0 <= ny < self.n:     # mirror to the neighbour
            if present:
                grid[nx][ny] |= BIT[OPP[d]]
            else:
                grid[nx][ny] &= ~BIT[OPP[d]]

    def has_wall(self, grid, x: int, y: int, d: int) -> bool:
        return bool(grid[x][y] & BIT[d])

    def toggle_actual(self, x: int, y: int, d: int) -> None:
        """Hand-paint: flip a wall in the ACTUAL (ground-truth) layer."""
        self._set_wall(self.act, x, y, d, not self.has_wall(self.act, x, y, d))
        self.dirty = True

    def wall_state(self, x: int, y: int, d: int) -> str:
        """How this edge looks for drawing: 'both', 'disc', 'act', or 'none'.
        'disc' = robot saw a wall you didn't paint (possible phantom);
        'act'  = a real wall the robot hasn't found (missed / not yet there)."""
        dd = self.has_wall(self.disc, x, y, d)
        aa = self.has_wall(self.act, x, y, d)
        if dd and aa:
            return "both"
        if dd:
            return "disc"
        if aa:
            return "act"
        return "none"

    def clear_discovered(self) -> None:
        self.disc = [[0] * self.n for _ in range(self.n)]
        self.visited = [[False] * self.n for _ in range(self.n)]
        self.flood = [[None] * self.n for _ in range(self.n)]
        self.path = []
        self.dirty = True

    def clear_actual(self) -> None:
        self.act = [[0] * self.n for _ in range(self.n)]
        self._add_border(self.act)
        self.dirty = True

    def load_actual(self, n: int, grid) -> None:
        """Replace the ACTUAL (ground-truth) maze with an n x n wall-bitmask grid
        (e.g. parsed from an MMS file). Resizes, clears the discovered overlay so
        a following sim/dump draws cleanly on top, and re-asserts the borders."""
        self.set_size(n)
        for x in range(n):
            for y in range(n):
                self.act[x][y] = grid[x][y] & 0xF
        self._add_border(self.act)          # belt-and-braces: outer walls present
        self.clear_discovered()
        self.dirty = True

    # -- mismatch report (for diagnostics) ---------------------------------
    def mismatches(self) -> List[dict]:
        """Edges where discovered and actual disagree. Only meaningful where
        you've painted the actual maze. Each: {x,y,dir,kind}."""
        out = []
        for x in range(self.n):
            for y in range(self.n):
                for d in range(4):
                    st = self.wall_state(x, y, d)
                    if st == "disc":
                        out.append({"x": x, "y": y, "dir": DIR_NAMES[d],
                                    "kind": "phantom (robot saw, none painted)"})
                    elif st == "act":
                        # only flag as 'missed' if the robot has visited this cell
                        if self.visited[x][y]:
                            out.append({"x": x, "y": y, "dir": DIR_NAMES[d],
                                        "kind": "missed (painted, robot didn't see)"})
        return out

    # -- line ingestion ----------------------------------------------------
    def parse_line(self, line: str) -> bool:
        """Feed a console line. Returns True if it changed the maze state."""
        kind, data = protocol.parse_event(line)
        if kind == "maze":
            if data["size"] != self.n:
                self.set_size(data["size"])
            return True
        if kind == "goal":
            if data["cells"]:
                self.set_goal(data["cells"])
            return True
        if kind == "start":
            self.set_start(data["x"], data["y"], data["facing"])
            return True
        if kind == "tcost":
            self.turn_cost = data["turn_cost"]
            self.dirty = True
            return True
        if kind == "solve":
            self._apply_solve(data)
            return True
        if kind == "solve_done":
            self.running = False
            self.dirty = True
            return True
        if kind == "dump_begin":
            # a fetch of a buffered offline run is starting: wipe the discovered
            # layer so the replayed SOLVE, lines rebuild it fresh over your maze.
            self.clear_discovered()
            self.running = True
            self.dirty = True
            return True
        if kind == "dump_end":
            self.running = False
            self.dirty = True
            return True
        return False

    def _apply_solve(self, d: dict) -> None:
        x, y = d["x"], d["y"]
        if not (0 <= x < self.n and 0 <= y < self.n):
            return
        self.running = True
        self.phase = d["phase"]
        self.pose = (x, y, d["facing"])
        self.visited[x][y] = True
        self.flood[x][y] = d["flood"]
        if not self.path or self.path[-1] != (x, y):
            self.path.append((x, y))
        # record discovered walls (union - sticky)
        for d_idx, key in enumerate(("wallN", "wallE", "wallS", "wallW")):
            if d[key]:
                self._set_wall(self.disc, x, y, d_idx, True)
        self.dirty = True

    # -- snapshot for diagnostics -----------------------------------------
    def snapshot(self) -> dict:
        return {
            "size": self.n,
            "start": list(self.start),
            "start_facing": self.start_facing,
            "goal": [list(c) for c in self.goal],
            "pose": list(self.pose),
            "phase": self.phase,
            "turn_cost": self.turn_cost,
            "path": [list(c) for c in self.path],
            "discovered_walls": self.disc,
            "actual_walls": self.act,
            "flood": self.flood,
            "mismatches": self.mismatches(),
        }

    # -- robot config commands --------------------------------------------
    def config_commands(self) -> List[str]:
        """Console commands to push the current size + start + goal to the robot."""
        cmds = [f"maze {self.n}"]
        # start cell + nose direction. Always the explicit form (not the corner
        # shortcut) so a custom nose heading is preserved - the shortcut would
        # reset facing to the corner default. 'maze' goes first so the cell is in
        # range on the robot.
        cmds.append(f"start {self.start[0]} {self.start[1]} "
                    f"{'NESW'[self.start_facing]}")
        if len(self.goal) == 1:
            cmds.append(f"goal {self.goal[0][0]} {self.goal[0][1]}")
        elif self.goal == self._center_goal(self.n):
            cmds.append("goal center")
        cmds.append(f"tcost {self.turn_cost}")     # flood turn penalty
        return cmds
