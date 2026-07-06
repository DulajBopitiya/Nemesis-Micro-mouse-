"""
maze_view.py - Qt widgets that draw the live maze for the Nemisis app.

  MazeCanvas - the grid: discovered walls, hand-painted actual walls, the robot
               pose, its path, per-cell flood values, start/goal. Click an edge
               (in Edit mode) to paint the real maze for diagnosis.
  MazePanel  - the canvas plus a control strip (size, goal, send-to-robot,
               clear, display toggles). Emits `command(str)` for the GUI to send.

Kept separate from gui.py so the maze feature is self-contained.
"""

from __future__ import annotations

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt, Signal, QPointF

import planner
from maze_model import MazeModel, DX, DY, BIT, MAX_SIZE
from theme import FlowLayout, chip as _chip, accent

# direction -> the two cell-corner offsets (in cell units) of that edge
#   N=top, E=right, S=bottom, W=left  (screen y grows downward; north is up)
_EDGE = {
    0: ((0, 0), (1, 0)),   # N (top)
    1: ((1, 0), (1, 1)),   # E (right)
    2: ((0, 1), (1, 1)),   # S (bottom)
    3: ((0, 0), (0, 1)),   # W (left)
}

C_BG       = QtGui.QColor("#0c0f12")
C_GRID     = QtGui.QColor("#232a33")
C_START    = QtGui.QColor(80, 220, 120, 60)
C_GOAL     = QtGui.QColor(240, 200, 70, 70)
C_VISITED  = QtGui.QColor(70, 120, 200, 45)
C_PATH     = QtGui.QColor("#33dddd")
C_PLAN     = QtGui.QColor("#46d17a")            # turn-weighted planned route
C_PLAN_CMP = QtGui.QColor(150, 160, 175, 130)   # cell-only route (tcost=0) for compare
C_ROBOT    = QtGui.QColor("#ff5577")
C_FLOOD    = QtGui.QColor("#7b8794")
C_WALL_BOTH = QtGui.QColor("#e8edf2")   # robot saw it AND it's painted: confirmed
C_WALL_DISC = QtGui.QColor("#ffa53a")   # robot only: discovered (phantom if painted maze)
C_WALL_ACT  = QtGui.QColor("#5b6673")   # painted only: real wall robot hasn't found


class MazeCanvas(QtWidgets.QWidget):
    wallEdited = Signal()

    def __init__(self, model: MazeModel):
        super().__init__()
        self.m = model
        self.edit = False
        self.show_flood = True
        self.show_path = True
        self.show_actual = True
        # turn-weighted planner preview (app-side mirror of the robot's flood)
        self.show_plan = True
        self.show_plan_cmp = True            # also draw the cell-only (tcost=0) route
        self.plan_source = "auto"            # "auto" | "disc" | "act"
        self.plan = None                     # planner.Plan for the current tcost
        self.plan_cmp = None                 # planner.Plan for tcost=0
        self.plan_label = ""                 # which grid the plan ran on
        self.setMinimumSize(360, 360)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)

    # geometry: returns (ox, oy, cell)
    def _geom(self):
        n = self.m.n
        side = min(self.width(), self.height()) - 16
        cell = side / n
        ox = (self.width() - cell * n) / 2
        oy = (self.height() - cell * n) / 2
        return ox, oy, cell

    def _cell_topleft(self, x, y, ox, oy, cell):
        # screen top-left of cell (x,y); north is up so row from top = n-1-y
        return ox + x * cell, oy + (self.m.n - 1 - y) * cell

    # -- planner preview ---------------------------------------------------
    def _has_interior(self, grid) -> bool:
        """True if `grid` has any wall that isn't a maze-border edge (i.e. the
        map has actually been painted/discovered, not just the outer box)."""
        m = self.m
        for x in range(m.n):
            for y in range(m.n):
                for d in range(4):
                    if grid[x][y] & BIT[d]:
                        nx, ny = x + DX[d], y + DY[d]
                        if 0 <= nx < m.n and 0 <= ny < m.n:
                            return True
        return False

    def _plan_grid(self):
        """Pick which wall map to plan on -> (grid, label). 'auto' prefers the
        hand-painted ACTUAL maze (so you can design + preview a route), and falls
        back to what the robot DISCOVERED once it has explored."""
        m = self.m
        if self.plan_source == "disc":
            return m.disc, "discovered"
        if self.plan_source == "act":
            return m.act, "actual"
        if self._has_interior(m.act):
            return m.act, "actual"
        return m.disc, "discovered"

    def recompute_plan(self) -> None:
        """Recompute the previewed route for the current tcost (and the cell-only
        tcost=0 route for comparison). Cheap for <=16x16; call when the map, goal,
        start or tcost changes - not every repaint."""
        m = self.m
        grid, self.plan_label = self._plan_grid()
        self.plan = planner.plan_path(grid, m.n, m.start, m.start_facing,
                                      m.goal, m.turn_cost)
        self.plan_cmp = planner.plan_path(grid, m.n, m.start, m.start_facing,
                                          m.goal, 0)

    def _draw_plan(self, p, route, color, ox, oy, cell, dashed=False):
        if not route or len(route.cells) < 2:
            return
        pts = []
        for (x, y) in route.cells:
            cx, cy = self._cell_topleft(x, y, ox, oy, cell)
            pts.append(QPointF(cx + cell / 2, cy + cell / 2))
        pen = QtGui.QPen(color, max(2.0, cell * 0.08))
        pen.setJoinStyle(Qt.RoundJoin)
        pen.setCapStyle(Qt.RoundCap)
        if dashed:
            pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        p.drawPolyline(QtGui.QPolygonF(pts))

    # -- painting ----------------------------------------------------------
    def paintEvent(self, _ev):
        m = self.m
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.fillRect(self.rect(), C_BG)
        ox, oy, cell = self._geom()

        # cell fills: start / goal / visited
        for x in range(m.n):
            for y in range(m.n):
                cx, cy = self._cell_topleft(x, y, ox, oy, cell)
                rect = QtCore.QRectF(cx, cy, cell, cell)
                if (x, y) == m.start:
                    p.fillRect(rect, C_START)
                elif (x, y) in m.goal:
                    p.fillRect(rect, C_GOAL)
                elif m.visited[x][y]:
                    p.fillRect(rect, C_VISITED)

        # faint grid posts
        p.setPen(QtGui.QPen(C_GRID, 1))
        for i in range(m.n + 1):
            p.drawLine(QPointF(ox + i * cell, oy),
                       QPointF(ox + i * cell, oy + m.n * cell))
            p.drawLine(QPointF(ox, oy + i * cell),
                       QPointF(ox + m.n * cell, oy + i * cell))

        # flood values
        if self.show_flood:
            f = QtGui.QFont()
            f.setPointSizeF(max(6.0, cell * 0.24))
            p.setFont(f)
            p.setPen(C_FLOOD)
            for x in range(m.n):
                for y in range(m.n):
                    v = m.flood[x][y]
                    if v is None or v >= 999:
                        continue
                    cx, cy = self._cell_topleft(x, y, ox, oy, cell)
                    p.drawText(QtCore.QRectF(cx, cy, cell, cell),
                               Qt.AlignCenter, str(v))

        # planned route preview (turn-weighted; under the driven path). The faint
        # dashed line is the cell-only (tcost=0) route, drawn first when it differs
        # so you can SEE how the turn penalty straightens the green route.
        if self.show_plan:
            if (self.show_plan_cmp and self.plan_cmp and self.plan
                    and self.plan_cmp.cells != self.plan.cells):
                self._draw_plan(p, self.plan_cmp, C_PLAN_CMP, ox, oy, cell,
                                dashed=True)
            self._draw_plan(p, self.plan, C_PLAN, ox, oy, cell)

        # path trail
        if self.show_path and len(m.path) >= 2:
            p.setPen(QtGui.QPen(C_PATH, max(1.5, cell * 0.06)))
            pts = []
            for (x, y) in m.path:
                cx, cy = self._cell_topleft(x, y, ox, oy, cell)
                pts.append(QPointF(cx + cell / 2, cy + cell / 2))
            p.drawPolyline(QtGui.QPolygonF(pts))

        # walls (drawn last so they sit on top)
        wpen = max(2.0, cell * 0.10)
        for x in range(m.n):
            for y in range(m.n):
                cx, cy = self._cell_topleft(x, y, ox, oy, cell)
                for d in range(4):
                    st = m.wall_state(x, y, d)
                    if st == "none":
                        continue
                    if st == "act" and not self.show_actual:
                        continue
                    (ax, ay), (bx, by) = _EDGE[d]
                    a = QPointF(cx + ax * cell, cy + ay * cell)
                    b = QPointF(cx + bx * cell, cy + by * cell)
                    if st == "both":
                        pen = QtGui.QPen(C_WALL_BOTH, wpen)
                    elif st == "disc":
                        pen = QtGui.QPen(C_WALL_DISC, wpen)
                    else:  # act only
                        pen = QtGui.QPen(C_WALL_ACT, max(1.5, cell * 0.06))
                        pen.setStyle(Qt.DashLine)
                    p.setPen(pen)
                    p.drawLine(a, b)

        # robot
        rx, ry, facing = m.pose
        if 0 <= rx < m.n and 0 <= ry < m.n:
            cx, cy = self._cell_topleft(rx, ry, ox, oy, cell)
            self._draw_robot(p, cx + cell / 2, cy + cell / 2, cell * 0.30, facing)
        p.end()

    def _draw_robot(self, p, cx, cy, r, facing):
        # facing 0=N(up),1=E(right),2=S(down),3=W(left)
        import math
        ang = {0: -math.pi / 2, 1: 0.0, 2: math.pi / 2, 3: math.pi}[facing]
        tip = QPointF(cx + r * math.cos(ang), cy + r * math.sin(ang))
        l = QPointF(cx + r * math.cos(ang + 2.5), cy + r * math.sin(ang + 2.5))
        rr = QPointF(cx + r * math.cos(ang - 2.5), cy + r * math.sin(ang - 2.5))
        p.setPen(Qt.NoPen)
        p.setBrush(C_ROBOT)
        p.drawPolygon(QtGui.QPolygonF([tip, l, rr]))

    # -- click to paint actual walls --------------------------------------
    def mousePressEvent(self, ev):
        if not self.edit:
            return
        ox, oy, cell = self._geom()
        mx, my = ev.position().x(), ev.position().y()
        col = int((mx - ox) // cell)
        row_from_top = int((my - oy) // cell)
        x, y = col, self.m.n - 1 - row_from_top
        if not (0 <= x < self.m.n and 0 <= y < self.m.n):
            return
        lx = (mx - ox) - col * cell
        ly = (my - oy) - row_from_top * cell
        dist = {0: ly, 2: cell - ly, 3: lx, 1: cell - lx}   # N,S,W,E
        d = min(dist, key=dist.get)
        self.m.toggle_actual(x, y, d)
        self.wallEdited.emit()
        self.update()


class MazePanel(QtWidgets.QWidget):
    command = Signal(str)            # console command to send to the robot

    def __init__(self, model: MazeModel):
        super().__init__()
        self.m = model
        self.canvas = MazeCanvas(model)
        self.setObjectName("mazePanel")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)

        # ---- Configure & Run ------------------------------------------------
        self.size_spin = QtWidgets.QSpinBox()
        self.size_spin.setRange(2, 16)
        self.size_spin.setValue(model.n)
        self.size_spin.valueChanged.connect(self._on_size)

        self.goal_edit = QtWidgets.QLineEdit(self._goal_text())
        self.goal_edit.setMaximumWidth(90)
        self.goal_edit.setToolTip("'center' or 'x y' for a single cell")
        self.goal_edit.editingFinished.connect(self._on_goal)

        self.start_combo = QtWidgets.QComboBox()
        self.start_combo.addItems(["SW", "SE", "NW", "NE"])
        self.start_combo.setToolTip(
            "Which corner you place the mouse in (SW=bottom-left … NE=top-right).\n"
            "Picking a corner presets the nose direction (bottom→N, top→S); change\n"
            "it independently with 'Nose'. Applied with '→ Robot' / 'Solve'.")
        self.start_combo.currentIndexChanged.connect(self._on_start)

        self.nose_combo = QtWidgets.QComboBox()
        self.nose_combo.addItems(["N", "E", "S", "W"])
        self.nose_combo.setToolTip(
            "Which way the mouse's nose points at the start (N=up, E=right,\n"
            "S=down, W=left). Set this to match how you physically place it.\n"
            "Applied with '→ Robot' / 'Solve'.")
        self.nose_combo.currentIndexChanged.connect(self._on_nose)

        send_btn = QtWidgets.QPushButton("→ Robot")
        send_btn.setToolTip("Send maze size + goal to the mouse")
        send_btn.clicked.connect(self._send_config)

        solve_btn = QtWidgets.QPushButton("Solve")
        accent(solve_btn, "green")
        solve_btn.setToolTip("Pushes maze size + goal, then starts the search run")
        solve_btn.clicked.connect(self._solve)

        fast_btn = QtWidgets.QPushButton("Fast run")
        accent(fast_btn, "cyan")
        fast_btn.setToolTip("Speed run on the map the last search learned. On the "
                            "'Fastest' profile it takes corners as SMOOTH (non-stop) "
                            "turns; otherwise it pivots. Run a Solve to the goal first.")
        fast_btn.clicked.connect(self._fast)

        cfg = self._section("Configure & run")
        for w in (_chip("Size", self.size_spin), _chip("Goal", self.goal_edit),
                  _chip("Start", self.start_combo), _chip("Nose", self.nose_combo),
                  send_btn, solve_btn, fast_btn):
            cfg.addWidget(w)
        lay.addWidget(cfg.box)

        # ---- Alignment & offline run ---------------------------------------
        self.align_chk = QtWidgets.QCheckBox("Align turns")
        self.align_chk.setToolTip("Square to the wall ahead before each pivot "
                                  "(needs 'ircal front'). Sends 'align on/off'.")
        self.align_chk.toggled.connect(
            lambda v: self.command.emit("align on" if v else "align off"))

        square_btn = QtWidgets.QPushButton("Square now")
        square_btn.setToolTip("Run ONE alignment now to bench-test it "
                              "(face the mouse at a wall first)")
        square_btn.clicked.connect(lambda: self.command.emit("align"))

        fcal_btn = QtWidgets.QPushButton("Front cal")
        fcal_btn.setToolTip("Learn the centre->wall distance for post-stop "
                            "recenter. Place the mouse at a cell CENTRE facing a "
                            "wall, then click.")
        fcal_btn.clicked.connect(lambda: self.command.emit("frontcal"))

        dump_btn = QtWidgets.QPushButton("⬇ Fetch run")
        accent(dump_btn, "amber")
        dump_btn.setToolTip("Pull the last OFFLINE run's map + raw IR from the "
                            "robot ('dump') and overlay it on your drawn maze. "
                            "Reconnect first; the data is held in RAM until the "
                            "next run or power-off.")
        dump_btn.clicked.connect(self._fetch)

        # BENCH TEST: make the ESP32 bridge synthesise a run for 'Fetch' (no maze
        # / no STM32 needed) so the log-fetch + auto-refetch path can be tried on
        # the bench. First fetch after enabling arrives short on purpose.
        self.fake_dump = False
        self.faketest_chk = QtWidgets.QCheckBox("🧪 Fake dump (ESP)")
        self.faketest_chk.setToolTip(
            "TEST ONLY. Makes the ESP32 bridge fabricate a run for 'Fetch run' "
            "instead of pulling a real one from the STM32 — no maze required. The "
            "first fetch deliberately arrives SHORT so you can watch the app "
            "auto re-fetch and recover. Turn OFF for normal fetches.")
        self.faketest_chk.toggled.connect(self._on_faketest)

        self.edit_chk = QtWidgets.QCheckBox("✏ Edit walls")
        self.edit_chk.toggled.connect(self._on_edit)

        run = self._section("Align & offline run")
        for w in (self.align_chk, square_btn, fcal_btn, dump_btn,
                  self.faketest_chk, self.edit_chk):
            run.addWidget(w)
        lay.addWidget(run.box)

        # ---- Display + map tools -------------------------------------------
        disp = self._section("Display & map")
        for label, attr in [("Flood", "show_flood"), ("Path", "show_path"),
                            ("Actual walls", "show_actual")]:
            chk = QtWidgets.QCheckBox(label)
            chk.setChecked(getattr(self.canvas, attr))
            chk.toggled.connect(lambda v, a=attr: self._toggle(a, v))
            disp.addWidget(chk)
        clr_d = QtWidgets.QPushButton("Clear discovered")
        clr_d.clicked.connect(self._clear_disc)
        clr_a = QtWidgets.QPushButton("Clear actual")
        clr_a.clicked.connect(self._clear_act)
        save_m = QtWidgets.QPushButton("Save maze")
        save_m.setToolTip("Save the hand-painted maze (walls + size + start + goal) to a .json file")
        save_m.clicked.connect(self._save_maze)
        load_m = QtWidgets.QPushButton("Load maze")
        load_m.setToolTip("Load a maze saved earlier as the ACTUAL maze")
        load_m.clicked.connect(self._load_maze)
        for w in (clr_d, clr_a, save_m, load_m):
            disp.addWidget(w)
        lay.addWidget(disp.box)

        # ---- Planner preview -----------------------------------------------
        self.tcost_spin = QtWidgets.QSpinBox()
        self.tcost_spin.setRange(0, 50)
        self.tcost_spin.setValue(self.m.turn_cost)
        self.tcost_spin.setToolTip(
            "Flood turn penalty: extra cost (in cells) the planner pays per 90°\n"
            "pivot. Higher = straighter routes with fewer, bunched turns; 0 = plan\n"
            "by cell count only. Updates the green preview live; click '→ Robot'\n"
            "(or Solve) to send it to the mouse as 'tcost'.")
        self.tcost_spin.valueChanged.connect(self._on_tcost)

        tcost_send = QtWidgets.QPushButton("→ Robot")
        tcost_send.setToolTip("Send just the turn cost to the mouse ('tcost N')")
        tcost_send.clicked.connect(
            lambda: self.command.emit(f"tcost {self.m.turn_cost}"))

        self.plan_chk = QtWidgets.QCheckBox("Plan")
        self.plan_chk.setChecked(self.canvas.show_plan)
        self.plan_chk.setToolTip("Overlay the route the flood would drive for this "
                                 "turn cost (green), computed app-side from the map.")
        self.plan_chk.toggled.connect(lambda v: self._toggle("show_plan", v))

        self.plancmp_chk = QtWidgets.QCheckBox("vs cells")
        self.plancmp_chk.setChecked(self.canvas.show_plan_cmp)
        self.plancmp_chk.setToolTip("Also draw the cell-count-only route (tcost=0) "
                                    "as a faint dashed line, to compare.")
        self.plancmp_chk.toggled.connect(lambda v: self._toggle("show_plan_cmp", v))

        self.plansrc_combo = QtWidgets.QComboBox()
        self.plansrc_combo.addItems(["Auto", "Discovered", "Actual"])
        self.plansrc_combo.setToolTip(
            "Which map to plan on: Auto = your painted maze if any, else what the\n"
            "robot discovered; Discovered = robot's map; Actual = your painted maze.")
        self.plansrc_combo.currentIndexChanged.connect(self._on_plansrc)

        plan = self._section("Planner preview")
        for w in (_chip("Turn cost", self.tcost_spin), tcost_send,
                  self.plan_chk, self.plancmp_chk,
                  _chip("on", self.plansrc_combo)):
            plan.addWidget(w)
        lay.addWidget(plan.box)

        # ---- MMS simulation: load a real MMS maze, push it to the robot, run the
        # on-MCU `sim` against it (with optional fault injection), and overlay the
        # result on this maze - same maze the PC MMS would solve, for comparison.
        sim_btn = QtWidgets.QPushButton("Load MMS → sim")
        sim_btn.setToolTip(
            "Open an MMS .maz/.num file, load it as the ACTUAL maze, push it to the\n"
            "robot and run the headless on-MCU 'sim' against it, then fetch + overlay\n"
            "the discovered map. Set the fault rates first to stress the solver.")
        sim_btn.clicked.connect(self._load_mms_sim)

        self.sim_miss = QtWidgets.QSpinBox(); self.sim_miss.setRange(0, 100)
        self.sim_miss.setToolTip("Chance a real wall is NOT sensed (missed wall).")
        self.sim_false = QtWidgets.QSpinBox(); self.sim_false.setRange(0, 100)
        self.sim_false.setToolTip("Chance an open side reads as a wall (phantom).")
        self.sim_desync = QtWidgets.QSpinBox(); self.sim_desync.setRange(0, 100)
        self.sim_desync.setToolTip(
            "Chance, per cell driven, that the true position slips a cell\n"
            "(models a ram-induced odometry desync).")
        self.sim_mode = QtWidgets.QComboBox()
        self.sim_mode.addItems(["Explore", "Shortest"])
        self.sim_mode.setToolTip(
            "Phase-0 search strategy:\n"
            "Explore = current (prefers unvisited, maps more of the maze)\n"
            "Shortest = optimistic shortest-path (chases the quickest route,\n"
            "explores far less). Compare path/optimal/explored% in the SIM line.")

        sim = self._section("Simulation")
        for w in (sim_btn,
                  _chip("faults % miss", self.sim_miss),
                  _chip("false", self.sim_false),
                  _chip("desync", self.sim_desync),
                  _chip("search", self.sim_mode)):
            sim.addWidget(w)
        lay.addWidget(sim.box)

        lay.addWidget(self.canvas, 1)

        self.status = QtWidgets.QLabel("")
        self.status.setObjectName("mazeStatus")
        lay.addWidget(self.status)

        self.canvas.wallEdited.connect(self.refresh)
        self.refresh()

    def _section(self, title: str):
        """A titled group box whose controls FLOW (wrap) so nothing clips when
        the Maze tab is narrow. Returns an object with .box (the widget) and
        .addWidget(w) for populating it."""
        box = QtWidgets.QGroupBox(title)
        flow = FlowLayout(box, margin=2, hspacing=6, vspacing=6)

        class _Sec:
            pass
        s = _Sec()
        s.box = box
        s.addWidget = flow.addWidget
        return s

    def _goal_text(self) -> str:
        if self.m.goal == self.m._center_goal(self.m.n):
            return "center"
        if len(self.m.goal) == 1:
            return f"{self.m.goal[0][0]} {self.m.goal[0][1]}"
        return "center"

    # -- handlers ----------------------------------------------------------
    def _on_size(self, v):
        self.m.set_size(v)
        self.goal_edit.setText(self._goal_text())
        self.refresh()

    def _on_goal(self):
        t = self.goal_edit.text().strip().lower()
        if t in ("center", "centre"):
            self.m.set_goal(self.m._center_goal(self.m.n))
            self.refresh()
            return
        parts = t.split()
        try:
            if len(parts) != 2:
                raise ValueError
            x, y = int(parts[0]), int(parts[1])
        except ValueError:
            self.status.setText(f"goal: type 'center' or 'x y' (0..{self.m.n - 1})")
            self.goal_edit.setText(self._goal_text())
            return
        if not self.m.set_goal([(x, y)]):
            self.status.setText(f"goal ({x},{y}) out of range — valid 0..{self.m.n - 1}")
            self.goal_edit.setText(self._goal_text())
            return
        self.refresh()

    def _on_start(self, idx):
        corner = ["sw", "se", "nw", "ne"][idx]
        x, y, f = self.m.corner_cell(corner)      # corner presets the nose dir
        self.m.set_start(x, y, f)
        # mirrors Size/Goal: update the model now; push with '→ Robot' / 'Solve'.
        self.refresh()

    def _on_nose(self, idx):
        # nose direction only - keep the current start cell, change the heading.
        sx, sy = self.m.start
        self.m.set_start(sx, sy, idx)
        self.refresh()

    def _on_tcost(self, v):
        self.m.turn_cost = int(v)               # preview only; '→ Robot'/Solve sends
        self.refresh()

    def _on_plansrc(self, idx):
        self.canvas.plan_source = ["auto", "disc", "act"][idx]
        self.refresh()

    def _on_edit(self, v):
        self.canvas.edit = v
        self.canvas.setCursor(Qt.CrossCursor if v else Qt.ArrowCursor)

    def _toggle(self, attr, v):
        setattr(self.canvas, attr, v)
        self.canvas.update()

    def _send_config(self):
        for cmd in self.m.config_commands():
            self.command.emit(cmd)

    def _solve(self):
        # push the current size + goal first so the robot solves to YOUR goal,
        # not whatever it last had (the 'always center' surprise).
        self._send_config()
        self.command.emit("solve")

    def _fast(self):
        # speed run on the learned map; smooth turns if the 'Fastest' profile is active
        self.status.setText("Fast run… (smooth turns if on the Fastest profile)")
        self.command.emit("fast")

    def _on_faketest(self, on):
        # It's the ESP32 bridge that fabricates the data (##DUMPTEST## lines are
        # intercepted there and never reach the STM32); the app just flips which
        # command 'Fetch' sends.
        self.fake_dump = on
        self.command.emit("##DUMPTEST##on" if on else "##DUMPTEST##off")
        self.status.setText(
            "ESP fake-dump ON — 'Fetch run' now pulls SYNTHETIC data (first try is "
            "short on purpose; watch it auto re-fetch)" if on
            else "ESP fake-dump OFF — 'Fetch run' pulls the real offline run")

    def _fetch(self):
        # pull the buffered offline run; the robot replays DUMP,begin .. SOLVE/SIR
        # .. DUMP,end, and the model clears + rebuilds the discovered overlay.
        if getattr(self, "fake_dump", False):
            self.status.setText("Fetching SYNTHETIC run from ESP (fake-dump test)…")
            self.command.emit("##DUMPTEST##")
            return
        self.status.setText("Fetching last offline run… (overlaying on your maze)")
        self.command.emit("dump")

    def _load_mms_sim(self):
        # Load an MMS maze, push it to the robot, run the on-MCU sim against it,
        # then fetch the result so it overlays on this (now ground-truth) maze.
        import os
        from mms_maze import load_mms
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open MMS maze", "", "MMS maze (*.maz *.num);;All files (*)")
        if not path:
            return
        try:
            n, grid = load_mms(path)
        except Exception as e:                       # noqa: BLE001 - show any error
            self.status.setText(f"maze load failed: {e}")
            return
        if n < 2 or n > MAX_SIZE:
            self.status.setText(f"maze {n}x{n} unsupported (max {MAX_SIZE})")
            return

        self.m.load_actual(n, grid)                  # show it as the actual maze
        self.refresh()

        # push to the robot + run the headless sim against the exact same maze
        self.command.emit(f"maze {n}")
        self.command.emit("sim clear")
        for y in range(n):
            row = "".join(format(grid[x][y] & 0xF, "x") for x in range(n))
            self.command.emit(f"sim row {y} {row}")
        miss, fls, des = (self.sim_miss.value(), self.sim_false.value(),
                          self.sim_desync.value())
        self.command.emit(f"sim mode {self.sim_mode.currentIndex()}")
        self.command.emit(f"sim run {miss} {fls} {des}")
        self._fetch()                                # dump -> overlay discovered map
        self.status.setText(
            f"sim: {os.path.basename(path)} {n}x{n} faults({miss}/{fls}/{des}) "
            f"— running on MCU + fetching map")

    def _clear_disc(self):
        self.m.clear_discovered()
        self.refresh()

    def _clear_act(self):
        self.m.clear_actual()
        self.refresh()

    def _save_maze(self):
        # Save the hand-painted (ACTUAL) maze + its size/start/goal to JSON so a
        # maze you built by hand can be reloaded later for diagnosis / sim.
        import json, os
        default = os.path.join(os.path.dirname(__file__), "mazes")
        os.makedirs(default, exist_ok=True)
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save maze", os.path.join(default, "maze.json"),
            "Nemisis maze (*.json);;All files (*)")
        if not path:
            return
        data = {
            "size": self.m.n,
            "start": list(self.m.start),
            "start_facing": self.m.start_facing,
            "goal": [list(c) for c in self.m.goal],
            "actual_walls": self.m.act,          # n x n wall bitmasks (N/E/S/W)
        }
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:                       # noqa: BLE001 - surface any error
            self.status.setText(f"maze save failed: {e}")
            return
        self.status.setText(f"saved maze → {os.path.basename(path)}")

    def _load_maze(self):
        # Load a maze saved by _save_maze back into the ACTUAL layer (+ size/start/
        # goal) so you can diagnose against it or push it to the sim.
        import json, os
        default = os.path.join(os.path.dirname(__file__), "mazes")
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load maze", default if os.path.isdir(default) else "",
            "Nemisis maze (*.json);;All files (*)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            n = int(data["size"])
            grid = data["actual_walls"]
        except Exception as e:                       # noqa: BLE001
            self.status.setText(f"maze load failed: {e}")
            return
        if n < 2 or n > MAX_SIZE or len(grid) != n:
            self.status.setText(f"maze {n}x{n} unsupported (max {MAX_SIZE})")
            return
        self.m.load_actual(n, grid)                  # replaces actual + re-adds border
        if "goal" in data:
            self.m.set_goal([tuple(c) for c in data["goal"]])
        if "start" in data:
            sx, sy = data["start"]
            self.m.set_start(int(sx), int(sy), int(data.get("start_facing", 0)))
        self.refresh()
        self.status.setText(f"loaded maze ← {os.path.basename(path)}")

    # -- refresh -----------------------------------------------------------
    def refresh(self):
        if self.size_spin.value() != self.m.n:
            self.size_spin.blockSignals(True)
            self.size_spin.setValue(self.m.n)
            self.size_spin.blockSignals(False)
        ci = {"sw": 0, "se": 1, "nw": 2, "ne": 3}.get(self.m.corner_name(), 0)
        if self.start_combo.currentIndex() != ci:
            self.start_combo.blockSignals(True)
            self.start_combo.setCurrentIndex(ci)
            self.start_combo.blockSignals(False)
        if self.nose_combo.currentIndex() != self.m.start_facing:
            self.nose_combo.blockSignals(True)
            self.nose_combo.setCurrentIndex(self.m.start_facing)
            self.nose_combo.blockSignals(False)
        # keep the tcost spin in step with the model (e.g. after a robot echo)
        if self.tcost_spin.value() != self.m.turn_cost:
            self.tcost_spin.blockSignals(True)
            self.tcost_spin.setValue(self.m.turn_cost)
            self.tcost_spin.blockSignals(False)

        self.canvas.recompute_plan()            # refresh the route preview
        mism = len(self.m.mismatches())
        rx, ry, rf = self.m.pose
        ph = "search" if self.m.phase == 0 else "return"
        run = "running" if self.m.running else "idle"
        extra = f"  ·  {mism} wall mismatch(es)" if mism else ""
        self.status.setText(
            f"{self.m.n}×{self.m.n}  ·  robot ({rx},{ry}) {'NESW'[rf]}  ·  "
            f"{ph} · {run}  ·  {len(self.m.path)} cells{extra}  ·  {self._plan_text()}")
        self.canvas.update()
        self.m.dirty = False

    def _plan_text(self) -> str:
        """Compact summary of the previewed route for the status bar."""
        pl = self.canvas.plan
        if pl is None:
            return "plan: —"
        if not pl.reached:
            return f"plan({self.canvas.plan_label}): goal unreachable"
        cmp = self.canvas.plan_cmp
        base = (f"plan({self.canvas.plan_label}) @tcost={self.m.turn_cost}: "
                f"{pl.length} cells / {pl.turns} turns")
        if cmp is not None and cmp.reached and cmp.turns != pl.turns:
            base += f"  vs cells-only: {cmp.length}/{cmp.turns} turns"
        return base
