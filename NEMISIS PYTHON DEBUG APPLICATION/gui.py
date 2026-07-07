#!/usr/bin/env python3
"""
gui.py - graphical debug / tuning console for the Nemisis micromouse.

PySide6 (Qt) UI + pyqtgraph live plots over the WiFi bridge (ESP32-C3 -> STM32
debug console). Same text protocol as the J-Link RTT terminal, just prettier and
with real-time graphs - the foundation for PID tuning.

Run:
    pip install -r requirements.txt
    python gui.py                       # defaults to 192.168.4.1:3333

Layout:
    [host][port][Connect]                         <- connection bar
    [encoder][imu][sensors][battery][buzzer][stop] <- quick subsystem buttons
    +------------------+  +----------------------+
    |   live plot      |  | console output       |
    | (active stream)  |  +----------------------+
    |                  |  | motor + vacuum panel |
    +------------------+  +----------------------+
    [ command line .......................][Send]
"""

from __future__ import annotations

import re
import sys
import time
import threading
from collections import deque
from pathlib import Path
from typing import Dict, Optional

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt, Signal
import pyqtgraph as pg

from connection import NemisisLink
from maze_model import MazeModel
from maze_view import MazePanel
from recorder import Recorder
from protocol import parse_event
from theme import apply_theme, accent
import serial_provision
from ota_upload import OtaUploader

# ----------------------------------------------------------------------------
# Telemetry parsing: turn console text lines into numeric series for plotting.
# Each entry: group key -> (compiled regex, [series names]). Capture groups in
# the regex map 1:1 to the series names.
# ----------------------------------------------------------------------------
PARSERS = {
    "imu": (
        re.compile(
            r"acc\[mg\]\s*X=(-?\d+)\s*Y=(-?\d+)\s*Z=(-?\d+)\s*\|\s*"
            r"gyr\[mdps\]\s*X=(-?\d+)\s*Y=(-?\d+)\s*Z=(-?\d+)"
        ),
        ["acc X", "acc Y", "acc Z", "gyro X", "gyro Y", "gyro Z"],
    ),
    "enc": (
        re.compile(
            r"ENC\s+L\s+raw=\s*(\d+).*?cnt=\s*(-?\d+).*?"
            r"R\s+raw=\s*(\d+).*?cnt=\s*(-?\d+)"
        ),
        ["L raw", "L cnt", "R raw", "R cnt"],
    ),
    "ir": (
        re.compile(
            r"IR\s+L_LM=\s*(-?\d+)\s+L_M=\s*(-?\d+)\s+L_F=\s*(-?\d+)\s+"
            r"R_F=\s*(-?\d+)\s+R_M=\s*(-?\d+)\s+R_RM=\s*(-?\d+)"
        ),
        ["L_LM", "L_M", "L_F", "R_F", "R_M", "R_RM"],
    ),
    "batt": (
        re.compile(r"BATT\s+(-?\d+)\s*mV\s*\((-?\d+)\s*mV/cell\)\s+([\d.]+)\s*%"),
        ["pack mV", "cell mV", "SoC %"],
    ),
    # Filtered wall sensors: firmware streams "WALL,L_LM,L_M,L_F,R_F,R_M,R_RM".
    "walls": (
        re.compile(r"WALL,\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)"),
        ["L_LM", "L_M", "L_F", "R_F", "R_M", "R_RM"],
    ),
    # Future PID telemetry: firmware streams "TLM,<t>,<setpoint>,<meas>,<out>".
    "tlm": (
        re.compile(r"TLM,\s*([\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)"),
        ["t", "setpoint", "measured", "output"],
    ),
}

GROUP_TITLES = {
    "imu": "IMU  (acc mg / gyro mdps)",
    "enc": "Encoders  (raw 0..16383 / quad count)",
    "ir": "IR reflectance",
    "walls": "Wall sensors  (IR, filtered)",
    "batt": "Battery",
    "tlm": "PID telemetry",
}

# ----------------------------------------------------------------------------
# Battery safety state. The firmware tags lines with its monitor state so the
# app can show a persistent indicator and mirror the high-current lockout:
#   stream : "BATT 8104 mV (4052 mV/cell)  86.34 %  -4368 m%/hr  [OK]"
#   boot   : "BATTERY monitor armed (crit<6600mV low<7000mV) - state=OK"
#   gate   : "BATTERY CRITICAL - motors disabled. Charge the pack."
# ----------------------------------------------------------------------------
BATT_LINE_RX = re.compile(
    r"BATT\s+(-?\d+)\s*mV.*?([\d.]+)\s*%(?:.*?\[(\w+)\])?")
BATT_ARMED_RX = re.compile(r"BATTERY\s+monitor\s+armed.*?state=(\w+)", re.I)
BATT_GATE_RX = re.compile(r"BATTERY\s+(OK|LOW|CRITICAL)\b.*?disabled", re.I)

# Firmware echoes gains as integers x1000, e.g. "  lvel kp=400 ki=2000 kd=0".
# We parse these so the tuning panel stays in sync with the mouse (a free "Get"
# whenever the user — or the app — sends a bare `pid`).
GAIN_RX = re.compile(
    r"\b(lvel|rvel|head)\s+kp=(-?\d+)\s+ki=(-?\d+)\s+kd=(-?\d+)", re.I)

# Velocity feedforward echo, e.g. "vel feedforward = 150 (x1000 ...)" or
# "vel ff -> 200 (x1000 ...)". Value is the gain x1000.
FF_RX = re.compile(r"(?:feedforward\s*=|vel ff ->)\s*(-?\d+)", re.I)

# Auto-tune result line from the firmware:
#   "ATUNE,ok,<kp>,<ki>,<kd>,<ku>,<tu_ms>"  (gains/ku x1000, tu in ms)  or
#   "ATUNE,fail"
ATUNE_OK_RX   = re.compile(r"ATUNE,ok,(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)", re.I)
ATUNE_FAIL_RX = re.compile(r"ATUNE,fail", re.I)

# "MOVE: done, travelled 1003 mm" — a distance-limited run finished.
MOVE_DONE_RX = re.compile(r"MOVE:\s*done,\s*travelled\s*(-?\d+)\s*mm", re.I)

# Standalone matcher for the per-tick head-loop telemetry, used by the arc logger:
# "TLM,<t>,<setpoint>,<measured>,<output>". setpoint/measured are deg x100 for the
# head loop; output is the heading trim (cps).
ARC_TLM_RX = re.compile(r"TLM,\s*([\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)")

# "TURN: done, heading 90 deg" — a pivot turn finished (heading actually reached).
TURN_DONE_RX = re.compile(r"TURN:\s*done,\s*heading\s*(-?\d+)\s*deg", re.I)

# "CELL,3,1,0,1" — an advance reached cell 3: left / right / front wall flags.
CELL_RX = re.compile(r"CELL,\s*(\d+),\s*([01]),\s*([01]),\s*([01])", re.I)
# "cell pitch = 180 mm" — the maze cell pitch echo.
CELL_MM_RX = re.compile(r"cell\s+pitch\s*=\s*(\d+)\s*mm", re.I)

# Nav sequencer (path) progress: "PATH 2/5: advance 3 cells" / "PATH 3/5: turn 90 deg".
PATH_STEP_RX = re.compile(r"PATH\s+(\d+)/(\d+):\s*(advance|turn)\s+(-?\d+)", re.I)
# "PATH: sequence done" / "PATH: aborted".
PATH_DONE_RX = re.compile(r"PATH:\s*(sequence done|aborted)", re.I)

# Pivot slew echo: "tcfg rate = 240  accel = 1200  (deg/s, deg/s^2)". Plain ints.
TCFG_RX = re.compile(r"tcfg\s+rate\s*=\s*(\d+)\s+accel\s*=\s*(\d+)", re.I)

# Ramp rates echo: "accel = 30000  decel = 30000  (cps/s)". Plain ints (cps/s).
ACCEL_RX = re.compile(r"accel\s*=\s*(\d+)\s+decel\s*=\s*(\d+)", re.I)

# Velocity filter echo: "vfilt window = 20 ms  alpha = 300 (x1000)".
VFILT_RX = re.compile(r"vfilt\s+window\s*=\s*(\d+)\s*ms\s+alpha\s*=\s*(\d+)", re.I)

# Smooth-arc tuning echoes (the UKMARSBOT-style feed-forward turn):
#   "arcff = 180 (x10)  ..."  -> curvature feed-forward, value is x10 (=> 18.0)
#   "arckp = 80  ..."         -> gentle heading-correction gain (plain int)
#   "arckd = 12  ..."         -> heading damping / D on filtered error (plain int)
# Fast-run smooth-turn launch marker: "FLOW,<deg>,<radius>,<cps>"
FLOW_RX = re.compile(r"FLOW,\s*(-?\d+),\s*(\d+),\s*(\d+)")
ARCFFL_RX = re.compile(r"arcffl\s*=\s*(-?\d+)\s*\(x10\)", re.I)
ARCFFR_RX = re.compile(r"arcffr\s*=\s*(-?\d+)\s*\(x10\)", re.I)
ARCKP_RX = re.compile(r"arckp\s*=\s*(-?\d+)", re.I)
ARCKD_RX = re.compile(r"arckd\s*=\s*(-?\d+)", re.I)
ARCGKD_RX = re.compile(r"arcgkd\s*=\s*(-?\d+)\s*\(x10\)", re.I)
# Heading D-term low-pass echo: "dfilt = 80 (x1000) ..." (straight/pivot loop).
DFILT_RX = re.compile(r"dfilt\s*=\s*(-?\d+)\s*\(x1000\)", re.I)

# IR timing echo: "irset settle = 150 us  samples = 8 ...". Plain ints.
IRSET_RX = re.compile(r"irset\s+settle\s*=\s*(\d+)\s*us\s+samples\s*=\s*(\d+)", re.I)

# Corridor-centring echoes. Show line: "follow ON   gain = 50 (x1000 deg/count)".
# Action lines: "follow ON - ..." / "follow off - ...". Gain line: "fcfg gain = 50 ...".
FOLLOW_RX  = re.compile(r"follow\s+(ON|off)\s+gain\s*=\s*(-?\d+)", re.I)
FOLLOW_ST_RX = re.compile(r"follow\s+(ON|off)\b", re.I)
FGAIN_RX   = re.compile(r"fcfg\s+gain\s*=\s*(-?\d+)", re.I)

# Boot geometry log: "CONTROL: geom 2451 counts/mm (x100) ...". The number is
# counts/mm x100, so we can show ramp rates in real m/s^2.
GEOM_RX = re.compile(r"geom\s+(\d+)\s+counts/mm", re.I)

# Tuning-profile sync. Firmware emits these on profile switch / save / load:
#   "PROFILE,<idx>,<name>"   active profile (also at boot via the menu)
#   "SAVE,ok,<idx>,<name>" | "SAVE,fail"
#   "LOAD,<idx>,<name>"
PROFILE_RX = re.compile(r"PROFILE,(\d+),(\w+)", re.I)
SAVE_RX    = re.compile(r"SAVE,(ok|fail)(?:,(\d+),(\w+))?", re.I)
LOAD_RX    = re.compile(r"LOAD,(\d+),(\w+)", re.I)

# PID loops the firmware exposes. Order matches the combo box.
PID_LOOPS = ["vel", "lvel", "rvel", "head"]

# Tuning profiles stored in the robot's flash. Order matches the firmware index.
PROFILES = ["Search", "Fast", "Fastest"]


class ArcLogger:
    """Captures one arc run to a self-describing CSV so the turn can be analysed
    offline (the whole point: stop tuning blind off pasted telemetry).

    One file per arc, written to ./logs/arc/. Each file records the arc params,
    the live arc gains, then every head-loop TLM tick (setpoint / measured /
    output / error), and the final result (heading reached, exit distance). The
    raw values are logged untouched; any analysis (lag, overshoot, settle) is done
    from the CSV afterwards.
    """

    DIR = Path(__file__).resolve().parent / "logs" / "arc"

    def __init__(self):
        self._f = None
        self.path: Optional[Path] = None
        self._rows = 0
        self.exit_mm = 0

    @property
    def active(self) -> bool:
        return self._f is not None

    def start(self, params: dict, gains: dict) -> Path:
        self.stop("superseded by a new arc")          # close any open log first
        self.DIR.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        p = params
        self.exit_mm = int(p["exit"])
        self.path = self.DIR / (f"arc_{ts}_d{p['deg']}_c{p['cps']}"
                                f"_r{p['radius']}_e{p['exit']}.csv")
        self._f = open(self.path, "w", encoding="utf-8", newline="")
        self._rows = 0
        self._f.write(f"# arc log {ts}\n")
        self._f.write(f"# params: deg={p['deg']} cps={p['cps']} "
                      f"radius_mm={p['radius']} exit_mm={p['exit']}\n")
        self._f.write(f"# ui gains (may be stale after a reflash): "
                      f"arcffl={gains.get('arcffl')} arcffr={gains.get('arcffr')} "
                      f"arckp={gains.get('arckp')} arckd={gains.get('arckd')}\n")
        self._f.write("# (authoritative 'fw <gain>=' lines below are queried live "
                      "from the firmware)\n")
        self._f.write("# setpoint/measured = heading deg x100; "
                      "output = heading trim (cps); error = setpoint-measured\n")
        self._f.write("t_ms,setpoint_cdeg,measured_cdeg,output,error_cdeg\n")
        return self.path

    def row(self, t: float, sp: float, meas: float, out: float):
        if self._f:
            self._f.write(f"{t:g},{sp:g},{meas:g},{out:g},{sp - meas:g}\n")
            self._rows += 1

    def note(self, text: str):
        if self._f:
            self._f.write(f"# {text}\n")

    def stop(self, reason: str = "") -> Optional[Path]:
        if not self._f:
            return None
        if reason:
            self._f.write(f"# end: {reason} ({self._rows} rows)\n")
        self._f.close()
        self._f = None
        return self.path


class FastLog:
    """Captures a whole FAST run to one CSV (logs/fast/) so a crash / jizzle in the
    smooth turns can be analysed: every head-loop TLM tick, plus marker comments for
    each flow-turn start (FLOW), each cell decision (SOLVE) and turn completions.
    Stays open for the whole run (unlike ArcLogger which is one turn)."""

    DIR = Path(__file__).resolve().parent / "logs" / "fast"

    def __init__(self):
        self._f = None
        self.path: Optional[Path] = None
        self._rows = 0

    @property
    def active(self) -> bool:
        return self._f is not None

    def start(self) -> Path:
        self.stop("superseded")
        self.DIR.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.path = self.DIR / f"fast_{ts}.csv"
        self._f = open(self.path, "w", encoding="utf-8", newline="")
        self._rows = 0
        self._f.write(f"# fast run log {ts}\n")
        self._f.write("# head-loop TLM ticks + marker comments. FLOW,deg,r,cps = a "
                      "smooth turn launched; SOLVE = a cell decision; TURN = turn done\n")
        self._f.write("# setpoint/measured = heading deg x100; output = trim (cps)\n")
        self._f.write("t_ms,setpoint_cdeg,measured_cdeg,output,error_cdeg\n")
        return self.path

    def row(self, t: float, sp: float, meas: float, out: float):
        if self._f:
            self._f.write(f"{t:g},{sp:g},{meas:g},{out:g},{sp - meas:g}\n")
            self._rows += 1

    def note(self, text: str):
        if self._f:
            self._f.write(f"# {text}\n")

    def stop(self, reason: str = "") -> Optional[Path]:
        if not self._f:
            return None
        if reason:
            self._f.write(f"# end: {reason} ({self._rows} rows)\n")
        self._f.close()
        self._f = None
        return self.path


class GripLog:
    """Captures a vacuum grip/downforce test to one CSV (logs/grip/). The firmware
    sweeps the fan through several duty levels, hard-brakes from a set speed at each,
    and emits GRIP,* lines: a per-level row (duty, v0_cps, brake_mm, a_max) and a
    downforce summary (GRIP,res,...) at the end. We log them raw so the run can be
    reviewed / re-plotted offline (a_max vs duty is the grip curve for the profiler)."""

    DIR = Path(__file__).resolve().parent / "logs" / "grip"

    def __init__(self):
        self._f = None
        self.path: Optional[Path] = None

    @property
    def active(self) -> bool:
        return self._f is not None

    def start(self) -> Path:
        self.stop("superseded")
        self.DIR.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.path = self.DIR / f"grip_{ts}.csv"
        self._f = open(self.path, "w", encoding="utf-8", newline="")
        self._f.write(f"# vacuum grip / downforce test {ts}\n")
        self._f.write("# per-level:  GRIP,<duty%>,<v0_cps>,<brake_mm>,<a_max_mm_s2>\n")
        self._f.write("# summary:    GRIP,res,<duty%>,<a_max>,<ratio_x1000>,<downforce_gf>,<grip_pct>\n")
        self._f.write("# downforce cancels friction: F_down = weight*(a_on/a_off - 1)\n")
        return self.path

    def raw(self, line: str):
        if self._f:
            self._f.write(line.strip() + "\n")

    def stop(self, reason: str = "") -> Optional[Path]:
        if not self._f:
            return None
        if reason:
            self._f.write(f"# end: {reason}\n")
        self._f.close()
        self._f = None
        return self.path


class MultiRunFetch:
    """Splits a multi-run 'dump' (the launcher's Search->Fast->Fastest session) into
    ONE CSV per run under logs/runs/<ts>/, each holding the full motion+sensor trace
    (encoders, heading, gyro-Z, accel-Z, all 6 IR, cell, flood, pack voltage) plus
    the per-cell decision (# SOLVE) and sensed-IR (# SIR) markers - everything needed
    to diagnose that run offline. Fed raw console lines; keys off the DUMP,run / TRACE
    / SOLVE / SIR markers the firmware emits during 'dump'."""

    DIR = Path(__file__).resolve().parent / "logs" / "runs"
    TRACE_HDR = ("t_ms,posL,posR,head_deg,gyroz_dps,accelz_g,"
                 "L_LM,L_M,L_F,R_F,R_M,R_RM,x,y,flood,vbat_mv")

    def __init__(self):
        self._sess: Optional[Path] = None
        self._f = None
        self._rows = 0
        self._exp_trace = 0
        self._cur_run = 0
        self._cur_label = ""
        self.files: list = []
        # per-run (run, label, rows, expected) for the LAST session; survives
        # end() so the console can decide whether a fetch needs re-trying.
        self.run_stats: list = []

    @property
    def active(self) -> bool:
        return self._sess is not None

    @property
    def incomplete_runs(self) -> list:
        """Runs from the last session that came back short: [(label, rows, expected)]."""
        return [(lbl, rows, exp) for (_r, lbl, rows, exp) in self.run_stats
                if exp and rows < exp]

    def begin(self, nruns: int) -> Path:
        self.end()                                   # close any half-open session
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._sess = self.DIR / ts
        self._sess.mkdir(parents=True, exist_ok=True)
        self.files = []
        self.run_stats = []
        return self._sess

    def _close_run(self):
        if self._f:
            self._f.write(f"# rows: {self._rows} trace (expected {self._exp_trace})\n")
            if self._exp_trace and self._rows < self._exp_trace:
                self._f.write("# WARNING: fewer trace rows than expected - link "
                              "dropped data; fetch again (robot kept the log)\n")
            self._f.close()
            self._f = None
            self.run_stats.append(
                (self._cur_run, self._cur_label, self._rows, self._exp_trace))

    def start_run(self, run: int, label: str, nsolve: int, ntrace: int):
        if not self._sess:
            return
        self._close_run()
        path = self._sess / f"run{run}_{label}.csv"
        self._f = open(path, "w", encoding="utf-8", newline="")
        self._rows = 0
        self._exp_trace = ntrace
        self._cur_run = run
        self._cur_label = label
        self._f.write(f"# run {run} ({label}) - {nsolve} decisions, {ntrace} trace samples\n")
        self._f.write("# TRACE rows below; '# SOLVE'/'# SIR' = per-cell decision + sensed IR\n")
        self._f.write(self.TRACE_HDR + "\n")
        self.files.append(path)

    def feed(self, line: str):
        """Route one dump line into the current run's CSV. Returns True if consumed."""
        if not self._f:
            return False
        kind, data = parse_event(line)
        if kind == "trace":
            ir = data["ir"]
            self._f.write(
                f'{data["t_ms"]},{data["posL"]},{data["posR"]},'
                f'{data["heading_deg"]:g},{data["gyroz_dps"]:g},{data["accelz_g"]:g},'
                f'{ir["L_LM"]},{ir["L_M"]},{ir["L_F"]},{ir["R_F"]},{ir["R_M"]},{ir["R_RM"]},'
                f'{data["x"]},{data["y"]},{data["flood"]},'
                f'{data["vbat_mv"] if data["vbat_mv"] is not None else ""}\n')
            self._rows += 1
            return True
        if kind == "solve":
            self._f.write(f"# SOLVE {line.strip()}\n")
            return True
        if kind == "sir":
            self._f.write(f"# SIR {line.strip()}\n")
            return True
        return False

    def end(self) -> Optional[Path]:
        if not self._sess:
            return None
        self._close_run()
        sess = self._sess
        self._sess = None
        return sess
# How many times to automatically re-issue a 'dump' when the link drops rows.
# The robot keeps the full run in RAM, so a re-fetch replays the same data - we
# just keep trying until every announced TRACE row arrives (or we give up and
# keep the most-complete copy).
FETCH_MAX_RETRIES = 3

# Starting gains (mirror the firmware defaults in control.c).
PID_DEFAULTS = {
    "vel":  (0.4, 2.0, 0.0),
    "lvel": (0.4, 2.0, 0.0),
    "rvel": (0.4, 2.0, 0.0),
    "head": (200.0, 0.0, 8.0),
}

BATT_COLORS = {
    "OK": "#55ff7f",
    "LOW": "#ffcc44",
    "CRITICAL": "#ff6666",
}

# ----------------------------------------------------------------------------
# Plot layout per group. Each group becomes one or more stacked panels so very
# different quantities don't share an axis. A panel is:
#   title, y-axis label, optional fixed y-range, and a list of traces, each:
#     (series_name_from_PARSERS, legend_label, color_index, scale)
# `scale` converts the raw parsed value to display units (e.g. mg -> g).
# ----------------------------------------------------------------------------
PANELS = {
    "imu": [
        {"title": "Acceleration", "ylabel": "g", "yrange": None, "traces": [
            ("acc X", "X", 0, 0.001), ("acc Y", "Y", 1, 0.001), ("acc Z", "Z", 2, 0.001)]},
        {"title": "Gyroscope", "ylabel": "°/s", "yrange": None, "traces": [
            ("gyro X", "X", 0, 0.001), ("gyro Y", "Y", 1, 0.001), ("gyro Z", "Z", 2, 0.001)]},
    ],
    "enc": [
        {"title": "Wheel angle", "ylabel": "deg", "yrange": (0, 360), "traces": [
            ("L raw", "Left", 0, 360.0 / 16384.0), ("R raw", "Right", 2, 360.0 / 16384.0)]},
        {"title": "Quadrature count", "ylabel": "counts", "yrange": None, "traces": [
            ("L cnt", "Left", 0, 1.0), ("R cnt", "Right", 2, 1.0)]},
    ],
    "ir": [
        {"title": "IR — Left", "ylabel": "reflected", "yrange": None, "traces": [
            ("L_LM", "L_LM", 0, 1.0), ("L_M", "L_M", 1, 1.0), ("L_F", "L_F", 2, 1.0)]},
        {"title": "IR — Right", "ylabel": "reflected", "yrange": None, "traces": [
            ("R_F", "R_F", 3, 1.0), ("R_M", "R_M", 4, 1.0), ("R_RM", "R_RM", 5, 1.0)]},
    ],
    "walls": [
        {"title": "Walls — Left side / Front", "ylabel": "reflected", "yrange": None, "traces": [
            ("L_LM", "L_LM (side)", 0, 1.0), ("L_M", "L_M (mid)", 1, 1.0),
            ("L_F", "L_F (front)", 2, 1.0)]},
        {"title": "Walls — Right side / Front", "ylabel": "reflected", "yrange": None, "traces": [
            ("R_F", "R_F (front)", 3, 1.0), ("R_M", "R_M (mid)", 4, 1.0),
            ("R_RM", "R_RM (side)", 5, 1.0)]},
    ],
    "batt": [
        {"title": "Voltage", "ylabel": "V", "yrange": None, "traces": [
            ("pack mV", "pack", 0, 0.001), ("cell mV", "cell", 2, 0.001)]},
        {"title": "State of charge", "ylabel": "%", "yrange": (0, 100), "traces": [
            ("SoC %", "SoC", 1, 1.0)]},
    ],
    "tlm": [
        {"title": "PID response", "ylabel": "", "yrange": None, "traces": [
            ("setpoint", "setpoint", 0, 1.0), ("measured", "measured", 2, 1.0),
            ("output", "output", 3, 1.0)]},
    ],
}

MAX_POINTS = 600  # rolling window per series


# ----------------------------------------------------------------------------
# Bridge NemisisLink's background reader thread into the Qt event loop.
# ----------------------------------------------------------------------------
class LinkBridge(QtCore.QObject):
    lineReceived = Signal(str)
    connected = Signal()
    failed = Signal(str)
    closed = Signal()

    def __init__(self, host: str, port: int):
        super().__init__()
        self.link = NemisisLink(host, port)
        self.link.on_line(self._on_line)

    def _on_line(self, line: str):
        if line == "[link closed]":
            self.closed.emit()
        else:
            self.lineReceived.emit(line)

    def connect_async(self):
        def worker():
            try:
                self.link.connect()
                self.connected.emit()
            except OSError as e:
                self.failed.emit(str(e))

        threading.Thread(target=worker, daemon=True).start()

    def send(self, text: str):
        try:
            self.link.send_line(text)
        except Exception as e:  # noqa: BLE001 - surface any send error in UI
            self.lineReceived.emit(f"[send error: {e}]")

    def close(self):
        self.link.close()


# ----------------------------------------------------------------------------
# Marshal OTA worker-thread callbacks onto the Qt GUI thread (queued signals).
# ----------------------------------------------------------------------------
class OtaSignals(QtCore.QObject):
    progress = Signal(int, int)     # (bytes done, total)
    log = Signal(str)
    done = Signal(bool, str)        # (ok, error-or-empty)


# ----------------------------------------------------------------------------
# Rolling data store per group/series, feeding the live plot.
# ----------------------------------------------------------------------------
class Series:
    def __init__(self):
        self.x: deque = deque(maxlen=MAX_POINTS)
        self.y: deque = deque(maxlen=MAX_POINTS)


class DataStore:
    def __init__(self):
        self.t0 = time.monotonic()
        self.groups: Dict[str, Dict[str, Series]] = {
            g: {name: Series() for name in names} for g, (_, names) in PARSERS.items()
        }
        self.dirty = False

    def ingest(self, line: str) -> Optional[str]:
        for group, (rx, names) in PARSERS.items():
            m = rx.search(line)
            if not m:
                continue
            t = time.monotonic() - self.t0
            for name, val in zip(names, m.groups()):
                s = self.groups[group][name]
                s.x.append(t)
                s.y.append(float(val))
            self.dirty = True
            return group
        return None


class GripPanel(QtWidgets.QWidget):
    """Vacuum grip / downforce test panel: launch the fan-duty brake sweep and
    visualise the result. The bar chart shows peak braking deceleration (a_max) per
    fan level — the grip curve that feeds the speed profiler — and the table gives
    brake distance, the ratio to fan-off, grip gain % and absolute downforce (grams-
    force, needs the robot weight). Fed the raw GRIP,* console lines via feed()."""

    command = Signal(str)

    # column sets per test mode (the middle two differ; downforce math is shared)
    COLS_BRAKE = ["fan %", "brake mm", "a_max mm/s²", "×base", "grip %", "downforce gf"]
    COLS_SPIN = ["fan %", "spin dps", "decel dps/s", "×base", "grip %", "downforce gf"]
    COLS = COLS_BRAKE

    def __init__(self):
        super().__init__()
        lay = QtWidgets.QVBoxLayout(self)

        # controls: weight (for absolute downforce) + test speed + launch/stop
        ctl = QtWidgets.QHBoxLayout()
        ctl.addWidget(QtWidgets.QLabel("Weight"))
        self.weight = QtWidgets.QSpinBox()
        self.weight.setRange(0, 5000)
        self.weight.setValue(250)
        self.weight.setSuffix(" g")
        self.weight.setToolTip("Robot weight — weigh it once on a scale. Needed for the "
                               "absolute downforce (gf); grip % works without it.")
        ctl.addWidget(self.weight)
        ctl.addWidget(QtWidgets.QLabel("Speed"))
        self.speed = QtWidgets.QSpinBox()
        self.speed.setRange(3000, 30000)
        self.speed.setSingleStep(1000)
        self.speed.setValue(12000)
        self.speed.setSuffix(" cps")
        self.speed.setToolTip("Speed to brake from (~0.5 m/s ≈ 12000 cps). Raise it if the "
                              "brake distance doesn't shrink as fan % rises.")
        ctl.addWidget(self.speed)
        self.run_btn = QtWidgets.QPushButton("Grip test")
        self.run_btn.setToolTip("Braking grip: accelerate to Speed, hard-brake, measure "
                                "stopping distance. Needs the runway shown.")
        self.run_btn.clicked.connect(
            lambda: self.command.emit(f"griptest {self.weight.value()} {self.speed.value()}"))
        ctl.addWidget(self.run_btn)
        self.spin_btn = QtWidgets.QPushButton("Spin test")
        self.spin_btn.setToolTip("Downforce via spin-up + locked-wheel skid — spins in "
                                 "place, no runway. The cleaner grip measurement. Uses 400 dps.")
        self.spin_btn.clicked.connect(
            lambda: self.command.emit(f"spintest {self.weight.value()} 400"))
        ctl.addWidget(self.spin_btn)
        stop = QtWidgets.QPushButton("Stop")
        stop.clicked.connect(lambda: self.command.emit("stop"))
        ctl.addWidget(stop)
        ctl.addStretch(1)
        lay.addLayout(ctl)

        self.status = QtWidgets.QLabel(
            "idle — clear ~0.5 m of runway ahead; it returns to start between fan levels")
        self.status.setStyleSheet("color:#8fbf9f;")
        lay.addWidget(self.status)

        # bar chart: peak deceleration (grip) vs fan duty
        self.plot = pg.PlotWidget()
        self.plot.setBackground("#101317")
        self.plot.setLabel("left", "peak decel a_max", units="mm/s²")
        self.plot.setLabel("bottom", "fan duty", units="%")
        self.plot.showGrid(x=False, y=True, alpha=0.25)
        self.plot.setXRange(-10, 110)
        self.bars = pg.BarGraphItem(x=[], height=[], width=14, brush="#4fc3f7")
        self.plot.addItem(self.bars)
        lay.addWidget(self.plot, 1)

        # numeric results
        self.table = QtWidgets.QTableWidget(0, len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setMaximumHeight(190)
        lay.addWidget(self.table)

        self._set_mode("brake")
        self._reset()

    # -- data ---------------------------------------------------------------
    def _set_mode(self, mode: str):
        """Relabel the middle columns + Y axis for the brake vs spin test."""
        if mode == "spin":
            cols, ylabel, yunits = self.COLS_SPIN, "skid decel", "deg/s²"
        else:
            cols, ylabel, yunits = self.COLS_BRAKE, "peak decel a_max", "mm/s²"
        self.table.setHorizontalHeaderLabels(cols)
        self.plot.setLabel("left", ylabel, units=yunits)

    def _reset(self):
        self._duty: list = []
        self._amax: list = []
        self._rows: dict = {}          # duty% -> table row index
        self.table.setRowCount(0)
        self.bars.setOpts(x=[], height=[])

    def _row_for(self, duty: int) -> int:
        if duty not in self._rows:
            r = self.table.rowCount()
            self.table.insertRow(r)
            self._rows[duty] = r
            self._set(r, 0, duty)
        return self._rows[duty]

    def _set(self, r: int, c: int, text):
        item = QtWidgets.QTableWidgetItem(str(text))
        item.setTextAlignment(Qt.AlignCenter)
        self.table.setItem(r, c, item)

    def _redraw(self):
        self.bars.setOpts(x=self._duty, height=self._amax, width=14)
        if self._amax:
            self.plot.setYRange(0, max(self._amax) * 1.15)

    def feed(self, line: str):
        s = line.strip()
        if not s.startswith("GRIP,"):
            return
        parts = s.split(",")
        if len(parts) < 2:
            return
        tag = parts[1]
        try:
            if tag == "begin":
                mode = parts[4] if len(parts) >= 5 else "brake"
                self._set_mode(mode)
                self._reset()
                self.status.setText(
                    ("spinning — lock-and-skid sweep in progress …" if mode == "spin"
                     else "running — braking sweep in progress …"))
                self.status.setStyleSheet("color:#ffd66d;")
            elif tag == "summary":
                pass
            elif tag == "end":
                self.status.setText("done — a_max (grip) should rise with fan %, "
                                    "brake distance should shrink")
                self.status.setStyleSheet("color:#8fbf9f;")
            elif tag == "res":
                # GRIP,res,<duty>,<amax>,<ratio_x1000>,<df_g>,<df_pct>
                duty = int(parts[2])
                r = self._row_for(duty)
                self._set(r, 2, int(parts[3]))
                self._set(r, 3, f"{int(parts[4]) / 1000.0:.3f}")
                self._set(r, 4, f"{int(parts[6]):+d}")
                self._set(r, 5, f"{int(parts[5]):+d}")
            else:
                # per-level raw: GRIP,<duty>,<v0>,<brake_mm>,<amax>
                duty = int(parts[1])
                brake = int(parts[3])
                amax = int(parts[4])
                r = self._row_for(duty)
                self._set(r, 1, brake)
                self._set(r, 2, amax)
                if duty in self._duty:
                    self._amax[self._duty.index(duty)] = amax
                else:
                    self._duty.append(duty)
                    self._amax.append(amax)
                self._redraw()
        except (ValueError, IndexError):
            pass


# ----------------------------------------------------------------------------
# Main window
# ----------------------------------------------------------------------------
PEN_COLORS = ["#ff5555", "#55ff7f", "#5599ff", "#ffcc44", "#cc66ff", "#33dddd"]


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, host: str, port: int):
        super().__init__()
        self.setWindowTitle("NEMISIS Debug")
        self.resize(1100, 680)

        self.bridge: Optional[LinkBridge] = None
        self.store = DataStore()
        self.active_group: Optional[str] = None
        self.curves: Dict[str, pg.PlotDataItem] = {}

        # live maze map + diagnostic recorder (both in their own modules)
        self.maze = MazeModel(size=6)
        self.recorder = Recorder()
        # per-run arc telemetry logger (CSV under ./logs/arc) so turns can be
        # analysed offline instead of tuned blind off pasted console output
        self.arclog = ArcLogger()
        # whole-run telemetry logger for FAST runs (CSV under ./logs/fast) so a
        # smooth-turn crash/jizzle can be read from real data
        self.fastlog = FastLog()
        # splits a multi-run 'dump' (Search/Fast/Fastest session) into one CSV per
        # run under ./logs/runs/<ts>/ with the full sensor trace, for offline diag
        self.multirun = MultiRunFetch()
        # vacuum grip/downforce test capture (CSV under ./logs/grip): logs the
        # per-fan-level braking rows + the downforce summary the firmware emits
        self.griplog = GripLog()

        # battery monitor state (mirrors the firmware's safety FSM)
        self._batt_state: Optional[str] = None
        self._batt_mv: Optional[int] = None
        self._batt_soc: Optional[float] = None
        self._batt_blink_on = False

        self._build_ui(host, port)

        # Refresh plots on a timer instead of per-line (smooth, cheap).
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh_plot)
        self.timer.start(33)  # ~30 fps

        # Slow blinker for the CRITICAL battery indicator (started on demand).
        self.batt_blink = QtCore.QTimer(self)
        self.batt_blink.timeout.connect(self._blink_batt)

        self._set_connected(False)  # also paints the initial "unknown" indicator

    # -- UI construction ---------------------------------------------------
    def _build_ui(self, host: str, port: int):
        pg.setConfigOptions(antialias=True, background="#101317", foreground="#d8dee9")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # connection bar
        bar = QtWidgets.QHBoxLayout()
        self.host_edit = QtWidgets.QLineEdit(host)
        self.host_edit.setMaximumWidth(160)
        self.port_edit = QtWidgets.QLineEdit(str(port))
        self.port_edit.setMaximumWidth(70)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        accent(self.connect_btn, "cyan")
        self.connect_btn.clicked.connect(self._toggle_connect)
        self.wifi_setup_btn = QtWidgets.QPushButton("WiFi setup…")
        self.wifi_setup_btn.setToolTip(
            "Point the mouse at a different WiFi router without reflashing"
        )
        self.wifi_setup_btn.clicked.connect(self._wifi_setup_clicked)
        self.ota_btn = QtWidgets.QPushButton("Update firmware…")
        self.ota_btn.setToolTip(
            "Stage a firmware .bin into the mouse's external QSPI flash over WiFi"
        )
        self.ota_btn.clicked.connect(self._ota_clicked)
        self.status_lbl = QtWidgets.QLabel("● disconnected")
        # persistent battery status indicator (voltage / SoC / safety state)
        self.batt_lbl = QtWidgets.QLabel("🔋 —")
        self.batt_lbl.setToolTip("Battery monitor state reported by the mouse")
        bar.addWidget(QtWidgets.QLabel("Host"))
        bar.addWidget(self.host_edit)
        bar.addWidget(QtWidgets.QLabel("Port"))
        bar.addWidget(self.port_edit)
        bar.addWidget(self.connect_btn)
        bar.addWidget(self.wifi_setup_btn)
        bar.addWidget(self.ota_btn)
        bar.addStretch(1)
        bar.addWidget(self.batt_lbl)
        bar.addSpacing(16)
        bar.addWidget(self.status_lbl)
        root.addLayout(bar)

        # quick subsystem buttons
        btns = QtWidgets.QHBoxLayout()
        for label, group in [
            ("encoder", "enc"), ("imu", "imu"), ("sensors", "ir"),
            ("walls", "walls"), ("battery", "batt"),
        ]:
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _=False, c=label, g=group: self._start_stream(c, g))
            btns.addWidget(b)
        for label, cmd in [("buzzer", "buzzer")]:
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _=False, c=cmd: self._send(c))
            btns.addWidget(b)
        self.stop_btn = QtWidgets.QPushButton("STOP")
        accent(self.stop_btn, "red")
        self.stop_btn.clicked.connect(lambda: self._send("stop"))
        btns.addWidget(self.stop_btn)
        btns.addStretch(1)
        # diagnostic capture: write a structured JSONL log for offline analysis
        self.record_btn = QtWidgets.QPushButton("⏺ Record")
        self.record_btn.setCheckable(True)
        self.record_btn.toggled.connect(self._toggle_record)
        btns.addWidget(self.record_btn)
        root.addLayout(btns)

        # main split: plot | (console + controls)
        split = QtWidgets.QSplitter(Qt.Horizontal)
        root.addWidget(split, 1)

        # left side is a tabbed view: live plots | live maze map
        self.left_tabs = QtWidgets.QTabWidget()
        self.plot = pg.GraphicsLayoutWidget()
        self._show_placeholder()
        self.left_tabs.addTab(self.plot, "Plot")

        self.maze_panel = MazePanel(self.maze)
        self.maze_panel.command.connect(self._on_maze_command)
        self.left_tabs.addTab(self.maze_panel, "Maze")

        # vacuum grip / downforce test: launch button + bar chart + results table
        self.grip_panel = GripPanel()
        self.grip_panel.command.connect(self._send)
        self.left_tabs.addTab(self.grip_panel, "Grip")
        split.addWidget(self.left_tabs)

        right = QtWidgets.QWidget()
        rlay = QtWidgets.QVBoxLayout(right)
        rlay.setContentsMargins(0, 0, 0, 0)

        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(2000)
        # font here; colours/border come from the app theme (QPlainTextEdit rule)
        self.console.setStyleSheet("font-family:Consolas,monospace; font-size:12px;")
        rlay.addWidget(self.console, 1)
        rlay.addWidget(self._build_controls())
        rlay.addWidget(self._build_tuning())
        split.addWidget(right)
        split.setSizes([620, 460])

        # command line
        cmd = QtWidgets.QHBoxLayout()
        self.cmd_edit = QtWidgets.QLineEdit()
        self.cmd_edit.setPlaceholderText("type a console command (e.g. 'help', 'imu', 'vacuum 30') …")
        self.cmd_edit.returnPressed.connect(self._send_cmd_line)
        send = QtWidgets.QPushButton("Send")
        accent(send, "cyan")
        send.clicked.connect(self._send_cmd_line)
        cmd.addWidget(self.cmd_edit, 1)
        cmd.addWidget(send)
        root.addLayout(cmd)

    def _build_controls(self) -> QtWidgets.QWidget:
        box = QtWidgets.QGroupBox("Controls")
        lay = QtWidgets.QGridLayout(box)

        # motor row
        lay.addWidget(QtWidgets.QLabel("Motor"), 0, 0)
        self.motor_target = QtWidgets.QComboBox()
        self.motor_target.addItems(["1", "2", "b"])
        lay.addWidget(self.motor_target, 0, 1)
        self.motor_speed = QtWidgets.QSpinBox()
        self.motor_speed.setRange(-100, 100)
        self.motor_speed.setValue(40)
        self.motor_speed.setSuffix(" %")
        lay.addWidget(self.motor_speed, 0, 2)
        self.motor_ms = QtWidgets.QSpinBox()
        self.motor_ms.setRange(50, 10000)
        self.motor_ms.setSingleStep(100)
        self.motor_ms.setValue(800)
        self.motor_ms.setSuffix(" ms")
        lay.addWidget(self.motor_ms, 0, 3)
        self.motor_run_btn = QtWidgets.QPushButton("Run")
        self.motor_run_btn.clicked.connect(self._run_motor)
        lay.addWidget(self.motor_run_btn, 0, 4)

        # vacuum row
        lay.addWidget(QtWidgets.QLabel("Vacuum"), 1, 0)
        self.vac_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.vac_slider.setRange(0, 100)
        self.vac_lbl = QtWidgets.QLabel("0 %")
        self.vac_slider.valueChanged.connect(lambda v: self.vac_lbl.setText(f"{v} %"))
        self.vac_slider.sliderReleased.connect(
            lambda: self._send(f"vacuum {self.vac_slider.value()}")
        )
        lay.addWidget(self.vac_slider, 1, 1, 1, 3)
        lay.addWidget(self.vac_lbl, 1, 4)

        # stream-rate row: sets the live-stream sample rate on the mouse
        lay.addWidget(QtWidgets.QLabel("Rate"), 2, 0)
        self.rate_spin = QtWidgets.QSpinBox()
        self.rate_spin.setRange(1, 200)
        self.rate_spin.setValue(20)
        self.rate_spin.setSuffix(" Hz")
        lay.addWidget(self.rate_spin, 2, 1)
        rate_set = QtWidgets.QPushButton("Set rate")
        rate_set.clicked.connect(lambda: self._send(f"rate {self.rate_spin.value()}"))
        lay.addWidget(rate_set, 2, 2)
        rate_def = QtWidgets.QPushButton("Default")
        rate_def.clicked.connect(lambda: self._send("rate default"))
        lay.addWidget(rate_def, 2, 3)

        # IR acquisition timing (bench diagnostic): emitter settle + oversampling.
        # Sweep settle to find where the weak IR signal stops growing.
        lay.addWidget(QtWidgets.QLabel("IR settle"), 3, 0)
        self.irsettle_spin = QtWidgets.QSpinBox()
        self.irsettle_spin.setRange(20, 5000)
        self.irsettle_spin.setSingleStep(50)
        self.irsettle_spin.setValue(150)
        self.irsettle_spin.setSuffix(" µs")
        self.irsettle_spin.setToolTip("Emitter settle before sampling — raise it until the 'walls' "
                                      "values stop growing (the phototransistor's response time)")
        lay.addWidget(self.irsettle_spin, 3, 1)
        self.irsamp_spin = QtWidgets.QSpinBox()
        self.irsamp_spin.setRange(1, 64)
        self.irsamp_spin.setValue(8)
        self.irsamp_spin.setSuffix(" smp")
        self.irsamp_spin.setToolTip("ADC oversampling per phase — more = less noise, slower sweep")
        lay.addWidget(self.irsamp_spin, 3, 2)
        irset_btn = QtWidgets.QPushButton("Set IR")
        irset_btn.clicked.connect(self._apply_irset)
        lay.addWidget(irset_btn, 3, 3)

        # Wall-sensor calibration. Place the mouse on the bench, capture each
        # canonical position, then Save (its own flash page — does NOT touch the
        # PID tuning profiles). Order: Dark first, then Side / Front.
        lay.addWidget(QtWidgets.QLabel("IR cal"), 4, 0)
        cal_row = QtWidgets.QWidget()
        cal_lay = QtWidgets.QHBoxLayout(cal_row)
        cal_lay.setContentsMargins(0, 0, 0, 0)
        cal_lay.setSpacing(3)
        for label, cmd, tip in (
            ("Dark",  "ircal dark",  "Open cell, nothing near — the no-wall floor. Do this first."),
            ("Side",  "ircal side",  "Centred between a left and right wall — sets L_LM/R_RM reference."),
            ("Front", "ircal front", "At the stop point facing a front wall — sets L_F/R_F reference."),
            ("Save",  "ircsave",     "Commit calibration to flash (stop any run first)."),
            ("Show",  "ircshow",     "Print the stored calibration table."),
        ):
            b = QtWidgets.QPushButton(label)
            b.setToolTip(tip)
            b.clicked.connect(lambda _=False, c=cmd: self._send(c))
            cal_lay.addWidget(b)
        lay.addWidget(cal_row, 4, 1, 1, 3)
        return box

    def _build_tuning(self) -> QtWidgets.QWidget:
        """PID tuning panel: pick a loop, fine-tune Kp/Ki/Kd, drive a step, and
        stream that loop's response onto the plot."""
        box = QtWidgets.QGroupBox("PID tuning")
        lay = QtWidgets.QGridLayout(box)

        # loop selector
        lay.addWidget(QtWidgets.QLabel("Loop"), 0, 0)
        self.pid_loop = QtWidgets.QComboBox()
        self.pid_loop.addItems(PID_LOOPS)
        self.pid_loop.currentTextChanged.connect(self._load_gains_into_ui)
        lay.addWidget(self.pid_loop, 0, 1)

        # velocity feedforward (global, not per-loop) — duty per cps
        lay.addWidget(QtWidgets.QLabel("FF"), 0, 2)
        self.ff_spin = QtWidgets.QDoubleSpinBox()
        self.ff_spin.setDecimals(3)
        self.ff_spin.setRange(0.0, 10.0)
        self.ff_spin.setSingleStep(0.01)
        self.ff_spin.setValue(0.15)
        self.ff_spin.setMaximumWidth(110)
        self.ff_spin.setToolTip("Velocity feedforward: motor duty applied per cps of target")
        self.ff_spin.valueChanged.connect(self._maybe_autosend_ff)
        lay.addWidget(self.ff_spin, 0, 3)
        ff_btn = QtWidgets.QPushButton("Set FF")
        ff_btn.clicked.connect(self._apply_ff)
        lay.addWidget(ff_btn, 0, 4, 1, 2)

        # cache of last-known gains per loop (kept in sync via GAIN_RX)
        self._gains = {k: list(v) for k, v in PID_DEFAULTS.items()}

        # Kp / Ki / Kd spin boxes for manual fine-tuning
        def gain_spin():
            s = QtWidgets.QDoubleSpinBox()
            s.setDecimals(3)
            s.setRange(0.0, 100000.0)
            s.setSingleStep(0.1)
            s.setMaximumWidth(110)
            s.valueChanged.connect(self._maybe_autosend)
            return s

        self.kp_spin, self.ki_spin, self.kd_spin = gain_spin(), gain_spin(), gain_spin()
        for col, (lbl, sp) in enumerate([("Kp", self.kp_spin),
                                         ("Ki", self.ki_spin),
                                         ("Kd", self.kd_spin)]):
            lay.addWidget(QtWidgets.QLabel(lbl), 1, col * 2)
            lay.addWidget(sp, 1, col * 2 + 1)

        # apply controls
        self.pid_auto = QtWidgets.QCheckBox("auto-apply")
        self.pid_auto.setToolTip("Send gains to the mouse as you edit them")
        lay.addWidget(self.pid_auto, 2, 0, 1, 2)
        set_btn = QtWidgets.QPushButton("Set gains")
        set_btn.clicked.connect(self._apply_gains)
        lay.addWidget(set_btn, 2, 2, 1, 2)
        get_btn = QtWidgets.QPushButton("Get")
        get_btn.setToolTip("Ask the mouse for its current gains + ramp rates")
        get_btn.clicked.connect(self._get_all)
        lay.addWidget(get_btn, 2, 4, 1, 2)

        # step / run controls
        lay.addWidget(QtWidgets.QLabel("Speed"), 3, 0)
        self.pid_speed = QtWidgets.QSpinBox()
        self.pid_speed.setRange(0, 60000)
        self.pid_speed.setSingleStep(500)
        self.pid_speed.setValue(4000)
        self.pid_speed.setSuffix(" cps")
        self.pid_speed.setToolTip("Target wheel speed, encoder counts/second")
        lay.addWidget(self.pid_speed, 3, 1)
        drive_btn = QtWidgets.QPushButton("Drive")
        drive_btn.setStyleSheet("font-weight:bold; color:#55ff7f;")
        drive_btn.clicked.connect(self._drive_step)
        lay.addWidget(drive_btn, 3, 2)
        stop2 = QtWidgets.QPushButton("Stop")
        stop2.setStyleSheet("font-weight:bold; color:#ff6666;")
        stop2.clicked.connect(lambda: self._send("stop"))
        lay.addWidget(stop2, 3, 3)
        stream_btn = QtWidgets.QPushButton("Plot this loop")
        stream_btn.setToolTip("Stream the selected loop's telemetry to the graph")
        stream_btn.clicked.connect(self._stream_loop)
        lay.addWidget(stream_btn, 3, 4, 1, 2)

        # auto-tune: one click finds vel Kp/Ki via a relay test (no manual trial)
        self.autotune_btn = QtWidgets.QPushButton("Auto-tune vel")
        self.autotune_btn.setStyleSheet("font-weight:bold; color:#ffcc55;")
        self.autotune_btn.setToolTip(
            "Relay auto-tune of the velocity loop at the Speed above.\n"
            "Robot wobbles forward while it measures, then applies the gains.\n"
            "Keep it on the floor with space ahead.")
        self.autotune_btn.clicked.connect(self._autotune)
        lay.addWidget(self.autotune_btn, 4, 0, 1, 2)
        self.autotune_lbl = QtWidgets.QLabel("")
        self.autotune_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.autotune_lbl, 4, 2, 1, 4)

        # distance-limited run: drive at Speed, auto-stop after N mm (safe way to
        # tune at real run speeds without the robot bolting across the floor)
        lay.addWidget(QtWidgets.QLabel("Dist"), 5, 0)
        self.move_dist = QtWidgets.QSpinBox()
        self.move_dist.setRange(10, 10000)
        self.move_dist.setSingleStep(100)
        self.move_dist.setValue(1000)
        self.move_dist.setSuffix(" mm")
        self.move_dist.setToolTip("Run this far at the Speed above, then ease to a stop")
        lay.addWidget(self.move_dist, 5, 1)
        move_btn = QtWidgets.QPushButton("Move")
        move_btn.setStyleSheet("font-weight:bold; color:#55ccff;")
        move_btn.setToolTip("Closed-loop straight run that auto-stops after Dist mm")
        move_btn.clicked.connect(self._move_step)
        lay.addWidget(move_btn, 5, 2)
        self.move_lbl = QtWidgets.QLabel("")
        self.move_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.move_lbl, 5, 3, 1, 3)

        # acceleration / deceleration ramp (cps/s). Separate rates so you can,
        # e.g., accelerate gently but brake hard. counts/mm comes from the boot
        # log so we can show the equivalent in m/s^2.
        self.counts_per_mm = 24.514     # default; updated from the geom log line
        lay.addWidget(QtWidgets.QLabel("Accel"), 6, 0)
        self.accel_spin = self._ramp_spin()
        lay.addWidget(self.accel_spin, 6, 1)
        lay.addWidget(QtWidgets.QLabel("Decel"), 6, 2)
        self.decel_spin = self._ramp_spin()
        lay.addWidget(self.decel_spin, 6, 3)
        accel_btn = QtWidgets.QPushButton("Set ramp")
        accel_btn.clicked.connect(self._apply_accel)
        lay.addWidget(accel_btn, 6, 4, 1, 2)
        self.accel_lbl = QtWidgets.QLabel("")
        self.accel_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.accel_lbl, 7, 0, 1, 6)
        self._update_accel_label()

        # velocity-feedback filter: window (ms) + EMA alpha. Heavier filtering
        # (longer window / smaller alpha) smooths the measured-velocity noise the
        # loop reacts to, trading a little phase lag.
        lay.addWidget(QtWidgets.QLabel("V-filt win"), 8, 0)
        self.vwin_spin = QtWidgets.QSpinBox()
        self.vwin_spin.setRange(2, 50)
        self.vwin_spin.setValue(20)
        self.vwin_spin.setSuffix(" ms")
        self.vwin_spin.setToolTip("Velocity sliding-window length — longer = smoother, more lag")
        self.vwin_spin.valueChanged.connect(self._maybe_autosend_vfilt)
        lay.addWidget(self.vwin_spin, 8, 1)
        lay.addWidget(QtWidgets.QLabel("alpha"), 8, 2)
        self.valpha_spin = QtWidgets.QDoubleSpinBox()
        self.valpha_spin.setDecimals(2)
        self.valpha_spin.setRange(0.02, 1.0)
        self.valpha_spin.setSingleStep(0.05)
        self.valpha_spin.setValue(0.30)
        self.valpha_spin.setToolTip("EMA smoothing — smaller = smoother, more lag")
        self.valpha_spin.valueChanged.connect(self._maybe_autosend_vfilt)
        lay.addWidget(self.valpha_spin, 8, 3)
        vfilt_btn = QtWidgets.QPushButton("Set filter")
        vfilt_btn.clicked.connect(self._apply_vfilt)
        lay.addWidget(vfilt_btn, 8, 4, 1, 2)

        # ---- persistent tuning profiles (stored in the robot's flash) -------
        # One full tuning set per speed regime, surviving a power cycle. Switch
        # the active profile, Save the live values into it, or recall (Load) it.
        lay.addWidget(QtWidgets.QLabel("Profile"), 9, 0)
        self.profile_combo = QtWidgets.QComboBox()
        self.profile_combo.addItems(PROFILES)
        self.profile_combo.setToolTip("Active tuning profile on the robot — switching applies it live")
        self.profile_combo.activated.connect(self._apply_profile)
        lay.addWidget(self.profile_combo, 9, 1)
        save_btn = QtWidgets.QPushButton("Save to bot")
        save_btn.setToolTip("Store the current live tuning into this profile in flash (robot must be stopped)")
        save_btn.clicked.connect(self._save_profile)
        lay.addWidget(save_btn, 9, 2, 1, 2)
        load_btn = QtWidgets.QPushButton("Load from bot")
        load_btn.setToolTip("Recall this profile's stored tuning into the live controller")
        load_btn.clicked.connect(self._load_profile)
        lay.addWidget(load_btn, 9, 4, 1, 2)
        self.profile_lbl = QtWidgets.QLabel("")
        self.profile_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.profile_lbl, 10, 0, 1, 6)

        # ---- pivot turn-in-place --------------------------------------------
        # Fixed 90/180 turns closed on the gyro. Angle picks the turn; Rate/Accel
        # shape the heading-setpoint slew (the 'head' PID does the rotating).
        lay.addWidget(QtWidgets.QLabel("Turn"), 11, 0)
        self.turn_angle = QtWidgets.QComboBox()
        # label -> signed degrees sent to the firmware (l = +90, r = -90)
        self._turn_opts = [("Left 90°", 90), ("Right 90°", -90), ("180°", 180)]
        for label, _deg in self._turn_opts:
            self.turn_angle.addItem(label)
        self.turn_angle.setToolTip("Pivot in place by this angle (gyro-closed)")
        lay.addWidget(self.turn_angle, 11, 1)
        turn_btn = QtWidgets.QPushButton("Turn")
        turn_btn.setStyleSheet("font-weight:bold; color:#ffaaff;")
        turn_btn.setToolTip("Pivot in place by the selected angle, eases to a stop")
        turn_btn.clicked.connect(self._turn_step)
        lay.addWidget(turn_btn, 11, 2)
        self.turn_lbl = QtWidgets.QLabel("")
        self.turn_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.turn_lbl, 11, 3, 1, 3)

        lay.addWidget(QtWidgets.QLabel("Turn rate"), 12, 0)
        self.trate_spin = QtWidgets.QSpinBox()
        self.trate_spin.setRange(20, 1200)
        self.trate_spin.setSingleStep(20)
        self.trate_spin.setValue(300)
        self.trate_spin.setSuffix(" °/s")
        self.trate_spin.setToolTip("Pivot slew-rate cap — how fast the turn spins")
        self.trate_spin.valueChanged.connect(self._maybe_autosend_tcfg)
        lay.addWidget(self.trate_spin, 12, 1)
        lay.addWidget(QtWidgets.QLabel("accel"), 12, 2)
        self.taccel_spin = QtWidgets.QSpinBox()
        self.taccel_spin.setRange(100, 10000)
        self.taccel_spin.setSingleStep(100)
        self.taccel_spin.setValue(1800)
        self.taccel_spin.setSuffix(" °/s²")
        self.taccel_spin.setToolTip("Pivot slew accel/decel — higher = snappier in/out of the turn")
        self.taccel_spin.valueChanged.connect(self._maybe_autosend_tcfg)
        lay.addWidget(self.taccel_spin, 12, 3)
        tcfg_btn = QtWidgets.QPushButton("Set turn")
        tcfg_btn.clicked.connect(self._apply_tcfg)
        lay.addWidget(tcfg_btn, 12, 4, 1, 2)

        # IR corridor centring (follow): a checkable toggle + centring gain.
        # Needs 'ircal side' done; with no walls it falls back to heading-hold.
        lay.addWidget(QtWidgets.QLabel("Follow"), 13, 0)
        self.follow_btn = QtWidgets.QPushButton("Centre: OFF")
        self.follow_btn.setCheckable(True)
        self.follow_btn.setToolTip("IR corridor centring on forward runs (needs ircal side)")
        self.follow_btn.toggled.connect(self._on_follow_toggle)
        lay.addWidget(self.follow_btn, 13, 1, 1, 2)
        lay.addWidget(QtWidgets.QLabel("gain"), 13, 3)
        self.fgain_spin = QtWidgets.QDoubleSpinBox()
        self.fgain_spin.setRange(-1.0, 1.0)
        self.fgain_spin.setSingleStep(0.01)
        self.fgain_spin.setDecimals(3)
        self.fgain_spin.setValue(0.05)
        self.fgain_spin.setToolTip("Heading-offset °/lateral-count. Raise until it centres "
                                   "crisply without weaving; negative flips direction.")
        lay.addWidget(self.fgain_spin, 13, 4)
        fcfg_btn = QtWidgets.QPushButton("Set")
        fcfg_btn.clicked.connect(lambda: self._send(f"fcfg {self.fgain_spin.value():.3f}"))
        lay.addWidget(fcfg_btn, 13, 5)

        # Cell-based advance (Navigator primitive): drive N maze cells non-stop.
        # Reports CELL,idx,L,R,F per cell; stops at the end or at a front wall.
        lay.addWidget(QtWidgets.QLabel("Advance"), 14, 0)
        self.adv_cells = QtWidgets.QSpinBox()
        self.adv_cells.setRange(1, 256)
        self.adv_cells.setValue(1)
        self.adv_cells.setSuffix(" cells")
        self.adv_cells.setToolTip("Drive this many maze cells in one continuous run "
                                  "at the Speed above (needs ircal side/front for walls)")
        lay.addWidget(self.adv_cells, 14, 1)
        adv_btn = QtWidgets.QPushButton("Advance")
        adv_btn.setStyleSheet("font-weight:bold; color:#55ccff;")
        adv_btn.setToolTip("Non-stop multi-cell run; eases to a stop at the end or a front wall")
        adv_btn.clicked.connect(self._advance_step)
        lay.addWidget(adv_btn, 14, 2)
        self.adv_lbl = QtWidgets.QLabel("")
        self.adv_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.adv_lbl, 14, 3)
        lay.addWidget(QtWidgets.QLabel("cell"), 14, 4)
        self.cell_mm = QtWidgets.QSpinBox()
        self.cell_mm.setRange(20, 1000)
        self.cell_mm.setSingleStep(10)
        self.cell_mm.setValue(180)
        self.cell_mm.setSuffix(" mm")
        self.cell_mm.setToolTip("Maze cell pitch used by Advance (classic micromouse = 180 mm)")
        self.cell_mm.editingFinished.connect(lambda: self._send(f"cell {self.cell_mm.value()}"))
        lay.addWidget(self.cell_mm, 14, 5)

        # Nav sequencer (path): a token string run as a back-to-back sequence.
        # F<n>=forward n cells, L/R=turn 90, U=180. The maze solver will drive
        # the same queue later; this is the manual front-end.
        lay.addWidget(QtWidgets.QLabel("Path"), 15, 0)
        self.path_edit = QtWidgets.QLineEdit()
        self.path_edit.setPlaceholderText("F2 L F1 R F3")
        self.path_edit.setToolTip("Move sequence: F<n>=forward n cells, L/R=turn 90°, "
                                  "U=180°. Runs straights non-stop, pivots at turns, "
                                  "at the Speed above.")
        self.path_edit.returnPressed.connect(self._run_path)
        lay.addWidget(self.path_edit, 15, 1, 1, 3)
        path_btn = QtWidgets.QPushButton("Run path")
        path_btn.setStyleSheet("font-weight:bold; color:#55ccff;")
        path_btn.setToolTip("Queue + run the move sequence ('stop' aborts)")
        path_btn.clicked.connect(self._run_path)
        lay.addWidget(path_btn, 15, 4)
        self.path_lbl = QtWidgets.QLabel("")
        self.path_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.path_lbl, 15, 5)

        # ---- smooth (arc) turn ----------------------------------------------
        # Curve forward through the angle along a circle of Radius at Speed,
        # instead of pivoting in place. The fast-run turn primitive; bench-test
        # the geometry here (firmware 'arc <deg> <cps> <r_mm>', stops on finish).
        lay.addWidget(QtWidgets.QLabel("Arc"), 16, 0)
        self.arc_angle = QtWidgets.QComboBox()
        self._arc_opts = [("Left 90°", 90), ("Right 90°", -90), ("Left 45°", 45),
                          ("Right 45°", -45)]
        for label, _deg in self._arc_opts:
            self.arc_angle.addItem(label)
        self.arc_angle.setToolTip("Smooth-curve through this angle while driving forward")
        lay.addWidget(self.arc_angle, 16, 1)
        self.arc_cps = QtWidgets.QSpinBox()
        self.arc_cps.setRange(2000, 20000)
        self.arc_cps.setSingleStep(500)
        self.arc_cps.setValue(12000)
        self.arc_cps.setSuffix(" cps")
        self.arc_cps.setToolTip("Forward speed held through the arc (~24500 cps = 1 m/s)")
        lay.addWidget(self.arc_cps, 16, 2)
        self.arc_radius = QtWidgets.QSpinBox()
        self.arc_radius.setRange(40, 250)
        self.arc_radius.setSingleStep(5)
        self.arc_radius.setValue(110)
        self.arc_radius.setSuffix(" mm")
        self.arc_radius.setToolTip("Turn radius — smaller = tighter. ~half a cell is typical")
        lay.addWidget(self.arc_radius, 16, 3)
        self.arc_exit = QtWidgets.QSpinBox()
        self.arc_exit.setRange(0, 1000)
        self.arc_exit.setSingleStep(50)
        self.arc_exit.setValue(0)
        self.arc_exit.setSuffix(" mm")
        self.arc_exit.setToolTip("Exit straight: 0 = stop after the arc (geometry test); "
                                 ">0 = drive on this far in the new heading WITHOUT stopping "
                                 "(smooth turn into a corridor)")
        lay.addWidget(self.arc_exit, 16, 4)
        arc_btn = QtWidgets.QPushButton("Arc")
        arc_btn.setStyleSheet("font-weight:bold; color:#ffaaff;")
        arc_btn.setToolTip("Smooth turn: curve forward through the angle; stop or drive on per Exit")
        arc_btn.clicked.connect(self._arc_step)
        lay.addWidget(arc_btn, 16, 5)
        self.arc_lbl = QtWidgets.QLabel("")
        self.arc_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.arc_lbl, 16, 6)

        # ---- smooth-arc tuning (the feed-forward turn controller) ------------
        # FF draws the arc open-loop (sets accuracy); Kp gently nulls drift; Kd is
        # the damping (D on the FILTERED heading error) that removes hunt/overshoot
        # WITHOUT the gyro buzz. Tuning recipe: set FF for accuracy, raise Kd to
        # smooth. Auto-send mirrors the PID panel's checkbox.
        # FF is PER TURN DIRECTION (a real mouse rotates more easily one way): tune
        # L and R separately so both 90° turns complete. L is typically lower.
        lay.addWidget(QtWidgets.QLabel("Arc FF L/R"), 17, 0)
        self.arcffl_spin = QtWidgets.QDoubleSpinBox()
        self.arcffl_spin.setDecimals(1)
        self.arcffl_spin.setRange(0.0, 60.0)
        self.arcffl_spin.setSingleStep(0.5)
        self.arcffl_spin.setValue(25.5)
        self.arcffl_spin.setToolTip("LEFT-turn curvature feed-forward (cps per °/s). Sets left-turn "
                                    "ACCURACY — raise if it under-turns, lower if it over-turns")
        self.arcffl_spin.valueChanged.connect(self._maybe_autosend_arctune)
        lay.addWidget(self.arcffl_spin, 17, 1)
        self.arcffr_spin = QtWidgets.QDoubleSpinBox()
        self.arcffr_spin.setDecimals(1)
        self.arcffr_spin.setRange(0.0, 60.0)
        self.arcffr_spin.setSingleStep(0.5)
        self.arcffr_spin.setValue(27.5)
        self.arcffr_spin.setToolTip("RIGHT-turn curvature feed-forward (cps per °/s)")
        self.arcffr_spin.valueChanged.connect(self._maybe_autosend_arctune)
        lay.addWidget(self.arcffr_spin, 17, 2)
        lay.addWidget(QtWidgets.QLabel("Kp"), 17, 3)
        self.arckp_spin = QtWidgets.QSpinBox()
        self.arckp_spin.setRange(0, 400)
        self.arckp_spin.setSingleStep(10)
        self.arckp_spin.setValue(80)
        self.arckp_spin.setToolTip("Gentle heading-correction gain (cps per ° error). "
                                   "Only nulls slow drift — leave low to avoid hunting")
        self.arckp_spin.valueChanged.connect(self._maybe_autosend_arctune)
        lay.addWidget(self.arckp_spin, 17, 4)
        lay.addWidget(QtWidgets.QLabel("Kd"), 17, 5)
        self.arckd_spin = QtWidgets.QSpinBox()
        self.arckd_spin.setRange(0, 200)
        self.arckd_spin.setSingleStep(2)
        self.arckd_spin.setValue(0)
        self.arckd_spin.setToolTip("Arc DAMPING (D on the filtered error). Raise to kill "
                                   "turn overshoot/hunt; filtered, so it won't bring back the buzz")
        self.arckd_spin.valueChanged.connect(self._maybe_autosend_arctune)
        lay.addWidget(self.arckd_spin, 17, 6)
        arctune_btn = QtWidgets.QPushButton("Set arc")
        arctune_btn.clicked.connect(self._apply_arctune)
        lay.addWidget(arctune_btn, 17, 7)

        # ---- heading D-term low-pass (straight/pivot buzz filter) ------------
        # Separate from the arc: this filters the heading PID's derivative on
        # straight runs and pivots. Lower = smoother (less motor buzz), more lag.
        lay.addWidget(QtWidgets.QLabel("Head D-filt"), 18, 0)
        self.dfilt_spin = QtWidgets.QDoubleSpinBox()
        self.dfilt_spin.setDecimals(3)
        self.dfilt_spin.setRange(0.01, 1.0)
        self.dfilt_spin.setSingleStep(0.01)
        self.dfilt_spin.setValue(0.08)
        self.dfilt_spin.setToolTip("Heading derivative low-pass for straight-hold/pivot. "
                                   "Lower = smoother / less buzz, more lag. 1.0 = raw gyro D")
        self.dfilt_spin.valueChanged.connect(self._maybe_autosend_dfilt)
        lay.addWidget(self.dfilt_spin, 18, 1)
        dfilt_btn = QtWidgets.QPushButton("Set D-filt")
        dfilt_btn.clicked.connect(self._apply_dfilt)
        lay.addWidget(dfilt_btn, 18, 2, 1, 2)

        # ---- flow (maze) smooth turn -----------------------------------------
        # The full maze turn: lead-in straight (Entry) -> arc -> lead-out straight
        # (Exit), then stop centred. Uses the Arc row's Angle / Speed / Radius; the
        # Entry lead-in is what starts the curve a radius BEFORE the corner centre so
        # the mouse lands centred on the next corridor. This is the fast-run
        # 'Fastest' profile turn primitive - bench-test the geometry here.
        lay.addWidget(QtWidgets.QLabel("Flow Turn"), 19, 0)
        self.ft_entry = QtWidgets.QSpinBox()
        self.ft_entry.setRange(0, 1000)
        self.ft_entry.setSingleStep(10)
        self.ft_entry.setValue(90)
        self.ft_entry.setSuffix(" in")
        self.ft_entry.setToolTip("Lead-in straight before the curve (mm). ~ cell pitch − radius")
        lay.addWidget(self.ft_entry, 19, 1)
        self.ft_exit = QtWidgets.QSpinBox()
        self.ft_exit.setRange(10, 1000)
        self.ft_exit.setSingleStep(10)
        self.ft_exit.setValue(90)
        self.ft_exit.setSuffix(" out")
        self.ft_exit.setToolTip("Lead-out straight after the curve (mm). ~ cell pitch − radius")
        lay.addWidget(self.ft_exit, 19, 2)
        ft_btn = QtWidgets.QPushButton("Flow Turn")
        ft_btn.setStyleSheet("font-weight:bold; color:#ffaaff;")
        ft_btn.setToolTip("Maze smooth turn using the Arc row's Angle/Speed/Radius "
                          "+ this Entry/Exit; stops centred for a geometry test")
        ft_btn.clicked.connect(self._flowturn_step)
        lay.addWidget(ft_btn, 19, 3)
        self.ft_lbl = QtWidgets.QLabel("")
        self.ft_lbl.setStyleSheet("color:#aaaaaa;")
        lay.addWidget(self.ft_lbl, 19, 4, 1, 3)

        self.move_run_btn = move_btn     # mirror battery lockout too
        self.adv_run_btn = adv_btn      # mirror battery lockout too
        self.path_run_btn = path_btn    # mirror battery lockout too
        self.drive_run_btn = drive_btn  # mirror battery lockout like the motor btn
        self.turn_run_btn = turn_btn    # mirror battery lockout too
        self.arc_run_btn = arc_btn      # mirror battery lockout too
        self.ft_run_btn = ft_btn        # mirror battery lockout too
        self._load_gains_into_ui(self.pid_loop.currentText())
        return box

    def _ramp_spin(self) -> QtWidgets.QSpinBox:
        s = QtWidgets.QSpinBox()
        s.setRange(1000, 400000)
        s.setSingleStep(5000)
        s.setValue(30000)
        s.setSuffix(" cps/s")
        s.setMaximumWidth(120)
        s.valueChanged.connect(self._update_accel_label)
        s.valueChanged.connect(self._maybe_autosend_accel)
        return s

    # -- tuning helpers ----------------------------------------------------
    def _load_gains_into_ui(self, loop: str):
        kp, ki, kd = self._gains.get(loop, (0.0, 0.0, 0.0))
        for sp, val in ((self.kp_spin, kp), (self.ki_spin, ki), (self.kd_spin, kd)):
            sp.blockSignals(True)   # don't trigger auto-apply while loading
            sp.setValue(val)
            sp.blockSignals(False)

    def _apply_gains(self):
        loop = self.pid_loop.currentText()
        kp, ki, kd = self.kp_spin.value(), self.ki_spin.value(), self.kd_spin.value()
        self._gains[loop] = [kp, ki, kd]
        self._send(f"pid {loop} {kp:g} {ki:g} {kd:g}")

    def _maybe_autosend(self):
        if getattr(self, "pid_auto", None) and self.pid_auto.isChecked():
            self._apply_gains()

    def _apply_ff(self):
        self._send(f"ff {self.ff_spin.value():g}")

    def _maybe_autosend_ff(self):
        if getattr(self, "pid_auto", None) and self.pid_auto.isChecked():
            self._apply_ff()

    def _drive_step(self):
        # make sure the loop we're watching is what we plot, then start the run
        self._stream_loop()
        self._send(f"drive {self.pid_speed.value()}")

    def _get_all(self):
        # pull gains, feedforward (shown by 'pid'), ramp rates, vel filter, turn
        self._send("pid")
        self._send("accel")
        self._send("vfilt")
        self._send("tcfg")
        # smooth-arc tuning + heading D-filter
        self._send("arcff")
        self._send("arckp")
        self._send("arckd")
        self._send("dfilt")

    def _apply_vfilt(self):
        self._send(f"vfilt {self.vwin_spin.value()} {self.valpha_spin.value():g}")

    def _apply_arctune(self):
        # push the smooth-arc gains: FF per direction (L/R), then Kp / Kd
        self._send(f"arcff l {self.arcffl_spin.value():g}")
        self._send(f"arcff r {self.arcffr_spin.value():g}")
        self._send(f"arckp {self.arckp_spin.value()}")
        self._send(f"arckd {self.arckd_spin.value()}")

    def _maybe_autosend_arctune(self):
        if getattr(self, "pid_auto", None) and self.pid_auto.isChecked():
            self._apply_arctune()

    def _apply_dfilt(self):
        self._send(f"dfilt {self.dfilt_spin.value():g}")

    def _maybe_autosend_dfilt(self):
        if getattr(self, "pid_auto", None) and self.pid_auto.isChecked():
            self._apply_dfilt()

    def _apply_irset(self):
        self._send(f"irset {self.irsettle_spin.value()} {self.irsamp_spin.value()}")

    def _maybe_autosend_vfilt(self):
        if getattr(self, "pid_auto", None) and self.pid_auto.isChecked():
            self._apply_vfilt()

    def _sync_from_bot(self):
        # pull the robot's current state so the panel mirrors it (used on
        # connect): which profile is active + all the live tuning values.
        self._send("profile")   # -> PROFILE,idx,name : sync the combo + label
        self._get_all()         # -> pid/ff/accel/vfilt : sync the spin boxes
        self._send("irset")     # -> IR settle/samples : sync the IR spin boxes
        self._send("follow")    # -> follow state + gain : sync the centring toggle
        self._send("cell")      # -> cell pitch mm : sync the advance cell-size spin

    # -- tuning profiles (flash) -------------------------------------------
    def _current_profile(self) -> str:
        return self.profile_combo.currentText().lower()

    def _apply_profile(self, _idx=None):
        # user picked a profile in the combo -> make it active on the robot
        # (the firmware echoes its gains/ff/accel/vfilt, which re-syncs the UI)
        self._send(f"profile {self._current_profile()}")

    def _save_profile(self):
        name = self.profile_combo.currentText()
        self.profile_lbl.setText(f"saving {name}…")
        self._send(f"save {self._current_profile()}")

    def _load_profile(self):
        self._send(f"load {self._current_profile()}")

    def _apply_accel(self):
        self._send(f"accel {self.accel_spin.value()} {self.decel_spin.value()}")

    def _maybe_autosend_accel(self):
        if getattr(self, "pid_auto", None) and self.pid_auto.isChecked():
            self._apply_accel()

    def _update_accel_label(self):
        cpm = getattr(self, "counts_per_mm", 24.514) or 24.514
        a = self.accel_spin.value() / cpm / 1000.0   # cps/s -> mm/s^2 -> m/s^2
        d = self.decel_spin.value() / cpm / 1000.0
        self.accel_lbl.setText(f"≈ {a:.2f} m/s² accel  /  {d:.2f} m/s² decel")

    def _move_step(self):
        # plot the loop, then run a fixed distance at the Speed value
        self._stream_loop()
        self.move_lbl.setText(f"running {self.move_dist.value()} mm…")
        self._send(f"move {self.move_dist.value()} {self.pid_speed.value()}")

    def _advance_step(self):
        # watch the heading loop (so centring is visible), then run N cells non-stop
        self._stream_loop()
        n = self.adv_cells.value()
        self.adv_lbl.setText(f"advancing {n} cell{'s' if n != 1 else ''}…")
        self._send(f"advance {n} {self.pid_speed.value()}")

    def _run_path(self):
        # run a token sequence (F2 L F1 R ...) back-to-back at the Speed value
        tokens = self.path_edit.text().strip()
        if not tokens:
            return
        self._stream_loop()              # watch the heading loop as it runs
        self.path_lbl.setText("running…")
        self._send(f"path {self.pid_speed.value()} {tokens}")

    def _turn_step(self):
        # watch the heading loop while the pivot runs, then kick off the turn
        deg = self._turn_opts[self.turn_angle.currentIndex()][1]
        self._send("tlm head on")
        self._set_plot_group("tlm")
        self.turn_lbl.setText(f"turning {deg}°…")
        self._send(f"turn {deg}")

    def _arc_step(self):
        # smooth (arc) turn: watch the heading loop, then curve forward through it
        deg = self._arc_opts[self.arc_angle.currentIndex()][1]
        cps = self.arc_cps.value()
        radius = self.arc_radius.value()
        exit_mm = self.arc_exit.value()
        # open a fresh CSV log for this run (records params, gains, every tick,
        # and the result) - never let a logging hiccup block the command
        try:
            p = self.arclog.start(
                params={"deg": deg, "cps": cps, "radius": radius, "exit": exit_mm},
                gains={"arcffl": self.arcffl_spin.value(),
                       "arcffr": self.arcffr_spin.value(),
                       "arckp": self.arckp_spin.value(),
                       "arckd": self.arckd_spin.value()})
            self._log(f"[arc log → {p.name}]")
        except Exception as e:                           # noqa: BLE001
            self._log(f"[arc log: could not start: {e}]")
        # query the firmware's ACTUAL live gains so the log records the truth (the
        # UI spinboxes can be stale after a reflash); the echoes are captured into
        # the log - and also re-sync the spinboxes via _parse_arctune
        self._send("arcff")
        self._send("arckp")
        self._send("arckd")
        self._send("arcgkd")
        self._send("tlm head on")
        self._set_plot_group("tlm")
        tail = f" +{exit_mm}mm" if exit_mm else ""
        self.arc_lbl.setText(f"arc {deg}° R{radius}{tail}…")
        self._send(f"arc {deg} {cps} {radius} {exit_mm}")

    def _flowturn_step(self):
        # maze smooth turn: Arc row's angle/speed/radius + this entry/exit lead-ins.
        deg = self._arc_opts[self.arc_angle.currentIndex()][1]
        cps = self.arc_cps.value()
        radius = self.arc_radius.value()
        entry = self.ft_entry.value()
        exit_mm = self.ft_exit.value()
        # reuse the arc CSV logger (same heading-loop TLM); the exit leg fires
        # MOVE done, so the log closes on completion just like an exit-arc
        try:
            p = self.arclog.start(
                params={"deg": deg, "cps": cps, "radius": radius, "exit": exit_mm},
                gains={"arcffl": self.arcffl_spin.value(),
                       "arcffr": self.arcffr_spin.value(),
                       "arckp": self.arckp_spin.value(),
                       "arckd": self.arckd_spin.value()})
            self.arclog.note(f"flowturn entry={entry} mm")
            self._log(f"[flowturn log → {p.name}]")
        except Exception as e:                           # noqa: BLE001
            self._log(f"[flowturn log: could not start: {e}]")
        self._send("arcff"); self._send("arckp")
        self._send("arckd"); self._send("arcgkd")
        self._send("tlm head on")
        self._set_plot_group("tlm")
        self.ft_lbl.setText(f"flow {deg}° R{radius} {entry}→{exit_mm}mm…")
        self._send(f"flowturn {deg} {cps} {radius} {entry} {exit_mm}")

    def _on_maze_command(self, cmd: str):
        # Intercept the maze panel's commands so a FAST run auto-opens a telemetry
        # log + turns on head TLM (to capture the smooth turns); everything else just
        # passes through to the robot.
        c = cmd.strip().lower()
        if c == "fast":
            try:
                p = self.fastlog.start()
                self._log(f"[fast log → {p.name}]")
            except Exception as e:                       # noqa: BLE001
                self._log(f"[fast log: could not start: {e}]")
            self._send("tlm head on")
            self._set_plot_group("tlm")
        self._send(cmd)

    def _fast_capture(self, line: str):
        """Feed each line to the open FAST-run log: head TLM ticks while it runs,
        marker comments for flow-turns / cell steps / turn-done, and close it when
        the run ends."""
        if not self.fastlog.active:
            return
        m = ARC_TLM_RX.search(line)
        if m:
            self.fastlog.row(float(m.group(1)), float(m.group(2)),
                             float(m.group(3)), float(m.group(4)))
            return
        m = FLOW_RX.search(line)
        if m:
            self.fastlog.note(f"FLOW deg={m.group(1)} r={m.group(2)} cps={m.group(3)}")
            return
        if line.startswith("SOLVE,"):
            self.fastlog.note(line.strip())
            return
        m = TURN_DONE_RX.search(line)
        if m:
            self.fastlog.note(f"TURN done, heading {m.group(1)} deg")
            return
        if "SOLVE:" in line:                             # "SOLVE: done"/"aborted" = end
            self.fastlog.note(line.strip())
            p = self.fastlog.stop("run ended")
            self._send("tlm head off")
            if p:
                self._log(f"[fast log saved → {p.name}]")
            return
        # catch-all: log every other non-TLM line as a note (the FAST: banner with the
        # smooth/pivot indicator, gain echoes, errors) so the log is self-explanatory
        s = line.strip()
        if s and s not in (">",):
            self.fastlog.note(s)

    def _grip_capture(self, line: str):
        """Capture a vacuum grip/downforce test to its own CSV. Opens on the
        firmware's 'GRIP,begin', logs every GRIP,* row + human 'grip:' line, and
        closes on 'GRIP,end'. Also surfaces the summary rows in the log pane."""
        s = line.strip()
        if s.startswith("GRIP,begin"):
            try:
                p = self.griplog.start()
                self.griplog.raw(s)
                self._log(f"[grip log → {p.name}]")
            except Exception as e:                       # noqa: BLE001
                self._log(f"[grip log: could not start: {e}]")
            return
        if not self.griplog.active:
            return
        if s.startswith("GRIP") or s.startswith("grip:"):
            self.griplog.raw(s)
        if s.startswith("GRIP,end"):
            p = self.griplog.stop("done")
            if p:
                self._log(f"[grip log saved → {p.name}]")

    def _arc_capture(self, line: str):
        """Feed each incoming line to the open arc log: TLM ticks while it runs,
        then close on completion (TURN done for a stop-test, MOVE done for an
        exit run)."""
        if not self.arclog.active:
            return
        m = ARC_TLM_RX.search(line)
        if m:
            self.arclog.row(float(m.group(1)), float(m.group(2)),
                            float(m.group(3)), float(m.group(4)))
            return
        # firmware gain echoes -> record the gains ACTUALLY in effect (the UI can
        # be stale); arcff is echoed x10, arckp/arckd as plain ints
        for rx, label, scale in ((ARCFFL_RX, "arcffl", 0.1),
                                 (ARCFFR_RX, "arcffr", 0.1),
                                 (ARCGKD_RX, "arcgkd", 0.1),
                                 (ARCKP_RX, "arckp", 1.0),
                                 (ARCKD_RX, "arckd", 1.0)):
            gm = rx.search(line)
            if gm:
                self.arclog.note(f"fw {label}={int(gm.group(1)) * scale:g}")
                return
        m = TURN_DONE_RX.search(line)
        if m:
            self.arclog.note(f"TURN done, heading {m.group(1)} deg")
            if self.arclog.exit_mm == 0:                 # no exit straight follows
                p = self.arclog.stop("turn complete (exit=0)")
                if p:
                    self._log(f"[arc log saved → {p.name}]")
            return
        m = MOVE_DONE_RX.search(line)
        if m:
            self.arclog.note(f"MOVE done, travelled {m.group(1)} mm")
            p = self.arclog.stop("arc + exit complete")
            if p:
                self._log(f"[arc log saved → {p.name}]")

    def _apply_tcfg(self):
        self._send(f"tcfg {self.trate_spin.value()} {self.taccel_spin.value()}")

    def _maybe_autosend_tcfg(self):
        if getattr(self, "pid_auto", None) and self.pid_auto.isChecked():
            self._apply_tcfg()

    def _on_follow_toggle(self, checked: bool):
        self.follow_btn.setText("Centre: ON" if checked else "Centre: OFF")
        self._send("follow on" if checked else "follow off")

    def _set_follow_button(self, on: bool):
        """Reflect the mouse's reported follow state without re-sending."""
        self.follow_btn.blockSignals(True)
        self.follow_btn.setChecked(on)
        self.follow_btn.setText("Centre: ON" if on else "Centre: OFF")
        self.follow_btn.blockSignals(False)

    def _stream_loop(self):
        loop = self.pid_loop.currentText()
        self._send(f"tlm {loop} on")
        self._set_plot_group("tlm")

    def _autotune(self):
        # plot the wobble live, then kick off the relay test at the Speed value
        self._send("tlm vel on")
        self._set_plot_group("tlm")
        self.autotune_lbl.setText("measuring… keep robot on the floor")
        self._send(f"autotune {self.pid_speed.value()}")

    # -- WiFi provisioning ---------------------------------------------------
    def _wifi_setup_clicked(self):
        """Point the mouse at a different local router, no reflash required.

        Opens a dialog whose primary path is setting the credentials straight
        over the ESP32-C3's USB serial port (rock-solid: no AP / phone / DHCP /
        captive portal). The captive-portal method is still offered as a
        fallback. See serial_provision.py and WifiProvision in the ESP32-C3
        firmware.
        """
        dlg = WifiSetupDialog(self, connected=bool(self.bridge and self.bridge.link.connected))
        dlg.portalRequested.connect(lambda: self._send("##WIFI_SETUP##"))
        dlg.exec()

    def _ota_clicked(self):
        """Stage a firmware .bin into the mouse's external QSPI flash over the
        WiFi link (Tier A: staging + CRC verify only; nothing is flashed to the
        running app here). See ota_upload.py and lib/ota in the firmware."""
        if not (self.bridge and self.bridge.link.connected):
            self._log("[not connected]")
            return

        # Default to the PlatformIO build output if it's where we expect it.
        default_dir = ""
        guess = (Path(__file__).resolve().parent.parent
                 / "Nemisis Firmware" / ".pio" / "build" / "nucleo_g474re"
                 / "firmware.bin")
        if guess.exists():
            default_dir = str(guess)

        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select firmware image", default_dir,
            "Firmware image (*.bin);;All files (*)")
        if not path:
            return

        prog = QtWidgets.QProgressDialog("Staging firmware to QSPI…", "Cancel",
                                         0, 100, self)
        prog.setWindowTitle("Update firmware")
        prog.setWindowModality(Qt.WindowModal)
        prog.setMinimumDuration(0)
        prog.setAutoClose(False)
        prog.setAutoReset(False)
        prog.setValue(0)

        sig = OtaSignals(self)

        def _on_progress(done: int, total: int):
            pct = int(done * 100 / total) if total else 0
            prog.setValue(pct)
            prog.setLabelText(f"Staging firmware to QSPI…  {done}/{total} bytes")

        def _on_done(ok: bool, err: str):
            prog.close()
            if ok:
                self._log("[ota] firmware staged + verified in QSPI ✓")
                QtWidgets.QMessageBox.information(
                    self, "Update firmware",
                    "Image staged and CRC-verified in external flash.\n\n"
                    "(Tier A: staging only — the bootloader that copies it into "
                    "the app slot is the next step.)")
            else:
                QtWidgets.QMessageBox.warning(
                    self, "Update firmware", f"Staging failed:\n\n{err}")

        sig.progress.connect(_on_progress)
        sig.log.connect(self._log)
        sig.done.connect(_on_done)

        up = OtaUploader(
            self.bridge.link, path,
            on_progress=lambda d, t: sig.progress.emit(d, t),
            on_log=lambda m: sig.log.emit(m),
        )
        prog.canceled.connect(up.cancel)
        # keep refs alive for the duration of the transfer
        self._ota_sig = sig
        self._ota_up = up
        up.upload_async(lambda ok, err: sig.done.emit(ok, err or ""))

    # -- connection --------------------------------------------------------
    def _toggle_connect(self):
        if self.bridge and self.bridge.link.connected:
            self.bridge.close()
            self._set_connected(False)
            return
        host = self.host_edit.text().strip()
        try:
            port = int(self.port_edit.text())
        except ValueError:
            self._log("[invalid port]")
            return
        self.bridge = LinkBridge(host, port)
        self.bridge.lineReceived.connect(self._on_line)
        self.bridge.connected.connect(lambda: (self._set_connected(True),
                                               self._log("[connected]"),
                                               self.bridge.send(""),
                                               self._sync_from_bot()))
        self.bridge.failed.connect(lambda e: (self._set_connected(False),
                                              self._log(f"[connect failed: {e}]")))
        self.bridge.closed.connect(lambda: (self._set_connected(False),
                                            self._log("[link closed]")))
        self.status_lbl.setText("● connecting…")
        self.connect_btn.setEnabled(False)
        self.bridge.connect_async()

    def _set_connected(self, ok: bool):
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("Disconnect" if ok else "Connect")
        self.status_lbl.setText(
            "<span style='color:#55ff7f'>● connected</span>" if ok
            else "<span style='color:#ff6666'>● disconnected</span>"
        )
        if not ok:
            # battery state is stale once the link is gone
            self._batt_state = self._batt_mv = self._batt_soc = None
            self._update_battery()

    # -- sending -----------------------------------------------------------
    def _send(self, cmd: str):
        if not (self.bridge and self.bridge.link.connected):
            self._log("[not connected]")
            return
        self._last_cmd = cmd          # remembered so a short dump can auto re-fetch
        self.bridge.send(cmd)
        self._log(f"> {cmd}")

    def _send_cmd_line(self):
        text = self.cmd_edit.text()
        if text == "":
            self._send("")
        else:
            self._send(text)
        self.cmd_edit.clear()

    def _eval_fetch(self, sess):
        """Called at DUMP,end. Checks every run arrived in full (rows vs the
        firmware-announced ntrace). If the link dropped rows and we have retries
        left, automatically re-send the fetch command - the robot still holds the
        whole log in RAM, so a re-fetch replays the same data. Keeps the
        most-complete session across attempts and reports the outcome loudly, so
        a truncated transfer can never masquerade as a good run."""
        short = self.multirun.incomplete_runs          # [(label, rows, expected)]
        missing = sum(exp - rows for (_l, rows, exp) in short)
        attempt = getattr(self, "_fetch_attempt", 0)

        # keep a pointer to the most-complete copy seen so far this cycle
        best_missing = getattr(self, "_fetch_best_missing", None)
        if sess and (best_missing is None or missing < best_missing):
            self._fetch_best_missing = missing
            self._fetch_best_sess = sess

        def _tries(n):
            return "1 retry" if n == 1 else f"{n} retries"

        if not short:
            done = "" if attempt == 0 else f" after {_tries(attempt)}"
            self._log(f"[fetch: COMPLETE - all runs arrived in full{done}]")
            self.maze_panel.status.setText("Fetch complete - all runs full")
            return

        detail = ", ".join(f"{lbl} {rows}/{exp}" for (lbl, rows, exp) in short)
        if attempt < FETCH_MAX_RETRIES and getattr(self, "_fetch_cmd", None):
            self._fetch_attempt = attempt + 1
            self._fetch_retrying = True
            self._log(f"[fetch: SHORT ({detail}) - {missing} rows dropped by the "
                      f"link; auto re-fetching, attempt "
                      f"{self._fetch_attempt}/{FETCH_MAX_RETRIES}]")
            self.maze_panel.status.setText(
                f"Fetch short by {missing} rows - retry "
                f"{self._fetch_attempt}/{FETCH_MAX_RETRIES}...")
            QtCore.QTimer.singleShot(600, self._auto_refetch)
        else:
            best = getattr(self, "_fetch_best_sess", None) or sess
            self._log(f"[fetch: STILL SHORT after {_tries(attempt)} ({detail}); "
                      f"best copy kept -> {best}]")
            self.maze_panel.status.setText(
                f"Fetch incomplete - {missing} rows still missing (best copy kept)")

    def _auto_refetch(self):
        """Re-issue the remembered fetch command for the next auto-refetch attempt."""
        cmd = getattr(self, "_fetch_cmd", None)
        if not cmd:
            self._fetch_retrying = False
            return
        if not (self.bridge and self.bridge.link.connected):
            self._fetch_retrying = False
            self._log("[fetch: link down - cannot auto re-fetch]")
            return
        self.bridge.send(cmd)
        self._log(f"> {cmd}   (auto re-fetch)")

    def _run_motor(self):
        self._send(f"motor {self.motor_target.currentText()} "
                   f"{self.motor_speed.value()} {self.motor_ms.value()}")

    def _start_stream(self, cmd: str, group: str):
        self._send(cmd)
        self._set_plot_group(group)

    # -- incoming ----------------------------------------------------------
    def _on_line(self, line: str):
        self._log(line)
        self._parse_battery(line)
        self._parse_gains(line)
        self._parse_ff(line)
        self._parse_autotune(line)
        self._parse_move(line)
        self._parse_cell(line)
        self._parse_turn(line)
        self._parse_path(line)
        self._parse_accel(line)
        self._parse_geom(line)
        self._parse_vfilt(line)
        self._parse_arctune(line)
        self._parse_dfilt(line)
        self._parse_tcfg(line)
        self._parse_irset(line)
        self._parse_follow(line)
        self._parse_profile(line)
        self._arc_capture(line)
        self._fast_capture(line)
        self._grip_capture(line)
        self.grip_panel.feed(line)
        if "GRIP,begin" in line:                         # bring the visual forward
            self.left_tabs.setCurrentWidget(self.grip_panel)
        # surface the on-MCU sim result right under the maze (it's the authoritative
        # number - learned vs optimal - computed on the bot with full data)
        if line.startswith("SIM:") or line.startswith("SIM STATS:"):
            self.maze_panel.status.setText(line.strip())
        # feed the live maze map and the diagnostic recorder
        self.maze.parse_line(line)
        # auto-save an offline-run fetch ('dump') to its own JSONL - your drawn
        # map (in the snapshot) plus the dumped map / raw IR / motion trace - even
        # if you didn't hit Record. Also flags a TRUNCATED transfer (link dropped
        # lines) so an incomplete dump never looks like a stuck robot.
        if "DUMP,begin" in line:
            if not getattr(self, "_fetch_retrying", False):
                # a fresh, user-initiated fetch → start a new auto-refetch cycle
                self._fetch_cmd = getattr(self, "_last_cmd", "dump")
                self._fetch_attempt = 0
                self._fetch_best_sess = None
                self._fetch_best_missing = None
            self._fetch_retrying = False
            try:
                self._dump_expected = int(line.split("begin,")[1].split(",")[0])
            except Exception:                            # noqa: BLE001
                self._dump_expected = None
            self._dump_got = 0
            self._dump_active = True
            # a multi-run session dump ('DUMP,begin,<nruns>') → split into per-run CSVs
            try:
                sess = self.multirun.begin(self._dump_expected or 1)
                self._log(f"[fetch: per-run CSVs → {sess}]")
            except Exception as e:                       # noqa: BLE001
                self._log(f"[fetch: could not start per-run logs: {e}]")
            if not self.recorder.active:
                try:
                    p = self.recorder.start(maze_model=self.maze)
                    self._dump_autorec = True
                    self._log(f"[fetch: saving run to {p}]")
                except Exception as e:                   # noqa: BLE001
                    self._log(f"[fetch: could not start log: {e}]")
        # start a new per-run CSV when a run block begins
        if "DUMP,run," in line:
            _, d = parse_event(line)
            if d:
                self.multirun.start_run(d["run"], d["label"], d["nsolve"], d["ntrace"])
                self._log(f"[fetch: run {d['run']} ({d['label']}): "
                          f"{d['nsolve']} cells, {d['ntrace']} samples]")
        # feed trace/solve/sir rows into the active per-run CSV
        if self.multirun.active:
            self.multirun.feed(line)
        if getattr(self, "_dump_active", False) and "SOLVE," in line:
            self._dump_got = getattr(self, "_dump_got", 0) + 1
        self.recorder.ingest(line)
        if "DUMP,end" in line:
            self._dump_active = False
            sess = self.multirun.end()
            if sess:
                n = len(self.multirun.files)
                self._log(f"[fetch: {n} run CSV(s) saved → {sess}]")
            if getattr(self, "_dump_autorec", False):
                self.recorder.stop()
                self._dump_autorec = False
                self._log("[fetch: run saved + overlaid]")
            self.maze_panel.refresh()
            self._eval_fetch(sess)
        group = self.store.ingest(line)
        # auto-switch the plot to whatever is actively streaming
        if group and group != self.active_group:
            self._set_plot_group(group)

    def _log(self, line: str):
        self.console.appendPlainText(line)

    # -- gain sync ---------------------------------------------------------
    def _parse_gains(self, line: str):
        """Keep the tuning panel in sync with gains the mouse reports (x1000)."""
        m = GAIN_RX.search(line)
        if not m:
            return
        loop = m.group(1).lower()
        kp, ki, kd = (int(m.group(2)) / 1000.0,
                      int(m.group(3)) / 1000.0,
                      int(m.group(4)) / 1000.0)
        self._gains[loop] = [kp, ki, kd]
        # the 'vel' selection edits both wheels; mirror the left loop's report
        if loop == "lvel":
            self._gains["vel"] = [kp, ki, kd]
        # refresh the visible kp/ki/kd if this echo is for the loop on display
        # (the 'vel' view is fed by the lvel echo, so catch that too)
        shown = self.pid_loop.currentText()
        if shown == loop or (shown == "vel" and loop == "lvel"):
            self._load_gains_into_ui(shown)

    def _parse_ff(self, line: str):
        """Sync the FF spin box with the feedforward the mouse reports."""
        m = FF_RX.search(line)
        if not m:
            return
        self.ff_spin.blockSignals(True)
        self.ff_spin.setValue(int(m.group(1)) / 1000.0)
        self.ff_spin.blockSignals(False)

    def _parse_autotune(self, line: str):
        """Show the auto-tune outcome. The firmware also echoes the new gains as
        a normal 'pid' report, so _parse_gains already updates the spinboxes —
        here we just surface Ku/Tu and the status text."""
        m = ATUNE_OK_RX.search(line)
        if m:
            kp, ki, kd, ku, tu = (int(m.group(1)) / 1000.0,
                                  int(m.group(2)) / 1000.0,
                                  int(m.group(3)) / 1000.0,
                                  int(m.group(4)) / 1000.0,
                                  int(m.group(5)))
            self.autotune_lbl.setText(
                f"done: Ku={ku:.2f} Tu={tu} ms → vel Kp={kp:.3f} Ki={ki:.3f}")
            return
        if ATUNE_FAIL_RX.search(line):
            self.autotune_lbl.setText("failed — set a rough FF first, or raise Speed")

    def _parse_move(self, line: str):
        """Show how far a distance-limited run actually went."""
        m = MOVE_DONE_RX.search(line)
        if m:
            self.move_lbl.setText(f"done: {int(m.group(1))} mm")

    def _parse_cell(self, line: str):
        """Show the walls latched at each cell centre during an advance, and keep
        the cell-pitch spin in sync with the mouse."""
        m = CELL_RX.search(line)
        if m:
            idx, L, R, F = (int(m.group(i)) for i in range(1, 5))
            walls = "".join(c for c, b in (("L", L), ("F", F), ("R", R)) if b) or "—"
            self.adv_lbl.setText(f"cell {idx}: {walls}")
            return
        m = CELL_MM_RX.search(line)
        if m:
            self.cell_mm.blockSignals(True)
            self.cell_mm.setValue(int(m.group(1)))
            self.cell_mm.blockSignals(False)

    def _parse_turn(self, line: str):
        """Show the heading a pivot turn actually reached."""
        m = TURN_DONE_RX.search(line)
        if m:
            # A pivot AND a smooth arc both report completion on this line; show
            # the reached heading on whichever was last kicked off.
            done = f"done: {int(m.group(1))}°"
            self.turn_lbl.setText(done)
            if hasattr(self, "arc_lbl") and self.arc_lbl.text().startswith("arc"):
                self.arc_lbl.setText(done)

    def _parse_path(self, line: str):
        """Track nav-sequencer (path) progress in the Path label."""
        m = PATH_STEP_RX.search(line)
        if m:
            i, n, typ, arg = m.group(1), m.group(2), m.group(3).lower(), m.group(4)
            self.path_lbl.setText(f"{i}/{n}: fwd {arg}" if typ == "advance"
                                  else f"{i}/{n}: turn {arg}°")
            return
        m = PATH_DONE_RX.search(line)
        if m:
            self.path_lbl.setText("done" if "done" in m.group(1).lower() else "aborted")

    def _parse_tcfg(self, line: str):
        """Sync the turn rate/accel spin boxes with what the mouse reports."""
        m = TCFG_RX.search(line)
        if not m:
            return
        for sp, val in ((self.trate_spin, int(m.group(1))),
                        (self.taccel_spin, int(m.group(2)))):
            sp.blockSignals(True)
            sp.setValue(val)
            sp.blockSignals(False)

    def _parse_irset(self, line: str):
        """Sync the IR settle/samples spin boxes with what the mouse reports."""
        m = IRSET_RX.search(line)
        if not m:
            return
        for sp, val in ((self.irsettle_spin, int(m.group(1))),
                        (self.irsamp_spin, int(m.group(2)))):
            sp.blockSignals(True)
            sp.setValue(val)
            sp.blockSignals(False)

    def _parse_follow(self, line: str):
        """Sync the centring toggle + gain with what the mouse reports."""
        m = FOLLOW_RX.search(line)          # show line: state + gain
        if m:
            self._set_follow_button(m.group(1).lower() == "on")
            self._set_fgain(int(m.group(2)) / 1000.0)
            return
        m = FOLLOW_ST_RX.search(line)       # action line: state only
        if m:
            self._set_follow_button(m.group(1).lower() == "on")
            return
        m = FGAIN_RX.search(line)           # gain echo from 'fcfg'
        if m:
            self._set_fgain(int(m.group(1)) / 1000.0)

    def _set_fgain(self, val: float):
        self.fgain_spin.blockSignals(True)
        self.fgain_spin.setValue(val)
        self.fgain_spin.blockSignals(False)

    def _parse_accel(self, line: str):
        """Sync the accel/decel spin boxes with the rates the mouse reports."""
        m = ACCEL_RX.search(line)
        if not m:
            return
        for sp, val in ((self.accel_spin, int(m.group(1))),
                        (self.decel_spin, int(m.group(2)))):
            sp.blockSignals(True)
            sp.setValue(val)
            sp.blockSignals(False)
        self._update_accel_label()

    def _parse_geom(self, line: str):
        """Pick up counts/mm from the boot log so the m/s^2 readout is right."""
        m = GEOM_RX.search(line)
        if m:
            self.counts_per_mm = int(m.group(1)) / 100.0
            self._update_accel_label()

    def _parse_vfilt(self, line: str):
        """Sync the velocity-filter spin boxes with what the mouse reports."""
        m = VFILT_RX.search(line)
        if not m:
            return
        self.vwin_spin.blockSignals(True)
        self.vwin_spin.setValue(int(m.group(1)))
        self.vwin_spin.blockSignals(False)
        self.valpha_spin.blockSignals(True)
        self.valpha_spin.setValue(int(m.group(2)) / 1000.0)
        self.valpha_spin.blockSignals(False)

    def _parse_arctune(self, line: str):
        """Sync the smooth-arc FF/Kp/Kd spin boxes with the robot's echoes."""
        m = ARCFFL_RX.search(line)
        if m:
            self._set_spin(self.arcffl_spin, int(m.group(1)) / 10.0)
            return
        m = ARCFFR_RX.search(line)
        if m:
            self._set_spin(self.arcffr_spin, int(m.group(1)) / 10.0)
            return
        m = ARCKP_RX.search(line)
        if m:
            self._set_spin(self.arckp_spin, int(m.group(1)))
            return
        m = ARCKD_RX.search(line)
        if m:
            self._set_spin(self.arckd_spin, int(m.group(1)))
            return

    def _parse_dfilt(self, line: str):
        """Sync the heading D-filter spin box (straight/pivot loop)."""
        m = DFILT_RX.search(line)
        if m:
            self._set_spin(self.dfilt_spin, int(m.group(1)) / 1000.0)

    @staticmethod
    def _set_spin(spin, value):
        spin.blockSignals(True)
        spin.setValue(value)
        spin.blockSignals(False)

    def _parse_profile(self, line: str):
        """Sync the profile combo + status with the robot's profile reports."""
        m = PROFILE_RX.search(line)
        if m:
            idx = int(m.group(1))
            if 0 <= idx < self.profile_combo.count():
                self.profile_combo.blockSignals(True)
                self.profile_combo.setCurrentIndex(idx)
                self.profile_combo.blockSignals(False)
            self.profile_lbl.setText(f"active: {m.group(2)}")
            return
        m = SAVE_RX.search(line)
        if m:
            if m.group(1).lower() == "ok":
                name = m.group(3) or self.profile_combo.currentText()
                self.profile_lbl.setText(f"saved → {name} (flash)")
            else:
                self.profile_lbl.setText("save failed — stop the robot first")
            return
        m = LOAD_RX.search(line)
        if m:
            self.profile_lbl.setText(f"loaded: {m.group(2)}")

    # -- battery indicator -------------------------------------------------
    def _parse_battery(self, line: str):
        """Pull battery voltage/SoC/state out of any line that carries it and
        refresh the indicator + the high-current lockout."""
        m = BATT_LINE_RX.search(line)
        if m:
            self._update_battery(state=m.group(3),
                                 pack_mv=int(m.group(1)),
                                 soc=float(m.group(2)))
            return
        m = BATT_ARMED_RX.search(line) or BATT_GATE_RX.search(line)
        if m:
            self._update_battery(state=m.group(1))

    def _update_battery(self, *, state: Optional[str] = None,
                        pack_mv: Optional[int] = None,
                        soc: Optional[float] = None):
        if state:
            self._batt_state = state.upper()
        if pack_mv is not None:
            self._batt_mv = pack_mv
        if soc is not None:
            self._batt_soc = soc

        st = self._batt_state
        parts = []
        if self._batt_mv is not None:
            parts.append(f"{self._batt_mv / 1000:.2f} V")
        if self._batt_soc is not None:
            parts.append(f"{self._batt_soc:.0f}%")
        parts.append(st or "—")
        self.batt_lbl.setText("🔋 " + "  ·  ".join(parts))

        critical = (st == "CRITICAL")
        # Mirror the firmware lockout in the UI so the buttons match reality.
        self.motor_run_btn.setEnabled(not critical)
        if hasattr(self, "drive_run_btn"):
            self.drive_run_btn.setEnabled(not critical)
        if hasattr(self, "move_run_btn"):
            self.move_run_btn.setEnabled(not critical)
        if hasattr(self, "turn_run_btn"):
            self.turn_run_btn.setEnabled(not critical)
        if hasattr(self, "arc_run_btn"):
            self.arc_run_btn.setEnabled(not critical)
        if hasattr(self, "ft_run_btn"):
            self.ft_run_btn.setEnabled(not critical)
        if hasattr(self, "adv_run_btn"):
            self.adv_run_btn.setEnabled(not critical)
        if hasattr(self, "path_run_btn"):
            self.path_run_btn.setEnabled(not critical)
        self.vac_slider.setEnabled(not critical)
        tip = ("Battery CRITICAL — the mouse has locked out motors & vacuum"
               if critical else "")
        self.motor_run_btn.setToolTip(tip)
        self.vac_slider.setToolTip(tip)

        if critical:
            if not self.batt_blink.isActive():
                self._batt_blink_on = False
                self.batt_blink.start(450)
            return  # the blinker paints the style while critical
        self.batt_blink.stop()
        self._paint_batt(False)

    def _paint_batt(self, alarm: bool):
        st = self._batt_state
        if alarm:
            self.batt_lbl.setStyleSheet(
                "color:#ffffff; background:#cc2222; font-weight:bold;"
                "padding:1px 6px; border-radius:3px;")
            return
        color = BATT_COLORS.get(st, "#6b7785")
        weight = "bold" if st in ("LOW", "CRITICAL") else "normal"
        self.batt_lbl.setStyleSheet(
            f"color:{color}; font-weight:{weight}; padding:1px 6px;")

    def _blink_batt(self):
        self._batt_blink_on = not self._batt_blink_on
        self._paint_batt(self._batt_blink_on)

    # -- plotting ----------------------------------------------------------
    def _show_placeholder(self):
        self.plot.clear()
        self.curves = {}
        lbl = self.plot.addLabel(
            "no stream — click a subsystem above", color="#6b7785", size="11pt")
        self._placeholder = lbl

    def _set_plot_group(self, group: str):
        panels = PANELS.get(group)
        if panels is None:
            return
        self.active_group = group
        self.plot.clear()
        # curves maps series_name -> (PlotDataItem, scale)
        self.curves = {}
        first_plot = None
        for row, panel in enumerate(panels):
            p = self.plot.addPlot(row=row, col=0)
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setTitle(panel["title"], size="10pt")
            p.setLabel("left", panel["ylabel"])
            if row == len(panels) - 1:
                p.setLabel("bottom", "time", "s")
            if panel["yrange"]:
                p.setYRange(*panel["yrange"])
            else:
                p.enableAutoRange(axis="y")
            p.addLegend(offset=(8, 8), labelTextSize="8pt")
            # share the x-axis so panels pan/zoom together
            if first_plot is None:
                first_plot = p
            else:
                p.setXLink(first_plot)
            for name, label, cidx, scale in panel["traces"]:
                pen = pg.mkPen(PEN_COLORS[cidx % len(PEN_COLORS)], width=2)
                self.curves[name] = (p.plot([], [], pen=pen, name=label), scale)

    def _refresh_plot(self):
        # repaint the maze map when it changed (cheap; own dirty flag)
        if self.maze.dirty:
            self.maze_panel.refresh()
        if not self.active_group or not self.store.dirty:
            return
        self.store.dirty = False
        series = self.store.groups[self.active_group]
        for name, (curve, scale) in self.curves.items():
            s = series.get(name)
            if s and s.x:
                if scale == 1.0:
                    curve.setData(list(s.x), list(s.y))
                else:
                    curve.setData(list(s.x), [v * scale for v in s.y])

    # -- diagnostic recording ----------------------------------------------
    def _toggle_record(self, on: bool):
        if on:
            path = self.recorder.start(maze_model=self.maze)
            self.record_btn.setText("⏹ Recording")
            self.record_btn.setStyleSheet("color:#ff6666; font-weight:bold;")
            self._log(f"[recording -> {path}]")
        else:
            summary = self.recorder.stop()
            self.record_btn.setText("⏺ Record")
            self.record_btn.setStyleSheet("")
            if summary:
                self._log(f"[recording stopped: {summary['duration_s']}s, "
                          f"{sum(summary['event_counts'].values())} events -> "
                          f"{self.recorder.path}]")

    # -- shutdown ----------------------------------------------------------
    def closeEvent(self, event: QtGui.QCloseEvent):
        if self.recorder.active:
            self.recorder.stop()
        if self.arclog.active:
            self.arclog.stop("app closed")
        if self.fastlog.active:
            self.fastlog.stop("app closed")
        if self.bridge:
            self.bridge.close()
        super().closeEvent(event)


class WifiSetupDialog(QtWidgets.QDialog):
    """Point the mouse at a different WiFi network without reflashing.

    Primary path: send the credentials over the ESP32-C3's USB serial port
    (serial_provision.py drives the firmware's SSID=/PASS=/SAVE commands). This
    sidesteps the captive-portal AP entirely, which on some boards/phones never
    hands out an IP ("connects but the page never loads"). The old portal method
    is still offered as a fallback.
    """

    portalRequested = Signal()          # emitted to fire ##WIFI_SETUP## over TCP
    _logLine = Signal(str)              # worker thread -> UI log (queued)

    def __init__(self, parent=None, connected: bool = False):
        super().__init__(parent)
        self.setWindowTitle("WiFi setup")
        self.setMinimumWidth(520)
        self._connected = connected
        self._worker: Optional[threading.Thread] = None

        root = QtWidgets.QVBoxLayout(self)

        intro = QtWidgets.QLabel(
            "Set the mouse's WiFi over its <b>USB cable</b> — the reliable way. "
            "Plug the ESP32-C3 into this PC, pick its COM port, enter your "
            "<b>2.4&nbsp;GHz</b> network and password, and hit Send. The mouse "
            "saves it and reboots onto that network."
        )
        intro.setWordWrap(True)
        root.addWidget(intro)

        # -- port picker --
        prow = QtWidgets.QHBoxLayout()
        prow.addWidget(QtWidgets.QLabel("USB port"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                      QtWidgets.QSizePolicy.Preferred)
        prow.addWidget(self.port_combo, 1)
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        prow.addWidget(self.refresh_btn)
        root.addLayout(prow)

        # -- credentials --
        form = QtWidgets.QFormLayout()
        self.ssid_edit = QtWidgets.QLineEdit()
        self.ssid_edit.setPlaceholderText("your 2.4 GHz network name")
        self.ssid_edit.setMaxLength(32)
        form.addRow("SSID", self.ssid_edit)

        prow2 = QtWidgets.QHBoxLayout()
        self.pass_edit = QtWidgets.QLineEdit()
        self.pass_edit.setPlaceholderText("leave blank if the network is open")
        self.pass_edit.setMaxLength(63)
        self.pass_edit.setEchoMode(QtWidgets.QLineEdit.Password)
        prow2.addWidget(self.pass_edit, 1)
        show = QtWidgets.QCheckBox("Show")
        show.toggled.connect(
            lambda on: self.pass_edit.setEchoMode(
                QtWidgets.QLineEdit.Normal if on else QtWidgets.QLineEdit.Password))
        prow2.addWidget(show)
        form.addRow("Password", prow2)
        root.addLayout(form)

        # -- send --
        self.send_btn = QtWidgets.QPushButton("Send over USB → save & reboot")
        self.send_btn.clicked.connect(self._send_serial)
        root.addWidget(self.send_btn)

        # -- log --
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(150)
        self.log.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
        root.addWidget(self.log)

        # -- captive-portal fallback --
        fb = QtWidgets.QGroupBox("Fallback: captive-portal (no USB cable)")
        fbl = QtWidgets.QVBoxLayout(fb)
        note = QtWidgets.QLabel(
            "If you can't use USB: put the mouse into portal mode, then join the "
            "open WiFi <b>NEMISIS-SETUP</b> from a phone and browse to "
            "http://192.168.4.1 (turn mobile data OFF first)."
        )
        note.setWordWrap(True)
        fbl.addWidget(note)
        self.portal_btn = QtWidgets.QPushButton("Reboot mouse into portal (over WiFi)")
        self.portal_btn.setEnabled(self._connected)
        self.portal_btn.setToolTip(
            "" if self._connected else "Only available while connected to the mouse")
        self.portal_btn.clicked.connect(self._request_portal)
        fbl.addWidget(self.portal_btn)
        root.addWidget(fb)

        # -- close --
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
        btns.rejected.connect(self.reject)
        root.addWidget(btns)

        self._logLine.connect(self._append)
        if not serial_provision.pyserial_available():
            self._append("pyserial not installed — run:  pip install pyserial")
            self.send_btn.setEnabled(False)
            self.port_combo.setEnabled(False)
            self.refresh_btn.setEnabled(False)
        else:
            self._refresh_ports()

    # -- helpers --
    def _append(self, line: str) -> None:
        self.log.appendPlainText(line)

    def _refresh_ports(self) -> None:
        self.port_combo.clear()
        ports = serial_provision.list_serial_ports()
        if not ports:
            self.port_combo.addItem("(no serial ports found)", userData=None)
            self.send_btn.setEnabled(False)
            return
        for p in ports:
            self.port_combo.addItem(p.label, userData=p.device)
        self.send_btn.setEnabled(True)

    def _request_portal(self) -> None:
        self.portalRequested.emit()
        self._append("Asked the mouse to reboot into the setup portal (over WiFi).")

    def _send_serial(self) -> None:
        if self._worker and self._worker.is_alive():
            return
        device = self.port_combo.currentData()
        if not device:
            self._append("Pick a USB port first.")
            return
        ssid = self.ssid_edit.text().strip()
        if not ssid:
            self._append("Enter your network name (SSID) first.")
            return
        password = self.pass_edit.text()

        self.send_btn.setEnabled(False)
        self._append(f"Opening {device} at {serial_provision.BAUD} baud…")

        def work():
            try:
                serial_provision.send_credentials(
                    device, ssid, password, log=self._logLine.emit)
                self._logLine.emit("Done. The mouse is rebooting to join "
                                   f"\"{ssid}\". Reconnect in ~15 s "
                                   "(try host 'nemisis.local').")
            except Exception as e:  # surface any serial error to the user
                self._logLine.emit(f"ERROR: {e}")
            finally:
                # re-enable the button back on the UI thread
                QtCore.QMetaObject.invokeMethod(
                    self.send_btn, "setEnabled", QtCore.Qt.QueuedConnection,
                    QtCore.Q_ARG(bool, True))

        self._worker = threading.Thread(target=work, daemon=True)
        self._worker.start()


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser(description="Nemisis micromouse GUI debug client")
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=3333)
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    apply_theme(app)
    win = MainWindow(args.host, args.port)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
