"""
protocol.py - shared parsing of the Nemisis console text protocol.

The GUI's live plots have their own PARSERS table (tuned for pyqtgraph series);
this module is the structured side used by the maze view and the diagnostics
recorder, where we want one tidy (kind, dict) per line rather than plot series.

Keeping it here means maze_model.py and recorder.py don't each re-invent the
regexes, and the firmware line formats live in exactly one place app-side.
"""

from __future__ import annotations

import re
from typing import Optional, Tuple

# --- maze / solver -----------------------------------------------------------
# "SOLVE,x,y,facing,flood,phase,wN,wE,wS,wW" - one per cell decision point.
SOLVE_RX = re.compile(
    r"SOLVE,\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),"
    r"\s*([01]),\s*([01]),\s*([01]),\s*([01])")
# "SOLVE: done - back at start" / "SOLVE: aborted"
SOLVE_DONE_RX = re.compile(r"SOLVE:\s*(done\b.*|aborted)", re.I)
# "SIR,x,y,L_LM,L_M,L_F,R_F,R_M,R_RM,thrL_F,thrR_F" - raw IR at a sense point.
SIR_RX = re.compile(
    r"SIR,\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),"
    r"\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)")
# "MAZE,6" - maze edge length.
MAZE_RX = re.compile(r"MAZE,\s*(\d+)")
# "GOAL,4,2,2,2,3,3,2,3,3" - count then count*(x,y) pairs.
GOAL_RX = re.compile(r"GOAL,\s*(\d+)((?:\s*,\s*-?\d+)*)")
# "START,x,y,facing" - the start corner (cell + heading 0..3 = N,E,S,W).
START_RX = re.compile(r"START,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)")
# "tcost = 2  (flood turn penalty, cells per 90 pivot)" - robot's flood turn cost.
TCOST_RX = re.compile(r"tcost\s*=\s*(\d+)", re.I)
# "DUMP,begin,<n>" / "DUMP,end" - brackets a replay of a buffered offline run
# (the 'dump' command). The SOLVE,/SIR, lines in between are the run's data.
DUMP_BEGIN_RX = re.compile(r"DUMP,\s*begin,\s*(\d+)", re.I)
# "DUMP,run,<idx>,<label>,<nsolve>,<ntrace>" - starts one run's block inside a
# multi-run fetch (launcher session: 0=search, 1=fast, 2=fastest).
DUMP_RUN_RX = re.compile(r"DUMP,\s*run,\s*(\d+)\s*,\s*(\w+)\s*,\s*(\d+)\s*,\s*(\d+)", re.I)
DUMP_TRACE_RX = re.compile(r"DUMP,\s*trace,\s*(\d+)", re.I)
DUMP_END_RX = re.compile(r"DUMP,\s*end", re.I)
# "TRACE,t_ms,posL,posR,head_ddeg,gyroz_ddps,accelz_mg,s0..s5,x,y,flood[,vbat_mv]"
# - one periodic motion/sensor sample from a run (replayed by 'dump'). The
# trailing vbat_mv (pack voltage) is optional so older 15-field logs still parse.
TRACE_RX = re.compile(r"TRACE," + r",".join([r"\s*(-?\d+)"] * 15) + r"(?:,\s*(-?\d+))?")

# --- live telemetry (for the recorder) --------------------------------------
IMU_RX = re.compile(
    r"acc\[mg\]\s*X=(-?\d+)\s*Y=(-?\d+)\s*Z=(-?\d+)\s*\|\s*"
    r"gyr\[mdps\]\s*X=(-?\d+)\s*Y=(-?\d+)\s*Z=(-?\d+)")
ENC_RX = re.compile(
    r"ENC\s+L\s+raw=\s*(\d+).*?cnt=\s*(-?\d+).*?R\s+raw=\s*(\d+).*?cnt=\s*(-?\d+)")
IR_RX = re.compile(
    r"IR\s+L_LM=\s*(-?\d+)\s+L_M=\s*(-?\d+)\s+L_F=\s*(-?\d+)\s+"
    r"R_F=\s*(-?\d+)\s+R_M=\s*(-?\d+)\s+R_RM=\s*(-?\d+)")
WALL_RX = re.compile(
    r"WALL,\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)")
BATT_RX = re.compile(
    r"BATT\s+(-?\d+)\s*mV\s*\((-?\d+)\s*mV/cell\)\s+([\d.]+)\s*%"
    r"(?:\s+(-?\d+)\s*m%/hr)?(?:\s*\[(\w+)\])?")
TLM_RX = re.compile(r"TLM,\s*([\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)")
CELL_RX = re.compile(r"CELL,\s*(\d+),\s*([01]),\s*([01]),\s*([01])")
MOVE_DONE_RX = re.compile(r"MOVE:\s*done,\s*travelled\s*(-?\d+)\s*mm", re.I)
TURN_DONE_RX = re.compile(r"TURN:\s*done,\s*heading\s*(-?\d+)\s*deg", re.I)
# "TURNDIAG,idx,cmd_ddeg,ach_ddeg,resid_ddeg,peak_ddps,ms,reason[,posL,posR]" -
# per-turn drift diagnostics from a 'turn' (idx -1) or a 'benchturn' run. Angles/rate
# in 0.1 units; reason 0=arrived clean, 1=snappy early-release, 2=timeout. posL/posR
# (per-wheel encoder counts this turn) are optional so older 7-field logs still parse.
TURNDIAG_RX = re.compile(
    r"TURNDIAG,\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)"
    r"(?:,\s*(-?\d+),\s*(-?\d+))?")
# "TURNSUM,turns,cum_resid_ddeg,mean_resid_ddeg[,drift_ddeg]" - benchturn summary.
# cum_resid = sum of per-turn in-frame shortfalls (hcarry-invariant); drift (optional,
# newer firmware) = true accumulated heading - ideal = the REAL error carried out of the
# sequence, which is what the hcarry fix drives toward ~0.
TURNSUM_RX = re.compile(r"TURNSUM,\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)(?:,\s*(-?\d+))?")

IR_NAMES   = ["L_LM", "L_M", "L_F", "R_F", "R_M", "R_RM"]


def parse_event(line: str) -> Tuple[Optional[str], Optional[dict]]:
    """Classify one console line. Returns (kind, data) or (None, None).

    `kind` is a short tag ('imu','enc','ir','walls','batt','tlm','solve',
    'solve_done','cell','move','turn','maze','goal'); `data` is a plain dict of
    the parsed fields (JSON-serialisable) for the recorder / maze model.
    """
    m = SOLVE_RX.search(line)
    if m:
        g = [int(v) for v in m.groups()]
        return "solve", {
            "x": g[0], "y": g[1], "facing": g[2], "flood": g[3], "phase": g[4],
            "wallN": g[5], "wallE": g[6], "wallS": g[7], "wallW": g[8],
        }
    m = SIR_RX.search(line)
    if m:
        g = [int(v) for v in m.groups()]
        return "sir", {
            "x": g[0], "y": g[1],
            "ir": dict(zip(IR_NAMES, g[2:8])),
            "thr_L_F": g[8], "thr_R_F": g[9],
        }
    m = SOLVE_DONE_RX.search(line)
    if m:
        return "solve_done", {"ok": not m.group(1).lower().startswith("abort"),
                              "text": m.group(1)}
    m = MAZE_RX.search(line)
    if m:
        return "maze", {"size": int(m.group(1))}
    m = GOAL_RX.search(line)
    if m:
        nums = [int(v) for v in re.findall(r"-?\d+", m.group(2))]
        cells = list(zip(nums[0::2], nums[1::2]))
        return "goal", {"count": int(m.group(1)), "cells": cells}
    m = START_RX.search(line)
    if m:
        return "start", {"x": int(m.group(1)), "y": int(m.group(2)),
                         "facing": int(m.group(3))}
    m = TCOST_RX.search(line)
    if m:
        return "tcost", {"turn_cost": int(m.group(1))}
    m = DUMP_BEGIN_RX.search(line)
    if m:
        return "dump_begin", {"count": int(m.group(1))}
    m = DUMP_RUN_RX.search(line)
    if m:
        return "dump_run", {"run": int(m.group(1)), "label": m.group(2),
                            "nsolve": int(m.group(3)), "ntrace": int(m.group(4))}
    m = DUMP_TRACE_RX.search(line)
    if m:
        return "dump_trace", {"count": int(m.group(1))}
    if DUMP_END_RX.search(line):
        return "dump_end", {}
    m = TRACE_RX.search(line)
    if m:
        gr = m.groups()
        g = [int(v) for v in gr[:15]]
        vbat_mv = int(gr[15]) if len(gr) > 15 and gr[15] is not None else None
        return "trace", {
            "t_ms": g[0], "posL": g[1], "posR": g[2],
            "heading_deg": g[3] / 10.0, "gyroz_dps": g[4] / 10.0,
            "accelz_g": g[5] / 1000.0,
            "ir": dict(zip(IR_NAMES, g[6:12])),
            "x": g[12], "y": g[13], "flood": g[14],
            "vbat_mv": vbat_mv,
            "vbat_v": (vbat_mv / 1000.0) if vbat_mv else None,
        }

    m = IMU_RX.search(line)
    if m:
        g = [int(v) for v in m.groups()]
        return "imu", {"acc_mg": g[0:3], "gyro_mdps": g[3:6]}
    m = ENC_RX.search(line)
    if m:
        g = [int(v) for v in m.groups()]
        return "enc", {"L_raw": g[0], "L_cnt": g[1], "R_raw": g[2], "R_cnt": g[3]}
    m = IR_RX.search(line)
    if m:
        return "ir", {n: int(v) for n, v in zip(IR_NAMES, m.groups())}
    m = WALL_RX.search(line)
    if m:
        return "walls", {n: int(v) for n, v in zip(IR_NAMES, m.groups())}
    m = BATT_RX.search(line)
    if m:
        return "batt", {
            "pack_mV": int(m.group(1)), "cell_mV": int(m.group(2)),
            "soc_pct": float(m.group(3)),
            "rate_mpct_hr": int(m.group(4)) if m.group(4) else None,
            "state": m.group(5),
        }
    m = TLM_RX.search(line)
    if m:
        return "tlm", {"t": float(m.group(1)), "setpoint": float(m.group(2)),
                       "measured": float(m.group(3)), "output": float(m.group(4))}
    m = CELL_RX.search(line)
    if m:
        g = [int(v) for v in m.groups()]
        return "cell", {"idx": g[0], "left": g[1], "right": g[2], "front": g[3]}
    m = MOVE_DONE_RX.search(line)
    if m:
        return "move", {"travelled_mm": int(m.group(1))}
    m = TURN_DONE_RX.search(line)
    if m:
        return "turn", {"heading_deg": int(m.group(1))}
    m = TURNDIAG_RX.search(line)
    if m:
        g = m.groups()
        d = {
            "idx": int(g[0]),
            "cmd_deg": int(g[1]) / 10.0, "achieved_deg": int(g[2]) / 10.0,
            "residual_deg": int(g[3]) / 10.0, "peak_dps": int(g[4]) / 10.0,
            "ms": int(g[5]), "reason": int(g[6]),
        }
        if g[7] is not None and g[8] is not None:
            pl, pr = int(g[7]), int(g[8])
            d["posL"] = pl
            d["posR"] = pr
            # wheel balance: |right| / |left| (1.0 = balanced; <1 = right lags/stalls)
            d["wheel_ratio"] = (abs(pr) / abs(pl)) if pl else 0.0
        return "turndiag", d
    m = TURNSUM_RX.search(line)
    if m:
        d = {"turns": int(m.group(1)),
             "cum_residual_deg": int(m.group(2)) / 10.0,
             "mean_residual_deg": int(m.group(3)) / 10.0}
        if m.group(4) is not None:
            d["drift_deg"] = int(m.group(4)) / 10.0
        return "turnsum", d
    return None, None
