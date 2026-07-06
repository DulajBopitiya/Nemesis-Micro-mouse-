#!/usr/bin/env python3
"""
analyze_drift.py - quantify fast-run DRIFT from the buffered run CSVs.

No maze / robot needed - pure offline post-mortem of the logs the app already
saved (logs/runs/<ts>/run*_*.csv). Answers the P3 question with numbers instead
of guesses: which axis of position-desync dominates the fast-run crashes?

Metrics per run (all derived from the TRACE columns
  t_ms,posL,posR,head_deg,gyroz_dps,accelz_g,L_LM,L_M,L_F,R_F,R_M,R_RM,x,y,flood,vbat_mv):

  * heading drift on straights   - when NOT rotating (|gyroz| small), how far is
                                    the heading from the nearest cardinal (0/90/..)?
                                    = accumulated heading error the robot drives with.
  * pivot rotation error         - each detected pivot's net heading change vs the
                                    nearest multiple of 90 deg (under/over-rotation).
  * L/R wheel imbalance          - on straights, right-wheel travel / left-wheel
                                    travel. <1 = right lags (curves right); a near-0
                                    ratio flags a right-wheel STALL.
  * battery sag                  - min pack voltage, and voltage under pivot load.
  * impact spikes                - |accelz - 1g| excursions = rams / hits.

Usage:  python analyze_drift.py [logs/runs]     (defaults to ./logs/runs)
"""
from __future__ import annotations

import csv
import glob
import math
import os
import sys

# --- tunables (heuristic segment detection) ---------------------------------
ROT_DPS      = 40.0     # |gyroz| above this = actively rotating (a pivot)
STRAIGHT_DPS = 25.0     # |gyroz| below this = driving straight (heading settled)
POS_MOVING   = 400      # |enc counts| above this = a real straight segment (not noise)
IMPACT_G     = 0.6      # |accelz - 1g| above this = an impact/vibration spike


def load_rows(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            p = line.rstrip("\n").split(",")
            if len(p) < 16 or not (p[0].lstrip("-").isdigit()):
                continue
            try:
                rows.append({
                    "t": int(p[0]), "posL": int(p[1]), "posR": int(p[2]),
                    "head": float(p[3]), "gyroz": float(p[4]), "az": float(p[5]),
                    "L_M": int(p[7]), "R_M": int(p[10]),
                    "x": int(p[12]), "y": int(p[13]),
                    "vbat": int(p[15]) if p[15] not in ("", "-") else 0,
                })
            except ValueError:
                continue
    return rows


def cardinal_err(h):
    """Smallest angle from h to the nearest multiple of 90 deg."""
    e = h - round(h / 90.0) * 90.0
    return abs(e)


def analyze(path):
    rows = load_rows(path)
    if len(rows) < 10:
        return None

    # --- heading drift on straights ---
    straight_err = [cardinal_err(r["head"]) for r in rows
                    if abs(r["gyroz"]) < STRAIGHT_DPS]
    # --- L/R wheel balance on straights (both wheels driving same direction, moving) ---
    ratios, stalls = [], 0
    lat_bias = []
    for r in rows:
        if abs(r["gyroz"]) < STRAIGHT_DPS and \
           (r["posL"] > POS_MOVING and r["posR"] > POS_MOVING or
            r["posL"] < -POS_MOVING and r["posR"] < -POS_MOVING):
            l, rr = abs(r["posL"]), abs(r["posR"])
            if l > 0:
                ratios.append(rr / l)
                if rr / l < 0.5:        # right wheel barely turning while left drives
                    stalls += 1
        # lateral centring: when both mid side sensors see a wall, their imbalance
        if r["L_M"] > 20 and r["R_M"] > 20:
            lat_bias.append(r["L_M"] - r["R_M"])

    # --- per-turn REALIZED rotation, measured settled-segment -> settled-segment ---
    # A turn's true net rotation = the heading difference between the settled heading
    # BEFORE it and the settled heading AFTER it fully settles (so late-finishing
    # rotation that the following straight's heading-hold completes is INCLUDED).
    # We collapse runs of low-|gyroz| samples into "settled" segments (median heading),
    # and the jump between consecutive settled segments is one realized turn.
    settled = []            # (median_heading) per settled segment
    i, n = 0, len(rows)
    while i < n:
        if abs(rows[i]["gyroz"]) < STRAIGHT_DPS:
            j = i
            hs = []
            while j < n and abs(rows[j]["gyroz"]) < STRAIGHT_DPS:
                hs.append(rows[j]["head"])
                j += 1
            if len(hs) >= 2:                          # a real settled dwell, not a blip
                hs.sort()
                settled.append(hs[len(hs) // 2])       # median heading
            i = j
        else:
            i += 1

    pivots = []
    for k in range(1, len(settled)):
        dhead = settled[k] - settled[k - 1]
        if abs(dhead) > 30:                            # a turn happened between them
            target = round(dhead / 90.0) * 90.0
            signed = (dhead - target) * (1 if dhead >= 0 else -1)   # <0 = under-rotate
            pivots.append((dhead, signed))

    # plausible pack voltage only (2-cell li-po ~6.0-8.4V); the truncated logs
    # merged fields on some rows -> junk like 1 mV or 7173634 mV, filter them out.
    vbats = [r["vbat"] for r in rows if 6000 <= r["vbat"] <= 8500]
    impacts = sum(1 for r in rows if abs(r["az"] - 1.0) > IMPACT_G)

    def stats(xs):
        if not xs:
            return (0.0, 0.0, 0.0)
        return (sum(xs) / len(xs), max(xs), min(xs))

    sh_mean, sh_max, _ = stats(straight_err)
    pv_err = [abs(e) for _, e in pivots]
    pv_signed = [e for _, e in pivots]              # keep sign: <0 = under-rotate
    pv_mean, pv_max, _ = stats(pv_err)
    pv_signed_mean = (sum(pv_signed) / len(pv_signed)) if pv_signed else 0.0
    r_mean, r_max, r_min = stats(ratios)

    return {
        "path": path,
        "rows": len(rows),
        "head_drift_mean": sh_mean, "head_drift_max": sh_max,
        "n_pivots": len(pivots), "pivot_err_mean": pv_mean, "pivot_err_max": pv_max,
        "pivot_err_signed_mean": pv_signed_mean,
        "wheel_ratio_mean": r_mean, "wheel_ratio_min": r_min, "wheel_stall_samples": stalls,
        "lat_bias_mean": (sum(lat_bias) / len(lat_bias)) if lat_bias else 0.0,
        "vbat_min": min(vbats) if vbats else 0,
        "vbat_max": max(vbats) if vbats else 0,
        "impacts": impacts,
    }


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "logs/runs"
    files = sorted(glob.glob(os.path.join(base, "*", "*fast*.csv")))
    if not files:
        print(f"no fast-run CSVs under {base}")
        return

    results = [a for a in (analyze(f) for f in files) if a]
    print(f"Analysed {len(results)} fast-run traces under {base}\n")

    hdr = ("run", "rows", "headDrift(deg)", "pivotErr(deg)", "wheelR/L",
           "stall", "vbat(min-max)", "hits")
    print(f"{hdr[0]:<26}{hdr[1]:>5}  {hdr[2]:>15}  {hdr[3]:>15}  "
          f"{hdr[4]:>10}  {hdr[5]:>5}  {hdr[6]:>13}  {hdr[7]:>4}")
    print("-" * 110)
    for a in results:
        run = os.path.basename(os.path.dirname(a["path"])) + "/" + \
              os.path.basename(a["path"]).replace("run0_", "").replace(".csv", "")
        print(f"{run:<26}{a['rows']:>5}  "
              f"{a['head_drift_mean']:>6.1f}/{a['head_drift_max']:<7.1f}  "
              f"{a['pivot_err_mean']:>6.1f}/{a['pivot_err_max']:<7.1f}  "
              f"{a['wheel_ratio_mean']:>9.2f}  {a['wheel_stall_samples']:>5}  "
              f"{a['vbat_min']:>5}-{a['vbat_max']:<7}  {a['impacts']:>4}")

    # --- aggregate ---
    def avg(k):
        xs = [a[k] for a in results]
        return sum(xs) / len(xs) if xs else 0.0

    print("-" * 110)
    print("AGGREGATE (mean across runs):")
    print(f"  heading drift on straights : {avg('head_drift_mean'):.2f} deg mean, "
          f"{max(a['head_drift_max'] for a in results):.1f} deg worst")
    print(f"  pivot rotation error       : {avg('pivot_err_mean'):.2f} deg mean abs, "
          f"{max(a['pivot_err_max'] for a in results):.1f} deg worst")
    print(f"  pivot error SIGNED         : {avg('pivot_err_signed_mean'):+.2f} deg mean "
          f"(negative = systematic UNDER-rotation)")
    print(f"  wheel R/L travel ratio     : {avg('wheel_ratio_mean'):.3f} "
          f"(1.00 = balanced; <1 = right lags)")
    print(f"  right-wheel stall samples  : {sum(a['wheel_stall_samples'] for a in results)} total")
    vb = [a["vbat_min"] for a in results if a["vbat_min"]]
    print(f"  battery min under load     : {min(vb) if vb else 'n/a (logs corrupted)'} mV")
    print(f"  impact spikes (|az-1g|>{IMPACT_G}g): {sum(a['impacts'] for a in results)} total")


if __name__ == "__main__":
    main()
