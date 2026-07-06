#!/usr/bin/env python3
"""
analyze_turns.py - summarise TURNDIAG / TURNSUM turn-accuracy data.

Feed it either a recorded .jsonl (from the app's Record button) OR a plain text
file / paste of the console output from a `benchturn` run. It tabulates, per turn,
commanded vs actually-achieved rotation and the residual (under-rotation), plus
the cumulative drift - the numbers behind the fast-run turn diagnosis.

Usage:  python analyze_turns.py <log.jsonl | console.txt>
"""
from __future__ import annotations

import json
import sys

import protocol

REASON = {0: "clean", 1: "snappy-early", 2: "timeout"}


def rows_from(path):
    turns, summ = [], None
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            data = None
            if line.startswith("{"):                      # a JSONL record
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                if rec.get("kind") == "turndiag":
                    data = ("turndiag", rec)
                elif rec.get("kind") == "turnsum":
                    data = ("turnsum", rec)
            else:                                          # raw console text
                kind, d = protocol.parse_event(line)
                if kind in ("turndiag", "turnsum"):
                    data = (kind, d)
            if not data:
                continue
            if data[0] == "turndiag":
                turns.append(data[1])
            else:
                summ = data[1]
    return turns, summ


def main():
    if len(sys.argv) < 2:
        print("usage: python analyze_turns.py <log.jsonl | console.txt>")
        return
    turns, summ = rows_from(sys.argv[1])
    if not turns:
        print("no TURNDIAG lines found")
        return

    has_wheels = any("wheel_ratio" in t for t in turns)
    wcol = f"  {'posL':>7}  {'posR':>7}  {'R/L':>5}" if has_wheels else ""
    print(f"{'idx':>4}  {'cmd':>7}  {'achieved':>9}  {'residual':>9}  "
          f"{'peak dps':>9}  {'ms':>5}  {'release':<12}{wcol}")
    print("-" * (62 + (26 if has_wheels else 0)))
    cum = 0.0
    for t in turns:
        cum += t["residual_deg"]
        idx = t["idx"]
        w = ""
        if "wheel_ratio" in t:
            w = f"  {t['posL']:>7}  {t['posR']:>7}  {t['wheel_ratio']:>5.2f}"
        print(f"{idx:>4}  {t['cmd_deg']:>6.1f}°  {t['achieved_deg']:>8.1f}°  "
              f"{t['residual_deg']:>+8.1f}°  {t['peak_dps']:>8.1f}  {t['ms']:>5}  "
              f"{REASON.get(t['reason'], t['reason']):<12}{w}")
    print("-" * (62 + (26 if has_wheels else 0)))
    if has_wheels:
        ratios = [t["wheel_ratio"] for t in turns if "wheel_ratio" in t]
        print(f"wheel balance R/L: mean {sum(ratios)/len(ratios):.3f}, "
              f"min {min(ratios):.3f}  (1.00 = balanced; <1 = right wheel lags/stalls)")
    resids = [t["residual_deg"] for t in turns]
    mean = sum(resids) / len(resids)
    print(f"{len(turns)} turns   mean residual {mean:+.2f}°   "
          f"cumulative drift {cum:+.1f}°   (perfect = 0)")
    worst = min(resids) if mean < 0 else max(resids)
    print(f"worst single turn {worst:+.1f}°")
    if summ:
        print(f"firmware TURNSUM: {summ['turns']} turns, "
              f"cum {summ['cum_residual_deg']:+.1f}°, mean {summ['mean_residual_deg']:+.1f}°")


if __name__ == "__main__":
    main()
