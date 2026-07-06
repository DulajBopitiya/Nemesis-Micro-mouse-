"""
recorder.py - structured diagnostic capture for the Nemisis app.

Records a session to a JSON Lines file: one timestamped JSON object per parsed
telemetry line (imu / enc / ir / walls / batt / tlm / solve / cell / move /
turn), plus the raw text for anything unrecognised. On stop it writes a run
summary (duration, per-kind counts, the solve path, final pose, wall mismatches).

This is the file you hand to Claude to diagnose the mouse: it's compact, fully
structured, and cross-stream (everything shares one monotonic clock), so the
encoder counts, IMU yaw, IR wall reads and the maze path all line up in time.

    rec = Recorder()
    path = rec.start(maze_model=maze)     # -> logs/nemisis-YYYYmmdd-HHMMSS.jsonl
    rec.ingest(line)                      # for every console line
    summary = rec.stop()                  # writes <file> + <file>.summary.json
"""

from __future__ import annotations

import json
import os
import time
from collections import Counter
from typing import Optional

import protocol

LOG_DIR = "logs"


class Recorder:
    # Flushing to disk on every single line (the old behaviour) is the
    # bottleneck during a fast burst (e.g. 'dump' replaying a whole run, or a
    # high-rate 'tlm'/'imu' stream): each flush is a syscall, and if enough of
    # them stack up the recorder can fall behind the incoming line rate.
    # Python's file object already buffers writes internally, so batching the
    # flush - by count and by time - keeps data safely on disk within a
    # fraction of a second without paying for a syscall per line. stop()
    # still flushes unconditionally so a session's tail is never left behind.
    _FLUSH_EVERY_N = 25
    _FLUSH_EVERY_S = 0.25

    def __init__(self):
        self._fh = None
        self.path: Optional[str] = None
        self._t0 = 0.0
        self._counts: Counter = Counter()
        self._raw_lines = 0
        self._maze = None
        self._last_solve = None
        self._unflushed = 0
        self._last_flush = 0.0

    @property
    def active(self) -> bool:
        return self._fh is not None

    def start(self, maze_model=None, directory: str = LOG_DIR) -> str:
        if self.active:
            return self.path
        os.makedirs(directory, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.path = os.path.join(directory, f"nemisis-{stamp}.jsonl")
        self._fh = open(self.path, "w", encoding="utf-8")
        self._t0 = time.monotonic()
        self._counts = Counter()
        self._raw_lines = 0
        self._maze = maze_model
        self._last_solve = None
        self._unflushed = 0
        self._last_flush = time.monotonic()
        self._write({"kind": "session_start",
                     "wall_clock": time.strftime("%Y-%m-%d %H:%M:%S"),
                     "maze": maze_model.snapshot() if maze_model else None})
        return self.path

    def ingest(self, line: str) -> None:
        if not self.active:
            return
        line = line.rstrip("\r\n")
        if not line:
            return
        t = round(time.monotonic() - self._t0, 4)
        kind, data = protocol.parse_event(line)
        if kind is None:
            # keep unrecognised text too (boot logs, acks, errors) - cheap and
            # often the exact context you need when diagnosing.
            self._raw_lines += 1
            self._write({"t": t, "kind": "text", "line": line})
            return
        self._counts[kind] += 1
        if kind == "solve":
            self._last_solve = data
        rec = {"t": t, "kind": kind}
        rec.update(data)
        self._write(rec)

    def stop(self) -> Optional[dict]:
        if not self.active:
            return None
        summary = {
            "kind": "session_end",
            "duration_s": round(time.monotonic() - self._t0, 2),
            "event_counts": dict(self._counts),
            "raw_text_lines": self._raw_lines,
            "last_solve": self._last_solve,
            "maze": self._maze.snapshot() if self._maze else None,
        }
        self._write(summary, force_flush=True)
        self._fh.close()
        self._fh = None
        # also drop the summary as its own small file for quick eyeballing
        try:
            with open(self.path + ".summary.json", "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)
        except OSError:
            pass
        return summary

    # -- internals ---------------------------------------------------------
    def _write(self, obj: dict, force_flush: bool = False) -> None:
        try:
            self._fh.write(json.dumps(obj) + "\n")
            self._unflushed += 1
            now = time.monotonic()
            if (force_flush or self._unflushed >= self._FLUSH_EVERY_N
                    or (now - self._last_flush) >= self._FLUSH_EVERY_S):
                self._fh.flush()
                self._unflushed = 0
                self._last_flush = now
        except (OSError, ValueError):
            pass
