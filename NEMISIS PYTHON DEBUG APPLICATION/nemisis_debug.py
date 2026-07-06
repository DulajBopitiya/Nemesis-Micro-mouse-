#!/usr/bin/env python3
"""
nemisis_debug.py - interactive debug client for the Nemisis micromouse.

Connects over WiFi (TCP, via the ESP32-C3 bridge) to the STM32 debug console
and gives you the same menu you'd get over J-Link RTT - encoder / imu /
sensors / motor / vacuum / battery / buzzer - plus a foundation for PID
tuning and live plotting later.

Quick start:
    1. Flash the ESP32-C3 bridge firmware.
    2. Join WiFi  "NEMISIS-DBG"  (password: micromouse).
    3. python nemisis_debug.py
       (defaults to 192.168.4.1:3333; override with --host/--port)

Then just type commands ('help' for the menu, 'stop' to end a live stream).
Local meta-commands start with ':' (e.g. ':quit', ':log run1.txt').
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import threading
from datetime import datetime
from typing import Optional, TextIO

from connection import NemisisLink, wait_for_bridge

PROMPT = "nemisis> "

# Colourise battery lines so the safety state is obvious in the terminal.
_COLOR = sys.stdout.isatty()
if _COLOR and os.name == "nt":
    os.system("")  # enable ANSI escape handling on Windows consoles
_BATT_STATE_RX = re.compile(r"\b(CRITICAL|LOW|OK)\b")
_BATT_ANSI = {"CRITICAL": "\033[1;31m", "LOW": "\033[1;33m", "OK": "\033[32m"}
_ANSI_RESET = "\033[0m"


def _colorize(line: str) -> str:
    """Tint battery-monitor lines by state; leave everything else untouched."""
    if not _COLOR or "BATT" not in line.upper():
        return line
    m = _BATT_STATE_RX.search(line)
    color = _BATT_ANSI.get(m.group(1)) if m else None
    return f"{color}{line}{_ANSI_RESET}" if color else line


class App:
    def __init__(self, link: NemisisLink):
        self.link = link
        self._logfile: Optional[TextIO] = None
        self._log_lock = threading.Lock()

    # -- incoming line handling -------------------------------------------
    def handle_line(self, line: str) -> None:
        # Reprint the prompt cleanly under async output.
        sys.stdout.write("\r" + _colorize(line) + "\n" + PROMPT)
        sys.stdout.flush()
        with self._log_lock:
            if self._logfile:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self._logfile.write(f"{ts}  {line}\n")
                self._logfile.flush()

    # -- local meta-commands ----------------------------------------------
    def meta(self, cmd: str) -> bool:
        """Handle ':' commands locally. Returns False to exit the app."""
        parts = cmd[1:].split()
        if not parts:
            return True
        name, *args = parts

        if name in ("quit", "q", "exit"):
            return False
        if name in ("help", "h", "?"):
            print(_META_HELP)
        elif name == "log":
            self._toggle_log(args[0] if args else None)
        elif name == "raw" and args:
            # send bytes verbatim (no trailing CR), e.g. ':raw \\n'
            self.link.send_line(args[0])
        else:
            print(f"unknown meta-command ':{name}' (try ':help')")
        return True

    def _toggle_log(self, path: Optional[str]) -> None:
        with self._log_lock:
            if self._logfile:
                self._logfile.close()
                self._logfile = None
                print("logging stopped")
            elif path:
                self._logfile = open(path, "a", encoding="utf-8")
                print(f"logging session to {path}")
            else:
                print("usage: :log <file>   (run again with no args to stop)")

    # -- main loop ---------------------------------------------------------
    def run(self) -> None:
        self.link.on_line(self.handle_line)
        print(f"Connected to {self.link.host}:{self.link.port}. "
              f"Type 'help' for the menu, ':help' for app commands, ':quit' to exit.")
        self.link.send_line("")          # nudge the console to print its menu
        try:
            while True:
                try:
                    text = input(PROMPT)
                except EOFError:
                    break
                if text.startswith(":"):
                    if not self.meta(text):
                        break
                    continue
                if not self.link.connected:
                    print("link is closed.")
                    break
                self.link.send_line(text)
        except KeyboardInterrupt:
            pass
        finally:
            with self._log_lock:
                if self._logfile:
                    self._logfile.close()
            self.link.close()
            print("\nbye.")


_META_HELP = """\
local app commands (handled here, not sent to the mouse):
  :help            this help
  :log <file>      start logging the session to <file> (run again to stop)
  :raw <text>      send text with no trailing carriage return
  :quit            exit the app
everything else is sent straight to the micromouse console.
"""


def main() -> int:
    ap = argparse.ArgumentParser(description="Nemisis micromouse WiFi debug client")
    ap.add_argument("--host", default="192.168.4.1",
                    help="bridge IP (default: 192.168.4.1, the AP gateway)")
    ap.add_argument("--port", type=int, default=3333, help="bridge TCP port")
    ap.add_argument("--wait", action="store_true",
                    help="poll until the bridge is reachable before connecting")
    args = ap.parse_args()

    if args.wait:
        print(f"waiting for bridge at {args.host}:{args.port} ...")
        if not wait_for_bridge(args.host, args.port):
            print("bridge not reachable - are you on the NEMISIS-DBG network?")
            return 1

    link = NemisisLink(args.host, args.port)
    try:
        link.connect()
    except OSError as e:
        print(f"connect failed: {e}")
        print("  - is the ESP32 bridge powered and are you on its WiFi?")
        return 1

    App(link).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
