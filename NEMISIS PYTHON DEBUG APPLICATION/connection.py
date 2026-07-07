"""
connection.py - transport layer for the Nemisis debug app.

Talks to the micromouse over a TCP socket exposed by the ESP32-C3 WiFi bridge.
The bridge is transparent, so this speaks the exact same text protocol the
STM32 debug console uses over J-Link RTT.

The transport is deliberately isolated behind `NemisisLink` so a BLE (or direct
USB-serial) backend can be added later without touching the UI: anything that
provides connect / send_line / a line callback / close will do.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Optional


class NemisisLink:
    """A line-oriented TCP link to the micromouse bridge."""

    def __init__(self, host: str = "192.168.4.1", port: int = 3333,
                 timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._on_line: Optional[Callable[[str], None]] = None
        # Optional additive tap on every received line, used by bulk/synchronous
        # flows (e.g. OTA firmware upload) that need to await specific replies
        # without disturbing the normal GUI line callback. Called on the rx
        # thread, in addition to (before) _on_line.
        self._sniffer: Optional[Callable[[str], None]] = None

    # -- lifecycle ---------------------------------------------------------
    def connect(self) -> None:
        self._sock = socket.create_connection((self.host, self.port),
                                              timeout=self.timeout)
        self._sock.settimeout(None)              # blocking reads in rx thread
        # Match the bridge's setNoDelay(true): don't let Nagle's algorithm
        # coalesce/delay small command writes.
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # Let the OS hold more unread bytes if the rx thread/GUI thread falls
        # behind for a moment (e.g. a burst from 'dump' or a fast telemetry
        # stream) - TCP itself never loses bytes once accepted here, but a
        # bigger window gives the app more slack before that matters.
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 18)
        except OSError:
            pass
        self._running = True
        self._rx_thread = threading.Thread(target=self._reader, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._running = False
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None and self._running

    # -- io ----------------------------------------------------------------
    def on_line(self, callback: Callable[[str], None]) -> None:
        """Register a callback invoked once per received text line."""
        self._on_line = callback

    def set_sniffer(self, callback: Optional[Callable[[str], None]]) -> None:
        """Install (or clear, with None) an additive per-line tap. Runs on the
        rx thread alongside the main on_line callback - keep it fast and
        non-blocking (e.g. push to a queue)."""
        self._sniffer = callback

    def send_line(self, text: str) -> None:
        """Send one command. A trailing CR is added (the console terminates on
        CR or LF) so it works identically to typing in RTT."""
        if not self._sock:
            raise ConnectionError("not connected")
        if not text.endswith("\r") and not text.endswith("\n"):
            text += "\r"
        self._sock.sendall(text.encode("utf-8", errors="replace"))

    # -- internals ---------------------------------------------------------
    def _reader(self) -> None:
        buf = b""
        while self._running and self._sock:
            try:
                data = self._sock.recv(8192)
            except OSError:
                break
            if not data:
                break                            # peer closed
            buf += data
            # split on CR or LF, keep the trailing partial line in buf
            buf = buf.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace")
                if self._sniffer:
                    try:
                        self._sniffer(line)
                    except Exception:       # a bad tap must never kill the rx loop
                        pass
                if self._on_line:
                    self._on_line(line)
        self._running = False
        if self._on_line:
            self._on_line("[link closed]")


def wait_for_bridge(host: str, port: int, attempts: int = 20,
                    delay: float = 0.5) -> bool:
    """Poll until the bridge TCP port accepts a connection (handy right after
    joining the AP). Returns True if reachable."""
    for _ in range(attempts):
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return True
        except OSError:
            time.sleep(delay)
    return False
