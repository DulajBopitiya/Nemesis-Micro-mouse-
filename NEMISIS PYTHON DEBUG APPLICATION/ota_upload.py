"""
ota_upload.py - stream a firmware .bin to the mouse's external QSPI flash.

Tier A of the OTA plan: the app sends the image over the existing WiFi link and
the STM32 stages it in the W25Q32JW, CRC-verifying it there. Nothing jumps to or
overwrites the running firmware here - this only proves + performs the staging,
so it cannot brick the board.

Protocol (mirrors lib/console cmd_otarx + lib/ota in the firmware):
    app -> otarx <size> <crc32hex>
    mcu -> OTARX,READY,<maxchunk>
    app -> <base64 chunk>                (<= maxchunk decoded bytes)
    mcu -> OTARX,ACK,<total>             (per chunk; NAK/ERR on failure)
    ...until <total> == size...
    mcu -> OTARX,DONE,OK,crc=...,rx=...  |  OTARX,DONE,CRCFAIL,...

base64 keeps the payload free of the ESP bridge's '##...##' magic strings, and
the per-chunk ACK gate paces the sender so the STM32's RX ring + ~400 ms sector
erases can never overrun the transfer.
"""
from __future__ import annotations

import base64
import queue
import threading
import time
import zlib
from pathlib import Path
from typing import Callable, Optional


class OtaError(Exception):
    pass


class OtaUploader:
    # Generous per-reply timeouts: a WiFi round-trip plus a QSPI sector erase
    # (up to ~0.4 s) can sit behind any single ACK.
    READY_TIMEOUT = 15.0
    ACK_TIMEOUT = 10.0
    DONE_TIMEOUT = 30.0

    def __init__(self, link, path: str,
                 on_progress: Optional[Callable[[int, int], None]] = None,
                 on_log: Optional[Callable[[str], None]] = None):
        self.link = link
        self.path = path
        self.on_progress = on_progress or (lambda done, total: None)
        self.on_log = on_log or (lambda msg: None)
        self._q: "queue.Queue[str]" = queue.Queue()
        self._cancel = False

    def cancel(self):
        """Request an abort; the transfer stops at the next chunk boundary and
        tells the firmware to invalidate the half-written slot."""
        self._cancel = True

    # -- response tap (runs on the link's rx thread) -----------------------
    def _sniff(self, line: str):
        s = line.strip()
        if s.startswith("OTARX"):           # OTA replies only; GUI still sees all
            self._q.put(s)

    def _wait(self, prefix: str, timeout: float) -> str:
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise OtaError(f"timed out waiting for {prefix}")
            try:
                line = self._q.get(timeout=min(0.25, remaining))
            except queue.Empty:
                continue
            if line.startswith(prefix):
                return line
            if ",NAK," in line or ",ERR," in line:
                raise OtaError(f"firmware rejected transfer: {line}")

    # -- main flow ---------------------------------------------------------
    def upload(self) -> bool:
        data = Path(self.path).read_bytes()
        size = len(data)
        if size == 0:
            raise OtaError("firmware file is empty")
        crc = zlib.crc32(data) & 0xFFFFFFFF

        self.on_log(f"[ota] {Path(self.path).name}: {size} bytes, crc32={crc:08x}")
        self.link.set_sniffer(self._sniff)
        try:
            while not self._q.empty():          # drop any stale replies
                self._q.get_nowait()

            self.link.send_line(f"otarx {size} {crc:08x}")
            ready = self._wait("OTARX,READY", self.READY_TIMEOUT)
            try:
                maxchunk = int(ready.split(",")[2])
            except (IndexError, ValueError):
                maxchunk = 384
            maxchunk = max(48, min(maxchunk, 384))
            self.on_log(f"[ota] staging started (chunk={maxchunk} B)")

            sent = 0
            for off in range(0, size, maxchunk):
                if self._cancel:
                    self.link.send_line("OTAABORT")
                    raise OtaError("cancelled by user")
                piece = data[off:off + maxchunk]
                self.link.send_line(base64.b64encode(piece).decode("ascii"))
                ack = self._wait("OTARX,ACK", self.ACK_TIMEOUT)
                try:
                    total = int(ack.split(",")[2])
                except (IndexError, ValueError):
                    total = sent + len(piece)
                sent += len(piece)
                if total != sent:
                    raise OtaError(f"ACK desync: mcu={total} app={sent}")
                self.on_progress(sent, size)

            done = self._wait("OTARX,DONE", self.DONE_TIMEOUT)
            if ",OK," not in done:
                raise OtaError(f"verify failed: {done}")
            self.on_log(f"[ota] {done}")
            self.on_log("[ota] image staged + CRC verified in QSPI  ✓")
            return True
        finally:
            self.link.set_sniffer(None)

    def upload_async(self, on_done: Callable[[bool, Optional[str]], None]):
        def worker():
            try:
                ok = self.upload()
                on_done(ok, None)
            except Exception as e:              # noqa: BLE001 - surfaced in UI
                self.on_log(f"[ota] FAILED: {e}")
                on_done(False, str(e))
        threading.Thread(target=worker, daemon=True).start()
