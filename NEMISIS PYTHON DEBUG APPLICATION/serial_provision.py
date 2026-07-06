"""
serial_provision.py - set the mouse's WiFi credentials over the ESP32-C3's USB
serial port.

This is the reliable fallback for when the captive-provisioning portal
(NEMISIS-SETUP AP + http://192.168.4.1) won't cooperate - no AP, phone, DHCP or
captive browser involved. It drives the exact same line commands the ESP32-C3
firmware accepts on its USB serial monitor (see WifiProvision::handleSerialLine
in the ESP32-C3 firmware):

    SSID=<network name>
    PASS=<password>
    SAVE

On SAVE the board writes the credentials to NVS and reboots to join that
network. Because these commands are only *read* from USB serial (the WiFi bridge
otherwise ignores serial input), this works whether the board is sitting in the
setup portal or already running as the transparent bridge.

Requires pyserial (`pip install pyserial>=3.5`).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, List, Optional

try:
    import serial                       # pyserial
    from serial.tools import list_ports
    _HAVE_PYSERIAL = True
except ImportError:                     # keep import of this module harmless
    serial = None                       # type: ignore
    list_ports = None                   # type: ignore
    _HAVE_PYSERIAL = False


BAUD = 115200


@dataclass
class Port:
    """A serial port candidate for the picker."""
    device: str                         # e.g. "COM7"
    description: str                    # human label from the OS

    @property
    def label(self) -> str:
        return f"{self.device} — {self.description}" if self.description else self.device


def pyserial_available() -> bool:
    return _HAVE_PYSERIAL


def list_serial_ports() -> List[Port]:
    """All serial ports the OS knows about, likeliest-first.

    The ESP32-C3 typically enumerates as a USB-JTAG/serial device or a
    CP210x/CH340 USB-UART bridge, so we float those to the top but never hide
    anything - the user may know better which COM port is theirs.
    """
    if not _HAVE_PYSERIAL:
        return []
    ports = [Port(p.device, p.description or "") for p in list_ports.comports()]

    def score(p: Port) -> int:
        d = (p.description or "").lower()
        hints = ("esp32", "usb jtag", "usb-serial", "cp210", "ch340", "ch910",
                 "silicon labs", "usb serial", "wch")
        return 0 if any(h in d for h in hints) else 1

    ports.sort(key=lambda p: (score(p), p.device))
    return ports


def send_credentials(port: str, ssid: str, password: str,
                     log: Optional[Callable[[str], None]] = None,
                     read_secs: float = 2.5) -> None:
    """Open `port`, push SSID=/PASS=/SAVE, and stream the board's replies to
    `log`. Raises on any serial error (bad port, in use, unplugged).

    The board reboots itself on SAVE, so the port may drop mid-read - that's
    expected and not treated as a failure once SAVE has been sent.
    """
    if not _HAVE_PYSERIAL:
        raise RuntimeError(
            "pyserial is not installed - run:  pip install pyserial")
    if not ssid:
        raise ValueError("SSID must not be empty")

    def emit(msg: str) -> None:
        if log:
            log(msg)

    # Build the port WITHOUT opening first so we can clear DTR/RTS: many
    # ESP32 boards' USB-UART bridges auto-reset the chip when those lines pulse
    # on open. Suppressing that keeps a live board alive; if a reset happens
    # anyway (native USB-CDC varies), the settle below covers the reboot.
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = 0.2
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    ser.open()
    try:
        # Let the board settle (and finish booting if the open did reset it) so
        # it's actively reading serial before we send. The firmware accepts
        # SSID=/PASS=/SAVE both in the portal loop and the normal bridge loop,
        # so whichever state it lands in will pick these up. Drain any banner.
        emit("   (waiting for the board to be ready…)")
        time.sleep(1.8)
        _drain(ser, emit, 0.6)

        for line in (f"SSID={ssid}", f"PASS={password}", "SAVE"):
            shown = line if not line.startswith("PASS=") else "PASS=" + "*" * len(password)
            emit(f">> {shown}")
            ser.write((line + "\r\n").encode("utf-8", errors="replace"))
            ser.flush()
            _drain(ser, emit, 0.4)

        # After SAVE the firmware prints "Saved ... rebooting..." then resets.
        _drain(ser, emit, read_secs)
    finally:
        try:
            ser.close()
        except Exception:
            pass


def _drain(ser, emit: Callable[[str], None], secs: float) -> None:
    """Read whatever the board sends for up to `secs`, emitting whole lines."""
    end = time.time() + secs
    buf = b""
    while time.time() < end:
        try:
            chunk = ser.read(256)
        except Exception:
            break                        # port went away (expected after reboot)
        if chunk:
            buf += chunk.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", errors="replace").strip()
                if text:
                    emit(f"   {text}")
    if buf:
        text = buf.decode("utf-8", errors="replace").strip()
        if text:
            emit(f"   {text}")
