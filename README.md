<div align="center">

<img src="assets/fabvolt-banner.png" alt="FabVolt Technologies" width="720">

# 🐭 NEMESIS — Micromouse

### A custom STM32G474 half-size maze-solving robot, its WiFi debug bridge, and a live tuning cockpit.

<sup>A **FabVolt Technologies** project</sup>

[![MCU](https://img.shields.io/badge/MCU-STM32G474RET6-03234b?logo=stmicroelectronics&logoColor=white)](https://www.st.com/en/microcontrollers-microprocessors/stm32g474re.html)
[![Bridge](https://img.shields.io/badge/Link-ESP32--C3%20WiFi-e7352c?logo=espressif&logoColor=white)](https://www.espressif.com/en/products/socs/esp32-c3)
[![Toolchain](https://img.shields.io/badge/Build-PlatformIO-orange?logo=platformio&logoColor=white)](https://platformio.org/)
[![Debug app](https://img.shields.io/badge/App-PySide6%20%2B%20pyqtgraph-3776ab?logo=python&logoColor=white)](https://www.qt.io/qt-for-python)
[![Algorithm](https://img.shields.io/badge/Solver-Flood--fill-46d17a)]()

</div>

---

## What is this?

**NEMESIS** is a from-scratch [micromouse](https://en.wikipedia.org/wiki/Micromouse) — an autonomous robot that maps an unknown 16×16 maze, floods it to find the shortest route, and then speed-runs to the centre. This repository holds the **whole ecosystem**, not just the robot:

1. **🤖 STM32 firmware** — the real-time brain: 1 kHz cascaded PID control, sensor fusion, flood-fill search, and speed-run profiling.
2. **📡 ESP32-C3 WiFi bridge** — turns the robot's debug console into a wireless link so it can be tuned untethered, mid-run.
3. **🖥️ Python debug cockpit** — a live PySide6 app: real-time plots, a rendered maze map, PID tuning, and diagnostic capture.

Together they make tuning a micromouse feel less like flashing-and-praying and more like driving a car with a dashboard.

---

## 🧭 Architecture

```
        ┌──────────────────────────────┐        ┌───────────────────────┐
        │        NEMESIS robot          │        │      Your laptop       │
        │                               │  WiFi  │                        │
        │  ┌────────────┐   UART        │◀──────▶│  ┌──────────────────┐  │
        │  │  STM32G474 │◀────────────▶ │ ESP32  │  │  Python cockpit   │  │
        │  │  firmware  │   console     │  -C3   │  │  gui.py           │  │
        │  └────┬───────┘   protocol    │ bridge │  │  · live plots     │  │
        │       │                       │        │  │  · maze map       │  │
        │  IMU · encoders · IR walls ·  │        │  │  · PID tuning     │  │
        │  motors · vacuum · fuel gauge │        │  │  · JSONL recorder │  │
        └──────────────────────────────┘        │  └──────────────────┘  │
                    ▲                            └───────────────────────┘
                    │  SWD / J-Link (flashing + ultimate recovery net)
```

The **same human-readable text protocol** flows over the J-Link RTT terminal *and* the WiFi link, so anything you can do on the wire you can do wirelessly.

---

## 📂 Repository layout

| Folder | What it is |
|---|---|
| [`Nemisis Firmware/`](./Nemisis%20Firmware) | STM32G474 firmware — PlatformIO + CubeMX. Control loop, solver, sensor & motor drivers, persistent maze store. |
| [`Nemisis ESP32C3/`](./Nemisis%20ESP32C3) | ESP32-C3 WiFi ↔ UART bridge + captive-portal WiFi provisioning. |
| [`NEMISIS PYTHON DEBUG APPLICATION/`](./NEMISIS%20PYTHON%20DEBUG%20APPLICATION) | PySide6 desktop cockpit: plots, maze view, tuning, recorder. |

---

## 🔩 Hardware

| Subsystem | Part | Bus |
|---|---|---|
| MCU | STM32G474RET6 (170 MHz, 512 KB flash) | — |
| IMU | ICM-42688-P (gyro + accel) | SPI1 |
| Wheel encoders | AS5047P magnetic (×2) | SPI3 + TIM quadrature |
| Motor driver | DRV8874 (×2) | TIM3 PWM |
| Fuel gauge | MAX17049 | I²C3 |
| Wall sensing | IR emitter/phototransistor array | ADC + DMA |
| Downforce | Vacuum impeller | PWM |
| WiFi link | ESP32-C3 | UART |
| Status | WS2812 RGB | — |

> Custom board — **24 MHz HSE crystal** (not the 8 MHz Nucleo default), so builds force `-D HSE_VALUE=24000000`.

---

## 🧠 Firmware highlights

- **1 kHz cascaded controller** — outer velocity + heading loops feed an inner motor loop; hardware-timer PWM keeps it deterministic.
- **Flood-fill solver** — sim-proven, ported to `lib/solver`; searches to the goal, then plans a turn-weighted shortest route home.
- **Navigator** — cell-accurate odometry with non-stop multi-cell runs and per-cell wall latching.
- **Persistent maze memory** — the learned map survives a battery swap (stored in the top pages of internal flash).
- **Speed-run profiling** — √(a·L) per-segment velocity planning, smooth (non-stop) turns, and vacuum downforce for grip.
- **Safety first** — battery-voltage cutoff for motors/vacuum; SWD/J-Link is always the ultimate recovery path.

---

## 🚀 Getting started

### Firmware & bridge (PlatformIO)
```bash
# STM32 firmware
cd "Nemisis Firmware"
pio run                 # build
pio run -t upload       # flash via J-Link / SWD

# ESP32-C3 bridge
cd "../Nemisis ESP32C3"
pio run -t upload
```

### Debug cockpit (Python)
```bash
cd "NEMISIS PYTHON DEBUG APPLICATION"
python -m venv .venv && .venv/Scripts/activate     # Windows
pip install -r requirements.txt
python gui.py            # connects to the bridge at 192.168.4.1:3333
```

---

## 🗺️ Roadmap

- [ ] **OTA firmware updates** over the WiFi link (staged via external flash, CRC-validated, one-button rollback) — no more nerve-wracking SWD sessions.
- [ ] Diagonal / smooth speed-run geometry tuned against logged real-run data.
- [ ] Auto-calibration on the start cell (self-centering + adaptive thresholds).

---

<div align="center">

<img src="assets/fabvolt-banner.png" alt="FabVolt Technologies" width="360">

**© FabVolt Technologies** · Built with too much coffee and a soldering iron. 🔧

</div>
