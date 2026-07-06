# Nemisis Python Debug Application

A WiFi debug/tuning client for the Nemisis micromouse. It talks to the STM32G474
debug console through the **ESP32-C3 WiFi bridge** over a TCP socket, using the
same text protocol the firmware exposes over J-Link RTT.

```
[ PC: this app ] --TCP/WiFi--> [ ESP32-C3 bridge ] --UART 115200--> [ STM32G474 console ]
```

Because the protocol is identical on both links, anything you can do in the
J-Link RTT Viewer you can do here, and vice-versa.

There are two front-ends sharing one transport (`connection.py`):

- **`gui.py`** - graphical app (PySide6 + pyqtgraph) with buttons, a console
  panel, motor/vacuum controls, and **live plots** of the streaming data.
- **`nemisis_debug.py`** - lightweight terminal client (stdlib only).

## Setup

```
python -m venv .venv
.venv\Scripts\activate            # Windows
pip install -r requirements.txt   # PySide6 + pyqtgraph (for the GUI)
```

The CLI client needs no dependencies; only the GUI needs the packages above.

## Run

1. Flash the ESP32-C3 bridge (see `../Nemisis ESP32C3`).
2. Power the mouse; join the WiFi network **`NEMISIS-DBG`** (password `micromouse`).
3. Launch a front-end:

```
python gui.py                      # graphical (defaults to 192.168.4.1:3333)
python gui.py --host 192.168.4.1 --port 3333

python nemisis_debug.py            # terminal version
python nemisis_debug.py --wait     # wait until the bridge is reachable
```

## GUI guide

- **Connection bar** - set host/port, click **Connect** (status dot turns green).
- **Subsystem buttons** - `encoder` / `imu` / `sensors` / `battery` start the
  matching live stream *and* switch the plot to it. `buzzer` chimes; **STOP**
  ends streams, coasts the motors and turns the vacuum off.
- **Live plot** (left) - auto-follows whichever stream is active and lays it out
  in tidy stacked panels with real units (600-sample rolling window, shared
  time axis):
  - **IMU** → Acceleration (g) X/Y/Z  +  Gyro (°/s) X/Y/Z
  - **Encoders** → Wheel angle (°, 0-360) L/R  +  Quadrature count L/R
  - **IR** → Left (L_LM/L_M/L_F)  +  Right (R_F/R_M/R_RM)
  - **Battery** → Voltage (V) pack/cell  +  State of charge (%, 0-100)
- **Plot / Maze tabs** (left) - the live plot, or a live **maze map**:
  - Set the grid **Size** (your home maze is 6×6) and **Goal** (`center`, or
    `x y` for a single cell), then **→ Robot** to push `maze`/`goal` to the
    mouse, and **Solve** to start an autonomous run.
  - As the mouse solves, the map fills in: **white** walls = robot saw it *and*
    you painted it, **orange** = robot-only (possible phantom), **dashed grey**
    = a wall you painted that the robot hasn't found. The robot, its **path**,
    and per-cell **flood** values are drawn live.
  - Tick **Edit walls** and click cell edges to paint the *actual* maze; the
    status line then reports **wall mismatches** (sensor errors at a glance).
- **⏺ Record** (top bar) - capture a structured **JSON Lines** diagnostic log to
  `logs/nemisis-<timestamp>.jsonl` (+ a `.summary.json`): every IMU/encoder/IR/
  wall/solve event on one shared clock, for offline analysis. Stop to finalise.
- **Console** (right) - full text log of everything sent/received.
- **Controls** - motor target (1/2/both) + speed + duration + **Run**, and a
  vacuum slider (sends `vacuum <n>` on release).
- **Command line** (bottom) - type any console command directly, Enter to send.

The plot understands these console lines automatically: IMU, encoders, IR
sensors, battery, and the future `TLM,...` PID telemetry (see Roadmap).

## On-device commands (work from either front-end)

| Command                         | What it does                                    |
|---------------------------------|-------------------------------------------------|
| `encoder` / `1`                 | live wheel encoder angles + quadrature counts   |
| `imu` / `2`                     | live accel / gyro / temperature                 |
| `sensors` / `3`                 | live 6× IR reflectance                          |
| `motor 1\|2\|b <-100..100> [ms]`| drive a motor (or both) at a duty for some ms   |
| `motor test`                    | scripted drive sequence (put it on a stand!)    |
| `motor stop`                    | coast both motors                               |
| `vacuum <0..100>` / `vacuum off`| set vacuum fan duty                             |
| `battery` / `6`                 | live fuel-gauge readout                         |
| `buzzer` / `buzzer <hz> <ms>`   | chime / custom tone                             |
| `drive <cps>` / `move <mm>`     | closed-loop straight run (continuous / fixed)   |
| `turn <l\|r\|180\|deg>`         | pivot in place by a fixed angle (gyro-closed)   |
| `tcfg <rate> [accel]`           | pivot slew rate + accel (°/s; `tcfg` = show)    |
| `maze [n]` / `goal [center\|x y]`| set maze size + goal for the solver            |
| `solve [cps]`                   | autonomous flood-fill search to goal + back     |
| `stop` (or Enter)               | end the active live stream / abort a solve      |

### CLI-only local commands (start with `:`)

| Command         | What it does                                  |
|-----------------|-----------------------------------------------|
| `:help`         | list these local commands                     |
| `:log <file>`   | record the session to a file (run again to stop) |
| `:raw <text>`   | send text with no trailing carriage return    |
| `:quit`         | exit                                          |

## Files

- `gui.py` - graphical client (PySide6 + pyqtgraph)
- `nemisis_debug.py` - terminal client
- `connection.py` - `NemisisLink`, the TCP transport (line-oriented, threaded reader)

## Roadmap (PID tuning & plotting)

The transport (`connection.py`) is isolated so the UI never cares how bytes
move. The GUI already plots a `TLM,<t>,<setpoint>,<measured>,<output>` line, so:

1. **Firmware:** add console commands `pid kp <x>` / `ki` / `kd`, `step <speed>`,
   and `tlm on`/`tlm off` that streams those CSV lines at a fixed rate.
2. **App:** add sliders/inputs that send the `pid ...` commands; the live plot
   already visualises the response.
3. **Optional BLE backend:** implement the same `connect/send_line/on_line`
   surface as `NemisisLink` and both front-ends work unchanged.
