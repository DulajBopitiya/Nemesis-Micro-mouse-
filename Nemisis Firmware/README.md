# Nemisis Firmware

Firmware for the **Nemisis** micromouse board — a custom PCB built around the
**STM32G474RET6** (Cortex-M4F @ 170 MHz). Built with **PlatformIO** using the
**STM32Cube (HAL)** framework, with peripherals configured in **STM32CubeMX**
(`NEMSIS.ioc`).

> Note: the PlatformIO environment is named `nucleo_g474re` and reports as a
> Nucleo board, but the actual target is the custom micromouse PCB. Only the
> MCU (STM32G474RE) matters for the toolchain; pin assignments below are for
> the custom board.

## Toolchain / project layout

This project uses the STM32CubeMX directory layout, so `platformio.ini` points
PlatformIO at it instead of the default `src/` / `include/`:

```ini
[platformio]
src_dir     = Core/Src    ; CubeMX-generated sources
include_dir = Core/Inc    ; CubeMX-generated headers

[env:nucleo_g474re]
platform        = ststm32
board           = nucleo_g474re
framework       = stm32cube
upload_protocol = jlink
debug_tool      = jlink
```

| Path            | Purpose                                                       |
|-----------------|---------------------------------------------------------------|
| `Core/Src`      | CubeMX-generated app sources (`main.c`, IT, MSP, system)      |
| `Core/Inc`      | CubeMX-generated headers (`main.h`, HAL config, pin defines)  |
| `lib/`          | Project libraries — auto-compiled by PlatformIO (see below)   |
| `NEMSIS.ioc`    | STM32CubeMX project — regenerating only touches `Core/`       |

Regenerating from CubeMX is safe: it only rewrites `Core/`, and all custom code
lives either inside `/* USER CODE */` guards or in `lib/`.

## Building & flashing

`pio` is not on PATH; use the PlatformIO penv binary (or the VS Code toolbar):

```sh
# build
~/.platformio/penv/Scripts/platformio.exe run -e nucleo_g474re

# upload (see docs/FLASHING.md for the J-Link details)
~/.platformio/penv/Scripts/platformio.exe run -e nucleo_g474re -t upload
```

Flashing is done over **SWD with a SEGGER J-Link**. There is an important
gotcha with the bundled vs. installed J-Link software — see
[docs/FLASHING.md](docs/FLASHING.md).

## Documentation

- [docs/HARDWARE.md](docs/HARDWARE.md) — pin map and peripheral configuration
- [docs/LIBRARIES.md](docs/LIBRARIES.md) — `buzzer` and `ws2812` driver usage
- [docs/FLASHING.md](docs/FLASHING.md) — J-Link / SWD upload setup and pitfalls
