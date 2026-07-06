# Flashing (SWD / J-Link)

The board is programmed over **SWD** with a **SEGGER J-Link**. `platformio.ini`
sets:

```ini
upload_protocol = jlink
debug_tool      = jlink
```

## The bundled-vs-installed J-Link gotcha

This setup uses a **J-Link CE** (China/OEM Edition) probe. Two J-Link software
installs are in play:

- **PlatformIO's bundled** `tool-jlink` (≈ V9.20) — **fails** to connect to the
  CE probe (`Connecting to J-Link via USB...FAILED: Cannot connect to J-Link`).
  Worse, J-Link Commander still returns exit code 0, so PlatformIO falsely
  reports `[SUCCESS]` even though nothing was flashed.
- **The locally installed** SEGGER software at
  `C:\Program Files\SEGGER\JLink_V924a` (**V9.24a**) — connects fine.

`C:\Program Files\SEGGER\JLink_V924a` has been added to the user `PATH`.

## Reliable flash via the installed J-Link

When the PlatformIO upload misbehaves, flash directly with the working V9.24a
using a J-Link command script (and `-ExitOnError 1` so it can't hang for
minutes on a failed connect):

```
device STM32G474RE
si SWD
speed 4000
connect
loadbin "<project>\.pio\build\nucleo_g474re\firmware.bin", 0x08000000
r
g
exit
```

```sh
& "C:\Program Files\SEGGER\JLink_V924a\JLink.exe" -ExitOnError 1 -CommanderScript flash.jlink
```

Check the probe is detected at all with `JLink.exe -CommanderScript` running
`ShowEmuList` — a healthy CE probe reports e.g.
`ProductName: J-Link CE`.

## SWD wiring (custom board)

| J-Link pin | Signal              |
|------------|---------------------|
| 1          | VTref → board 3V3 (required for the probe to sense target voltage) |
| 7          | SWDIO               |
| 9          | SWCLK               |
| 4 / 6 / 8  | GND                 |
| 15         | nRESET (optional)   |

If you see "target voltage too low / could not connect", VTref is almost always
the missing wire. If J-Link reports an unknown device, pin it explicitly:

```ini
board_build.jlink.device = STM32G474RE
```
