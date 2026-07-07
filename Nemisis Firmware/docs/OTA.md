# OTA firmware updates (over WiFi)

Update the mouse's firmware over the WiFi link instead of the SWD/J-Link cable.

## Flash map
```
0x08000000  bootloader   32 KB   (env:boot in bootloader/) - never OTA'd
0x08008000  app          468 KB  (env:app_ota) - the OTA-updatable firmware
0x0807D000  persistent   12 KB   maze / calibration / settings
```
External QSPI flash (W25Q32JW, 4 MB): incoming image slot @ 0x000000, metadata
@ 0x0F0000, golden @ 0x100000 (reserved for future rollback), scratch @ 0x3FF000.

## One-time setup (via SWD/J-Link)
Flash the bootloader **and** the relocated app together in one J-Link session
(separate `pio -t upload`s mass-erase each other):

```
# build both
pio run -e app_ota
cd bootloader && pio run && cd ..
# flash both at once
"C:\Program Files\SEGGER\JLink_V924a\JLink.exe" -CommanderScript tools/flash_ota.jlink -ExitOnError 1 -NoGui 1
```

## Everyday update (over WiFi, no cable)
1. Edit the firmware.
2. **Build the `app_ota` env** (NOT the default build button, which builds the
   0x08000000 recovery image):
   - `pio run -e app_ota`, or pick `env:app_ota` in the PlatformIO env switcher.
3. In the debug app: **"Update firmware…"** → the dialog defaults to
   `.pio/build/app_ota/firmware.bin`. Stage it.
   - Mouse LEDs fill **blue** as it uploads, **green** when CRC-verified.
4. Answer **"Install it now?"** → **Yes** (or type `ota apply` in the console).
   - Mouse resets → bootloader copies QSPI → app slot, verifies, reboots.
   - New firmware flashes the LEDs **green** on first boot ("installed and running").
   - The WiFi link drops during the reset/copy (~few seconds); reconnect.

## Console commands
- `qspi id | status | read <addr> [n] | test` - external flash driver/diagnostics
- `otarx <size> <crc32>` - firmware receive (the app drives this; base64 payload)
- `ota info | verify | apply` - inspect / re-CRC / install the staged image

## Safety net
- SWD/J-Link always recovers: flashing a normal `nucleo_g474re` build overwrites
  both the bootloader and the app slot and boots directly.
  `pio run -e nucleo_g474re -t upload` (or re-run `tools/flash_ota.jlink`).
- Staging can't brick anything (no jump). `ota apply` only proceeds if the staged
  image CRC-verifies. The bootloader only jumps to a sane app vector.
- **Current limit (single-slot):** a power loss *during* the bootloader's copy can
  leave a partial app - recover via SWD. Golden-image auto-rollback is the planned
  B3 step to close that window.
