# Libraries

Custom drivers live in `lib/`. PlatformIO's Library Dependency Finder compiles
each `lib/<name>/` automatically and exposes its header — no `platformio.ini`
changes are needed. Just `#include` the header from `Core/Src`.

---

## `lib/buzzer` — passive buzzer + tones (PB8)

Square-wave tone generation for the passive buzzer, with a note table and a
melody player for building tunes.

```c
#include "buzzer.h"

Buzzer_Init();                       // once, after clocks are up
Buzzer_Tone(NOTE_A4, 200);           // single note (Hz, ms); freq 0 = silence
Buzzer_StartupSound();               // built-in power-on chime
```

Build a melody from `BuzzerNote { freq, duration_ms }` entries:

```c
static const BuzzerNote tune[] = {
  { NOTE_E5, 150 }, { NOTE_REST, 50 }, { NOTE_G5, 300 },
};
Buzzer_PlayMelody(tune, sizeof(tune) / sizeof(tune[0]));
```

Note frequencies `NOTE_C4`…`NOTE_C7` and `NOTE_REST` are defined in `buzzer.h`
(equal temperament, A4 = 440 Hz). All calls are **blocking**. Edit
`Buzzer_StartupSound()` in `buzzer.c` to change the boot chime.

---

## `lib/ws2812` — WS2812B addressable RGB LEDs (PA2 / TIM15 + DMA)

Drives a chain of WS2812B-compatible LEDs (`XL-1010RGBC-2812B`) using TIM15 PWM
+ DMA. Colour bits are streamed to `TIM15->CCR1`; the CPU is free during the
transfer, which then blocks only until the DMA completion callback fires.

```c
#include "ws2812.h"

WS2812_Init(&htim15, TIM_CHANNEL_1); // once, after MX_TIM15_Init()

WS2812_SetAll(32, 0, 0);             // all LEDs red (R,G,B 0..255)
WS2812_Show();                       // push buffer to LEDs (blocking)

WS2812_SetPixel(2, 0, 0, 64);        // LED #2 blue (index 0..NUM-1)
WS2812_Show();

WS2812_Clear();                      // all off (in buffer)
WS2812_Show();
```

- **LED count:** `WS2812_NUM_LEDS` (default **4**) — override in `ws2812.h` or
  via a `-D WS2812_NUM_LEDS=N` build flag.
- **Colour order:** pass normal `(r, g, b)`; the driver emits WS2812B's internal
  GRB order. If colours look swapped on a particular part, adjust the byte order
  in `WS2812_SetPixel`.
- **Reset/latch:** 48 trailing zero slots (~56 µs low) terminate each frame.
- **Current:** values are full 0–255; keep them modest (the demo uses 32) since
  4 LEDs at full white draw significant current.
- **3.3 V data:** PA2 drives at 3.3 V. This usually works, but if the first LED
  is unreliable, a logic-level shifter on the data line is the fix.

> The driver implements `HAL_TIM_PWM_PulseFinishedCallback`. If you add other
> DMA-driven PWM timers, that weak callback is shared — guard on the `htim`
> instance (the driver already checks for its own timer).
