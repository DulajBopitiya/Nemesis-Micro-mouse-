/**
 ******************************************************************************
 * @file    ws2812.h
 * @brief   Driver for WS2812B-style addressable RGB LEDs (e.g. XL-1010RGBC-2812B)
 *          driven by a timer PWM channel + DMA.
 *
 * The LED is fed a single-wire 800 kHz-ish bitstream. Each colour bit is sent
 * as one PWM pulse: a short high time = logic '0', a long high time = logic '1'.
 * DMA streams the per-bit compare values into the timer's CCR register so the
 * CPU is free during transmission.
 *
 * Wiring on this board: data line = PA2 = TIM15_CH1 (DMA1_Channel1).
 ******************************************************************************
 */
#ifndef WS2812_H
#define WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* pulls in stm32g4xx_hal.h and the pin defines */

/* Number of LEDs in the chain. The XL-1010RGBC-2812B has one controller per
 * package, so set this to however many you have daisy-chained on PA2. */
#ifndef WS2812_NUM_LEDS
#define WS2812_NUM_LEDS   4U
#endif

/**
 * @brief  Bind the driver to a timer/channel and prepare timing.
 * @param  htim     PWM timer handle (here &htim15)
 * @param  channel  Timer channel (here TIM_CHANNEL_1)
 * @note   Call once after MX_TIM15_Init().
 */
void WS2812_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/** Set one pixel's colour in the frame buffer (0..255 each). Does not transmit. */
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/** Set every pixel to the same colour. Does not transmit. */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

/** Turn every pixel off in the frame buffer. Does not transmit. */
void WS2812_Clear(void);

/**
 * @brief  Push the frame buffer out to the LEDs over DMA.
 *         Blocks until the transfer (plus reset/latch) is complete.
 */
void WS2812_Show(void);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */
