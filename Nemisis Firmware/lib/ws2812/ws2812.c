/**
 ******************************************************************************
 * @file    ws2812.c
 * @brief   WS2812B (XL-1010RGBC-2812B) driver - timer PWM + DMA. See ws2812.h.
 ******************************************************************************
 */
#include "ws2812.h"

/* Bits per LED: 8 green + 8 red + 8 blue (WS2812B order is G,R,B, MSB first). */
#define WS2812_BITS_PER_LED   24U

/* Trailing low slots to generate the >50us reset/latch pulse after the data.
 * One slot is one PWM period (~1.18us here) of 0% duty, i.e. line held low.
 * 48 slots ~= 56us, comfortably above the 50us minimum. */
#define WS2812_RESET_SLOTS    48U

#define WS2812_BUF_LEN  (WS2812_NUM_LEDS * WS2812_BITS_PER_LED + WS2812_RESET_SLOTS)

/* Per-LED colour store, in transmit order (G, R, B). */
static uint8_t  ws_grb[WS2812_NUM_LEDS][3];

/* DMA-fed compare values. uint16_t because the DMA is half-word aligned. */
static uint16_t ws_dma[WS2812_BUF_LEN];

/* Compare value for a '0' bit (short high) and a '1' bit (long high).
 * Derived from the timer's auto-reload at init so the duty stays correct
 * regardless of the exact ARR chosen in CubeMX. */
static uint16_t ws_lo;
static uint16_t ws_hi;

static TIM_HandleTypeDef *ws_htim;
static uint32_t           ws_channel;
static volatile uint8_t   ws_busy;

void WS2812_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
  ws_htim    = htim;
  ws_channel = channel;
  ws_busy    = 0U;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);   /* 99 in this project */

  /* WS2812B nominal: '0' high ~0.4us, '1' high ~0.8us within a ~1.25us bit.
   * Expressed as a fraction of the bit period (ARR+1 counts): ~34% and ~66%. */
  ws_lo = (uint16_t)(((arr + 1U) * 34U) / 100U);
  ws_hi = (uint16_t)(((arr + 1U) * 66U) / 100U);

  WS2812_Clear();
}

void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
  if (index >= WS2812_NUM_LEDS)
  {
    return;
  }
  ws_grb[index][0] = g;
  ws_grb[index][1] = r;
  ws_grb[index][2] = b;
}

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
  for (uint16_t i = 0U; i < WS2812_NUM_LEDS; i++)
  {
    WS2812_SetPixel(i, r, g, b);
  }
}

void WS2812_Clear(void)
{
  for (uint16_t i = 0U; i < WS2812_NUM_LEDS; i++)
  {
    ws_grb[i][0] = 0U;
    ws_grb[i][1] = 0U;
    ws_grb[i][2] = 0U;
  }
}

void WS2812_Show(void)
{
  uint32_t slot = 0U;

  /* Expand every colour byte (MSB first) into one compare value per bit. */
  for (uint16_t led = 0U; led < WS2812_NUM_LEDS; led++)
  {
    for (uint8_t comp = 0U; comp < 3U; comp++)
    {
      uint8_t value = ws_grb[led][comp];
      for (uint8_t bit = 0U; bit < 8U; bit++)
      {
        ws_dma[slot++] = (value & 0x80U) ? ws_hi : ws_lo;
        value <<= 1U;
      }
    }
  }

  /* Reset/latch: hold the line low. */
  while (slot < WS2812_BUF_LEN)
  {
    ws_dma[slot++] = 0U;
  }

  /* Fire the transfer and wait for the completion callback below. */
  ws_busy = 1U;
  HAL_TIM_PWM_Start_DMA(ws_htim, ws_channel, (uint32_t *)ws_dma, WS2812_BUF_LEN);
  while (ws_busy) { }
}

/* Called from the DMA1_Channel1 IRQ (via HAL_DMA_IRQHandler) when the whole
 * buffer has been sent. Stop the PWM/DMA and release the Show() wait. */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == ws_htim)
  {
    HAL_TIM_PWM_Stop_DMA(ws_htim, ws_channel);
    ws_busy = 0U;
  }
}
