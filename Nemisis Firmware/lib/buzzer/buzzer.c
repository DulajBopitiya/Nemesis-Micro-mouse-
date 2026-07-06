/**
 ******************************************************************************
 * @file    buzzer.c
 * @brief   Passive buzzer driver (PB8). See buzzer.h.
 ******************************************************************************
 */
#include "buzzer.h"

/* Gap inserted between consecutive melody notes so repeated/adjacent notes
 * are articulated instead of blurring together. */
#define BUZZER_NOTE_GAP_MS   20U

/* --- Microsecond timing via the DWT cycle counter ------------------------- */
static void dwt_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

void Buzzer_Init(void)
{
  dwt_delay_init();
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void Buzzer_Tone(uint16_t freq_hz, uint16_t duration_ms)
{
  if (freq_hz == 0U)
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(duration_ms);
    return;
  }

  uint32_t half_period_us = 500000U / freq_hz;             /* half of one cycle */
  uint32_t cycles         = ((uint32_t)freq_hz * duration_ms) / 1000U;

  for (uint32_t i = 0U; i < cycles; i++)
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    delay_us(half_period_us);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    delay_us(half_period_us);
  }
}

void Buzzer_PlayMelody(const BuzzerNote *melody, uint16_t length)
{
  for (uint16_t i = 0U; i < length; i++)
  {
    Buzzer_Tone(melody[i].freq, melody[i].duration);
    HAL_Delay(BUZZER_NOTE_GAP_MS);
  }
}

void Buzzer_StartupSound(void)
{
  /* Rising major arpeggio with a little flourish - a clean "power up" chime. */
  static const BuzzerNote startup[] =
  {
    { NOTE_C5,  90 },
    { NOTE_E5,  90 },
    { NOTE_G5,  90 },
    { NOTE_C6, 130 },
    { NOTE_G5,  70 },
    { NOTE_C6, 220 },
  };

  Buzzer_PlayMelody(startup, sizeof(startup) / sizeof(startup[0]));
}
