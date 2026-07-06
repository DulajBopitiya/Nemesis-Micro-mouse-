/**
 ******************************************************************************
 * @file    buzzer.h
 * @brief   Passive buzzer driver (PB8 via MOSFET) with tone + melody support.
 *
 * A passive buzzer has no oscillator, so a tone is produced by toggling the
 * pin with a square wave at the desired frequency. Timing uses the DWT cycle
 * counter for microsecond accuracy (HAL_Delay is millisecond-only).
 *
 * Use the NOTE_* defines and the BuzzerNote/Buzzer_PlayMelody API to build
 * tunes. Frequencies are equal-tempered, A4 = 440 Hz.
 ******************************************************************************
 */
#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* BUZZER_Pin / BUZZER_GPIO_Port + HAL */

/* --- Note frequencies in Hz (0 = rest/silence) ---------------------------- */
#define NOTE_REST  0U

#define NOTE_C4    262U
#define NOTE_CS4   277U
#define NOTE_D4    294U
#define NOTE_DS4   311U
#define NOTE_E4    330U
#define NOTE_F4    349U
#define NOTE_FS4   370U
#define NOTE_G4    392U
#define NOTE_GS4   415U
#define NOTE_A4    440U
#define NOTE_AS4   466U
#define NOTE_B4    494U

#define NOTE_C5    523U
#define NOTE_CS5   554U
#define NOTE_D5    587U
#define NOTE_DS5   622U
#define NOTE_E5    659U
#define NOTE_F5    698U
#define NOTE_FS5   740U
#define NOTE_G5    784U
#define NOTE_GS5   831U
#define NOTE_A5    880U
#define NOTE_AS5   932U
#define NOTE_B5    988U

#define NOTE_C6    1047U
#define NOTE_D6    1175U
#define NOTE_E6    1319U
#define NOTE_F6    1397U
#define NOTE_G6    1568U
#define NOTE_A6    1760U
#define NOTE_B6    1976U
#define NOTE_C7    2093U

/* One entry in a melody: a pitch and how long to hold it. */
typedef struct
{
  uint16_t freq;       /* Hz, NOTE_REST for silence */
  uint16_t duration;   /* milliseconds */
} BuzzerNote;

/** Initialise timing (DWT cycle counter). Call once at startup. */
void Buzzer_Init(void);

/** Play a single tone for duration_ms (blocking). freq_hz == 0 => silence. */
void Buzzer_Tone(uint16_t freq_hz, uint16_t duration_ms);

/** Play an array of notes back-to-back (blocking). */
void Buzzer_PlayMelody(const BuzzerNote *melody, uint16_t length);

/** A short, cheerful power-on chime. */
void Buzzer_StartupSound(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
