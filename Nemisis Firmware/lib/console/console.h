/**
 ******************************************************************************
 * @file    console.h
 * @brief   Interactive debug console for the Nemisis micromouse.
 *
 * One command set, two transports at once:
 *   - SEGGER RTT   (over the J-Link/SWD link, view in "J-Link RTT Viewer")
 *   - USART1       (115200 8N1, to the ESP32-C3 -> WiFi -> Python app)
 *
 * Everything typed on either link is parsed the same way and every reply is
 * echoed to BOTH links, so the RTT terminal and the Python app see the same
 * session. This is the shared protocol the WiFi/BLE tooling builds on later.
 *
 * Type 'help' (or 'debug', or just Enter) for the menu. Subsystems:
 *   encoder / imu / sensors / battery   -> live streams ('stop' or Enter ends)
 *   motor / vacuum / buzzer             -> action commands
 *
 * Usage (in main.c):
 *     ConsoleCtx ctx = { .hspi_imu = &hspi1, ... , .huart = &huart1 };
 *     Console_Init(&ctx);
 *     while (1) { Console_Task(); }     // non-blocking, call every loop
 ******************************************************************************
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* All the peripheral handles the console needs to poke each subsystem.
   Fill this in main.c from the CubeMX-generated handles. */
typedef struct
{
  SPI_HandleTypeDef  *hspi_imu;    /* ICM-42688-P            (hspi1) */
  SPI_HandleTypeDef  *hspi_enc;    /* AS5047P x2 shared bus  (hspi3) */
  I2C_HandleTypeDef  *hi2c_fuel;   /* MAX17049 fuel gauge    (hi2c3) */
  ADC_HandleTypeDef  *hadc1;       /* 5 IR receivers                 */
  ADC_HandleTypeDef  *hadc4;       /* 1 IR receiver + motor sense    */
  TIM_HandleTypeDef  *htim_fan;    /* vacuum PWM             (htim8, CH3) */
  TIM_HandleTypeDef  *htim_mot;    /* motor PWM              (htim3) */
  TIM_HandleTypeDef  *htim_encL;   /* left  ABI quadrature   (htim1) */
  TIM_HandleTypeDef  *htim_encR;   /* right ABI quadrature   (htim2) */
  UART_HandleTypeDef *huart;       /* link to ESP32-C3       (huart1) */
} ConsoleCtx;

/** Bind handles, init the subsystems the console drives (encoders, motor
 *  timing/ADC cal), and print the banner + menu. Call once after the MX_*
 *  inits and the existing boot self-test. */
void Console_Init(const ConsoleCtx *ctx);

/** Non-blocking pump: poll both links for a command line, dispatch it, and
 *  emit one sample if a live stream is active. Call every superloop pass. */
void Console_Task(void);

/** Mute/unmute the USART1 (ESP32 -> WiFi/BLE) output. When muted the wireless
 *  link is silenced so an offline run has no comms traffic jittering the main
 *  loop; RTT (wired J-Link) is unaffected. Console_Task still runs (and still
 *  pumps the solver), it just doesn't transmit over the radio link. */
void Console_SetMuted(bool muted);
bool Console_IsMuted(void);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */
