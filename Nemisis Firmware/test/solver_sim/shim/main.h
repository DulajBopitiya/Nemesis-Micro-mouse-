/**
 * main.h - HOST-BUILD SHIM (not the firmware's real main.h).
 *
 * Lets lib/solver (and the public driver headers it includes: control.h,
 * sensors.h, battery.h) compile natively on a PC for the headless solver
 * regression tests, WITHOUT pulling in the STM32 HAL. Placed first on the
 * include path so it shadows Core/Inc/main.h.
 *
 * It only needs to satisfy what those headers reference: a few opaque HAL
 * handle types (they only ever appear as pointers in struct members / params),
 * plus the CMSIS bits the sim path actually reads (SystemCoreClock, DWT->CYCCNT,
 * HAL_GetTick). None of the real hardware is present; see stubs.c.
 */
#ifndef MAIN_H_HOST_SHIM
#define MAIN_H_HOST_SHIM

#include <stdint.h>
#include <stdbool.h>

/* Opaque HAL handle types - only pointers to these appear in the headers. */
typedef struct { int _dummy; } TIM_HandleTypeDef;
typedef struct { int _dummy; } SPI_HandleTypeDef;
typedef struct { int _dummy; } ADC_HandleTypeDef;
typedef struct { int _dummy; } I2C_HandleTypeDef;
typedef struct { int _dummy; } GPIO_TypeDef;

/* CMSIS pieces the solver's sim path touches. */
extern uint32_t SystemCoreClock;
uint32_t HAL_GetTick(void);

/* Minimal DWT: the sim reads DWT->CYCCNT for per-decision timing. On the host
   there's no cycle counter, so it reads 0 (the us metric is MCU-only). */
typedef struct { volatile uint32_t CYCCNT; } DWT_Type_Host;
extern DWT_Type_Host *DWT;

#endif /* MAIN_H_HOST_SHIM */
