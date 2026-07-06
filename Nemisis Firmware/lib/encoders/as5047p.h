/**
 ******************************************************************************
 * @file    as5047p.h
 * @brief   Driver / bring-up checker for the AMS AS5047P-ATSM 14-bit magnetic
 *          rotary encoder over SPI.
 *
 * The board has the LEFT encoder (ENCODERL) on SPI3 with a software chip-select
 * on PA4, plus its incremental ABI outputs feeding TIM1 (PC0=A, PA9=B) for
 * quadrature counting. This driver covers the SPI absolute-angle interface;
 * use TIM1 (htim1) separately for the incremental count.
 *
 * Wiring on this board (from CubeMX / hal_msp.c):
 *     PA4  -> CSn  (ENCODERL_CS, plain GPIO output, software NSS, idle HIGH)
 *     PC10 -> CLK  (SPI3_SCK)
 *     PC11 -> MISO (SPI3_MISO, data from encoder)
 *     PC12 -> MOSI (SPI3_MOSI, data to encoder)
 *
 * SPI mode 1 (CPOL=0, CPHA=1 -> SPI_PHASE_2EDGE), MSB first, 16-bit frames sent
 * as 2x 8-bit bytes, max clock 10 MHz. The AS5047P will NOT respond on mode 0.
 *
 * Frame format (16 bits, MSB first):
 *   bit15 = even parity over bits 14..0
 *   bit14 = R/W   (1 = read, 0 = write)  on commands
 *           error flag (EF)              on the returned data word
 *   bit13..0 = address (command) or data (response)
 *
 * Quick start:
 *     #include "as5047p.h"
 *     AS5047P_Handle enc;
 *     AS5047P_Init(&enc, &hspi3, ENCODERL_CS_GPIO_Port, ENCODERL_CS_Pin);
 *     uint16_t raw = AS5047P_ReadAngleRaw(&enc);   // 0..16383
 *     float deg    = AS5047P_ReadAngleDeg(&enc);   // 0..360
 ******************************************************************************
 */
#ifndef AS5047P_H
#define AS5047P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"        /* HAL + SPI_HandleTypeDef + ENCODERL_CS_* defines */
#include <stdint.h>
#include <stdbool.h>

/* Volatile (read) register addresses. */
#define AS5047P_REG_NOP        0x0000U
#define AS5047P_REG_ERRFL      0x0001U   /* error flags (clears on read) */
#define AS5047P_REG_PROG       0x0003U
#define AS5047P_REG_DIAAGC     0x3FFCU   /* diagnostics + automatic gain control */
#define AS5047P_REG_MAG        0x3FFDU   /* CORDIC magnitude */
#define AS5047P_REG_ANGLEUNC   0x3FFEU   /* angle, no dynamic error compensation */
#define AS5047P_REG_ANGLECOM   0x3FFFU   /* angle WITH dynamic error compensation */

#define AS5047P_RESOLUTION     16384U    /* 14-bit -> counts per revolution */

/* One encoder instance: SPI bus + its software chip-select pin. */
typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef      *cs_port;
  uint16_t           cs_pin;
} AS5047P_Handle;

/** Bind a handle to its SPI bus + CS pin and park CS high (deselected). */
void AS5047P_Init(AS5047P_Handle *enc, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

/** Read one register. Returns true if the frame parity checked out and the
 *  error flag was clear; *out holds the 14-bit payload. */
bool AS5047P_ReadRegister(AS5047P_Handle *enc, uint16_t addr, uint16_t *out);

/** Raw absolute angle 0..16383 (ANGLECOM). Returns 0xFFFF on a bad frame. */
uint16_t AS5047P_ReadAngleRaw(AS5047P_Handle *enc);

/** Absolute angle in degrees 0..360. Returns -1.0f on a bad frame. */
float AS5047P_ReadAngleDeg(AS5047P_Handle *enc);

/** Probe: read ERRFL + a valid angle frame. True if the chip is talking and
 *  reports no error. Good for a one-shot bring-up check. */
bool AS5047P_Probe(AS5047P_Handle *enc);

#ifdef __cplusplus
}
#endif

#endif /* AS5047P_H */
