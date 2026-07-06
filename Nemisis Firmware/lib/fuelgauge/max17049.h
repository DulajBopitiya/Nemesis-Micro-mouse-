/**
 ******************************************************************************
 * @file    max17049.h
 * @brief   Driver / bring-up checker for the Maxim MAX17049 2-cell ModelGauge
 *          Li-ion fuel gauge on I2C3.
 *
 * Wiring on this board (from CubeMX / hal_msp.c):
 *     PC9 -> FUEL_SDA (I2C3_SDA)
 *     PA8 -> FUEL_SCL (I2C3_SCL)
 *   NOTE: pins are configured NOPULL -> external I2C pull-ups (~4.7k) required.
 *
 * The MAX17049 starts gauging automatically at power-up; no init is needed to
 * read voltage / state-of-charge. All registers are 16-bit, big-endian (MSB
 * first). 7-bit I2C address is fixed at 0x36.
 *
 * Quick start:
 *     #include "max17049.h"
 *     if (MAX17049_Probe(&hi2c3)) {
 *         MAX17049_Status s;
 *         MAX17049_ReadStatus(&hi2c3, &s);   // s.pack_v, s.soc, s.crate ...
 *     }
 ******************************************************************************
 */
#ifndef MAX17049_H
#define MAX17049_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"        /* HAL + I2C_HandleTypeDef */
#include <stdint.h>
#include <stdbool.h>

/* 7-bit address 0x36, shifted to the 8-bit form the HAL expects. */
#define MAX17049_I2C_ADDR   (0x36U << 1)

/* MAX17049 is the 2-cell variant (MAX17048 = 1-cell). */
#define MAX17049_CELLS      2U

typedef struct
{
  uint16_t version;   /* chip silicon version (VERSION reg, sanity check)   */
  float    cell_v;    /* per-cell voltage [V]                               */
  float    pack_v;    /* pack voltage [V] = cell_v * MAX17049_CELLS         */
  float    soc;       /* state of charge [%] 0..100                         */
  float    crate;     /* charge/discharge rate [%/hr], + = charging         */
} MAX17049_Status;

/** True if the device ACKs on the bus. */
bool  MAX17049_Probe(I2C_HandleTypeDef *hi2c);

/** Raw 16-bit register access (handles big-endian byte order). */
bool  MAX17049_Read16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *val);
bool  MAX17049_Write16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t val);

/** Individual scaled reads. Return NAN-free values; check the bool. */
bool  MAX17049_ReadCellVoltage(I2C_HandleTypeDef *hi2c, float *volts);
bool  MAX17049_ReadSOC(I2C_HandleTypeDef *hi2c, float *percent);
bool  MAX17049_ReadChargeRate(I2C_HandleTypeDef *hi2c, float *pct_per_hr);

/** One-shot grab of everything useful. */
bool  MAX17049_ReadStatus(I2C_HandleTypeDef *hi2c, MAX17049_Status *out);

/** Force a power-on-reset of the gauge (CMD reg <- 0x5400). */
bool  MAX17049_Reset(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* MAX17049_H */
