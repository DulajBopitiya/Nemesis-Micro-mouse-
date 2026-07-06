/**
 ******************************************************************************
 * @file    icm42688.h
 * @brief   Minimal driver / bring-up checker for the TDK InvenSense
 *          ICM-42688-P 6-axis IMU on SPI1.
 *
 * Wiring on this board (from CubeMX / hal_msp.c):
 *     PA15 -> CS   (driven here as a plain GPIO output, NOT hardware NSS)
 *     PB3  -> SCLK (SPI1_SCK)
 *     PB4  -> SDO  (SPI1_MISO, data from IMU)
 *     PB5  -> SDI  (SPI1_MOSI, data to IMU)
 *
 * SPI mode 0 (CPOL=0, CPHA=0), MSB first, max 24 MHz. The chip also accepts
 * mode 3; CubeMX is currently set to mode 0 which is fine.
 *
 * Quick start:
 *     #include "icm42688.h"
 *     if (ICM42688_Init(&hspi1)) {  // probes WHO_AM_I + powers on sensors
 *         ICM42688_Raw raw;
 *         ICM42688_ReadData(&hspi1, &raw);
 *     }
 ******************************************************************************
 */
#ifndef ICM42688_H
#define ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"        /* HAL + SPI_HandleTypeDef */
#include <stdint.h>
#include <stdbool.h>

/* CS line. Change here if the board ever moves it off PA15. */
#define ICM42688_CS_PORT      GPIOA
#define ICM42688_CS_PIN       GPIO_PIN_15

/* WHO_AM_I (reg 0x75) returns this fixed value for a healthy ICM-42688-P. */
#define ICM42688_WHOAMI       0x47U

/* Raw 16-bit sensor counts straight off the registers. */
typedef struct
{
  int16_t accel[3];   /* X, Y, Z */
  int16_t gyro[3];    /* X, Y, Z */
  int16_t temp;       /* raw temperature */
} ICM42688_Raw;

/* Same data scaled to engineering units (using power-on default full scales:
   accel = +/-16 g, gyro = +/-2000 dps). */
typedef struct
{
  float accel_g[3];   /* g          */
  float gyro_dps[3];  /* deg/s      */
  float temp_c;       /* deg C      */
} ICM42688_Data;

/** Configure PA15 as a GPIO output for software chip-select (idle high). */
void    ICM42688_CS_Init(void);

/** Read WHO_AM_I. Returns true and fills *who if it matches 0x47. */
bool    ICM42688_Probe(SPI_HandleTypeDef *hspi, uint8_t *who);

/** Full bring-up: CS init, soft reset, WHO_AM_I check, power on accel+gyro.
 *  Returns true only if the chip answered correctly. */
bool    ICM42688_Init(SPI_HandleTypeDef *hspi);

/** Single register read/write (handles the read bit 0x80 for you). */
uint8_t ICM42688_ReadReg(SPI_HandleTypeDef *hspi, uint8_t reg);
void    ICM42688_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val);

/** Burst-read temperature + accel + gyro into raw counts. */
bool    ICM42688_ReadData(SPI_HandleTypeDef *hspi, ICM42688_Raw *out);

/** Convert raw counts to g / dps / degC using the power-on default scales. */
void    ICM42688_Convert(const ICM42688_Raw *raw, ICM42688_Data *out);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_H */
