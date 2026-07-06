/**
 ******************************************************************************
 * @file    max17049.c
 * @brief   MAX17049 2-cell fuel gauge driver. See max17049.h.
 ******************************************************************************
 */
#include "max17049.h"

/* ---- Register map (16-bit, big-endian) ----------------------------------- */
#define REG_VCELL     0x02U   /* battery voltage                              */
#define REG_SOC       0x04U   /* state of charge                              */
#define REG_MODE      0x06U   /* quick-start / enable bits                    */
#define REG_VERSION   0x08U   /* silicon version                             */
#define REG_HIBRT     0x0AU   /* hibernate thresholds                        */
#define REG_CONFIG    0x0CU   /* RCOMP, sleep, alert config                  */
#define REG_CRATE     0x16U   /* charge/discharge rate                       */
#define REG_VRESET    0x18U
#define REG_STATUS    0x1AU
#define REG_CMD       0xFEU   /* write 0x5400 = power-on-reset                */

/* Scaling (MAX17048/MAX17049 datasheet):
   VCELL : 78.125 uV per cell per LSB
   SOC   : 1%/256 per LSB
   CRATE : 0.208 %/hr per LSB (signed) */
#define VCELL_LSB_V   0.000078125f
#define SOC_LSB_PCT   (1.0f / 256.0f)
#define CRATE_LSB     0.208f

#define I2C_TIMEOUT_MS  100U

/* -------------------------------------------------------------------------- */

bool MAX17049_Read16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *val)
{
  uint8_t buf[2];

  if (HAL_I2C_Mem_Read(hi2c, MAX17049_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                       buf, 2U, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }
  *val = (uint16_t)((buf[0] << 8) | buf[1]);   /* MSB first */
  return true;
}

bool MAX17049_Write16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t val)
{
  uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFFU) };

  return (HAL_I2C_Mem_Write(hi2c, MAX17049_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, 2U, I2C_TIMEOUT_MS) == HAL_OK);
}

bool MAX17049_Probe(I2C_HandleTypeDef *hi2c)
{
  return (HAL_I2C_IsDeviceReady(hi2c, MAX17049_I2C_ADDR, 3U, I2C_TIMEOUT_MS) == HAL_OK);
}

bool MAX17049_ReadCellVoltage(I2C_HandleTypeDef *hi2c, float *volts)
{
  uint16_t raw;
  if (!MAX17049_Read16(hi2c, REG_VCELL, &raw))
  {
    return false;
  }
  *volts = (float)raw * VCELL_LSB_V;
  return true;
}

bool MAX17049_ReadSOC(I2C_HandleTypeDef *hi2c, float *percent)
{
  uint16_t raw;
  if (!MAX17049_Read16(hi2c, REG_SOC, &raw))
  {
    return false;
  }
  *percent = (float)raw * SOC_LSB_PCT;
  return true;
}

bool MAX17049_ReadChargeRate(I2C_HandleTypeDef *hi2c, float *pct_per_hr)
{
  uint16_t raw;
  if (!MAX17049_Read16(hi2c, REG_CRATE, &raw))
  {
    return false;
  }
  *pct_per_hr = (float)((int16_t)raw) * CRATE_LSB;   /* signed */
  return true;
}

bool MAX17049_ReadStatus(I2C_HandleTypeDef *hi2c, MAX17049_Status *out)
{
  uint16_t vcell, soc, crate;

  if (!MAX17049_Read16(hi2c, REG_VERSION, &out->version)) { return false; }
  if (!MAX17049_Read16(hi2c, REG_VCELL,   &vcell))        { return false; }
  if (!MAX17049_Read16(hi2c, REG_SOC,     &soc))          { return false; }
  if (!MAX17049_Read16(hi2c, REG_CRATE,   &crate))        { return false; }

  out->cell_v = (float)vcell * VCELL_LSB_V;
  out->pack_v = out->cell_v * (float)MAX17049_CELLS;
  out->soc    = (float)soc * SOC_LSB_PCT;
  out->crate  = (float)((int16_t)crate) * CRATE_LSB;
  return true;
}

bool MAX17049_Reset(I2C_HandleTypeDef *hi2c)
{
  /* CMD <- 0x5400 forces POR. The IC NACKs the stop on success, so don't
     treat a HAL error here as fatal; just issue it best-effort. */
  return MAX17049_Write16(hi2c, REG_CMD, 0x5400U);
}
