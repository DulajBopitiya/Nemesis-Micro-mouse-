/**
 ******************************************************************************
 * @file    icm42688.c
 * @brief   ICM-42688-P SPI driver / bring-up checker. See icm42688.h.
 ******************************************************************************
 */
#include "icm42688.h"

/* ---- Register map (Bank 0) ----------------------------------------------- */
#define REG_DEVICE_CONFIG   0x11U   /* bit0 = SOFT_RESET_CONFIG               */
#define REG_INT_STATUS      0x2DU   /* bit3 = DATA_RDY_INT                    */
#define REG_TEMP_DATA1      0x1DU   /* burst start: TEMP, ACCEL[XYZ], GYRO[XYZ] */
#define REG_PWR_MGMT0       0x4EU   /* gyro/accel power modes                 */
#define REG_WHO_AM_I        0x75U
#define REG_BANK_SEL        0x76U

/* PWR_MGMT0: GYRO_MODE=11 (low noise) | ACCEL_MODE=11 (low noise) = 0x0F */
#define PWR_MGMT0_ALL_LN    0x0FU

/* SPI read transactions set the MSB of the register address. */
#define ICM_READ_BIT            0x80U

#define SPI_TIMEOUT_MS      100U

/* Default full-scale sensitivities after reset (datasheet section 3):
   accel +/-16 g  -> 2048 LSB/g
   gyro  +/-2000  -> 16.4 LSB/(deg/s) */
#define ACCEL_LSB_PER_G     2048.0f
#define GYRO_LSB_PER_DPS    16.4f

/* -------------------------------------------------------------------------- */

static inline void cs_low(void)
{
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
}

void ICM42688_CS_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();            /* PA15 is on port A */

  /* Idle CS high BEFORE switching the pin to output so we never glitch low. */
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);

  gpio.Pin   = ICM42688_CS_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;        /* override the AF5_SPI1 NSS mode */
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ICM42688_CS_PORT, &gpio);

  cs_high();
}

uint8_t ICM42688_ReadReg(SPI_HandleTypeDef *hspi, uint8_t reg)
{
  uint8_t tx = (uint8_t)(reg | ICM_READ_BIT);
  uint8_t rx = 0U;

  cs_low();
  HAL_SPI_Transmit(hspi, &tx, 1U, SPI_TIMEOUT_MS);
  HAL_SPI_Receive(hspi, &rx, 1U, SPI_TIMEOUT_MS);
  cs_high();

  return rx;
}

void ICM42688_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)
{
  uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), val };

  cs_low();
  HAL_SPI_Transmit(hspi, tx, 2U, SPI_TIMEOUT_MS);
  cs_high();
}

bool ICM42688_Probe(SPI_HandleTypeDef *hspi, uint8_t *who)
{
  uint8_t id = ICM42688_ReadReg(hspi, REG_WHO_AM_I);
  if (who != NULL)
  {
    *who = id;
  }
  return (id == ICM42688_WHOAMI);
}

bool ICM42688_Init(SPI_HandleTypeDef *hspi)
{
  ICM42688_CS_Init();

  /* Soft reset, then wait the datasheet-mandated 1 ms before any access. */
  ICM42688_WriteReg(hspi, REG_DEVICE_CONFIG, 0x01U);
  HAL_Delay(2);

  /* Make sure we're on register bank 0 (where WHO_AM_I / data live). */
  ICM42688_WriteReg(hspi, REG_BANK_SEL, 0x00U);

  if (!ICM42688_Probe(hspi, NULL))
  {
    return false;                          /* wrong / no answer -> bail out */
  }

  /* Turn on accel + gyro in low-noise mode. Datasheet: do not issue another
     register write to PWR_MGMT0 within 200 us, and gyro needs settling time. */
  ICM42688_WriteReg(hspi, REG_PWR_MGMT0, PWR_MGMT0_ALL_LN);
  HAL_Delay(1);

  return true;
}

bool ICM42688_ReadData(SPI_HandleTypeDef *hspi, ICM42688_Raw *out)
{
  uint8_t tx = (uint8_t)(REG_TEMP_DATA1 | ICM_READ_BIT);
  uint8_t buf[14];                         /* temp(2) + accel(6) + gyro(6) */

  if (out == NULL)
  {
    return false;
  }

  cs_low();
  HAL_SPI_Transmit(hspi, &tx, 1U, SPI_TIMEOUT_MS);
  HAL_SPI_Receive(hspi, buf, sizeof(buf), SPI_TIMEOUT_MS);
  cs_high();

  /* All sensor words are big-endian (high byte first). */
  out->temp     = (int16_t)((buf[0]  << 8) | buf[1]);
  out->accel[0] = (int16_t)((buf[2]  << 8) | buf[3]);
  out->accel[1] = (int16_t)((buf[4]  << 8) | buf[5]);
  out->accel[2] = (int16_t)((buf[6]  << 8) | buf[7]);
  out->gyro[0]  = (int16_t)((buf[8]  << 8) | buf[9]);
  out->gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
  out->gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);

  return true;
}

void ICM42688_Convert(const ICM42688_Raw *raw, ICM42688_Data *out)
{
  for (int i = 0; i < 3; i++)
  {
    out->accel_g[i]  = (float)raw->accel[i] / ACCEL_LSB_PER_G;
    out->gyro_dps[i] = (float)raw->gyro[i]  / GYRO_LSB_PER_DPS;
  }
  /* Temperature in degC = (raw / 132.48) + 25 (datasheet). */
  out->temp_c = ((float)raw->temp / 132.48f) + 25.0f;
}
