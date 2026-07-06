/**
 ******************************************************************************
 * @file    as5047p.c
 * @brief   AMS AS5047P-ATSM SPI driver. See as5047p.h for wiring + protocol.
 ******************************************************************************
 */
#include "as5047p.h"

#define AS5047P_FLAG_READ   0x4000U   /* bit14 set = read command            */
#define AS5047P_FLAG_PARITY 0x8000U   /* bit15 = even parity over bits 14..0  */
#define AS5047P_DATA_MASK   0x3FFFU   /* bits 13..0 = payload                 */
#define AS5047P_EF_MASK     0x4000U   /* bit14 of a response = error flag     */

#define AS5047P_SPI_TIMEOUT 10U       /* ms per 16-bit transfer               */

/* Even parity: return 1 if bits 14..0 contain an ODD number of set bits, so
   ORing the result into bit15 makes the whole 16-bit word even parity. */
static uint16_t parity_even_bit(uint16_t frame)
{
  uint16_t v = frame & 0x7FFFU;
  v ^= v >> 8;
  v ^= v >> 4;
  v ^= v >> 2;
  v ^= v >> 1;
  return (uint16_t)(v & 1U);
}

/* Build a 16-bit command word for a register address with the read bit and
   the correct even-parity bit. */
static uint16_t make_command(uint16_t addr, bool read)
{
  uint16_t cmd = (uint16_t)(addr & AS5047P_DATA_MASK);
  if (read)
  {
    cmd |= AS5047P_FLAG_READ;
  }
  if (parity_even_bit(cmd))
  {
    cmd |= AS5047P_FLAG_PARITY;
  }
  return cmd;
}

/* Exchange one 16-bit frame (sent MSB-first as 2 bytes) while holding CS low.
   Returns the 16-bit word clocked back from the encoder. */
static uint16_t xfer16(AS5047P_Handle *enc, uint16_t tx)
{
  uint8_t txb[2] = { (uint8_t)(tx >> 8), (uint8_t)(tx & 0xFFU) };
  uint8_t rxb[2] = { 0, 0 };

  HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(enc->hspi, txb, rxb, 2, AS5047P_SPI_TIMEOUT);
  HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);

  return (uint16_t)((rxb[0] << 8) | rxb[1]);
}

void AS5047P_Init(AS5047P_Handle *enc, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  enc->hspi    = hspi;
  enc->cs_port = cs_port;
  enc->cs_pin  = cs_pin;

  /* Park CS high (deselected). CubeMX drives it low at boot, so undo that. */
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

bool AS5047P_ReadRegister(AS5047P_Handle *enc, uint16_t addr, uint16_t *out)
{
  /* AS5047P reads are pipelined: the first frame issues the read command, the
     SECOND frame clocks the requested data back. Use a NOP for the second. */
  (void)xfer16(enc, make_command(addr, true));
  uint16_t resp = xfer16(enc, make_command(AS5047P_REG_NOP, true));

  /* Verify even parity across the whole returned word. */
  if (parity_even_bit(resp) != ((resp >> 15) & 1U))
  {
    return false;
  }
  /* Bit14 set = the encoder flagged an error for the addressed register. */
  if (resp & AS5047P_EF_MASK)
  {
    return false;
  }

  if (out)
  {
    *out = (uint16_t)(resp & AS5047P_DATA_MASK);
  }
  return true;
}

uint16_t AS5047P_ReadAngleRaw(AS5047P_Handle *enc)
{
  uint16_t angle;
  if (!AS5047P_ReadRegister(enc, AS5047P_REG_ANGLECOM, &angle))
  {
    return 0xFFFFU;
  }
  return angle;
}

float AS5047P_ReadAngleDeg(AS5047P_Handle *enc)
{
  uint16_t raw = AS5047P_ReadAngleRaw(enc);
  if (raw == 0xFFFFU)
  {
    return -1.0f;
  }
  return (float)raw * (360.0f / (float)AS5047P_RESOLUTION);
}

bool AS5047P_Probe(AS5047P_Handle *enc)
{
  /* Clear any stale error flags first (ERRFL clears on read). */
  uint16_t errfl = 0;
  (void)AS5047P_ReadRegister(enc, AS5047P_REG_ERRFL, &errfl);

  /* Read DIAAGC: a live sensor returns a non-zero word (the AGC value lives in
     the low byte plus the LF/COF/MAGL/MAGH status bits). A stuck-low MISO
     (wrong SPI mode, no chip, broken wire) returns 0x0000, which would
     otherwise sail through the parity check as a bogus "valid" frame. So
     require both a clean frame AND a non-zero diagnostic word. */
  uint16_t diag = 0;
  if (!AS5047P_ReadRegister(enc, AS5047P_REG_DIAAGC, &diag) || diag == 0U)
  {
    return false;
  }
  return true;
}
