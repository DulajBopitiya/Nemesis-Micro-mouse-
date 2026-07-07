/*
 * qspiflash - W25Q32JW driver over QUADSPI1 (single-line indirect mode).
 * See qspiflash.h for the OTA rationale. All ops are blocking (poll WIP).
 */
#include "qspiflash.h"

/* ---- W25Q command set (standard SPI opcodes) ------------------------------ */
#define CMD_RESET_ENABLE   0x66u
#define CMD_RESET_DEVICE   0x99u
#define CMD_READ_JEDEC_ID  0x9Fu
#define CMD_READ_STATUS1   0x05u
#define CMD_WRITE_ENABLE   0x06u
#define CMD_READ_DATA      0x03u   /* single-line read, up to ~50 MHz */
#define CMD_PAGE_PROGRAM   0x02u
#define CMD_SECTOR_ERASE   0x20u   /* 4 KB  */
#define CMD_CHIP_ERASE     0xC7u

#define SR1_WIP            0x01u   /* write/erase in progress */
#define SR1_WEL            0x02u   /* write enable latch      */

/* Generous blocking timeouts (ms). Datasheet worst-case: page program ~3 ms,
   sector erase ~400 ms, chip erase a few seconds. */
#define TMO_CMD            HAL_QSPI_TIMEOUT_DEFAULT_VALUE
#define TMO_BUSY_MS        6000u

static QSPI_HandleTypeDef *H;   /* bound in QSpiFlash_Init */

/* ---- low-level command helpers -------------------------------------------- */

/* Fill a command struct with the common single-line defaults. */
static void base_cmd(QSPI_CommandTypeDef *c, uint8_t instr)
{
  c->InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  c->Instruction       = instr;
  c->AddressMode       = QSPI_ADDRESS_NONE;
  c->AddressSize       = QSPI_ADDRESS_24_BITS;
  c->Address           = 0;
  c->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  c->DataMode          = QSPI_DATA_NONE;
  c->DummyCycles       = 0;
  c->NbData            = 0;
  c->DdrMode           = QSPI_DDR_MODE_DISABLE;
  c->DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  c->SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
}

/* Single opcode, no address, no data (e.g. Write-Enable, Reset). */
static bool cmd_only(uint8_t instr)
{
  QSPI_CommandTypeDef c;
  base_cmd(&c, instr);
  return HAL_QSPI_Command(H, &c, TMO_CMD) == HAL_OK;
}

/* Poll SR1 WIP until clear or timeout. */
static bool wait_ready(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  for (;;)
  {
    uint8_t sr1;
    if (!QSpiFlash_ReadStatus(&sr1)) return false;
    if ((sr1 & SR1_WIP) == 0)        return true;
    if ((HAL_GetTick() - start) > timeout_ms) return false;
  }
}

/* Write-Enable, then confirm WEL latched. */
static bool write_enable(void)
{
  if (!cmd_only(CMD_WRITE_ENABLE)) return false;
  uint8_t sr1;
  if (!QSpiFlash_ReadStatus(&sr1)) return false;
  return (sr1 & SR1_WEL) != 0;
}

/* ---- public API ----------------------------------------------------------- */

bool QSpiFlash_ReadID(QSpiFlash_ID *id)
{
  if (!H || !id) return false;
  QSPI_CommandTypeDef c;
  base_cmd(&c, CMD_READ_JEDEC_ID);
  c.DataMode = QSPI_DATA_1_LINE;
  c.NbData   = 3;

  uint8_t rx[3] = { 0, 0, 0 };
  if (HAL_QSPI_Command(H, &c, TMO_CMD) != HAL_OK)       return false;
  if (HAL_QSPI_Receive(H, rx, TMO_CMD) != HAL_OK)       return false;

  id->mfr      = rx[0];
  id->mem_type = rx[1];
  id->capacity = rx[2];
  return true;
}

bool QSpiFlash_ReadStatus(uint8_t *sr1)
{
  if (!H || !sr1) return false;
  QSPI_CommandTypeDef c;
  base_cmd(&c, CMD_READ_STATUS1);
  c.DataMode = QSPI_DATA_1_LINE;
  c.NbData   = 1;
  if (HAL_QSPI_Command(H, &c, TMO_CMD) != HAL_OK) return false;
  return HAL_QSPI_Receive(H, sr1, TMO_CMD) == HAL_OK;
}

bool QSpiFlash_Read(uint32_t addr, uint8_t *buf, size_t len)
{
  if (!H || !buf) return false;
  if (len == 0)   return true;
  if (addr + len > QSPIFLASH_CHIP_SIZE) return false;

  QSPI_CommandTypeDef c;
  base_cmd(&c, CMD_READ_DATA);
  c.AddressMode = QSPI_ADDRESS_1_LINE;
  c.Address     = addr;
  c.DataMode    = QSPI_DATA_1_LINE;
  c.NbData      = (uint32_t)len;
  if (HAL_QSPI_Command(H, &c, TMO_CMD) != HAL_OK) return false;
  return HAL_QSPI_Receive(H, buf, TMO_CMD) == HAL_OK;
}

bool QSpiFlash_EraseSector(uint32_t addr)
{
  if (!H) return false;
  if (addr >= QSPIFLASH_CHIP_SIZE) return false;
  if (!write_enable()) return false;

  QSPI_CommandTypeDef c;
  base_cmd(&c, CMD_SECTOR_ERASE);
  c.AddressMode = QSPI_ADDRESS_1_LINE;
  c.Address     = addr & ~(QSPIFLASH_SECTOR_SIZE - 1u);
  if (HAL_QSPI_Command(H, &c, TMO_CMD) != HAL_OK) return false;
  return wait_ready(TMO_BUSY_MS);
}

bool QSpiFlash_EraseChip(void)
{
  if (!H) return false;
  if (!write_enable()) return false;
  if (!cmd_only(CMD_CHIP_ERASE)) return false;
  return wait_ready(TMO_BUSY_MS * 20u);   /* chip erase is the slow one */
}

bool QSpiFlash_Write(uint32_t addr, const uint8_t *buf, size_t len)
{
  if (!H || !buf) return false;
  if (addr + len > QSPIFLASH_CHIP_SIZE) return false;

  while (len > 0)
  {
    /* one Page-Program cannot cross a 256-byte page boundary */
    uint32_t page_off = addr & (QSPIFLASH_PAGE_SIZE - 1u);
    uint32_t chunk    = QSPIFLASH_PAGE_SIZE - page_off;
    if (chunk > len) chunk = (uint32_t)len;

    if (!write_enable()) return false;

    QSPI_CommandTypeDef c;
    base_cmd(&c, CMD_PAGE_PROGRAM);
    c.AddressMode = QSPI_ADDRESS_1_LINE;
    c.Address     = addr;
    c.DataMode    = QSPI_DATA_1_LINE;
    c.NbData      = chunk;
    if (HAL_QSPI_Command(H, &c, TMO_CMD) != HAL_OK)          return false;
    if (HAL_QSPI_Transmit(H, (uint8_t *)buf, TMO_CMD) != HAL_OK) return false;
    if (!wait_ready(TMO_BUSY_MS))                            return false;

    addr += chunk;
    buf  += chunk;
    len  -= chunk;
  }
  return true;
}

bool QSpiFlash_Init(QSPI_HandleTypeDef *hqspi)
{
  if (!hqspi) return false;
  H = hqspi;

  /* Software reset (0x66 then 0x99) to leave the chip in a known state, then
     the datasheet-required ~30 us recovery before the next command. */
  cmd_only(CMD_RESET_ENABLE);
  cmd_only(CMD_RESET_DEVICE);
  HAL_Delay(1);

  QSpiFlash_ID id;
  if (!QSpiFlash_ReadID(&id)) return false;
  return (id.mfr == QSPIFLASH_MFR_WINBOND) &&
         (id.capacity == QSPIFLASH_CAP_32MBIT);
}
