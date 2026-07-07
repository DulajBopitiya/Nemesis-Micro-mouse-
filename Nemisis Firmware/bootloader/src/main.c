/*
 * NEMESIS OTA bootloader - B2: install a staged image, then jump.
 *
 * Runs on reset from 0x08000000. Flow:
 *   1. If the "apply" flag (TAMP backup reg 0) is set, clear it (one-shot) and,
 *      if the QSPI incoming image CRC-verifies, copy it into the internal app
 *      slot (0x08008000) and verify the copy.
 *   2. Jump to the app at 0x08008000 (validating its vector first).
 *
 * It is NEVER OTA-updated. SWD/J-Link stays the ultimate recovery net: flashing
 * a normal 0x08000000-based build overwrites both this and the app slot.
 *
 * B2 is single-slot: a power loss DURING the copy can leave a partial app
 * (recover via SWD). B3 will add a golden image + rollback to close that window.
 *
 * Runs at the reset-default HSI (~16 MHz) - no PLL/HSE setup. QSPI is sourced
 * from SYSCLK, so ~4 MHz here; fine for a one-time image copy. The app sets up
 * its own 160 MHz clock after the jump.
 */
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdbool.h>

/* ---- flash map ---- */
#define APP_BASE            0x08008000u
#define RAM_START           0x20000000u
#define RAM_END             0x20020000u          /* 128 KB */

/* ---- shared OTA constants (MUST match lib/ota/ota.h) ---- */
#define OTA_IMAGE_MAX       (480u * 1024u)
#define QSPI_SLOT_INCOMING  0x000000u
#define QSPI_META_INCOMING  0x0F0000u
#define QSPI_SLOT_GOLDEN    0x100000u
#define QSPI_META_GOLDEN    0x1F0000u
#define OTA_META_MAGIC      0x4F544131u
#define OTA_STATUS_VALID    0xA5A5A5A5u
#define OTA_APPLY_MAGIC     0x0A7A0A7Au
#define OTA_APPLIED_MAGIC   0x0A9911EDu     /* -> app flashes green on next boot */
#define OTA_ROLLBACK_MAGIC  0x0B0B0B0Bu     /* restore golden slot instead */

typedef struct { uint32_t magic, size, crc32, status; } OtaMeta;

QSPI_HandleTypeDef hqspi1;

/* HAL_Init() enables the SysTick interrupt for its blocking-timeout tick, so we
   MUST provide the handler. Without it the first tick (~1 ms after HAL_Init)
   vectors to the weak Default_Handler (an infinite loop) and the bootloader
   freezes before doing anything. (The app has this in stm32g4xx_it.c, which the
   bootloader project doesn't include.) */
void SysTick_Handler(void) { HAL_IncTick(); }

/* ===========================================================================
 *  Apply flag (TAMP backup register 0)
 * ========================================================================= */
static void bkp_access_enable(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
}
static uint32_t apply_flag_read(void)  { return TAMP->BKP0R; }
static void     apply_flag_clear(void) { TAMP->BKP0R = 0u; }

/* ===========================================================================
 *  CRC-32 (zlib/IEEE, poly 0xEDB88320) - matches lib/ota Ota_Crc32
 * ========================================================================= */
static uint32_t crc32(uint32_t crc, const uint8_t *d, uint32_t n)
{
  crc = ~crc;
  for (uint32_t i = 0; i < n; i++)
  {
    crc ^= d[i];
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
  }
  return ~crc;
}

/* ===========================================================================
 *  QSPI (single-line indirect read; mirrors lib/qspiflash)
 * ========================================================================= */
void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
  if (hqspi->Instance != QUADSPI) return;

  RCC_PeriphCLKInitTypeDef pclk = {0};
  pclk.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
  pclk.QspiClockSelection   = RCC_QSPICLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&pclk);

  __HAL_RCC_QSPI_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF10_QUADSPI;
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;                         /* PA6/PA7 IO3/IO2 */
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11; /* PB0/1/10/11 */
  HAL_GPIO_Init(GPIOB, &g);
}

static bool qspi_init(void)
{
  hqspi1.Instance = QUADSPI;
  hqspi1.Init.ClockPrescaler     = 3;
  hqspi1.Init.FifoThreshold      = 4;
  hqspi1.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi1.Init.FlashSize          = 21;                  /* 4 MB */
  hqspi1.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
  hqspi1.Init.ClockMode          = QSPI_CLOCK_MODE_0;
  hqspi1.Init.FlashID            = QSPI_FLASH_ID_1;
  hqspi1.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
  return HAL_QSPI_Init(&hqspi1) == HAL_OK;
}

static bool qspi_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  QSPI_CommandTypeDef c = {0};
  c.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  c.Instruction       = 0x03;                            /* READ DATA */
  c.AddressMode       = QSPI_ADDRESS_1_LINE;
  c.AddressSize       = QSPI_ADDRESS_24_BITS;
  c.Address           = addr;
  c.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  c.DataMode          = QSPI_DATA_1_LINE;
  c.DummyCycles       = 0;
  c.NbData            = len;
  c.DdrMode           = QSPI_DDR_MODE_DISABLE;
  c.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  c.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&hqspi1, &c, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return false;
  return HAL_QSPI_Receive(&hqspi1, buf, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

/* CRC-32 the QSPI image by streaming it out in blocks. */
static bool qspi_crc(uint32_t base, uint32_t size, uint32_t *out)
{
  uint8_t buf[256];
  uint32_t crc = 0, off = 0;
  while (off < size)
  {
    uint32_t n = size - off;
    if (n > sizeof buf) n = sizeof buf;
    if (!qspi_read(base + off, buf, n)) return false;
    crc = crc32(crc, buf, n);
    off += n;
  }
  *out = crc;
  return true;
}

/* ===========================================================================
 *  Copy QSPI incoming image -> internal app slot, then verify
 * ========================================================================= */
static bool flash_program_app(uint32_t qspi_src, uint32_t size)
{
  /* DBANK-aware page geometry (same scheme as lib/mazestore). The app slot at
     0x08008000 is well within bank 1 in both single- and dual-bank modes. */
  uint32_t page_sz    = (FLASH->OPTR & FLASH_OPTR_DBANK) ? 0x800u : 0x1000u;
  uint32_t first_page = (APP_BASE - FLASH_BASE) / page_sz;
  uint32_t npages     = (size + page_sz - 1u) / page_sz;

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  FLASH_EraseInitTypeDef er = {0};
  er.TypeErase = FLASH_TYPEERASE_PAGES;
  er.Banks     = FLASH_BANK_1;
  er.Page      = first_page;
  er.NbPages   = npages;
  uint32_t page_err = 0;
  if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return false;
  }

  uint8_t buf[256];
  uint32_t off = 0;
  bool ok = true;
  while (off < size && ok)
  {
    uint32_t n = size - off;
    if (n > sizeof buf) n = sizeof buf;
    if (!qspi_read(qspi_src + off, buf, n)) { ok = false; break; }

    for (uint32_t i = 0; i < n; i += 8u)
    {
      uint64_t dw = 0xFFFFFFFFFFFFFFFFull;      /* pad the tail with erased 0xFF */
      uint32_t chunk = (n - i >= 8u) ? 8u : (n - i);
      memcpy(&dw, buf + i, chunk);
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                            APP_BASE + off + i, dw) != HAL_OK)
      {
        ok = false;
        break;
      }
    }
    off += n;
  }
  HAL_FLASH_Lock();
  return ok;
}

/* Install the CRC-valid image from a QSPI slot (incoming or golden) into the app
   slot, verifying source and result. Returns true on a fully verified install.
   Assumes qspi_init() already succeeded. */
static bool install_from(uint32_t qspi_slot, uint32_t qspi_meta)
{
  OtaMeta m;
  if (!qspi_read(qspi_meta, (uint8_t *)&m, sizeof m)) return false;
  if (m.magic != OTA_META_MAGIC || m.status != OTA_STATUS_VALID) return false;
  if (m.size == 0u || m.size > OTA_IMAGE_MAX) return false;

  /* Re-verify the source image in QSPI before touching internal flash. */
  uint32_t src_crc = 0;
  if (!qspi_crc(qspi_slot, m.size, &src_crc) || src_crc != m.crc32) return false;

  if (!flash_program_app(qspi_slot, m.size)) return false;

  /* Verify what actually landed in the app slot (memory-mapped read). */
  return crc32(0, (const uint8_t *)APP_BASE, m.size) == m.crc32;
}

/* Apply a staged update. On success flags the green boot-confirm. If the copy
   fails partway (app slot now bad) and a golden image exists, restore it so the
   board can't be stranded by a bad/interrupted apply. */
static void do_apply(void)
{
  if (!qspi_init()) return;
  if (install_from(QSPI_SLOT_INCOMING, QSPI_META_INCOMING))
    TAMP->BKP1R = OTA_APPLIED_MAGIC;               /* app flashes green */
  else
    (void)install_from(QSPI_SLOT_GOLDEN, QSPI_META_GOLDEN);  /* best-effort rescue */
  HAL_QSPI_DeInit(&hqspi1);
}

/* Explicit rollback: restore the golden image into the app slot. */
static void do_rollback(void)
{
  if (!qspi_init()) return;
  (void)install_from(QSPI_SLOT_GOLDEN, QSPI_META_GOLDEN);
  HAL_QSPI_DeInit(&hqspi1);
}

/* ===========================================================================
 *  Jump to the application (validates vector; restores IRQ state)
 * ========================================================================= */
static void jump_to_app(uint32_t base)
{
  uint32_t app_sp = *(volatile uint32_t *)(base);
  uint32_t app_pc = *(volatile uint32_t *)(base + 4u);

  if (app_sp < RAM_START || app_sp > RAM_END) return;   /* blank/invalid slot */

  __disable_irq();
  /* Leave clocks at the HSI reset default (HAL_Init didn't change them); the
     app's SystemClock_Config sets up HSE/PLL from any state - same as a cold
     boot. Just stop the SysTick we started so it can't fire into the app. */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;

  SCB->VTOR = base;
  __DSB();
  __ISB();
  __set_MSP(app_sp);
  __enable_irq();                       /* app expects PRIMASK=0 (see B1 note) */
  ((void (*)(void))app_pc)();
}

int main(void)
{
  HAL_Init();                           /* HSI ~16 MHz, SysTick for HAL timeouts */
  bkp_access_enable();

  uint32_t flag = apply_flag_read();
  if (flag == OTA_APPLY_MAGIC || flag == OTA_ROLLBACK_MAGIC)
  {
    apply_flag_clear();                 /* one-shot: clear BEFORE work so a failed
                                           op can't loop forever */
    if (flag == OTA_APPLY_MAGIC) do_apply();
    else                         do_rollback();
  }

  jump_to_app(APP_BASE);

  /* Only reached if the app slot is invalid. Hang for SWD recovery. */
  while (1) { __NOP(); }
}
