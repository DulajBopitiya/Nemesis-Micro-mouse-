/*
 * qspiflash - Winbond W25Q32JW (32 Mbit / 4 MB) NOR flash over the STM32
 * QUADSPI1 peripheral. Used to STAGE OTA firmware images (download to here,
 * CRC-verify, then the bootloader copies into the internal app slot) and to
 * hold a golden known-good image for one-button rollback. See the memory
 * `ota-external-flash-plan` for the full OTA design.
 *
 * Bring-up uses SINGLE-LINE (1-1-1) commands via the QUADSPI peripheral in
 * indirect mode - the most robust way to prove the bus. Quad-line fast read
 * (0xEB) is a later speed optimisation; single-line at ~40 MHz is already
 * plenty for staging a ~120 KB image.
 *
 * All calls are BLOCKING (poll WIP). Not ISR-safe. Wire the handle once:
 *     if (QSpiFlash_Init(&hqspi1)) { ... bus is alive ... }
 */
#ifndef QSPIFLASH_H
#define QSPIFLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"          /* QSPI_HandleTypeDef via the HAL */

#define QSPIFLASH_PAGE_SIZE     256u
#define QSPIFLASH_SECTOR_SIZE   4096u
#define QSPIFLASH_BLOCK_SIZE    65536u
#define QSPIFLASH_CHIP_SIZE     (4u * 1024u * 1024u)   /* 32 Mbit = 4 MB */

/* Winbond W25Q32JW JEDEC id (via 0x9F). We accept any Winbond 32 Mbit part:
   manufacturer must be 0xEF and the capacity byte 0x16; the memory-type byte
   (0x60 on the JW 1.8 V part) is reported for info but not gated on. */
#define QSPIFLASH_MFR_WINBOND   0xEFu
#define QSPIFLASH_CAP_32MBIT    0x16u

typedef struct
{
  uint8_t mfr;        /* manufacturer id  (Winbond = 0xEF)      */
  uint8_t mem_type;   /* memory type      (W25Q32JW = 0x60)     */
  uint8_t capacity;   /* capacity code    (32 Mbit = 0x16)      */
} QSpiFlash_ID;

/** Bind the QUADSPI handle, software-reset the chip, and probe the JEDEC id.
 *  Returns true iff a Winbond 32 Mbit part answered. Safe to call at boot. */
bool QSpiFlash_Init(QSPI_HandleTypeDef *hqspi);

/** Read the 3-byte JEDEC id (0x9F). Works even if Init() failed the gate,
 *  so callers can print whatever actually came back for diagnosis. */
bool QSpiFlash_ReadID(QSpiFlash_ID *id);

/** Read Status Register 1 (0x05). bit0 = WIP (busy), bit1 = WEL. */
bool QSpiFlash_ReadStatus(uint8_t *sr1);

/** Read `len` bytes from `addr` (0x03 normal read, 24-bit address). */
bool QSpiFlash_Read(uint32_t addr, uint8_t *buf, size_t len);

/** Erase the 4 KB sector containing `addr` (0x20). Blocks until done. */
bool QSpiFlash_EraseSector(uint32_t addr);

/** Erase the entire chip (0xC7). Blocks (can take several seconds). */
bool QSpiFlash_EraseChip(void);

/** Program `len` bytes at `addr` (0x02). Splits across 256-byte page
 *  boundaries automatically. The target range must already be erased. */
bool QSpiFlash_Write(uint32_t addr, const uint8_t *buf, size_t len);

#endif /* QSPIFLASH_H */
