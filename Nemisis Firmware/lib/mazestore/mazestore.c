/**
 ******************************************************************************
 * @file    mazestore.c
 * @brief   Persistent maze / speed-run memory in on-chip flash - see mazestore.h.
 ******************************************************************************
 */
#include "mazestore.h"
#include "control.h"       /* Control_IsActive - don't erase mid-run */
#include "main.h"          /* HAL + FLASH definitions */
#include "dbg.h"
#include <string.h>
#include <stddef.h>

/* ===========================================================================
 *  On-flash blob (THIRD-to-last page; last = settings, 2nd-last = sensor cal)
 * ========================================================================= */
#define MAZESTORE_MAGIC    0x4E454D4Du   /* "NEMM" */
#define MAZESTORE_VERSION  1u

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t best_len;                /* learned best path length (info only)     */
  MazeMap  map;                     /* 784 bytes                                */
  uint32_t crc;                     /* CRC32 over everything above              */
  uint32_t pad;                     /* keep total an 8-byte multiple            */
} Blob;

/* Compile-time guards: doubleword-aligned blob + dim matches the solver's. */
typedef char mazestore_blob_is_dw_aligned[(sizeof(Blob) % 8u == 0) ? 1 : -1];

/* RAM cache: valid iff a good blob was found at Init or written by Save. */
static bool     cache_valid;
static uint32_t reset_csr;          /* snapshot of the reset-cause flags        */
static bool     warm_reset;         /* Init treated this boot as a button reset */

/* ===========================================================================
 *  Flash geometry (DBANK-aware) + CRC  - same scheme as settings.c/sensors.c,
 *  one page further down (third from the end).
 * ========================================================================= */
typedef struct { uint32_t addr; uint32_t bank; uint32_t page; } FlashPage;

static FlashPage flash_geometry(void)
{
  FlashPage f;
  uint32_t total = (uint32_t)(*(volatile uint16_t *)FLASHSIZE_BASE) * 1024u;

  if (FLASH->OPTR & FLASH_OPTR_DBANK)
  {
    uint32_t page_sz = 0x800u;                        /* dual bank: 2 KB pages */
    f.addr = FLASH_BASE + total - 3u * page_sz;        /* third-to-last page   */
    f.bank = FLASH_BANK_2;
    f.page = (f.addr - (FLASH_BASE + total / 2u)) / page_sz;
  }
  else
  {
    uint32_t page_sz = 0x1000u;                        /* single bank: 4 KB     */
    f.addr = FLASH_BASE + total - 3u * page_sz;
    f.bank = FLASH_BANK_1;
    f.page = (f.addr - FLASH_BASE) / page_sz;
  }
  return f;
}

static uint32_t crc32(const uint8_t *d, uint32_t n)
{
  uint32_t c = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n; i++)
  {
    c ^= d[i];
    for (int k = 0; k < 8; k++)
      c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1u)));
  }
  return c ^ 0xFFFFFFFFu;
}

static uint32_t blob_crc(const Blob *b)
{
  return crc32((const uint8_t *)b, offsetof(Blob, crc));
}

static bool blob_ok(const Blob *f)
{
  return f->magic == MAZESTORE_MAGIC &&
         f->version == MAZESTORE_VERSION &&
         f->crc == blob_crc(f) &&
         f->map.n >= 2u && f->map.n <= MAZE_STORE_DIM;
}

/* ===========================================================================
 *  Erase / save
 * ========================================================================= */
static bool page_erase(void)
{
  FlashPage fp = flash_geometry();
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  FLASH_EraseInitTypeDef er = {0};
  er.TypeErase = FLASH_TYPEERASE_PAGES;
  er.Banks     = fp.bank;
  er.Page      = fp.page;
  er.NbPages   = 1;
  uint32_t page_err = 0;
  bool ok = (HAL_FLASHEx_Erase(&er, &page_err) == HAL_OK);
  HAL_FLASH_Lock();
  return ok;
}

void MazeStore_Erase(void)
{
  if (Control_IsActive()) return;    /* erase stalls the CPU/loop */
  if (page_erase())
    LOG("MAZESTORE: erased saved run\r\n");
  cache_valid = false;
}

bool MazeStore_Save(const MazeMap *m)
{
  if (!m) return false;
  if (Control_IsActive()) return false;    /* erase stalls the CPU/loop */

  Blob b;
  memset(&b, 0, sizeof(b));
  b.magic   = MAZESTORE_MAGIC;
  b.version = MAZESTORE_VERSION;
  b.map     = *m;
  b.best_len = 0;
  b.pad     = 0;
  b.crc     = blob_crc(&b);

  FlashPage fp = flash_geometry();
  bool ok = true;

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  FLASH_EraseInitTypeDef er = {0};
  er.TypeErase = FLASH_TYPEERASE_PAGES;
  er.Banks     = fp.bank;
  er.Page      = fp.page;
  er.NbPages   = 1;
  uint32_t page_err = 0;
  if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK)
  {
    ok = false;
  }
  else
  {
    const uint64_t *src = (const uint64_t *)(const void *)&b;
    uint32_t n64 = sizeof(Blob) / 8u;
    for (uint32_t i = 0; i < n64 && ok; i++)
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                            fp.addr + i * 8u, src[i]) != HAL_OK)
        ok = false;
    }
  }
  HAL_FLASH_Lock();

  if (ok)                                  /* verify what actually landed */
  {
    const Blob *f = (const Blob *)fp.addr;
    ok = (blob_ok(f) && memcmp(f, &b, offsetof(Blob, crc)) == 0);
  }
  cache_valid = ok;
  LOG("MAZESTORE: save %s (%dx%d map)\r\n", ok ? "ok" : "FAIL",
      (int)m->n, (int)m->n);
  return ok;
}

/* ===========================================================================
 *  Load / query
 * ========================================================================= */
bool MazeStore_Valid(void) { return cache_valid; }

bool MazeStore_Get(MazeMap *out)
{
  if (!out || !cache_valid) return false;
  const Blob *f = (const Blob *)flash_geometry().addr;
  if (!blob_ok(f)) { cache_valid = false; return false; }
  *out = f->map;
  return true;
}

/* ===========================================================================
 *  Boot: reset-cause discrimination, then load
 * ========================================================================= */
uint32_t    MazeStore_LastResetFlags(void) { return reset_csr; }
bool        MazeStore_WasWarmReset(void)   { return warm_reset; }

const char *MazeStore_LastResetName(void)
{
  /* Report the dominant cause. Order matters: a power-on also asserts PIN, so
     BOR is checked first. */
  if (reset_csr & RCC_CSR_BORRSTF)  return "power/BOR";
  if (reset_csr & RCC_CSR_LPWRRSTF) return "low-power";
  if (reset_csr & RCC_CSR_WWDGRSTF) return "wwdg";
  if (reset_csr & RCC_CSR_IWDGRSTF) return "iwdg";
  if (reset_csr & RCC_CSR_SFTRSTF)  return "software";
  if (reset_csr & RCC_CSR_OBLRSTF)  return "optionbyte";
  if (reset_csr & RCC_CSR_PINRSTF)  return "pin/button";
  return "unknown";
}

void MazeStore_Init(void)
{
  /* Snapshot the reset-cause flags, then clear them so the NEXT reset's cause is
     read cleanly (the flags are sticky until cleared or power is removed). */
  reset_csr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Erase ONLY on the exact "someone pressed NRST while powered" signature:
     the PIN flag set and NO power/software/watchdog/option-byte flag. Any other
     combination - crucially a cold power-on / battery swap, which sets BOR - is
     treated as KEEP. Default-safe: ambiguity never wipes a solved maze. */
  warm_reset =
       (reset_csr & RCC_CSR_PINRSTF) &&
     !(reset_csr & (RCC_CSR_BORRSTF  | RCC_CSR_SFTRSTF  | RCC_CSR_IWDGRSTF |
                    RCC_CSR_WWDGRSTF | RCC_CSR_LPWRRSTF | RCC_CSR_OBLRSTF));

  const Blob *f = (const Blob *)flash_geometry().addr;
  cache_valid = blob_ok(f);

  /* KEEP the saved map across every reset and power cycle. The map is only
     replaced by a new SEARCH (MazeStore_Save erases+writes) or an explicit
     `mem erase` - so you can power-cycle / reset the board and re-run the SAVED
     fast run as many times as you like without re-searching.

     The old boot-time "warm reset -> auto-erase" was REMOVED: it keyed on the
     PIN-reset-without-BOR flag signature, but whether a cold power-on asserts BOR
     is supply/ramp dependent - a clean power cycle that happened NOT to set BOR
     looked exactly like a button reset and wiped a good map (the intermittent
     "saved run lost after a power cycle"). warm_reset is still computed for the
     `mem` diagnostic, it just no longer triggers an erase. */
  LOG("MAZESTORE: reset=%s, saved run %s%s\r\n",
      MazeStore_LastResetName(),
      cache_valid ? "PRESENT" : "none",
      warm_reset ? " (kept; 'mem erase' or a new search clears it)" : "");
}
