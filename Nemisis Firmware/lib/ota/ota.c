/*
 * ota - firmware image staging in QSPI flash. See ota.h for the layout and
 * rationale. Built on lib/qspiflash (single-line blocking driver).
 */
#include "ota.h"
#include "qspiflash.h"

/* ---- staging cursor (one transfer at a time) ------------------------------ */
static uint32_t stage_size;      /* announced image size for the active stage  */
static uint32_t stage_written;   /* bytes committed to QSPI so far             */
static uint32_t stage_erased;    /* byte offset up to which sectors are erased */
static bool     stage_active;

/* ---- CRC-32 (zlib/IEEE, reflected, poly 0xEDB88320) ----------------------- */
uint32_t Ota_Crc32(uint32_t crc, const uint8_t *data, size_t len)
{
  crc = ~crc;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
  }
  return ~crc;
}

/* ---- metadata ------------------------------------------------------------- */
static bool write_meta(uint32_t meta_addr, const OtaMeta *m)
{
  if (!QSpiFlash_EraseSector(meta_addr)) return false;
  return QSpiFlash_Write(meta_addr, (const uint8_t *)m, sizeof(*m));
}

bool Ota_ReadMeta(OtaMeta *meta)
{
  if (!meta) return false;
  return QSpiFlash_Read(OTA_META_INCOMING, (uint8_t *)meta, sizeof(*meta));
}

/* ---- staging -------------------------------------------------------------- */
bool Ota_BeginStage(uint32_t size)
{
  stage_active  = false;
  stage_written = 0;
  stage_erased  = 0;
  stage_size    = 0;

  if (size == 0 || size > OTA_IMAGE_MAX) return false;

  /* Invalidate the old metadata first: if this transfer is interrupted, the
     slot must not look valid. (Sectors are erased on-the-fly during writes -
     see Ota_WriteChunk - so this call returns fast and the long erase cost is
     spread across the ACK-gated transfer instead of stalling before READY.) */
  OtaMeta bad = { 0, 0, 0, OTA_STATUS_INVALID };
  if (!write_meta(OTA_META_INCOMING, &bad)) return false;

  stage_size   = size;
  stage_active = true;
  return true;
}

/* Erase image sectors up to (but not yet including) byte offset `need`. Called
   incrementally as the write cursor advances; each call erases at most the one
   or two sectors the next chunk will touch, so any single erase stays hidden
   behind that chunk's ACK. */
static bool ensure_erased(uint32_t need)
{
  while (stage_erased < need)
  {
    if (!QSpiFlash_EraseSector(OTA_SLOT_INCOMING + stage_erased)) return false;
    stage_erased += QSPIFLASH_SECTOR_SIZE;    /* stays sector-aligned */
  }
  return true;
}

bool Ota_WriteChunk(const uint8_t *data, size_t len)
{
  if (!stage_active || !data) return false;
  if (len == 0) return true;
  if (stage_written + len > stage_size) return false;   /* overflow guard */

  if (!ensure_erased(stage_written + (uint32_t)len)) return false;
  if (!QSpiFlash_Write(OTA_SLOT_INCOMING + stage_written, data, len))
    return false;

  stage_written += (uint32_t)len;
  return true;
}

/* Read the slot back out of QSPI in blocks and CRC-32 it - authoritative
   check of what is actually stored (not just what we thought we wrote). */
static bool crc_slot(uint32_t base, uint32_t size, uint32_t *out_crc)
{
  uint8_t buf[256];
  uint32_t crc = 0;
  uint32_t off = 0;
  while (off < size)
  {
    uint32_t n = size - off;
    if (n > sizeof buf) n = sizeof buf;
    if (!QSpiFlash_Read(base + off, buf, n)) return false;
    crc = Ota_Crc32(crc, buf, n);
    off += n;
  }
  *out_crc = crc;
  return true;
}

bool Ota_FinishStage(uint32_t expected_crc, uint32_t *out_crc, uint32_t *out_received)
{
  if (out_received) *out_received = stage_written;
  if (out_crc)      *out_crc = 0;

  bool complete = stage_active && (stage_written == stage_size);
  uint32_t crc = 0;
  bool crc_ok = complete && crc_slot(OTA_SLOT_INCOMING, stage_size, &crc)
                         && (crc == expected_crc);

  OtaMeta m = {
    .magic  = crc_ok ? OTA_META_MAGIC : 0,
    .size   = stage_size,
    .crc32  = crc,
    .status = crc_ok ? OTA_STATUS_VALID : OTA_STATUS_INVALID,
  };
  write_meta(OTA_META_INCOMING, &m);

  if (out_crc) *out_crc = crc;
  stage_active = false;
  return crc_ok;
}

bool Ota_VerifyIncoming(void)
{
  OtaMeta m;
  if (!Ota_ReadMeta(&m)) return false;
  if (m.magic != OTA_META_MAGIC || m.status != OTA_STATUS_VALID) return false;
  if (m.size == 0 || m.size > OTA_IMAGE_MAX) return false;

  uint32_t crc;
  if (!crc_slot(OTA_SLOT_INCOMING, m.size, &crc)) return false;
  return crc == m.crc32;
}
