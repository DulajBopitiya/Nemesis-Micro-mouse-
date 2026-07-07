/*
 * ota - firmware image STAGING in the external W25Q32JW (QSPI) flash.
 *
 * Tier A of the OTA plan: receive a new app image over the existing link,
 * write it into the QSPI "incoming" slot, and CRC-verify it there. NO jump /
 * copy-to-internal here - that is the (later, higher-risk) bootloader's job.
 * This layer only proves the transport + staging pipeline, and it cannot
 * brick the board: a bad/interrupted transfer just leaves an invalid slot.
 *
 * On top of the raw lib/qspiflash driver. See memory `ota-external-flash-plan`.
 *
 * ---- QSPI 4 MB layout -----------------------------------------------------
 *   0x000000  incoming image slot   (<= 480 KB, matches the internal app slot)
 *   0x0F0000  incoming metadata     (1 sector: size + crc32 + status)
 *   0x100000  golden image slot     (known-good backup, for rollback)
 *   0x1F0000  golden metadata       (1 sector)
 *   0x3FF000  scratch               (used by `qspi test`)
 */
#ifndef OTA_H
#define OTA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Max image = internal app slot size once the bootloader carve-out exists
   (480 KB = 0x08008000..0x0807D000). Enforced now so a staged image can never
   be too big to copy later. */
#define OTA_IMAGE_MAX        (480u * 1024u)

#define OTA_SLOT_INCOMING    0x000000u
#define OTA_META_INCOMING    0x0F0000u
#define OTA_SLOT_GOLDEN      0x100000u
#define OTA_META_GOLDEN      0x1F0000u

#define OTA_META_MAGIC       0x4F544131u   /* "OTA1" */

/* "Apply this update on next boot" handshake between the app and the bootloader.
   The app writes OTA_APPLY_MAGIC to TAMP backup register 0 (survives a system
   reset) then resets; the bootloader reads it, does the copy, and clears it.
   MUST be kept in sync with the copy in bootloader/src/main.c. */
#define OTA_APPLY_MAGIC      0x0A7A0A7Au

/* "An update was just installed" handshake, the other direction: the bootloader
   writes OTA_APPLIED_MAGIC to TAMP backup register 1 right after a successful
   copy, and the app reads+clears it on boot to flash the LEDs green. MUST match
   the copy in bootloader/src/main.c. */
#define OTA_APPLIED_MAGIC    0x0A9911EDu

/* Metadata record stored (erased-then-written) in a dedicated QSPI sector. */
typedef struct
{
  uint32_t magic;      /* OTA_META_MAGIC when valid            */
  uint32_t size;       /* image length in bytes                */
  uint32_t crc32;      /* zlib/IEEE CRC-32 over the image       */
  uint32_t status;     /* OTA_STATUS_*                          */
} OtaMeta;

#define OTA_STATUS_VALID     0xA5A5A5A5u
#define OTA_STATUS_INVALID   0x00000000u

/* zlib-compatible CRC-32 (poly 0xEDB88320). Feed 0 as the initial `crc`. */
uint32_t Ota_Crc32(uint32_t crc, const uint8_t *data, size_t len);

/** Begin staging `size` bytes into the incoming slot: validates the size and
 *  erases exactly the sectors it will occupy. Returns false if size is 0 or
 *  exceeds OTA_IMAGE_MAX, or on a flash error. */
bool Ota_BeginStage(uint32_t size);

/** Append `len` bytes to the incoming slot at the current write cursor (call
 *  repeatedly during receive). Must be preceded by Ota_BeginStage. Returns
 *  false on overflow past the announced size or a flash error. */
bool Ota_WriteChunk(const uint8_t *data, size_t len);

/** Finish staging: read the slot back out of QSPI, CRC-32 it, and compare to
 *  `expected_crc`. On match, writes VALID metadata and returns true (the
 *  computed crc is stored in *out_crc). On mismatch, writes INVALID metadata
 *  and returns false. `*out_received` reports the byte count actually written. */
bool Ota_FinishStage(uint32_t expected_crc, uint32_t *out_crc, uint32_t *out_received);

/** Read the incoming-slot metadata. Returns false if unreadable. */
bool Ota_ReadMeta(OtaMeta *meta);

/** Re-verify the incoming slot against its stored metadata (independent CRC
 *  read-back). Returns true iff metadata is VALID and the CRC still matches. */
bool Ota_VerifyIncoming(void);

/** Set the "apply on next boot" flag (TAMP backup reg 0). The caller then
 *  resets (NVIC_SystemReset) so the bootloader copies the staged image into the
 *  app slot. Does NOT itself reset. Only call after Ota_VerifyIncoming(). */
void Ota_RequestApply(void);

/** Returns true exactly once after the bootloader installed an update this boot
 *  (reads and clears TAMP backup reg 1). Use it to show a boot confirmation. */
bool Ota_JustApplied(void);

#endif /* OTA_H */
