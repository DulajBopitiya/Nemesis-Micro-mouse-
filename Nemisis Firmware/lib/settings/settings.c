/**
 ******************************************************************************
 * @file    settings.c
 * @brief   Persistent tuning profiles in on-chip flash - see settings.h.
 ******************************************************************************
 */
#include "settings.h"
#include "control.h"
#include "main.h"          /* HAL + FLASH definitions */
#include "dbg.h"
#include <string.h>
#include <stddef.h>

/* ===========================================================================
 *  On-flash layout. One blob in the LAST flash page. The STM32G474RE can be
 *  single-bank (4 KB pages) or dual-bank (2 KB pages) depending on the DBANK
 *  option bit, so the page address / bank / number are computed at runtime
 *  (see flash_geometry). Code only uses ~12% of flash, so the last page is free
 *  without touching the linker script.
 * ========================================================================= */
#define SETTINGS_MAGIC    0x4E454D31u   /* "NEM1" */
#define SETTINGS_VERSION  2u             /* v2: + turn_mode + arc geometry per profile */

/* One profile's worth of controller tuning. `reserved` leaves headroom so turn
   parameters (pivot/arc) can be added later without breaking the flash format -
   bump SETTINGS_VERSION and consume reserved words. Kept an 8-byte multiple so
   the blob programs cleanly in 64-bit doublewords. */
typedef struct
{
  float   ff;                       /* velocity feedforward (duty/cps)        */
  float   vel_kp, vel_ki, vel_kd;   /* inner velocity PID                     */
  float   head_kp, head_ki, head_kd;/* heading-hold PID                       */
  float   accel, decel;             /* setpoint ramp rates (cps/s)            */
  int32_t vel_win;                  /* velocity filter window (ms)            */
  float   vel_lpf;                  /* velocity filter EMA alpha              */
  float   turn_rate;                /* pivot slew cap (deg/s)                 */
  float   turn_accel;               /* pivot slew accel (deg/s^2)             */
  int32_t turn_mode;                /* 0 = pivot turns, 1 = smooth (arc) turns */
  float   arc_radius;               /* smooth-turn radius (mm)                */
  float   arc_entry;                /* lead-in straight before the arc (mm)   */
  float   arc_exit;                 /* lead-out straight after the arc (mm)   */
  float   reserved[5];              /* future headroom                        */
} Profile;                          /* 88 bytes (layout unchanged)            */

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t active;
  Profile  prof[SETTINGS_NUM_PROFILES];
  uint32_t crc;                     /* CRC32 over everything above            */
  uint32_t pad;                     /* keep total an 8-byte multiple          */
} Blob;

/* Compile-time guard: the blob must be a whole number of doublewords. */
typedef char settings_blob_is_doubleword_aligned[(sizeof(Blob) % 8u == 0) ? 1 : -1];

static const char *const PROFILE_NAMES[SETTINGS_NUM_PROFILES] =
{
  "Search", "Fast", "Fastest"
};

/* The live, in-RAM copy. Flushed to flash by Settings_Save. */
static Blob g;

/* ===========================================================================
 *  Flash geometry (DBANK-aware) + CRC
 * ========================================================================= */
typedef struct { uint32_t addr; uint32_t bank; uint32_t page; } FlashPage;

static FlashPage flash_geometry(void)
{
  FlashPage f;
  uint32_t total = (uint32_t)(*(volatile uint16_t *)FLASHSIZE_BASE) * 1024u; /* KB->bytes */

  if (FLASH->OPTR & FLASH_OPTR_DBANK)
  {
    /* Dual bank: 2 KB pages, second bank starts at FLASH_BASE + total/2. */
    uint32_t page_sz = 0x800u;
    f.addr = FLASH_BASE + total - page_sz;          /* last page (in bank 2)  */
    f.bank = FLASH_BANK_2;
    f.page = (f.addr - (FLASH_BASE + total / 2u)) / page_sz;
  }
  else
  {
    /* Single bank: 4 KB pages. */
    uint32_t page_sz = 0x1000u;
    f.addr = FLASH_BASE + total - page_sz;
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

/* ===========================================================================
 *  Profile <-> live controller
 * ========================================================================= */
bool Settings_Capture(int idx)
{
  if (idx < 0 || idx >= SETTINGS_NUM_PROFILES) return false;
  Profile *p = &g.prof[idx];
  float kp, ki, kd;

  p->ff = Control_GetVelFF();
  Control_GetGains(CTRL_LOOP_LVEL, &kp, &ki, &kd);
  p->vel_kp = kp; p->vel_ki = ki; p->vel_kd = kd;
  Control_GetGains(CTRL_LOOP_HEAD, &kp, &ki, &kd);
  p->head_kp = kp; p->head_ki = ki; p->head_kd = kd;

  float a, d; Control_GetAccel(&a, &d);
  p->accel = a; p->decel = d;

  int w; float al; Control_GetVelFilter(&w, &al);
  p->vel_win = w; p->vel_lpf = al;

  float tr, ta; Control_GetTurn(&tr, &ta);
  p->turn_rate = tr; p->turn_accel = ta;

  float rad, ent, ex; Control_GetArcGeom(&rad, &ent, &ex);
  p->turn_mode  = Control_GetTurnMode();
  p->arc_radius = rad; p->arc_entry = ent; p->arc_exit = ex;
  return true;
}

bool Settings_Apply(int idx)
{
  if (idx < 0 || idx >= SETTINGS_NUM_PROFILES) return false;
  const Profile *p = &g.prof[idx];

  Control_SetVelFF(p->ff);
  Control_SetGains(CTRL_LOOP_VEL, p->vel_kp, p->vel_ki, p->vel_kd);
  Control_SetGains(CTRL_LOOP_HEAD, p->head_kp, p->head_ki, p->head_kd);
  Control_SetAccel(p->accel, p->decel);
  Control_SetVelFilter(p->vel_win, p->vel_lpf);
  Control_SetTurn(p->turn_rate, p->turn_accel);  /* 0 from an old blob = keep default */
  Control_SetTurnMode((int)p->turn_mode);
  Control_SetArcGeom(p->arc_radius, p->arc_entry, p->arc_exit);
  return true;
}

bool Settings_SetActive(int idx)
{
  if (idx < 0 || idx >= SETTINGS_NUM_PROFILES) return false;
  g.active = (uint16_t)idx;
  return Settings_Apply(idx);
}

int Settings_Active(void) { return (int)g.active; }

const char *Settings_ProfileName(int idx)
{
  if (idx < 0 || idx >= SETTINGS_NUM_PROFILES) return "?";
  return PROFILE_NAMES[idx];
}

int Settings_Parse(const char *s)
{
  if (!s || !*s) return -1;
  for (int i = 0; i < SETTINGS_NUM_PROFILES; i++)
  {
    const char *n = PROFILE_NAMES[i];
    int j = 0;
    while (n[j] && s[j])
    {
      char a = s[j], b = n[j];
      if (a >= 'A' && a <= 'Z') a += 32;
      if (b >= 'A' && b <= 'Z') b += 32;
      if (a != b) break;
      j++;
    }
    if (n[j] == '\0' && s[j] == '\0') return i;   /* full case-insensitive match */
  }
  /* single-digit index 0..N-1 */
  if (s[1] == '\0' && s[0] >= '0' && s[0] <= ('0' + SETTINGS_NUM_PROFILES - 1))
    return s[0] - '0';
  return -1;
}

/* ===========================================================================
 *  Flash load / save
 * ========================================================================= */
void Settings_Init(void)
{
  FlashPage fp = flash_geometry();
  const Blob *f = (const Blob *)fp.addr;

  if (f->magic == SETTINGS_MAGIC && f->version == SETTINGS_VERSION &&
      f->crc == blob_crc(f))
  {
    g = *f;                                  /* copy flash blob into RAM       */
    if (g.active >= SETTINGS_NUM_PROFILES) g.active = 0;
    Settings_Apply((int)g.active);           /* override baked defaults        */
    LOG("SETTINGS: loaded flash, active profile %d (%s)\r\n",
        (int)g.active, Settings_ProfileName((int)g.active));
  }
  else
  {
    /* Blank / corrupt / old version: seed every profile from the controller's
       current baked-in defaults. Not written to flash until the user saves. */
    g.magic   = SETTINGS_MAGIC;
    g.version = SETTINGS_VERSION;
    g.active  = 0;
    g.pad     = 0;
    for (int i = 0; i < SETTINGS_NUM_PROFILES; i++) Settings_Capture(i);

    /* Differentiate the three regimes from the common (search) base just captured.
       0 Search  = the baked gentle pivot tune (left as captured).
       1 Fast    = same loop tune, SNAPPIER pivots (~2x slew/accel) for quick turns.
       2 Fastest = SMOOTH (arc) turns - flow through corners without stopping.
       Values are starting points; tune live and `save`. Arc geometry assumes a
       ~180 mm cell: R=90 (half cell), entry/exit = pitch-R = 90 mm each. */
    g.prof[1].turn_rate  = 700.0f;   /* was 550 (300 baked) - quicker pivot */
    g.prof[1].turn_accel = 6000.0f;  /* was 4000 - snap into/out of the turn  */
    g.prof[1].turn_mode  = 0;        /* still pivots, just fast */

    g.prof[2].turn_rate  = 550.0f;
    g.prof[2].turn_accel = 4000.0f;
    g.prof[2].turn_mode  = 1;        /* smooth arc turns */
    g.prof[2].arc_radius = 90.0f;
    g.prof[2].arc_entry  = 90.0f;
    g.prof[2].arc_exit   = 90.0f;
    LOG("SETTINGS: flash empty - seeded Search/Fast/Fastest profiles (use 'save')\r\n");
  }
}

bool Settings_Save(void)
{
  if (Control_IsActive()) return false;       /* erase stalls the CPU/loop     */

  g.magic   = SETTINGS_MAGIC;
  g.version = SETTINGS_VERSION;
  g.pad     = 0;
  g.crc     = blob_crc(&g);

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
    const uint64_t *src = (const uint64_t *)(const void *)&g;
    uint32_t n64 = sizeof(Blob) / 8u;
    for (uint32_t i = 0; i < n64 && ok; i++)
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                            fp.addr + i * 8u, src[i]) != HAL_OK)
        ok = false;
    }
  }

  HAL_FLASH_Lock();

  /* Verify what actually landed in flash. */
  if (ok)
  {
    const Blob *f = (const Blob *)fp.addr;
    ok = (f->magic == SETTINGS_MAGIC && f->crc == blob_crc(f) &&
          memcmp(f, &g, offsetof(Blob, crc)) == 0);
  }
  return ok;
}
