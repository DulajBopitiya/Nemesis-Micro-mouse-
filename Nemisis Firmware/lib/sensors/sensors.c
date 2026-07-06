/**
 ******************************************************************************
 * @file    sensors.c
 * @brief   IR reflectance acquisition - polling stage. See sensors.h.
 ******************************************************************************
 */
#include "sensors.h"
#include <string.h>
#include <stddef.h>

/* --- tuning knobs (polling stage) ---------------------------------------- */
#define SETTLE_US_DEFAULT  150U   /* emitter rise/settle before sampling (was 1 ms) */
/* Averaging is now done by the ADC HARDWARE oversampler (8x, enabled in
   MX_ADC*_Init) - one conversion returns the mean of 8 samples. So the software
   loop below defaults to ONE conversion (was 8): same averaged reading, ~8x fewer
   ADC start/stop cycles per sweep. 'irset samples' can still stack extra software
   averaging on top of the hardware 8x if a channel ever needs it. */
#define ADC_SAMPLES_DEFAULT  1U   /* SW loops on top of the HW 8x oversampler       */
/* Receiver smoothing (smaller = steadier, laggier). Dropped 0.40 -> 0.30 when
   the sweep went to 200 Hz: at the faster rate a gentler alpha keeps the same
   (or better) noise rejection while the doubled sample rate is what cuts the
   lag, so net delay falls without getting noisier. Filter time-constant ~12 ms. */
#define EMA_ALPHA          0.30f

/* --- calibration ---------------------------------------------------------- */
#define CAL_SNAP_SWEEPS   16U     /* sweeps averaged per capture (mouse held still) */
#define CAL_THRESH_FRAC   0.40f   /* trip point = dark + FRAC*(ref-dark)            */
/* Hysteresis: a wall is ASSERTED when the reading rises past `thresh`, and only
   CLEARED when it falls back below this fraction of the dark->thresh span. The
   gap stops a reading hovering near the threshold from flickering wall/no-wall
   sweep to sweep - the "sometimes sees it, sometimes doesn't" failure. */
#define CAL_HYST_FRAC     0.65f   /* clear below dark + 0.65*(thresh-dark)          */

/* Live-tunable from the console ('irset') so the emitter settle / oversampling
   can be swept on the bench - the IR signal is weak, and the right settle is a
   property of the phototransistor we measure rather than guess. */
static uint32_t settle_us   = SETTLE_US_DEFAULT;
static uint32_t adc_samples = ADC_SAMPLES_DEFAULT;

typedef struct
{
  GPIO_TypeDef      *port;
  uint16_t           pin;
  ADC_HandleTypeDef *adc;
  uint32_t           ch;
} IRChan;

static IRChan chan[SENSOR_COUNT];
static float  filt[SENSOR_COUNT];     /* EMA state                            */
static int    out[SENSOR_COUNT];      /* published filtered values            */
static int    raw_refl[SENSOR_COUNT]; /* THIS sweep's unfiltered reflectance   */
static bool   wall_on[SENSOR_COUNT];  /* hysteretic wall-present latch         */
static bool   async_en;               /* non-blocking background sweep active   */

static void update_wall_latches(void);   /* defined after the cal table below   */
static int    amb_dbg[SENSOR_COUNT];  /* last raw ambient (emitter off)       */
static int    lit_dbg[SENSOR_COUNT];  /* last raw lit     (emitter on)        */

/* --- microsecond delay via the DWT cycle counter (same idea as motor.c) --- */
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < ticks) { }
}

/* Average ADC_SAMPLES single conversions of one channel. Keeps the 640.5-cycle
   sampling time the rest of the firmware uses for the high-Z receiver node. */
static uint16_t adc_read_avg(ADC_HandleTypeDef *adc, uint32_t ch)
{
  ADC_ChannelConfTypeDef c = {0};
  c.Channel      = ch;
  c.Rank         = ADC_REGULAR_RANK_1;
  c.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  c.SingleDiff   = ADC_SINGLE_ENDED;
  c.OffsetNumber = ADC_OFFSET_NONE;
  c.Offset       = 0;
  HAL_ADC_ConfigChannel(adc, &c);

  uint32_t n = adc_samples;
  uint32_t acc = 0;
  for (uint32_t i = 0; i < n; i++)
  {
    HAL_ADC_Start(adc);
    HAL_ADC_PollForConversion(adc, 5);
    acc += HAL_ADC_GetValue(adc);
    HAL_ADC_Stop(adc);
  }
  return (uint16_t)(acc / n);
}

/* Ambient-subtracted reflectance of one sensor: dark reading, then lit, minus.
   Clamped at 0 (a wall always reflects MORE than ambient). The raw amb/lit are
   reported back too for the 'irraw' bench diagnostic. */
static int ir_read(const IRChan *s, int *amb_out, int *lit_out)
{
  HAL_GPIO_WritePin(s->port, s->pin, GPIO_PIN_RESET);   /* emitter off         */
  delay_us(settle_us);
  int amb = (int)adc_read_avg(s->adc, s->ch);

  HAL_GPIO_WritePin(s->port, s->pin, GPIO_PIN_SET);     /* emitter on          */
  delay_us(settle_us);
  int lit = (int)adc_read_avg(s->adc, s->ch);

  HAL_GPIO_WritePin(s->port, s->pin, GPIO_PIN_RESET);   /* leave it off        */

  *amb_out = amb;
  *lit_out = lit;
  int refl = lit - amb;
  return (refl < 0) ? 0 : refl;
}

void Sensors_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc4)
{
  dwt_init();

  /* Emitter pin + ADC channel per sensor - mirrors the table in Console_Init.
     Five receivers on ADC1, the rightmost (R_RM) on ADC4. */
  chan[SENS_L_LM] = (IRChan){ LEFT_LM_EMMITER_GPIO_Port,     LEFT_LM_EMMITER_Pin,     hadc1, ADC_CHANNEL_1 };
  chan[SENS_L_M ] = (IRChan){ LEFT_M_EMMITER_GPIO_Port,      LEFT_M_EMMITER_Pin,      hadc1, ADC_CHANNEL_9 };
  chan[SENS_L_F ] = (IRChan){ LEFT_FRONT_EMMITER_GPIO_Port,  LEFT_FRONT_EMMITER_Pin,  hadc1, ADC_CHANNEL_8 };
  chan[SENS_R_F ] = (IRChan){ RIGHT_FRONT_EMMITER_GPIO_Port, RIGHT_FRONT_EMMITER_Pin, hadc1, ADC_CHANNEL_7 };
  chan[SENS_R_M ] = (IRChan){ RIGHT_M_EMMITER_GPIO_Port,     RIGHT_M_EMMITER_Pin,     hadc1, ADC_CHANNEL_4 };
  chan[SENS_R_RM] = (IRChan){ RIGHT_RM_EMMITER_GPIO_Port,    RIGHT_RM_EMMITER_Pin,    hadc4, ADC_CHANNEL_3 };

  for (int i = 0; i < SENSOR_COUNT; i++) { filt[i] = 0.0f; out[i] = 0; }
}

void Sensors_Update(void)
{
  if (async_en) return;                 /* the background pump owns the sweep now  */
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    int refl = ir_read(&chan[i], &amb_dbg[i], &lit_dbg[i]);
    raw_refl[i] = refl;                 /* unfiltered: leads the EMA for the brake */
    filt[i] += EMA_ALPHA * ((float)refl - filt[i]);
    out[i] = (int)filt[i];
  }
  update_wall_latches();
}

/* === Non-blocking IR sweep: TIM7-paced, ADC-interrupt driven ===============
 * The blocking Sensors_Update above spins the CPU ~3.4 ms per sweep (settle
 * busy-waits + PollForConversion). This runs the SAME sweep in the background off
 * hardware interrupts so the main loop is free:
 *   TIM7 update ISR  fires after the 150 us emitter settle -> launches the ADC
 *   ADC EOC ISR      the (oversampled) conversion is done -> reads the value and
 *                    arms the next phase (emitter + TIM7)
 * Because both the settle (a hardware timer) and the conversion (a hardware
 * interrupt) are timed in HARDWARE, the ambient->lit gap is a FIXED ~280 us
 * regardless of main-loop load - identical pacing to the blocking sweep that reads
 * correctly, which the earlier main-loop-polled attempt could NOT guarantee (its
 * gap rode the loop -> ambient-subtraction noise -> phantom walls). Same maths
 * (ir_read phases, EMA, wall latches) so readings + calibration are unchanged.
 * TIM7 + the two ADC IRQs sit BELOW the 1 kHz control loop (prio 5) so control
 * stays exact. Gated behind 'irasync'; OFF (default) = the blocking path runs. */
static TIM_HandleTypeDef htim7_ir;        /* settle timer (1 us tick, one-shot)     */
static bool  ir_tim_ready;                /* TIM7 configured once                   */
static int   sw_sensor;                   /* current sensor 0..SENSOR_COUNT-1       */
static int   sw_phase;                    /* 0 = ambient (emitter off), 1 = lit      */
static int   sw_amb;                      /* ambient captured for the current sensor */

static void ir_timer_init(void)
{
  __HAL_RCC_TIM7_CLK_ENABLE();
  /* APB1 timer clock (= PCLK1, doubled when the APB1 prescaler isn't /1) - mirror
     the derivation control.c uses for TIM6. */
  RCC_ClkInitTypeDef clk; uint32_t lat; HAL_RCC_GetClockConfig(&clk, &lat);
  uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
  uint32_t timclk = (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : pclk1 * 2U;

  htim7_ir.Instance               = TIM7;
  htim7_ir.Init.Prescaler         = (timclk / 1000000U) - 1U;   /* 1 MHz -> 1 us tick */
  htim7_ir.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim7_ir.Init.Period            = (settle_us ? settle_us : 1U) - 1U;
  htim7_ir.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* live settle_us */
  HAL_TIM_Base_Init(&htim7_ir);
  htim7_ir.Instance->EGR = TIM_EGR_UG;                 /* load PSC/ARR before first use */
  __HAL_TIM_CLEAR_FLAG(&htim7_ir, TIM_FLAG_UPDATE);

  HAL_NVIC_SetPriority(TIM7_DAC_IRQn, 6, 0);           /* below control (5)             */
  HAL_NVIC_EnableIRQ(TIM7_DAC_IRQn);
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 6, 0); HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  HAL_NVIC_SetPriority(ADC4_IRQn,   6, 0); HAL_NVIC_EnableIRQ(ADC4_IRQn);
  ir_tim_ready = true;
}

/* Drive the emitter for `phase` (off = ambient, on = lit), select the sensor's ADC
   channel on a new sensor, and start the TIM7 settle timer. When it elapses the
   TIM7 ISR launches the conversion. */
static void ir_arm_settle(int sensor, int phase)
{
  const IRChan *s = &chan[sensor];
  HAL_GPIO_WritePin(s->port, s->pin, phase ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (phase == 0)                         /* new sensor: its channel (same amb+lit)  */
  {
    ADC_ChannelConfTypeDef c = {0};
    c.Channel      = s->ch;
    c.Rank         = ADC_REGULAR_RANK_1;
    c.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    c.SingleDiff   = ADC_SINGLE_ENDED;
    c.OffsetNumber = ADC_OFFSET_NONE;
    c.Offset       = 0;
    HAL_ADC_ConfigChannel(s->adc, &c);
  }
  sw_sensor = sensor;
  sw_phase  = phase;
  __HAL_TIM_SET_COUNTER(&htim7_ir, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim7_ir, (settle_us ? settle_us : 1U) - 1U);
  __HAL_TIM_CLEAR_FLAG(&htim7_ir, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim7_ir);
}

/* Stop the background sweep hardware: TIM7 off, both ADC interrupts off, emitters
   off. Used on toggle-off and to fence a blocking calibration read. */
static void sw_stop_hw(void)
{
  if (ir_tim_ready) HAL_TIM_Base_Stop_IT(&htim7_ir);
  HAL_ADC_Stop_IT(chan[SENS_L_LM].adc);   /* hadc1 */
  HAL_ADC_Stop_IT(chan[SENS_R_RM].adc);   /* hadc4 */
  for (int i = 0; i < SENSOR_COUNT; i++)
    HAL_GPIO_WritePin(chan[i].port, chan[i].pin, GPIO_PIN_RESET);
}

/* --- interrupt handlers (override the weak startup stubs; NOT in it.c) ------ */

/* TIM7 elapsed = settle done -> fire the conversion. Serviced directly (not via
   HAL_TIM_IRQHandler) so it never touches control.c's TIM6 PeriodElapsed path. */
void TIM7_DAC_IRQHandler(void)
{
  if (__HAL_TIM_GET_FLAG(&htim7_ir, TIM_FLAG_UPDATE) &&
      __HAL_TIM_GET_IT_SOURCE(&htim7_ir, TIM_IT_UPDATE))
  {
    __HAL_TIM_CLEAR_IT(&htim7_ir, TIM_IT_UPDATE);
    HAL_TIM_Base_Stop_IT(&htim7_ir);      /* one-shot */
    if (async_en) HAL_ADC_Start_IT(chan[sw_sensor].adc);
  }
}

void ADC1_2_IRQHandler(void) { HAL_ADC_IRQHandler(chan[SENS_L_LM].adc); }  /* hadc1 */
void ADC4_IRQHandler(void)   { HAL_ADC_IRQHandler(chan[SENS_R_RM].adc); }  /* hadc4 */

/* One (oversampled) conversion finished. Read it and advance the sweep. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (!async_en || hadc != chan[sw_sensor].adc) return;   /* not our sweep          */
  int val = (int)HAL_ADC_GetValue(hadc);

  if (sw_phase == 0)                      /* ambient captured -> do the lit phase     */
  {
    sw_amb = val; amb_dbg[sw_sensor] = val;
    ir_arm_settle(sw_sensor, 1);          /* emitter on -> settle -> lit conversion   */
  }
  else                                    /* lit captured -> reflectance + EMA        */
  {
    lit_dbg[sw_sensor] = val;
    HAL_GPIO_WritePin(chan[sw_sensor].port, chan[sw_sensor].pin, GPIO_PIN_RESET);
    int refl = val - sw_amb; if (refl < 0) refl = 0;
    raw_refl[sw_sensor] = refl;
    filt[sw_sensor] += EMA_ALPHA * ((float)refl - filt[sw_sensor]);
    out[sw_sensor] = (int)filt[sw_sensor];

    if (sw_sensor + 1 >= SENSOR_COUNT)
    {
      update_wall_latches();              /* sweep done -> publish, loop continuously */
      ir_arm_settle(0, 0);
    }
    else ir_arm_settle(sw_sensor + 1, 0); /* next sensor, ambient                     */
  }
}

/* Kept for source compatibility (main.c calls it every pass). The sweep is now
   fully interrupt-driven, so there is nothing to pump. */
void Sensors_Pump(void) { }

void Sensors_SetAsync(bool on)
{
  if (on && !async_en)
  {
    if (!ir_tim_ready) ir_timer_init();
    async_en = true;
    ir_arm_settle(0, 0);                  /* kick the first phase; ISRs carry on      */
  }
  else if (!on && async_en)
  {
    async_en = false;                     /* ISR guards no-op from here               */
    sw_stop_hw();
  }
}
bool Sensors_GetAsync(void) { return async_en; }

int        Sensors_Get(SensorId id) { return out[id]; }
const int *Sensors_Raw(void)        { return out; }

void Sensors_GetRaw(int *amb, int *lit)
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (amb) amb[i] = amb_dbg[i];
    if (lit) lit[i] = lit_dbg[i];
  }
}

void Sensors_SetTiming(uint32_t settle, uint32_t samples)
{
  if (settle  > 0U)            settle_us   = settle;
  if (samples >= 1U && samples <= 64U) adc_samples = samples;
}

/* === Speed-adaptive sweep timing ==========================================
 * Faster the mouse moves, the more a slow sweep lags (it travels further while
 * the 6 sensors are read), so we drop the per-phase settle + ADC oversampling at
 * speed to finish the sweep quicker - trading a little extra noise (smoothed by
 * the EMA) for fresher position info. Stationary/slow we average hard for clean
 * readings, so calibration (done still) uses the cleanest timing. Bands have
 * hysteresis so the timing can't flap at a boundary. ON by default; turn off
 * for bench A/B with Sensors_SetAdaptive(false) (then 'irset' rules again). */
/* OFF by default. It changed the per-read timing (settle/oversampling) between
   speed bands - but calibration ('ircal') is captured STATIONARY (slow band:
   150 us / 8 samples), so at speed the live readings no longer matched the
   calibrated thresholds and wall detection got WORSE. The search isn't fast
   enough to need it, so we keep the cal timing everywhere. Re-enable only once
   calibration is captured at the SAME timing the fast run uses. */
static bool adaptive = false;
static int  sband    = 0;             /* 0 = slow/clean, 1 = medium, 2 = fast     */

void Sensors_SetAdaptive(bool on) { adaptive = on; }
bool Sensors_GetAdaptive(void)    { return adaptive; }

void Sensors_AdaptTiming(float cps)
{
  if (!adaptive) return;

  /* ~24500 cps = 1 m/s. Bands: <~0.2, ~0.2-0.45, >~0.45 m/s, with hysteresis. */
  if      (sband == 0) { if (cps > 5500.0f)  sband = 1; }
  else if (sband == 1) { if (cps > 11500.0f) sband = 2; else if (cps < 4000.0f) sband = 0; }
  else                 { if (cps < 9500.0f)  sband = 1; }

  switch (sband)
  {
    case 2:  settle_us =  60U; adc_samples = 2U; break;   /* fast: minimal lag     */
    case 1:  settle_us = 100U; adc_samples = 4U; break;   /* medium                */
    default: settle_us = 150U; adc_samples = 8U; break;   /* slow: cleanest        */
  }
}

void Sensors_GetTiming(uint32_t *settle, uint32_t *samples)
{
  if (settle)  *settle  = settle_us;
  if (samples) *samples = adc_samples;
}

/* ===========================================================================
 *  Calibration  (see sensors.h)
 *
 *  Stored in its OWN flash page - the page just below the settings/profile page
 *  - so committing calibration never disturbs the tuning profiles. Magic +
 *  version + CRC make it independently valid; a blank/garbled page just means
 *  "uncalibrated" (Walls_* return false, Sensors_Lateral returns 0).
 * ========================================================================= */
#define CAL_MAGIC    0x4E454943u   /* "NEIC" - Nemisis IR Calibration */
#define CAL_VERSION  1u
#define CAL_FLAG_DARK  0x1u
#define CAL_FLAG_SIDE  0x2u
#define CAL_FLAG_FRONT 0x4u

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t flags;                   /* which captures have been done            */
  int32_t  dark[SENSOR_COUNT];      /* no-wall floor (incl. crosstalk pedestal) */
  int32_t  ref[SENSOR_COUNT];       /* reference position reading               */
  int32_t  thresh[SENSOR_COUNT];    /* wall-present trip point                  */
  uint32_t crc;                     /* CRC32 over everything above              */
  uint32_t pad;                     /* keep total an 8-byte multiple            */
} CalBlob;                          /* 88 bytes                                 */

/* Compile-time guard: the blob must program in whole 64-bit doublewords. */
typedef char cal_blob_is_doubleword_aligned[(sizeof(CalBlob) % 8u == 0) ? 1 : -1];

static CalBlob cal;                 /* live, in-RAM copy                        */

/* --- flash geometry: the page just BELOW the settings page (DBANK-aware, same
       runtime computation settings.c uses, offset one extra page). ----------- */
typedef struct { uint32_t addr; uint32_t bank; uint32_t page; } CalFlashPage;

static CalFlashPage cal_flash_page(void)
{
  CalFlashPage f;
  uint32_t total = (uint32_t)(*(volatile uint16_t *)FLASHSIZE_BASE) * 1024u;

  if (FLASH->OPTR & FLASH_OPTR_DBANK)
  {
    uint32_t page_sz = 0x800u;                          /* dual bank: 2 KB pages */
    f.addr = FLASH_BASE + total - 2u * page_sz;         /* second-to-last page   */
    f.bank = FLASH_BANK_2;
    f.page = (f.addr - (FLASH_BASE + total / 2u)) / page_sz;
  }
  else
  {
    uint32_t page_sz = 0x1000u;                         /* single bank: 4 KB     */
    f.addr = FLASH_BASE + total - 2u * page_sz;
    f.bank = FLASH_BANK_1;
    f.page = (f.addr - FLASH_BASE) / page_sz;
  }
  return f;
}

static uint32_t cal_crc(const CalBlob *b)
{
  const uint8_t *d = (const uint8_t *)b;
  uint32_t n = offsetof(CalBlob, crc);
  uint32_t c = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n; i++)
  {
    c ^= d[i];
    for (int k = 0; k < 8; k++)
      c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1u)));
  }
  return c ^ 0xFFFFFFFFu;
}

/* One settled snapshot: average CAL_SNAP_SWEEPS fresh reads per sensor (bypasses
   the EMA so a just-placed mouse reads true, not the lagged filter state). */
static void cal_snapshot(int *vals)
{
  /* Calibration reads blocking (ir_read). If the background (ISR) sweep is running,
     pause it so the two paths don't fight over the ADC, then resume after. */
  bool was_async = async_en;
  if (was_async) { async_en = false; sw_stop_hw(); }
  long acc[SENSOR_COUNT] = {0};
  for (uint32_t s = 0; s < CAL_SNAP_SWEEPS; s++)
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      int a, l;
      acc[i] += ir_read(&chan[i], &a, &l);
    }
  for (int i = 0; i < SENSOR_COUNT; i++)
    vals[i] = (int)(acc[i] / (long)CAL_SNAP_SWEEPS);
  if (was_async) { async_en = true; ir_arm_settle(0, 0); }   /* resume the ISR sweep */
}

/* thresh = dark + FRAC*(ref-dark), floored just above dark so noise never trips. */
static int32_t cal_threshold(int32_t dark, int32_t ref)
{
  int32_t span = ref - dark;
  if (span < 1) span = 1;
  int32_t t = dark + (int32_t)(CAL_THRESH_FRAC * (float)span);
  if (t <= dark) t = dark + 1;
  return t;
}

void Sensors_CalDark(void)
{
  int v[SENSOR_COUNT];
  cal_snapshot(v);
  for (int i = 0; i < SENSOR_COUNT; i++) cal.dark[i] = v[i];
  cal.flags |= CAL_FLAG_DARK;
}

void Sensors_CalSide(void)
{
  int v[SENSOR_COUNT];
  cal_snapshot(v);
  const SensorId side[2] = { SENS_L_LM, SENS_R_RM };
  for (int k = 0; k < 2; k++)
  {
    SensorId id = side[k];
    cal.ref[id]    = v[id];
    cal.thresh[id] = cal_threshold(cal.dark[id], v[id]);
  }
  cal.flags |= CAL_FLAG_SIDE;
}

void Sensors_CalFront(void)
{
  int v[SENSOR_COUNT];
  cal_snapshot(v);
  const SensorId front[2] = { SENS_L_F, SENS_R_F };
  for (int k = 0; k < 2; k++)
  {
    SensorId id = front[k];
    cal.ref[id]    = v[id];
    cal.thresh[id] = cal_threshold(cal.dark[id], v[id]);
  }
  cal.flags |= CAL_FLAG_FRONT;
}

/* --- Button / competition self-cal (see sensors.h) ----------------------- */
#define CAL_BTN_MIN_SPAN  15      /* a real wall must read >= this above dark   */
#define CAL_BTN_MAX_RAIL  3500    /* reject a railed / nonsense reading         */

void Sensors_CalSnapshot(int out[SENSOR_COUNT])
{
  cal_snapshot(out);
}

bool Sensors_CalApplyButton(const int dark[SENSOR_COUNT], const int ref[SENSOR_COUNT])
{
  /* The four channels a button cal sets: side (centring) + front (wall stop). */
  const SensorId ids[4] = { SENS_L_LM, SENS_R_RM, SENS_L_F, SENS_R_F };

  /* Validate the whole capture BEFORE writing anything - all four references
     must clearly see their wall. No L/R symmetry check on purpose: the side
     sensors have a large, real per-sensor gain difference (e.g. L_LM ~85 vs
     R_RM ~131 centred), which is captured and cancels per-sensor. */
  for (int k = 0; k < 4; k++)
  {
    SensorId id = ids[k];
    int32_t d = (dark[id] > 0) ? dark[id] : 0;
    if (ref[id] - d < CAL_BTN_MIN_SPAN || ref[id] >= CAL_BTN_MAX_RAIL)
      return false;                 /* bad placement - leave stored cal intact */
  }

  for (int i = 0; i < SENSOR_COUNT; i++)
    cal.dark[i] = (dark[i] > 0) ? dark[i] : 0;
  for (int k = 0; k < 4; k++)
  {
    SensorId id = ids[k];
    cal.ref[id]    = ref[id];
    cal.thresh[id] = cal_threshold(cal.dark[id], ref[id]);
  }
  cal.flags |= CAL_FLAG_DARK | CAL_FLAG_SIDE | CAL_FLAG_FRONT;
  return true;
}

void Sensors_GetCal(int *dark, int *ref, int *thresh, uint16_t *flags)
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (dark)   dark[i]   = (int)cal.dark[i];
    if (ref)    ref[i]    = (int)cal.ref[i];
    if (thresh) thresh[i] = (int)cal.thresh[i];
  }
  if (flags) *flags = cal.flags;
}

void Sensors_CalLoad(void)
{
  CalFlashPage fp = cal_flash_page();
  const CalBlob *f = (const CalBlob *)fp.addr;

  if (f->magic == CAL_MAGIC && f->version == CAL_VERSION && f->crc == cal_crc(f))
  {
    cal = *f;
  }
  else
  {
    memset(&cal, 0, sizeof cal);
    cal.magic   = CAL_MAGIC;
    cal.version = CAL_VERSION;
  }
}

bool Sensors_CalSave(void)
{
  cal.magic   = CAL_MAGIC;
  cal.version = CAL_VERSION;
  cal.pad     = 0;
  cal.crc     = cal_crc(&cal);

  CalFlashPage fp = cal_flash_page();
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
    const uint64_t *src = (const uint64_t *)(const void *)&cal;
    uint32_t n64 = sizeof(CalBlob) / 8u;
    for (uint32_t i = 0; i < n64 && ok; i++)
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                            fp.addr + i * 8u, src[i]) != HAL_OK)
        ok = false;
    }
  }

  HAL_FLASH_Lock();

  if (ok)
  {
    const CalBlob *f = (const CalBlob *)fp.addr;
    ok = (f->magic == CAL_MAGIC && f->crc == cal_crc(f) &&
          memcmp(f, &cal, offsetof(CalBlob, crc)) == 0);
  }
  return ok;
}

/* --- wall booleans + lateral error (built on the live EMA values) ----------
 * Detection is HYSTERETIC: per sensor, the latch sets when the reading climbs
 * past `thresh` and only clears when it drops below CAL_HYST_FRAC of the
 * dark->thresh span. A reading sitting near the threshold therefore holds its
 * last state instead of flickering wall/no-wall sweep to sweep - the cure for
 * "sometimes detects a wall, sometimes doesn't". Updated once per sweep from
 * Sensors_Update; the Walls_* queries just read the latch. */
static void update_wall_latches(void)
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    int32_t thr = cal.thresh[i];
    if (thr <= 0) { wall_on[i] = false; continue; }   /* uncalibrated channel */
    int32_t lo = cal.dark[i] + (int32_t)(CAL_HYST_FRAC * (float)(thr - cal.dark[i]));
    if (lo >= thr) lo = thr - 1;
    if      (out[i] > thr) wall_on[i] = true;
    else if (out[i] < lo)  wall_on[i] = false;
    /* between lo and thr: hold the previous state */
  }
}

bool Walls_Left(void)  { return wall_on[SENS_L_LM]; }
bool Walls_Right(void) { return wall_on[SENS_R_RM]; }

bool Walls_Front(void)
{
  return wall_on[SENS_L_F] || wall_on[SENS_R_F];  /* either front-diagonal counts */
}

/* High-confidence front wall: BOTH diagonals must trip. A real front wall
   reflects both L_F and R_F strongly; a single diagonal crossing threshold is
   almost always a weak/partial reflection (e.g. a side wall caught at an angle,
   or R_F sitting just above its noise floor). Use this when COMMITTING a wall
   to the permanent maze model - a phantom there severs a passage for good.
   Walls_Front() stays OR for the emergency front-stop: better to stop short
   than to drive into a wall one diagonal missed. */
bool Walls_FrontConfident(void)
{
  return wall_on[SENS_L_F] && wall_on[SENS_R_F];
}

/* Early/sensitive front read for the FRONT-STOP only: compares THIS sweep's
   UNFILTERED reflectance to threshold, so it trips ~one EMA time-constant (~10 ms
   ≈ a few mm at search speed) before the filtered Walls_Front() latch would - the
   margin the brake needs at 8000 cps. OR of the two diagonals (sensitive: better
   to brake early than ram). Noisier than the latched read, so it must NOT be used
   to RECORD a wall - the solver re-checks Walls_FrontConfident() while stopped
   before committing one. Uncalibrated channel (thr<=0) never trips. */
bool Walls_FrontRaw(void)
{
  int thr[SENSOR_COUNT];
  Sensors_GetCal(NULL, NULL, thr, NULL);
  bool lf = (thr[SENS_L_F] > 0) && (raw_refl[SENS_L_F] > thr[SENS_L_F]);
  bool rf = (thr[SENS_R_F] > 0) && (raw_refl[SENS_R_F] > thr[SENS_R_F]);
  return lf || rf;
}

/* Nose is AT a wall (CONTACT grade) - filtered front >= ~2x the frontcal
   stop-distance ref, far above mere detection (~90mm) - the "at the wall" level
   Control_StartContactProbe uses. This is the reading the mouse shows when it has
   driven up to a wall and the wheels are slip-spinning against it. Needs 'frontcal'
   (CAL_FLAG_FRONT); uncalibrated -> never trips (so callers degrade to no-op). */
#define FRONT_CONTACT_MULT   2        /* x front stop-distance ref = "at the wall" */
#define FRONT_CONTACT_MIN  200        /* floor, matches Control contact_level       */
bool Walls_FrontContact(void)
{
  int ref[SENSOR_COUNT]; uint16_t flags = 0;
  Sensors_GetCal(NULL, ref, NULL, &flags);
  if (!(flags & CAL_FLAG_FRONT)) return false;      /* no front cal -> never trips */
  int lref = FRONT_CONTACT_MULT * ref[SENS_L_F];
  int rref = FRONT_CONTACT_MULT * ref[SENS_R_F];
  if (lref < FRONT_CONTACT_MIN) lref = FRONT_CONTACT_MIN;
  if (rref < FRONT_CONTACT_MIN) rref = FRONT_CONTACT_MIN;
  bool lf = (ref[SENS_L_F] > 0) && (out[SENS_L_F] >= lref);
  bool rf = (ref[SENS_R_F] > 0) && (out[SENS_R_F] >= rref);
  return lf || rf;
}

/* Front wall CLOSE - between plain detection and hard contact: filtered front above
   ~2.5x the ircal wall threshold. Detection (Walls_Front, 1x) trips ~90mm out, right
   at a cell CENTRE, so using it to block the fast-run creep stalls the creep short of
   the cell MARK -> the believed position lags a cell -> the next move rams. This trips
   only when the nose is ~40-50mm from the wall, so the creep still reaches the mark,
   then stops before ramming. Keyed on the ircal THRESHOLD (always calibrated) - unlike
   Walls_FrontContact it never silently no-ops when frontcal is absent. */
bool Walls_FrontClose(void)
{
  int thr[SENSOR_COUNT];
  Sensors_GetCal(NULL, NULL, thr, NULL);
  bool lf = (thr[SENS_L_F] > 0) && (2 * out[SENS_L_F] > 5 * thr[SENS_L_F]);  /* >2.5x */
  bool rf = (thr[SENS_R_F] > 0) && (2 * out[SENS_R_F] > 5 * thr[SENS_R_F]);
  return lf || rf;
}



int Sensors_Lateral(void)
{
  bool L = Walls_Left();    /* a wall actually present (above threshold) this side */
  bool R = Walls_Right();
  int le = (cal.ref[SENS_L_LM] > 0) ? (out[SENS_L_LM] - (int)cal.ref[SENS_L_LM]) : 0;
  int re = (cal.ref[SENS_R_RM] > 0) ? (out[SENS_R_RM] - (int)cal.ref[SENS_R_RM]) : 0;
  /* >0: closer than centred to that wall. Fallbacks: both walls -> centre
     between them; one wall -> hold the calibrated distance to it; none -> 0 so
     the controller falls back to plain heading-hold. */
  if (L && R) return le - re;     /* >0: drifted toward LEFT wall (steer right) */
  if (L)      return le;          /* left wall only:  hold left distance        */
  if (R)      return -re;         /* right wall only: hold right distance       */
  return 0;                       /* no walls: no lateral info                  */
}
