/**
 ******************************************************************************
 * @file    console.c
 * @brief   Interactive debug console (RTT + USART1) - see console.h.
 ******************************************************************************
 */
#include "console.h"

#include "SEGGER_RTT.h"
#include "icm42688.h"
#include "as5047p.h"
#include "max17049.h"
#include "motor.h"
#include "buzzer.h"
#include "battery.h"
#include "control.h"
#include "settings.h"
#include "sensors.h"
#include "solver.h"
#include "mazestore.h"
#include "qspiflash.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

/* ============================ state ====================================== */

static const ConsoleCtx *C;          /* bound handles */

static AS5047P_Handle encL, encR;    /* the two magnetic encoders on SPI3 */
static bool qspi_ok;                 /* W25Q32JW probed OK at boot (OTA flash) */

/* Live-stream selector. Action commands (motor/vacuum/buzzer) are one-shot
   and leave the mode at IDLE. */
typedef enum { MODE_IDLE = 0, MODE_IMU, MODE_ENC, MODE_IR, MODE_BATT, MODE_WALLS } Mode;
static Mode     mode;
static uint32_t stream_interval;     /* ms between samples, live value */
static uint32_t mode_default_ms;     /* the active mode's natural interval */
static uint32_t rate_override_ms;    /* 0 = use per-mode default; else forced */
static uint32_t last_sample;

/* Line assembly (shared by both transports). */
static char    line[80];
static uint8_t line_len;
static int     last_term;            /* to swallow the LF in a CR/LF pair */

/* ====================== interrupt-driven USART1 ========================= */
/* Blocking HAL_UART_Transmit stalled the superloop on every line and the
   single RDR (FIFO off) dropped input bytes while we were busy, which made
   streams stutter and commands garble. Instead we run USART1 fully on
   interrupts with TX/RX ring buffers, so the main loop never waits and no
   received byte is lost. The handler is USART1_IRQHandler (this board's bridge
   UART is USART1); it overrides the weak default in the startup file. */
#define TXR_SZ 2048U                 /* power-of-two not required, plain modulo */
#define RXR_SZ 256U
static volatile uint8_t  txr[TXR_SZ];
static volatile uint16_t txr_head, txr_tail;
static volatile uint8_t  rxr[RXR_SZ];
static volatile uint16_t rxr_head, rxr_tail;
static USART_TypeDef    *UARTx;      /* C->huart->Instance, cached for the ISR */

void USART1_IRQHandler(void)
{
  uint32_t isr = UARTx->ISR;

  if (isr & USART_ISR_RXNE_RXFNE)              /* byte received */
  {
    uint8_t d = (uint8_t)(UARTx->RDR & 0xFFU);
    uint16_t nh = (uint16_t)((rxr_head + 1U) % RXR_SZ);
    if (nh != rxr_tail) { rxr[rxr_head] = d; rxr_head = nh; }  /* else drop */
  }

  if ((UARTx->CR1 & USART_CR1_TXEIE) && (isr & USART_ISR_TXE_TXFNF))
  {
    if (txr_tail != txr_head)                  /* more to send */
    {
      UARTx->TDR = txr[txr_tail];
      txr_tail = (uint16_t)((txr_tail + 1U) % TXR_SZ);
    }
    else
    {
      UARTx->CR1 &= ~USART_CR1_TXEIE;          /* queue empty: stop TXE ints */
    }
  }

  if (isr & USART_ISR_ORE) { UARTx->ICR = USART_ICR_ORECF; }   /* clear overrun */
}

/* Queue bytes for the ISR to clock out. Spins only if the (large) TX ring is
   full, which won't happen at debug rates. */
static void uart_tx(const char *s, int len)
{
  if (!UARTx) return;
  for (int i = 0; i < len; i++)
  {
    uint16_t nh = (uint16_t)((txr_head + 1U) % TXR_SZ);
    if (nh == txr_tail)
    {
      /* Ring full: we must wait for space - but the TX interrupt has to be
         enabled FIRST or nothing drains and we spin forever. (This is what hung
         the boot once the help menu grew past TXR_SZ in one write: the old code
         only kicked TXEIE after the loop, so a single transmit larger than the
         ring deadlocked here.) Enabling it before the wait makes any size safe. */
      UARTx->CR1 |= USART_CR1_TXEIE;
      while (nh == txr_tail) { /* let the TX ISR drain the ring */ }
    }
    txr[txr_head] = (uint8_t)s[i];
    txr_head = nh;
  }
  UARTx->CR1 |= USART_CR1_TXEIE;               /* kick the TX interrupt */
}

/* ====================== transport (out + in) ============================= */

/* When muted, the USART1 (ESP32 -> WiFi/BLE) link is silenced so a run can go
   fully offline with no comms traffic to jitter the main loop. RTT (the wired
   J-Link debug) is left alone - it's non-blocking and not the wireless path. */
static volatile bool console_muted;

/* Announce offline/online transitions to the ESP32 bridge over USART1 so it can
   show an offline indicator on its RGB LED (it can't otherwise tell muted from
   just-idle). uart_tx() writes the USART1 ring directly, bypassing the mute in
   out(), so the marker still reaches the bridge as we go silent. Sent only on a
   real state change to avoid spamming (the launcher calls this repeatedly). */
void Console_SetMuted(bool m)
{
  if (m == console_muted) return;
  if (m)                                        /* going OFFLINE: tell the ESP first */
    uart_tx("\r\n##OFFLINE##\r\n", (int)strlen("\r\n##OFFLINE##\r\n"));
  console_muted = m;
  if (!m)                                        /* back ONLINE: announce now traffic is allowed */
    uart_tx("\r\n##ONLINE##\r\n", (int)strlen("\r\n##ONLINE##\r\n"));
}
bool Console_IsMuted(void)    { return console_muted; }

/* Write the same bytes to RTT (non-blocking) and the USART1 TX ring. */
static void out(const char *s, int len)
{
  if (len <= 0) return;
  SEGGER_RTT_Write(0, s, (unsigned)len);
  if (!console_muted) uart_tx(s, len);
}

static void puts_(const char *s) { out(s, (int)strlen(s)); }

/* printf to both links. NOTE: stick to integer formats - newlib-nano's float
   printf is usually compiled out, so we print milli-units like the rest of the
   firmware does. */
static void cprintf(const char *fmt, ...)
{
  static char b[200];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  out(b, n < (int)sizeof b ? n : (int)sizeof b - 1);
}

/* Pull one byte from whichever link has data; -1 if both are empty. */
static int read_byte(void)
{
  if (SEGGER_RTT_HasKey())
  {
    return SEGGER_RTT_GetKey();
  }
  if (rxr_tail != rxr_head)                    /* USART1 RX ring (ISR-filled) */
  {
    uint8_t d = rxr[rxr_tail];
    rxr_tail = (uint16_t)((rxr_tail + 1U) % RXR_SZ);
    return (int)d;
  }
  return -1;
}

/* Accumulate bytes into `line`. Returns true once a full line is ready
   (line is NUL-terminated; may be empty if the user just pressed Enter). */
static bool poll_line(void)
{
  int c;
  while ((c = read_byte()) >= 0)
  {
    if (c == '\r' || c == '\n')
    {
      if (last_term == '\r' && c == '\n') { last_term = c; continue; } /* eat LF of CRLF */
      last_term = c;
      out("\r\n", 2);
      line[line_len] = '\0';
      line_len = 0;
      return true;
    }
    last_term = 0;

    if (c == 0x08 || c == 0x7F)                 /* backspace / delete */
    {
      if (line_len) { line_len--; puts_("\b \b"); }
    }
    else if (line_len < sizeof(line) - 1)
    {
      line[line_len++] = (char)c;
      char ch = (char)c;
      out(&ch, 1);                              /* echo so typing is visible */
    }
  }
  return false;
}

/* ====================== small helpers =================================== */

static int ci_eq(const char *a, const char *b)
{
  while (*a && *b)
  {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca += 32;
    if (cb >= 'A' && cb <= 'Z') cb += 32;
    if (ca != cb) return 0;
    a++; b++;
  }
  return *a == *b;
}

static void print_menu(void)
{
  puts_("\r\n=== NEMISIS DEBUG ===\r\n"
        "  1  encoder   live wheel encoders (SPI angle + quad count)\r\n"
        "  2  imu       live accel / gyro / temp\r\n"
        "  3  sensors   live 6x IR reflectance (raw, blocking read)\r\n"
        "  walls        live 6x IR, EMA-filtered (the wall-sensing stream)\r\n"
        "  irset [us] [n]  IR settle-us + oversampling (bench diagnostic)\r\n"
        "  irasync [on|off] IR sweep: non-blocking background vs blocking\r\n"
        "  irraw        one-shot raw ambient/lit ADC per sensor (0..4095)\r\n"
        "  ircal <dark|side|front>  capture wall-sensor calibration on the bench\r\n"
        "  ircsave / ircshow  store calibration to flash | view it\r\n"
        "  4  motor     motor 1|2|b <-100..100> [ms] | motor test | motor stop\r\n"
        "  5  vacuum    vacuum <0..100> | vacuum off\r\n"
        "  griptest [weight_g] [cps]  measure vacuum downforce via braking grip (GRIP log)\r\n"
        "  spintest [weight_g] [dps]  downforce via spin+lock skid (in place, no runway)\r\n"
        "  6  battery   live fuel gauge\r\n"
        "  7  buzzer    buzzer | buzzer <freqHz> <ms>\r\n"
        "  --- closed-loop drive / PID tuning ---\r\n"
        "  drive <cps>  start straight run (counts/s) | speed <cps> change it\r\n"
        "  move <mm> [cps]  straight run that auto-stops after <mm> (eases to stop)\r\n"
        "  turn <l|r|180|deg> [rate]  pivot in place by a fixed angle (gyro)\r\n"
        "  arc <l|r|deg> <cps> <r_mm>  smooth turn: curve forward through deg (bench)\r\n"
        "  flowturn <l|r|deg> <cps> <r_mm> <entry> <exit>  maze smooth turn (bench)\r\n"
        "  benchturn <n> <deg> [rate]  turn-accuracy test in place (no walls); logs TURNDIAG\r\n"
        "  advance <cells> [cps]  drive N cells non-stop; CELL,idx,L,R,F per cell\r\n"
        "  path [cps] <F<n>|L|R|U ...>  run a move sequence (e.g. path F2 L F1 R)\r\n"
        "  solve [cps]  autonomous flood-fill search to goal + back (needs ircal)\r\n"
        "  fast [cps]   speed run on the learned map (smooth turns on 'fastest')\r\n"
        "  mem [save|load|erase]  persist the learned map to flash (survives power cycle)\r\n"
        "  sim <seed>|stats <n> [miss%% false%% desync%%]  headless solve on a virtual maze\r\n"
        "  maze [n] / goal [center|x y]  set maze size + goal for 'solve'\r\n"
        "  tcost [n]    flood turn penalty (cells per 90 pivot; straighter routes)\r\n"
        "  start [sw|se|nw|ne|x y [NESW]]  set the corner you place the mouse in\r\n"
        "  dump         replay the last (offline) run's map + raw IR to the app\r\n"
        "  align [on|off] / acfg <skew> <dist>  front-wall squaring (pre-turn)\r\n"
        "  frontcal [mm]  learn centre->wall distance for post-stop recenter\r\n"
        "  cell [mm]    maze cell pitch for 'advance' (cell alone = show)\r\n"
        "  tcfg <rate> [accel]  pivot slew rate + accel (deg/s; tcfg alone = show)\r\n"
        "  gyroscale [v|cal <deg>]  gyro turn-scale (turn 360, then 'gyroscale cal <actual>')\r\n"
        "  follow <on|off>  IR corridor centring on forward runs (needs ircal side)\r\n"
        "  fcfg <gain> [damp]  centring gain + weave damping (fcfg alone = show)\r\n"
        "  pid <loop> <kp> <ki> <kd>   loop: vel|lvel|rvel|head  (pid = show)\r\n"
        "  arcff [l|r] <v>  arc feed-forward per turn direction (arcff = show)\r\n"
        "  arckp [gain]  arc heading-correction gain (gentle; arckp = show)\r\n"
        "  arckd [gain]  arc heading damping (D on filtered error; arckd = show)\r\n"
        "  arcgkd [gain]  arc gyro-rate damping, direct-to-motor (the buzz fix; arcgkd = show)\r\n"
        "  arctrans [mm]  smooth-turn omega ramp length (speed-invariant; arctrans = show)\r\n"
        "  dfilt [alpha]  heading D-term low-pass (lower = less motor buzz; dfilt = show)\r\n"
        "  ff <value>   velocity feedforward (duty per cps)\r\n"
        "  accel <cps/s> [decel]  setpoint ramp rates (accel alone = show)\r\n"
        "  vfilt <ms> [alpha]  velocity filter: window + EMA (smoothness vs lag)\r\n"
        "  autotune [cps]  auto-find vel kp/ki (relay test; applies result)\r\n"
        "  profile <search|fast|fastest>  switch tuning profile (profile = show)\r\n"
        "  save [profile]   store live tuning to flash | load [profile]  recall it\r\n"
        "  tlm <loop> [on|off]         stream TLM,t,setpoint,measured,output\r\n"
        "  rate <hz>    set live-stream rate (rate / rate default to show/reset)\r\n"
        "  stop / Enter  stop drive / end a live stream\r\n"
        "  help / debug  show this menu\r\n"
        "> ");
}

/* ====================== IR reflectance (moved from main.c) =============== */

#define IR_AVG_SAMPLES  16U

typedef struct
{
  const char        *name;
  GPIO_TypeDef      *em_port;
  uint16_t           em_pin;
  ADC_HandleTypeDef *adc;
  uint32_t           channel;
} IR_Sensor;

#define IR_COUNT 6
static IR_Sensor ir_sensors[IR_COUNT];   /* filled in Console_Init (needs C) */

static uint16_t ADC_ReadAvg(ADC_HandleTypeDef *adc, uint32_t channel, uint32_t n)
{
  ADC_ChannelConfTypeDef c = {0};
  c.Channel      = channel;
  c.Rank         = ADC_REGULAR_RANK_1;
  c.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  c.SingleDiff   = ADC_SINGLE_ENDED;
  c.OffsetNumber = ADC_OFFSET_NONE;
  c.Offset       = 0;
  HAL_ADC_ConfigChannel(adc, &c);

  uint32_t acc = 0;
  for (uint32_t i = 0; i < n; i++)
  {
    HAL_ADC_Start(adc);
    HAL_ADC_PollForConversion(adc, 10);
    acc += HAL_ADC_GetValue(adc);
    HAL_ADC_Stop(adc);
  }
  return (uint16_t)(acc / n);
}

static void IR_Read(const IR_Sensor *s, uint16_t *amb, uint16_t *lit)
{
  HAL_GPIO_WritePin(s->em_port, s->em_pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  *amb = ADC_ReadAvg(s->adc, s->channel, IR_AVG_SAMPLES);

  HAL_GPIO_WritePin(s->em_port, s->em_pin, GPIO_PIN_SET);
  HAL_Delay(1);
  *lit = ADC_ReadAvg(s->adc, s->channel, IR_AVG_SAMPLES);

  HAL_GPIO_WritePin(s->em_port, s->em_pin, GPIO_PIN_RESET);
}

/* ====================== per-mode sample emitters ======================== */

static void sample_imu(void)
{
  ICM42688_Raw raw;
  ICM42688_Data d;
  if (ICM42688_ReadData(C->hspi_imu, &raw))
  {
    ICM42688_Convert(&raw, &d);
    cprintf("acc[mg] X=%ld Y=%ld Z=%ld | gyr[mdps] X=%ld Y=%ld Z=%ld | T=%ld dC/10\r\n",
            (long)(d.accel_g[0] * 1000.0f), (long)(d.accel_g[1] * 1000.0f), (long)(d.accel_g[2] * 1000.0f),
            (long)(d.gyro_dps[0] * 1000.0f), (long)(d.gyro_dps[1] * 1000.0f), (long)(d.gyro_dps[2] * 1000.0f),
            (long)(d.temp_c * 10.0f));
  }
  else
  {
    puts_("IMU read failed\r\n");
  }
}

static void sample_enc(void)
{
  uint16_t rawL = AS5047P_ReadAngleRaw(&encL);
  uint16_t rawR = AS5047P_ReadAngleRaw(&encR);
  int16_t  cntL = (int16_t)__HAL_TIM_GET_COUNTER(C->htim_encL);
  int16_t  cntR = (int16_t)__HAL_TIM_GET_COUNTER(C->htim_encR);
  cprintf("ENC L raw=%5u (%ld.%02ld deg) cnt=%6d  |  R raw=%5u (%ld.%02ld deg) cnt=%6d\r\n",
          rawL,
          (long)((rawL * 360L) / AS5047P_RESOLUTION),
          (long)(((rawL * 360L * 100L) / AS5047P_RESOLUTION) % 100L),
          cntL,
          rawR,
          (long)((rawR * 360L) / AS5047P_RESOLUTION),
          (long)(((rawR * 360L * 100L) / AS5047P_RESOLUTION) % 100L),
          cntR);
}

static void sample_ir(void)
{
  int refl[IR_COUNT];
  for (int i = 0; i < IR_COUNT; i++)
  {
    uint16_t amb, lit;
    IR_Read(&ir_sensors[i], &amb, &lit);
    refl[i] = (int)lit - (int)amb;
  }
  cprintf("IR  L_LM=%5d  L_M=%5d  L_F=%5d  R_F=%5d  R_M=%5d  R_RM=%5d\r\n",
          refl[0], refl[1], refl[2], refl[3], refl[4], refl[5]);
}

/* Filtered wall-sensor values from lib/sensors (swept in the main loop at
   ~100 Hz). Same 6 channels as 'sensors' but EMA-smoothed and live - this is
   the stream the wall-detect / centring layers will build on. Machine-parseable
   "WALL," prefix for the app. */
static void sample_walls(void)
{
  const int *r = Sensors_Raw();
  cprintf("WALL,%d,%d,%d,%d,%d,%d\r\n", r[0], r[1], r[2], r[3], r[4], r[5]);
}

/* irset [settle_us] [samples] : IR acquisition timing (bench diagnostic).
   'irset' alone shows current. Bigger settle = the phototransistor has longer
   to respond (bigger signal) at the cost of a slower sweep. */
static void cmd_irset(int argc, char **argv)
{
  if (argc < 2)
  {
    uint32_t s, n;
    Sensors_GetTiming(&s, &n);
    cprintf("irset settle = %lu us  samples = %lu  (sweep ~ %lu us)\r\n",
            (unsigned long)s, (unsigned long)n,
            (unsigned long)(6UL * 2UL * (s + n * 17UL)));   /* rough: 2 phases/sensor */
    puts_("usage: irset <settle_us> [samples 1..64]   (e.g. 'irset 1000' or 'irset 500 16')\r\n");
    return;
  }
  uint32_t s = (uint32_t)atoi(argv[1]);
  uint32_t n = (argc >= 3) ? (uint32_t)atoi(argv[2]) : 0U;   /* 0 = leave samples */
  Sensors_SetTiming(s, n);
  Sensors_GetTiming(&s, &n);
  cprintf("irset settle = %lu us  samples = %lu\r\n", (unsigned long)s, (unsigned long)n);
}

/* irasync [on|off] : switch the IR sweep between the proven BLOCKING sweep (off,
   default) and the non-blocking background state machine (on) that frees the
   ~3.4 ms the blocking sweep spins. Bare 'irasync' shows the current mode. */
static void cmd_irasync(int argc, char **argv)
{
  if (argc >= 2)
  {
    if      (ci_eq(argv[1], "on"))  Sensors_SetAsync(true);
    else if (ci_eq(argv[1], "off")) Sensors_SetAsync(false);
    else { puts_("usage: irasync [on|off]\r\n"); return; }
  }
  cprintf("irasync = %s  (%s sweep)\r\n",
          Sensors_GetAsync() ? "on" : "off",
          Sensors_GetAsync() ? "non-blocking background" : "blocking");
}

/* irraw : one-shot dump of the RAW ambient + lit ADC levels per sensor (0..4095)
   before subtraction. amb near 4095 = receiver saturated; near 0 = starved;
   mid-range = well-biased and the emitter current is the lever. */
static void cmd_irraw(int argc, char **argv)
{
  (void)argc; (void)argv;
  int amb[SENSOR_COUNT], lit[SENSOR_COUNT];
  Sensors_GetRaw(amb, lit);
  static const char *nm[SENSOR_COUNT] = { "L_LM","L_M","L_F","R_F","R_M","R_RM" };
  puts_("IRRAW (0..4095): sensor  ambient  lit   diff\r\n");
  for (int i = 0; i < SENSOR_COUNT; i++)
    cprintf("  %-5s  %5d  %5d  %5d\r\n", nm[i], amb[i], lit[i], lit[i] - amb[i]);
}

static const char *const IR_NAMES[SENSOR_COUNT] =
  { "L_LM", "L_M", "L_F", "R_F", "R_M", "R_RM" };

/* ircshow : print the stored wall-sensor calibration (dark / ref / thresh) and
   which captures are in. */
static void cmd_ircshow(int argc, char **argv)
{
  (void)argc; (void)argv;
  int d[SENSOR_COUNT], r[SENSOR_COUNT], t[SENSOR_COUNT];
  uint16_t flags;
  Sensors_GetCal(d, r, t, &flags);
  cprintf("IRCAL  captured:[%s%s%s]   sensor  dark    ref  thresh\r\n",
          (flags & 0x1u) ? "dark " : "",
          (flags & 0x2u) ? "side " : "",
          (flags & 0x4u) ? "front" : "");
  for (int i = 0; i < SENSOR_COUNT; i++)
    cprintf("  %-5s  %5d  %5d  %5d\r\n", IR_NAMES[i], d[i], r[i], t[i]);
}

/* ircal <dark|side|front> : capture one calibration position, mouse held still.
   Order matters - run 'dark' first (the side/front thresholds are measured
   relative to it). 'ircsave' commits the result to flash. */
static void cmd_ircal(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("usage: ircal <dark|side|front>\r\n"
          "  dark   open cell, nothing near        (do this first)\r\n"
          "  side   centred between L and R walls   (sets L_LM/R_RM)\r\n"
          "  front  at the stop point facing a wall (sets L_F/R_F)\r\n"
          "  then 'ircsave' to store, 'ircshow' to view\r\n");
    return;
  }
  if      (ci_eq(argv[1], "dark"))  { Sensors_CalDark();  puts_("ircal: dark baseline captured\r\n"); }
  else if (ci_eq(argv[1], "side"))  { Sensors_CalSide();  puts_("ircal: side reference captured (L_LM/R_RM)\r\n"); }
  else if (ci_eq(argv[1], "front")) { Sensors_CalFront(); puts_("ircal: front reference captured (L_F/R_F)\r\n"); }
  else { puts_("ircal: arg is dark|side|front\r\n"); return; }
  cmd_ircshow(0, NULL);
}

/* ircsave : commit calibration to its own flash page (does NOT touch the tuning
   profiles). Refused mid-run - a flash erase stalls the control loop. */
static void cmd_ircsave(int argc, char **argv)
{
  (void)argc; (void)argv;
  if (Control_IsActive())
  {
    puts_("ircsave: stop the run first (flash erase stalls the loop)\r\n");
    return;
  }
  if (Sensors_CalSave()) puts_("ircsave: calibration written to flash\r\n");
  else                   puts_("ircsave: FLASH WRITE FAILED\r\n");
}

static void sample_batt(void)
{
  MAX17049_Status s;
  if (MAX17049_ReadStatus(C->hi2c_fuel, &s))
  {
    cprintf("BATT %ld mV (%ld mV/cell)  %ld.%02ld %%  %ld m%%/hr  [%s]\r\n",
            (long)(s.pack_v * 1000.0f), (long)(s.cell_v * 1000.0f),
            (long)s.soc, (long)((s.soc - (long)s.soc) * 100.0f),
            (long)(s.crate * 1000.0f),
            Battery_StateName(Battery_State()));
  }
  else
  {
    puts_("FUEL gauge: no ACK (check I2C3 / pull-ups / battery)\r\n");
  }
}

/* ====================== mode + action control =========================== */

static void stop_streams(void)
{
  if (mode != MODE_IDLE)
  {
    mode = MODE_IDLE;
    puts_("(stream stopped)\r\n");
  }
}

static void start_stream(Mode m, uint32_t interval_ms, const char *label)
{
  mode = m;
  mode_default_ms = interval_ms;
  stream_interval = rate_override_ms ? rate_override_ms : interval_ms;
  last_sample = HAL_GetTick() - stream_interval;  /* fire first sample immediately */
  cprintf("[%s] streaming @ %lu Hz - 'rate <hz>' to change, 'stop'/Enter to end\r\n",
          label, (unsigned long)(1000UL / (stream_interval ? stream_interval : 1U)));
}

/* rate <hz> | rate default | rate  (show) - live-stream sample rate */
static void cmd_rate(int argc, char **argv)
{
  if (argc < 2)
  {
    if (rate_override_ms)
      cprintf("rate = %lu Hz (forced)\r\n", (unsigned long)(1000UL / rate_override_ms));
    else
      puts_("rate = per-stream default (imu 20, enc 10, sensors 5, batt 1 Hz)\r\n");
    return;
  }
  if (ci_eq(argv[1], "default") || ci_eq(argv[1], "0"))
  {
    rate_override_ms = 0;
    if (mode != MODE_IDLE) stream_interval = mode_default_ms;
    puts_("rate -> per-stream default\r\n");
    return;
  }
  int hz = atoi(argv[1]);
  if (hz < 1)   hz = 1;
  if (hz > 500) hz = 500;            /* IR/battery reads can't really hit the top */
  rate_override_ms = (uint32_t)(1000 / hz);
  if (rate_override_ms == 0) rate_override_ms = 1;
  if (mode != MODE_IDLE)
  {
    stream_interval = rate_override_ms;
    last_sample = HAL_GetTick() - stream_interval;
  }
  cprintf("rate -> %d Hz (%lu ms/sample)\r\n", hz, (unsigned long)rate_override_ms);
}

/* motor 1|2|b <speed> [ms] | motor test | motor stop */
static void cmd_motor(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("usage: motor 1|2|b <-100..100> [ms] | motor test | motor stop\r\n");
    return;
  }
  if (ci_eq(argv[1], "stop"))
  {
    Motor_Coast(MOTOR_1);
    Motor_Coast(MOTOR_2);
    puts_("motors coasting\r\n");
    return;
  }
  /* Battery gate: 'stop' above is always allowed; anything that drives the
     motors is blocked while the pack is critical. */
  if (!Battery_AllowHighCurrent())
  {
    cprintf("BATTERY %s - motors disabled. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
    return;
  }

  if (ci_eq(argv[1], "test"))
  {
    puts_("MOTOR demo in 3 s - put the robot on a stand / clear the bench...\r\n");
    HAL_Delay(3000);
    Motor_DriveSequence(C->hadc4);
    puts_("motor demo done\r\n");
    return;
  }

  int speed = (argc >= 3) ? atoi(argv[2]) : 0;
  uint32_t ms = (argc >= 4) ? (uint32_t)atoi(argv[3]) : 800U;
  if (speed > 100) speed = 100;
  if (speed < -100) speed = -100;

  if (ci_eq(argv[1], "1"))
  {
    cprintf("motor 1 @ %d%% for %lu ms\r\n", speed, (unsigned long)ms);
    Motor_SoftPWM(MOTOR_1, speed, ms);
    Motor_Coast(MOTOR_1);
  }
  else if (ci_eq(argv[1], "2"))
  {
    cprintf("motor 2 @ %d%% for %lu ms\r\n", speed, (unsigned long)ms);
    Motor_SoftPWM(MOTOR_2, speed, ms);
    Motor_Coast(MOTOR_2);
  }
  else if (ci_eq(argv[1], "b"))
  {
    int dir = (speed > 0) - (speed < 0);
    int duty = speed < 0 ? -speed : speed;
    cprintf("both motors @ %d%% for %lu ms\r\n", speed, (unsigned long)ms);
    Motor_DriveBoth(dir, dir, duty, ms);
  }
  else
  {
    puts_("motor: pick 1, 2, b, test or stop\r\n");
    return;
  }
  cprintf("  sense: M1=%ld mA  M2=%ld mA\r\n",
          (long)Motor_ReadCurrent_mA(C->hadc4, MOTOR_1),
          (long)Motor_ReadCurrent_mA(C->hadc4, MOTOR_2));
}

/* Set the vacuum fan PWM to a 0..100 % duty. Returns the duty actually applied,
   or -1 if a >0 request was refused by the battery gate (the fan is the biggest
   current draw on the robot, so it's blocked while the pack is critical). Shared
   by the `vacuum` command and the grip test. */
static int fan_set_pct(int pct)
{
  return Battery_SetVacuum(pct);   /* battery owns the fan + gate (single source) */
}

/* vacuum <0..100> | vacuum off */
static void cmd_vacuum(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("usage: vacuum <0..100> | vacuum off\r\n");
    return;
  }
  int pct = ci_eq(argv[1], "off") ? 0 : atoi(argv[1]);
  if (fan_set_pct(pct) < 0)
  {
    cprintf("BATTERY %s - vacuum disabled. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
    return;
  }
  if (pct > 100) pct = 100;
  if (pct < 0)   pct = 0;
  cprintf("vacuum duty=%d%% (CCR=%lu)\r\n", pct,
          (unsigned long)(((uint32_t)pct * 65535U) / 100U));
}

/* ===================== vacuum grip / downforce test ======================
 * Measures how much extra grip the vacuum buys, using the robot itself. At each
 * fan duty it accelerates to a set speed, slams a hard (traction-limited) brake
 * and measures the stopping distance from the encoders. The peak no-slip
 * deceleration a = v^2 / (2*d) rises with downforce; its ratio to the fan-off
 * baseline gives downforce as a fraction of the robot's weight - the friction
 * coefficient cancels:  F_down = weight * (a_on / a_off - 1).
 *
 * It's pure orchestration of existing, tested primitives (drive to speed, hard
 * decel to stop, recenter back to the start so the mouse stays roughly in place)
 * pumped from the main loop - NO 1 kHz-ISR changes. Emits GRIP,* lines for the
 * Python app's grip logger. */
static const int GT_DUTY[] = { 0, 25, 50, 75, 100 };
#define GT_NLEV          ((int)(sizeof(GT_DUTY) / sizeof(GT_DUTY[0])))
#define GT_SPOOL_MS      1200U      /* fan spin-up before a brake test         */
#define GT_ACCEL_TMO_MS  3000U      /* give up reaching test speed             */
#define GT_BRAKE_TMO_MS  2000U      /* safety cap on a single brake            */
#define GT_TEST_ACCEL    150000.0f  /* brisk accel up to test speed (cps/s)    */
#define GT_TEST_DECEL    600000.0f  /* huge: traction, not the ramp, limits    */
#define GT_ATSPEED_FRAC  0.90f      /* "at speed" once measured >= frac*target */
#define GT_STOP_CPS      300.0f     /* wheel speed below which we're stopped   */

typedef enum { GT_OFF = 0, GT_SPOOL, GT_ACCEL, GT_BRAKE, GT_RETURN, GT_FINISH } GtState;
static GtState  gt_state;
static int      gt_idx;
static uint32_t gt_t0;
static int      gt_v0_cps;
static float    gt_weight_g;
static float    gt_brakepos_mm;             /* travel at brake start (mm)      */
static float    gt_amax[GT_NLEV];           /* peak decel per level (mm/s^2)   */
static float    gt_brake_mm[GT_NLEV];       /* braking distance per level (mm) */
static float    gt_saved_accel, gt_saved_decel;

/* Mean forward travel since the last Control_Start, in mm (encoders zero there). */
static float gt_travel_mm(void)
{
  int32_t pl = 0, pr = 0;
  Control_GetState(&pl, &pr, NULL, NULL, NULL);
  float cpm = Control_GetCountsPerMM();
  if (cpm <= 0.0f) cpm = 1.0f;
  return 0.5f * (float)(pl + pr) / cpm;
}

/* Restore the world and stop the test (called on abort/finish). */
static void griptest_abort(const char *why)
{
  if (gt_state == GT_OFF) return;
  fan_set_pct(0);
  Control_Stop();
  Control_SetAccel(gt_saved_accel, gt_saved_decel);
  gt_state = GT_OFF;
  if (why) cprintf("griptest: %s\r\n", why);
  puts_("GRIP,end,0\r\n> ");
}

/* griptest [weight_g] [cps] - kick off the fan-duty grip sweep. */
static void cmd_griptest(int argc, char **argv)
{
  if (gt_state != GT_OFF) { puts_("griptest: already running ('stop' to abort)\r\n"); return; }
  if (Solver_Active() || Control_NavActive() || Control_IsActive())
  { puts_("griptest: stop the current run first\r\n"); return; }
  if (!Battery_AllowHighCurrent())
  {
    cprintf("BATTERY %s - griptest needs the fan + motors. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
    return;
  }

  gt_weight_g = (argc >= 2) ? (float)atof(argv[1]) : 0.0f;
  gt_v0_cps   = (argc >= 3) ? atoi(argv[2]) : 12000;
  if (gt_v0_cps < 3000) gt_v0_cps = 3000;

  Control_GetAccel(&gt_saved_accel, &gt_saved_decel);
  Control_SetAccel(GT_TEST_ACCEL, GT_TEST_DECEL);   /* brisk up, brake at grip limit */

  for (int i = 0; i < GT_NLEV; i++) { gt_amax[i] = 0.0f; gt_brake_mm[i] = 0.0f; }
  gt_idx   = 0;
  gt_state = GT_SPOOL;
  gt_t0    = HAL_GetTick();
  fan_set_pct(GT_DUTY[0]);

  float cpm = Control_GetCountsPerMM();
  cprintf("GRIP,begin,%ld,%d,brake\r\n", (long)gt_weight_g, gt_v0_cps);
  cprintf("griptest: %d fan levels, v0=%d cps (~%ld mm/s). Clear ~0.5 m of runway ahead;\r\n"
          "          it returns to start between levels. 'stop' aborts.\r\n",
          GT_NLEV, gt_v0_cps, (long)(cpm > 0.0f ? (float)gt_v0_cps / cpm : 0.0f));
}

/* Pump the grip test one step; call every main-loop pass (no-op when off). */
static void griptest_task(void)
{
  if (gt_state == GT_OFF) return;
  uint32_t now = HAL_GetTick();

  switch (gt_state)
  {
  case GT_SPOOL:
    if (now - gt_t0 < GT_SPOOL_MS) break;             /* let the fan spin up      */
    if (!Control_Start(gt_v0_cps))                    /* zeros encoders -> base 0 */
    { griptest_abort("drive blocked (battery?)"); return; }
    gt_state = GT_ACCEL; gt_t0 = now;
    break;

  case GT_ACCEL:
    if (Control_GetWheelSpeedCps() >= GT_ATSPEED_FRAC * (float)gt_v0_cps)
    {
      gt_brakepos_mm = gt_travel_mm();
      Control_SetSpeed(0);                             /* hard brake (decel huge)  */
      gt_state = GT_BRAKE; gt_t0 = now;
    }
    else if (now - gt_t0 > GT_ACCEL_TMO_MS)
    {
      cprintf("GRIP,%d,%d,0,0\r\n", GT_DUTY[gt_idx], gt_v0_cps);
      puts_("griptest: never reached test speed (runway/grip?) - skipping level\r\n");
      Control_Stop();
      gt_state = GT_RETURN; gt_t0 = now;
    }
    break;

  case GT_BRAKE:
  {
    bool stopped = (Control_GetWheelSpeedCps() < GT_STOP_CPS) || !Control_IsActive();
    if (!stopped && (now - gt_t0) <= GT_BRAKE_TMO_MS) break;

    float stoppos = gt_travel_mm();
    float bmm = stoppos - gt_brakepos_mm;
    if (bmm < 1.0f) bmm = 1.0f;                        /* guard the divide         */
    float cpm    = Control_GetCountsPerMM(); if (cpm <= 0.0f) cpm = 1.0f;
    float v0mmps = (float)gt_v0_cps / cpm;
    float amax   = (v0mmps * v0mmps) / (2.0f * bmm);   /* mm/s^2                    */
    gt_brake_mm[gt_idx] = bmm;
    gt_amax[gt_idx]     = amax;
    Control_Stop();

    cprintf("GRIP,%d,%d,%ld,%ld\r\n", GT_DUTY[gt_idx], gt_v0_cps, (long)bmm, (long)amax);
    cprintf("grip: fan=%3d%%  brake=%ld mm  a=%ld mm/s^2\r\n",
            GT_DUTY[gt_idx], (long)bmm, (long)amax);

    if (stoppos > 5.0f) Control_StartRecenter((int)stoppos);   /* back to start   */
    gt_state = GT_RETURN; gt_t0 = now;
    break;
  }

  case GT_RETURN:
    if (Control_RecenterActive()) break;
    Control_PopRecenterDone();                         /* drain the done flag      */
    if (++gt_idx >= GT_NLEV) { gt_state = GT_FINISH; break; }
    fan_set_pct(GT_DUTY[gt_idx]);
    gt_state = GT_SPOOL; gt_t0 = now;
    break;

  case GT_FINISH:
  {
    fan_set_pct(0);
    Control_SetAccel(gt_saved_accel, gt_saved_decel);
    float a0 = gt_amax[0];
    puts_("GRIP,summary\r\n--- grip test: downforce vs fan-off ---\r\n");
    for (int i = 0; i < GT_NLEV; i++)
    {
      float ratio  = (a0 > 1.0f) ? gt_amax[i] / a0 : 0.0f;
      long  rx1000 = (long)(ratio * 1000.0f);
      long  df_pct = (long)((ratio - 1.0f) * 100.0f);
      long  df_g   = (gt_weight_g > 0.0f) ? (long)(gt_weight_g * (ratio - 1.0f)) : 0;
      cprintf("GRIP,res,%d,%ld,%ld,%ld,%ld\r\n",
              GT_DUTY[i], (long)gt_amax[i], rx1000, df_g, df_pct);
      cprintf("  fan %3d%%: a=%4ld mm/s^2  x%ld.%03ld baseline  %+ld%% grip  %+ld gf\r\n",
              GT_DUTY[i], (long)gt_amax[i], rx1000 / 1000, rx1000 % 1000, df_pct, df_g);
    }
    if (gt_weight_g <= 0.0f)
      puts_("  (pass a weight in grams for absolute downforce: griptest <weight_g> [cps])\r\n");
    cprintf("GRIP,end,%ld\r\n> ", (long)a0);
    gt_state = GT_OFF;
    break;
  }

  default: break;
  }
}

/* ================== vacuum SPIN grip / downforce test ====================
 * The braking test showed straight-line stopping is motor/drivetrain-limited, not
 * grip-limited, so downforce didn't register. This measures GRIP directly: spin the
 * mouse up in place, LOCK the wheels, and time the tyre skid that bleeds the spin
 * off. Skid decel rises with downforce and (being a locked skid) is immune to the
 * motor drag that dominated braking. In place -> no runway. Reuses the GRIP,*
 * protocol (+ a 'spin' mode tag on GRIP,begin) so the app's panel/logger just work.
 * F_down = weight * (decel_on / decel_off - 1). */
typedef enum { ST_OFF = 0, ST_SPOOL, ST_WAIT, ST_FINISH } StState;
static StState  st_state;
static int      st_idx;
static uint32_t st_t0;
static int      st_spin_dps;
static float    st_weight_g;
static float    st_decel[GT_NLEV];

static void spintest_abort(const char *why)
{
  if (st_state == ST_OFF) return;
  fan_set_pct(0);
  Control_Stop();
  st_state = ST_OFF;
  if (why) cprintf("spintest: %s\r\n", why);
  puts_("GRIP,end,0\r\n> ");
}

/* spintest [weight_g] [spin_dps] - fan-swept spin-and-lock grip sweep. */
static void cmd_spintest(int argc, char **argv)
{
  if (st_state != ST_OFF || gt_state != GT_OFF)
  { puts_("spintest: a grip test is already running ('stop' to abort)\r\n"); return; }
  if (Solver_Active() || Control_NavActive() || Control_IsActive())
  { puts_("spintest: stop the current run first\r\n"); return; }
  if (!Battery_AllowHighCurrent())
  {
    cprintf("BATTERY %s - spintest needs the fan + motors. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
    return;
  }

  st_weight_g = (argc >= 2) ? (float)atof(argv[1]) : 0.0f;
  st_spin_dps = (argc >= 3) ? atoi(argv[2]) : 400;
  if (st_spin_dps < 120) st_spin_dps = 400;

  for (int i = 0; i < GT_NLEV; i++) st_decel[i] = 0.0f;
  st_idx   = 0;
  st_state = ST_SPOOL;
  st_t0    = HAL_GetTick();
  fan_set_pct(GT_DUTY[0]);
  cprintf("GRIP,begin,%ld,%d,spin\r\n", (long)st_weight_g, st_spin_dps);
  cprintf("spintest: spin up to %d dps then lock + skid, %d fan levels. Spins in "
          "place - no runway needed. 'stop' aborts.\r\n", st_spin_dps, GT_NLEV);
}

/* Pump the spin-grip test one step; call every main-loop pass (no-op when off). */
static void spintest_task(void)
{
  if (st_state == ST_OFF) return;
  uint32_t now = HAL_GetTick();

  switch (st_state)
  {
  case ST_SPOOL:
    if (now - st_t0 < GT_SPOOL_MS) break;
    if (!Control_StartSpinGrip(st_spin_dps))
    { spintest_abort("spin blocked (battery?)"); return; }
    st_state = ST_WAIT;
    break;

  case ST_WAIT:
  {
    bool ok; float decel = 0.0f, omega0 = 0.0f;
    if (!Control_PopSpinGripResult(&ok, &decel, &omega0)) break;
    if (ok) st_decel[st_idx] = decel;
    cprintf("GRIP,%d,%d,%ld,%ld\r\n", GT_DUTY[st_idx], st_spin_dps,
            (long)omega0, (long)decel);
    cprintf("spin: fan=%3d%%  w0=%ld dps  decel=%ld dps/s%s\r\n",
            GT_DUTY[st_idx], (long)omega0, (long)decel,
            ok ? "" : "  (never reached rate)");
    if (++st_idx >= GT_NLEV) { st_state = ST_FINISH; break; }
    fan_set_pct(GT_DUTY[st_idx]);
    st_state = ST_SPOOL; st_t0 = now;
    break;
  }

  case ST_FINISH:
  {
    fan_set_pct(0);
    float d0 = st_decel[0];
    puts_("GRIP,summary\r\n--- spin grip test: downforce vs fan-off ---\r\n");
    for (int i = 0; i < GT_NLEV; i++)
    {
      float ratio  = (d0 > 1.0f) ? st_decel[i] / d0 : 0.0f;
      long  rx1000 = (long)(ratio * 1000.0f);
      long  df_pct = (long)((ratio - 1.0f) * 100.0f);
      long  df_g   = (st_weight_g > 0.0f) ? (long)(st_weight_g * (ratio - 1.0f)) : 0;
      cprintf("GRIP,res,%d,%ld,%ld,%ld,%ld\r\n",
              GT_DUTY[i], (long)st_decel[i], rx1000, df_g, df_pct);
      cprintf("  fan %3d%%: decel=%5ld dps/s  x%ld.%03ld base  %+ld%% grip  %+ld gf\r\n",
              GT_DUTY[i], (long)st_decel[i], rx1000 / 1000, rx1000 % 1000, df_pct, df_g);
    }
    if (st_weight_g <= 0.0f)
      puts_("  (pass a weight for absolute downforce: spintest <weight_g> [dps])\r\n");
    cprintf("GRIP,end,%ld\r\n> ", (long)d0);
    st_state = ST_OFF;
    break;
  }

  default: break;
  }
}

/* buzzer | buzzer <freqHz> <ms> */
static void cmd_buzzer(int argc, char **argv)
{
  if (argc >= 3)
  {
    uint16_t f = (uint16_t)atoi(argv[1]);
    uint16_t ms = (uint16_t)atoi(argv[2]);
    cprintf("buzzer %u Hz for %u ms\r\n", f, ms);
    Buzzer_Tone(f, ms);
  }
  else
  {
    puts_("buzzer chime\r\n");
    Buzzer_StartupSound();
  }
}

/* ====================== closed-loop control / PID tuning ================ */

/* loop name -> enum. Returns -1 if unknown. */
static int parse_loop(const char *s)
{
  if (ci_eq(s, "vel")  || ci_eq(s, "v"))  return CTRL_LOOP_VEL;
  if (ci_eq(s, "lvel") || ci_eq(s, "l"))  return CTRL_LOOP_LVEL;
  if (ci_eq(s, "rvel") || ci_eq(s, "r"))  return CTRL_LOOP_RVEL;
  if (ci_eq(s, "head") || ci_eq(s, "h"))  return CTRL_LOOP_HEAD;
  return -1;
}

/* Print one loop's gains. Floats are shown x1000 (newlib-nano has no %f). */
static void print_gains(const char *name, CtrlLoop loop)
{
  float kp, ki, kd;
  Control_GetGains(loop, &kp, &ki, &kd);
  cprintf("  %-4s kp=%ld ki=%ld kd=%ld  (x1000)\r\n",
          name, (long)(kp * 1000.0f), (long)(ki * 1000.0f), (long)(kd * 1000.0f));
}

/* drive <cps> | go <cps> : start a closed-loop straight run */
static void cmd_drive(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("usage: drive <counts/s>   (e.g. 'drive 4000')\r\n");
    return;
  }
  int cps = atoi(argv[1]);
  if (Control_Start(cps))
    cprintf("DRIVE: closed loop @ %d counts/s - 'stop' to end\r\n", cps);
  else
    cprintf("BATTERY %s - drive disabled. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
}

/* move <mm> [cps] : closed-loop straight run that auto-stops after <mm> */
static void cmd_move(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("usage: move <mm> [cps]   (e.g. 'move 1000' or 'move 1500 24000')\r\n");
    return;
  }
  int mm  = atoi(argv[1]);
  int cps = (argc >= 3) ? atoi(argv[2]) : 8000;
  if (mm <= 0) { puts_("move: distance must be > 0 mm\r\n"); return; }
  if (Control_StartMove(mm, cps))
    cprintf("MOVE: %d mm @ %d cps - eases to a stop at the end ('stop' to abort)\r\n",
            mm, cps);
  else
    cprintf("BATTERY %s - move disabled. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
}

/* advance <cells> [cps] : drive N maze cells in one continuous run - does NOT
   stop between cells. Prints "CELL,<idx>,<L>,<R>,<F>" (walls 0/1) each time it
   reaches a cell centre, and eases to a stop at the last cell (or earlier and
   centred if a front wall is seen). Needs 'ircal side/front' for the walls. */
static void cmd_advance(int argc, char **argv)
{
  if (argc < 2)
  {
    cprintf("usage: advance <cells> [cps]   (cell pitch %ld mm, set with 'cell')\r\n",
            (long)Control_GetCellMM());
    return;
  }
  int cells = atoi(argv[1]);
  int cps   = (argc >= 3) ? atoi(argv[2]) : 8000;
  if (cells <= 0) { puts_("advance: cells must be > 0\r\n"); return; }
  if (Control_StartAdvance(cells, cps))
    cprintf("ADVANCE: %d cells @ %d cps - flows through cells, stops at the end "
            "(or at a front wall)\r\n", cells, cps);
  else
    cprintf("BATTERY %s - advance disabled. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
}

/* cell [mm] : show or set the maze cell pitch used by 'advance'. */
static void cmd_cell(int argc, char **argv)
{
  if (argc >= 2)
  {
    int mm = atoi(argv[1]);
    if (mm <= 0) { puts_("cell: pitch must be > 0 mm\r\n"); return; }
    Control_SetCellMM((float)mm);
  }
  cprintf("cell pitch = %ld mm\r\n", (long)Control_GetCellMM());
}

/* path [cps] <tokens...> : queue + run a sequence of moves back-to-back.
   Tokens: F<n> = advance n cells, L = turn left 90, R = turn right 90,
   U = turn 180. Optional leading number sets the advance speed (cps).
   e.g. 'path F2 L F1 R F3'  or  'path 10000 F4 L F2'. Runs straights non-stop;
   stops to pivot at each turn. 'stop' aborts. The maze solver will drive the
   same queue (Control_NavAdd*) later; this is the manual front-end. */
static void cmd_path(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("usage: path [cps] <F<n>|L|R|U ...>   e.g. 'path F2 L F1 R F3'\r\n");
    return;
  }

  int i = 1;
  int cps = 8000;                          /* default mapping speed              */
  if (argv[1][0] >= '0' && argv[1][0] <= '9') { cps = atoi(argv[1]); i = 2; }

  Control_NavAbort();                       /* drop any running/old sequence      */
  Control_NavReset();

  int steps = 0;
  for (; i < argc; i++)
  {
    char c = argv[i][0];
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');   /* upper-case          */
    bool ok = true;
    switch (c)
    {
      case 'F': { int n = atoi(&argv[i][1]); if (n <= 0) n = 1;
                  ok = Control_NavAddAdvance(n, cps); break; }
      case 'L':   ok = Control_NavAddTurn( 90, 0); break;   /* left  = +90        */
      case 'R':   ok = Control_NavAddTurn(-90, 0); break;   /* right = -90        */
      case 'U':
      case 'B':   ok = Control_NavAddTurn(180, 0); break;   /* u-turn / about     */
      default:
        cprintf("path: bad token '%s' (use F<n>, L, R, U)\r\n", argv[i]);
        Control_NavReset();
        return;
    }
    if (!ok) { puts_("path: too many steps (max 48)\r\n"); Control_NavReset(); return; }
    steps++;
  }

  if (steps == 0) { puts_("path: no steps\r\n"); return; }

  if (Control_NavStart())
    cprintf("PATH: %d steps @ %d cps - running ('stop' to abort)\r\n", steps, cps);
  else
    cprintf("BATTERY %s - path disabled. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
}

/* maze [n] : show or set the maze edge length (cells, 2..16). Re-centres the
   goal and clears the discovered map. The app sends this to match your maze. */
static void cmd_maze(int argc, char **argv)
{
  if (argc >= 2)
  {
    int n = atoi(argv[1]);
    Solver_SetSize(n);
  }
  int sz, gx[4], gy[4], gc;
  Solver_GetConfig(&sz, gx, gy, &gc);
  int sx, sy, sf;
  Solver_GetStart(&sx, &sy, &sf);
  cprintf("MAZE,%d\r\n", sz);
  cprintf("START,%d,%d,%d\r\n", sx, sy, sf);
  cprintf("maze = %dx%d, start (%d,%d) facing %c\r\n", sz, sz, sx, sy, "NESW"[sf]);
}

/* start [sw|se|nw|ne | <x> <y> [N|E|S|W]] : show or set the start corner - the
   cell + heading you physically place the mouse in. Corner shortcuts resolve
   against the current maze size (bottom corners face N, top corners face S).
   The explicit form takes a cell and an optional facing letter (default N). The
   solver floods back to THIS cell, and the app highlights it. */
static int parse_facing_arg(const char *s)
{
  if (ci_eq(s, "n")) return 0;
  if (ci_eq(s, "e")) return 1;
  if (ci_eq(s, "s")) return 2;
  if (ci_eq(s, "w")) return 3;
  int v = atoi(s);
  return ((v % 4) + 4) % 4;
}

static void cmd_start(int argc, char **argv)
{
  int sz, gx[4], gy[4], gc;
  Solver_GetConfig(&sz, gx, gy, &gc);
  if (argc >= 2)
  {
    int x, y, f;
    if      (ci_eq(argv[1], "sw")) { x = 0;      y = 0;      f = 0; }  /* face N */
    else if (ci_eq(argv[1], "se")) { x = sz - 1; y = 0;      f = 0; }  /* face N */
    else if (ci_eq(argv[1], "nw")) { x = 0;      y = sz - 1; f = 2; }  /* face S */
    else if (ci_eq(argv[1], "ne")) { x = sz - 1; y = sz - 1; f = 2; }  /* face S */
    else if (argc >= 3)
    {
      x = atoi(argv[1]);
      y = atoi(argv[2]);
      f = (argc >= 4) ? parse_facing_arg(argv[3]) : 0;
    }
    else
    {
      puts_("usage: start [sw|se|nw|ne | <x> <y> [N|E|S|W]]\r\n");
      return;
    }
    Solver_SetStart(x, y, f);
  }
  int x, y, f;
  Solver_GetStart(&x, &y, &f);
  cprintf("START,%d,%d,%d\r\n", x, y, f);
  cprintf("start = (%d,%d) facing %c  (place the mouse here, nose pointing %c)\r\n",
          x, y, "NESW"[f], "NESW"[f]);
}

/* goal [center | <x> <y>] : show or set the goal. No args shows the current
   goal; 'center' sets the middle 2x2; 'x y' sets a single custom cell. */
static void cmd_goal(int argc, char **argv)
{
  if (argc >= 2)
  {
    if (ci_eq(argv[1], "center") || ci_eq(argv[1], "centre"))
      Solver_SetGoalCenter();
    else if (argc >= 3)
      Solver_SetGoalCell(atoi(argv[1]), atoi(argv[2]));
    else
      { puts_("usage: goal [center | <x> <y>]\r\n"); return; }
  }
  int sz, gx[4], gy[4], gc;
  Solver_GetConfig(&sz, gx, gy, &gc);
  cprintf("GOAL,%d", gc);
  for (int i = 0; i < gc; i++) cprintf(",%d,%d", gx[i], gy[i]);
  cprintf("\r\n");
}

/* solve [cps] : autonomous flood-fill search run. Explores to the goal then
   floods back to the start cell, stop-and-go one cell at a time. Needs 'ircal
   side' + 'ircal front' done first (else every cell reads open). Reports
   "SOLVE,x,y,facing,flood,phase,wN,wE,wS,wW" per cell and "SOLVE: done" at the
   end. 'stop' aborts. Put the mouse in the start corner facing into the maze. */
static void cmd_solve(int argc, char **argv)
{
  int cps = (argc >= 2) ? atoi(argv[1]) : 0;     /* 0 -> solver default */

  if (Solver_Start(cps))
  {
    int sz, gx[4], gy[4], gc;
    Solver_GetConfig(&sz, gx, gy, &gc);
    int sx, sy, sf;
    Solver_GetStart(&sx, &sy, &sf);
    cprintf("MAZE,%d\r\n", sz);                  /* let the app size its grid */
    cprintf("START,%d,%d,%d\r\n", sx, sy, sf);   /* highlight the start corner */
    cprintf("GOAL,%d", gc);
    for (int i = 0; i < gc; i++) cprintf(",%d,%d", gx[i], gy[i]);
    cprintf("\r\n");
    puts_("SOLVE: search run started - exploring to goal ('stop' to abort)\r\n");
  }
  else
  {
    uint16_t flags = 0;
    Sensors_GetCal(NULL, NULL, NULL, &flags);
    if (!(flags & 0x2) || !(flags & 0x4))
      puts_("solve: wall sensors not calibrated - run 'ircal side' and "
            "'ircal front' first\r\n");
    else if (Solver_Active())
      puts_("solve: already running ('stop' to abort)\r\n");
    else
      cprintf("BATTERY %s - solve disabled. Charge the pack.\r\n",
              Battery_StateName(Battery_State()));
  }
}

/* fast [cps] : FAST (speed) run - replays the least-cost path on the map the last
   search LEARNED, start->goal->back, at higher speed. Needs a completed search
   (came home) first. On the 'fastest' profile the corners are taken as SMOOTH
   (non-stop) turns; otherwise it pivots. 'stop' aborts. */
/* ===== FAST-RUN BRAKING (console `fast`) — EDIT THESE for 18000 cps ==========
 * brake distance = v^2 / (2*decel), so LOWER decel = brake EARLIER. The flow now
 * brakes PREDICTIVELY into each turn cell; these set how hard.                   */
#define FAST_ACCEL      190000.0f   /* spin-up rate (cps/s)                        */
#define FAST_DECEL      105000.0f   /* brake rate (cps/s) - lower = brake earlier  */
#define FAST_BRAKE_REV    6000.0f   /* active reverse-brake force (cps); 0 = off   */
/* ============================================================================= */

static void cmd_fast(int argc, char **argv)
{
  int cps = (argc >= 2) ? atoi(argv[1]) : 0;     /* 0 -> solver default */
  Control_SetAccel(FAST_ACCEL, FAST_DECEL);      /* bake the fast-run braking in  */
  Control_SetBrakeReverse(FAST_BRAKE_REV);
  if (Solver_StartFast(cps))
  {
    cprintf("FAST: speed run started (%s turns) - 'stop' to abort\r\n",
            Control_GetTurnMode() ? "smooth" : "pivot");
  }
  else if (!Solver_FastReady())
    puts_("fast: no learned map yet - run 'solve' (search) to the goal and back first\r\n");
  else if (Solver_Active())
    puts_("fast: already running ('stop' to abort)\r\n");
  else
    cprintf("BATTERY %s or goal unreachable - fast disabled.\r\n",
            Battery_StateName(Battery_State()));
}

/* mem [save|load|erase] : persistent maze / speed-run memory (internal flash).
   'mem' alone shows status + the boot reset cause (to verify the button-erases /
   power-keeps rule). A completed search auto-saves on its own; these are manual /
   test controls. 'load' pulls the saved map back into the solver and arms a fast
   run (equivalent to the offline long-press). */
static void cmd_mem(int argc, char **argv)
{
  if (argc >= 2 && ci_eq(argv[1], "save"))
  {
    if (Solver_Active()) { puts_("mem: stop the run first\r\n"); return; }
    MazeMap m; Solver_CaptureMap(&m);
    cprintf("mem: save %s\r\n", MazeStore_Save(&m) ? "ok" : "FAILED");
    return;
  }
  if (argc >= 2 && ci_eq(argv[1], "load"))
  {
    if (Solver_Active()) { puts_("mem: stop the run first\r\n"); return; }
    MazeMap m;
    if (MazeStore_Valid() && MazeStore_Get(&m) && Solver_RestoreMap(&m))
      puts_("mem: loaded saved map - fast run armed ('fast' or long-press)\r\n");
    else
      puts_("mem: no valid saved map\r\n");
    return;
  }
  if (argc >= 2 && ci_eq(argv[1], "erase"))
  {
    if (Solver_Active()) { puts_("mem: stop the run first\r\n"); return; }
    MazeStore_Erase();
    puts_("mem: erased\r\n");
    return;
  }
  cprintf("MEM: saved run %s | boot reset = %s (CSR 0x%08lX)%s\r\n",
          MazeStore_Valid() ? "PRESENT" : "none",
          MazeStore_LastResetName(),
          (unsigned long)MazeStore_LastResetFlags(),
          MazeStore_WasWarmReset() ? " [warm -> erased]" : "");
  puts_("usage: mem [save|load|erase]\r\n");
}

/* sim ... : on-MCU headless simulation of the solver against a GENERATED virtual
   maze (no motors/IR). Validates the 16x16 brain + per-decision CPU time, and -
   with fault injection - stresses the phantom-wall recovery the way real sensor
   errors / position desyncs would. Set the size first ('maze 16'); a single run
   leaves a run-log so 'dump' draws the solved map in the app.
     sim <seed> [miss%] [false%] [desync%]
     sim stats <count> [miss%] [false%] [desync%]   (runs seeds 1..count) */
static int sim_hexval(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static void cmd_sim(int argc, char **argv)
{
  if (Solver_Active()) { puts_("sim: a real run is active ('stop' first)\r\n"); return; }
  if (argc < 2)
  {
    puts_("usage: sim <seed> [miss%] [false%] [desync%]   (generated maze)\r\n"
          "       sim stats <count> [miss%] [false%] [desync%]\r\n"
          "       sim clear | sim row <y> <hex> | sim run [miss%] [false%] [desync%]\r\n"
          "       sim mode <0|1|2>  0=explore  1=shortest  2=confirm-optimal (learns the best path)\r\n"
          "  set size first, e.g. 'maze 16'. faults are percentages (0 = perfect).\r\n"
          "  row hex = one nibble per cell (N1 E2 S4 W8), x=0..n-1; used by the app.\r\n");
    return;
  }

  int sz; Solver_GetConfig(&sz, NULL, NULL, NULL);

  if (ci_eq(argv[1], "mode"))
  {
    if (argc >= 3) Solver_SetSearchMode(atoi(argv[2]));
    int sm = Solver_GetSearchMode();
    const char *nm = (sm == 2) ? "confirm-optimal" : (sm == 1) ? "shortest-path" : "explore";
    cprintf("sim: search mode = %d (%s)\r\n", sm, nm);
    return;
  }

  /* --- maze upload (app pushes an MMS maze in row by row), then 'sim run' ---- */
  if (ci_eq(argv[1], "clear"))
  {
    Solver_SimClear();
    puts_("sim: maze cleared to borders\r\n");
    return;
  }
  if (ci_eq(argv[1], "row"))
  {
    if (argc < 4) { puts_("usage: sim row <y> <hex>\r\n"); return; }
    int y = atoi(argv[2]);
    const char *h = argv[3];
    int x = 0;
    for (; h[x] && x < sz; x++)
    {
      int v = sim_hexval(h[x]);
      if (v < 0) { cprintf("sim row: bad hex at col %d\r\n", x); return; }
      Solver_SimSetCell(x, y, v);
    }
    cprintf("sim: row %d loaded (%d cells)\r\n", y, x);
    return;
  }
  if (ci_eq(argv[1], "run"))
  {
    int miss = (argc >= 3) ? atoi(argv[2]) : 0;
    int fls  = (argc >= 4) ? atoi(argv[3]) : 0;
    int des  = (argc >= 5) ? atoi(argv[4]) : 0;
    Solver_SimSetFaults(miss, fls, des);
    SimResult r;
    Solver_RunSim(&r);
    int pct = r.cells ? (100 * r.explored) / r.cells : 0;
    cprintf("SIM: n=%d loaded mode=%d faults(%d/%d/%d) -> %s  learned=%d optimal=%d "
            "(journey=%d) explored=%d%% recov=%d us(worst=%d avg=%d)\r\n",
            sz, r.mode, miss, fls, des, r.reached ? "REACHED" : "FAILED",
            r.learned, r.optimal, r.path_len, pct, r.recoveries, r.worst_us, r.avg_us);
    puts_("sim: run-log ready - use 'dump' to view the simulated map in the app\r\n");
    return;
  }

  if (ci_eq(argv[1], "stats"))
  {
    int count = (argc >= 3) ? atoi(argv[2]) : 0;
    if (count < 1) { puts_("sim stats: need a count >= 1\r\n"); return; }
    int miss = (argc >= 4) ? atoi(argv[3]) : 0;
    int fls  = (argc >= 5) ? atoi(argv[4]) : 0;
    int des  = (argc >= 6) ? atoi(argv[5]) : 0;
    Solver_SimSetFaults(miss, fls, des);

    int  reached = 0, worst_us = 0, optimal_hits = 0;
    long sum_learn = 0, sum_opt = 0, sum_expl = 0, sum_recov = 0;
    int  learn_n = 0;
    for (int s = 1; s <= count; s++)
    {
      Solver_SimGenMaze((uint32_t)s);
      SimResult r;
      if (Solver_RunSim(&r)) reached++;
      if (r.worst_us > worst_us) worst_us = r.worst_us;
      if (r.learned >= 0 && r.optimal >= 0)
      {
        sum_learn += r.learned; sum_opt += r.optimal; learn_n++;
        if (r.learned == r.optimal) optimal_hits++;   /* found the true quickest path */
      }
      sum_expl  += r.explored;
      sum_recov += r.recoveries;
    }
    cprintf("SIM STATS: n=%d count=%d mode=%d faults(%d/%d/%d) -> reached=%d/%d  "
            "learned avg=%ld optimal avg=%ld  found-optimal=%d/%d  explored avg=%ld/%d  "
            "recov avg=%ld worst_us=%d\r\n",
            sz, count, Solver_GetSearchMode(), miss, fls, des, reached, count,
            learn_n ? sum_learn / learn_n : -1, learn_n ? sum_opt / learn_n : -1,
            optimal_hits, count, sum_expl / count, sz * sz, sum_recov / count, worst_us);
    return;
  }

  uint32_t seed = (uint32_t)atoi(argv[1]);
  int miss = (argc >= 3) ? atoi(argv[2]) : 0;
  int fls  = (argc >= 4) ? atoi(argv[3]) : 0;
  int des  = (argc >= 5) ? atoi(argv[4]) : 0;
  Solver_SimSetFaults(miss, fls, des);
  Solver_SimGenMaze(seed);

  SimResult r;
  Solver_RunSim(&r);
  int pct = r.cells ? (100 * r.explored) / r.cells : 0;
  cprintf("SIM: n=%d seed=%lu mode=%d faults(%d/%d/%d) -> %s  learned=%d optimal=%d "
          "(journey=%d) explored=%d%% recov=%d us(worst=%d avg=%d)\r\n",
          sz, (unsigned long)seed, r.mode, miss, fls, des, r.reached ? "REACHED" : "FAILED",
          r.learned, r.optimal, r.path_len, pct, r.recoveries, r.worst_us, r.avg_us);
  puts_("sim: run-log ready - use 'dump' to view the simulated map in the app\r\n");
}

/* align [on|off] : front-wall squaring. No arg = run ONE alignment now (bench
   test - face the mouse at a wall first). 'on'/'off' = whether the solver
   squares before each pivot. Needs 'ircal front'. Tune gains with 'acfg'. */
static void cmd_align(int argc, char **argv)
{
  if (argc >= 2)
  {
    if (ci_eq(argv[1], "on") || ci_eq(argv[1], "off"))
    {
      Solver_SetAlign(ci_eq(argv[1], "on"));
      cprintf("align in solve: %s\r\n", Solver_GetAlign() ? "ON" : "off");
      return;
    }
    puts_("usage: align [on|off]   (no arg = run one alignment now)\r\n");
    return;
  }
  if (Control_StartFrontAlign())
    puts_("ALIGN: squaring to the wall ahead...\r\n");
  else
    puts_("align: needs 'ircal front' (and a wall ahead), battery OK\r\n");
}

/* acfg [<kp_skew> <kp_dist>] : front-align gains (cps per IR count). kp_skew
   rotates to square (null L_F-R_F); kp_dist creeps to the cal stop distance.
   x1000 in the echo (no float printf). 'acfg' alone shows them. */
static void cmd_acfg(int argc, char **argv)
{
  if (argc >= 3)
    Control_SetAlign((float)atof(argv[1]), (float)atof(argv[2]));
  float ks, kd; Control_GetAlign(&ks, &kd);
  cprintf("acfg skew = %ld  dist = %ld  (x1000 cps/count)\r\n",
          (long)(ks * 1000.0f), (long)(kd * 1000.0f));
}

/* tcost [n] : turn penalty the flood planner adds per 90 pivot, in cell units.
   Higher => the solver plans straighter routes with fewer, bunched turns (faster,
   and the shape a later diagonal run wants); 0 => plan by cell count only (the old
   behaviour). 'tcost' alone shows it. Clamped 0..50. */
static void cmd_tcost(int argc, char **argv)
{
  if (argc >= 2) Solver_SetTurnCost(atoi(argv[1]));
  cprintf("tcost = %d  (flood turn penalty, cells per 90 pivot)\r\n",
          Solver_GetTurnCost());
}

/* frontcal [cps|mm] : learn the centre->front-wall-stop distance used to recentre
   after a front-stop. Place the mouse at a cell CENTRE facing a wall, then run
   'frontcal' - it drives to the wall and reverses back, storing the distance.
   'frontcal <mm>' sets it by hand; 'frontcal' alone (no wall) shows it. */
static void cmd_frontcal(int argc, char **argv)
{
  if (argc >= 2)
  {
    int v = atoi(argv[1]);
    if (v >= 10 && v <= 140)        /* a plausible distance -> set it directly   */
    {
      Solver_SetFrontWallMM(v);
      cprintf("frontcal = %d mm (centre->wall stop)\r\n", Solver_GetFrontWallMM());
      return;
    }
    /* otherwise treat the arg as a drive speed for the auto-measure */
  }
  if (Solver_StartFrontCal(argc >= 2 ? atoi(argv[1]) : 0))
    puts_("FRONTCAL: place at a cell CENTRE facing a wall - driving to it...\r\n");
  else
    cprintf("frontcal: needs 'ircal front' + battery OK + idle (now %d mm)\r\n",
            Solver_GetFrontWallMM());
}

/* turn <l|r|180|deg> [rate] : pivot in place by a fixed angle (gyro-closed).
   l/left = +90, r/right = -90, 180/around = 180; a bare number turns that many
   signed degrees. Optional [rate] overrides the slew cap (deg/s) for this turn. */
static void cmd_turn(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("usage: turn <l|r|180|deg> [rate dps]   (e.g. 'turn l', 'turn 180', 'turn -45')\r\n");
    return;
  }

  int deg;
  if      (ci_eq(argv[1], "l") || ci_eq(argv[1], "left"))             deg =  90;
  else if (ci_eq(argv[1], "r") || ci_eq(argv[1], "right"))           deg = -90;
  else if (ci_eq(argv[1], "180") || ci_eq(argv[1], "u") ||
           ci_eq(argv[1], "around") || ci_eq(argv[1], "about"))      deg = 180;
  else                                                                deg = atoi(argv[1]);

  if (deg == 0) { puts_("turn: angle must be non-zero\r\n"); return; }
  int rate = (argc >= 3) ? atoi(argv[2]) : 0;     /* 0 = use the tuned default */

  if (Control_StartPivot(deg, rate))
    cprintf("TURN: pivot %d deg - eases to a stop ('stop' to abort)\r\n", deg);
  else
    cprintf("BATTERY %s - turn disabled. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
}

/* arc <l|r|deg> <cps> <radius_mm> [exit_mm] : SMOOTH (arc) turn - drive forward at
   <cps> while curving through <deg> along a circle of <radius_mm>. exit_mm 0 (or
   omitted) stops when the arc completes (bench geometry); >0 drives on that far in
   the new heading WITHOUT stopping (smooth turn into a corridor). e.g.
   'arc l 12000 110' or 'arc l 12000 110 250'. */
static void cmd_arc(int argc, char **argv)
{
  if (argc < 4)
  {
    puts_("usage: arc <l|r|deg> <cps> <radius_mm> [exit_mm]   (e.g. 'arc l 12000 110 250')\r\n");
    return;
  }

  int deg;
  if      (ci_eq(argv[1], "l") || ci_eq(argv[1], "left"))  deg =  90;
  else if (ci_eq(argv[1], "r") || ci_eq(argv[1], "right")) deg = -90;
  else                                                     deg = atoi(argv[1]);
  if (deg == 0) { puts_("arc: angle must be non-zero\r\n"); return; }

  int cps    = atoi(argv[2]);
  int radius = atoi(argv[3]);
  int exit_mm = (argc >= 5) ? atoi(argv[4]) : 0;
  if (cps <= 0 || radius <= 0) { puts_("arc: cps and radius must be > 0\r\n"); return; }

  if (Control_StartSmoothTurn(deg, cps, radius, exit_mm))
    cprintf("ARC: %d deg @ %d cps, R=%d mm, exit=%d mm ('stop' to abort)\r\n",
            deg, cps, radius, exit_mm);
  else
    cprintf("BATTERY %s or bad arg - arc disabled.\r\n",
            Battery_StateName(Battery_State()));
}

/* ==========================================================================
 *  benchturn - turn-accuracy bench test (NO maze / walls needed, spins in place)
 *
 *  Runs N pivots of <deg> back-to-back in place and logs, per turn, what was
 *  commanded vs what the gyro actually reached - the data behind the fast-run
 *  under-rotation diagnosis. Each turn emits:
 *      TURNDIAG,idx,cmd_ddeg,ach_ddeg,resid_ddeg,peak_ddps,ms,reason
 *  (angles/rate x10; resid = ach-cmd, <0 = under-rotated; reason 0 clean /
 *   1 snappy early-release / 2 timeout), then at the end:
 *      TURNSUM,turns,cum_resid_ddeg,mean_resid_ddeg
 *  cum_resid is the total drift after all N turns (perfect = 0).
 * ======================================================================== */
static bool    bench_active;
static int     bench_remaining, bench_deg, bench_cps, bench_idx;
static int32_t bench_cum_ddeg;

static void bench_task(void)
{
  if (Control_IsActive()) return;                 /* wait for the pivot to finish  */
  int32_t deg;
  if (!Control_PopTurnDone(&deg)) return;         /* not done yet                  */

  int32_t cmd, ach, pk, ms, rs, pl, pr;
  Control_GetLastTurnDiag(&cmd, &ach, &pk, &ms, &rs, &pl, &pr);
  int32_t resid = ach - cmd;                      /* deci-deg, <0 = under-rotated  */
  bench_cum_ddeg += resid;
  cprintf("TURNDIAG,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
          bench_idx, (long)cmd, (long)ach, (long)resid, (long)pk, (long)ms, (long)rs,
          (long)pl, (long)pr);
  bench_idx++;

  if (--bench_remaining <= 0)
  {
    cprintf("TURNSUM,%d,%ld,%ld\r\n> ", bench_idx, (long)bench_cum_ddeg,
            (long)(bench_idx ? bench_cum_ddeg / bench_idx : 0));
    bench_active = false;
    return;
  }
  Control_StartPivot(bench_deg, bench_cps);       /* next turn                     */
}

static void cmd_benchturn(int argc, char **argv)
{
  if (argc < 3)
  {
    puts_("usage: benchturn <n> <deg> [rate dps]   (spins in place, no walls needed)\r\n"
          "  e.g. 'benchturn 4 90' - 4x 90 pivots; logs TURNDIAG per turn + TURNSUM\r\n");
    return;
  }
  int n    = atoi(argv[1]);
  int deg  = atoi(argv[2]);
  int rate = (argc >= 4) ? atoi(argv[3]) : 0;
  if (n < 1 || n > 64) { puts_("benchturn: n must be 1..64\r\n"); return; }
  if (deg == 0)        { puts_("benchturn: deg must be non-zero\r\n"); return; }

  bench_deg = deg; bench_cps = rate; bench_remaining = n;
  bench_idx = 0; bench_cum_ddeg = 0;
  if (!Control_StartPivot(deg, rate))
  {
    cprintf("BATTERY %s - benchturn disabled.\r\n", Battery_StateName(Battery_State()));
    return;
  }
  bench_active = true;
  cprintf("BENCHTURN: %d x %d deg in place. cols: "
          "TURNDIAG,idx,cmd,ach,resid,peak,ms,reason,posL,posR "
          "(deg/dps x10; posL/R = per-wheel counts, |L|vs|R| = wheel balance)\r\n", n, deg);
}

/* flowturn <l|r|deg> <cps> <radius_mm> <entry_mm> <exit_mm> : bench-test a MAZE
   smooth turn - lead-in straight, arc, lead-out straight, then stop centred. This is
   the geometry the fast-run smooth profile uses; tune radius/entry/exit so the mouse
   lands centred on the next corridor (for a 180 mm cell, R=90 entry=90 exit=90). */
static void cmd_flowturn(int argc, char **argv)
{
  if (argc < 6)
  {
    puts_("usage: flowturn <l|r|deg> <cps> <radius_mm> <entry_mm> <exit_mm>\r\n"
          "       (e.g. 'flowturn l 8000 90 90 90')\r\n");
    return;
  }
  int deg;
  if      (ci_eq(argv[1], "l") || ci_eq(argv[1], "left"))  deg =  90;
  else if (ci_eq(argv[1], "r") || ci_eq(argv[1], "right")) deg = -90;
  else                                                     deg = atoi(argv[1]);
  if (deg == 0) { puts_("flowturn: angle must be non-zero\r\n"); return; }

  int cps    = atoi(argv[2]);
  int radius = atoi(argv[3]);
  int entry  = atoi(argv[4]);
  int exit_m = atoi(argv[5]);
  if (cps <= 0 || radius <= 0 || exit_m <= 0)
  { puts_("flowturn: cps, radius, exit must be > 0\r\n"); return; }

  if (Control_StartFlowTurn(deg, cps, radius, entry, exit_m, false))
    cprintf("FLOWTURN: %d deg @ %d cps, R=%d, entry=%d, exit=%d mm ('stop' to abort)\r\n",
            deg, cps, radius, entry, exit_m);
  else
    cprintf("BATTERY %s or bad arg - flowturn disabled.\r\n",
            Battery_StateName(Battery_State()));
}

/* arcff [l|r] <gain> : smooth-arc curvature feed-forward (cps trim per deg/s of
   turn rate), PER TURN DIRECTION (a real mouse turns asymmetrically). 'arcff <v>'
   sets BOTH; 'arcff l <v>'/'arcff r <v>' set one; bare = show. Raise until the arc
   heading tracks without lag (safe now: completion is distance-gated). */
static void cmd_arcff(int argc, char **argv)
{
  if (argc >= 3 && (ci_eq(argv[1], "l") || ci_eq(argv[1], "left")))
    Control_SetArcFFLeft((float)atof(argv[2]));
  else if (argc >= 3 && (ci_eq(argv[1], "r") || ci_eq(argv[1], "right")))
    Control_SetArcFFRight((float)atof(argv[2]));
  else if (argc >= 2)
    Control_SetArcFF((float)atof(argv[1]));            /* set both directions      */
  cprintf("arcffl = %d (x10)\r\n", (int)(Control_GetArcFFLeft()  * 10.0f));
  cprintf("arcffr = %d (x10)  (cps trim per deg/s; per turn direction)\r\n",
          (int)(Control_GetArcFFRight() * 10.0f));
}

/* arckp [gain] : arc heading-correction gain (cps trim per deg of error). Gentle
   on purpose - arcff carries the turn, this only nulls drift. Bare = show. */
static void cmd_arckp(int argc, char **argv)
{
  if (argc >= 2) Control_SetArcKp((float)atof(argv[1]));
  cprintf("arckp = %d  (cps trim per deg of heading error)\r\n",
          (int)Control_GetArcKp());
}

/* arckd [gain] : arc heading DAMPING (derivative on the filtered error). Raise to
   remove turn overshoot/hunt; filtered, so no buzz comes back. Bare = show. */
static void cmd_arckd(int argc, char **argv)
{
  if (argc >= 2) Control_SetArcKd((float)atof(argv[1]));
  cprintf("arckd = %d  (damping; cps trim per deg/s of error change)\r\n",
          (int)Control_GetArcKd());
}

/* arcgkd [gain] : arc gyro-rate damping (motor duty per deg/s of rate error) -
   the fast, direct-to-motor damping that fixes the buzz/limit-cycle. Bare = show. */
static void cmd_arcgkd(int argc, char **argv)
{
  if (argc >= 2) Control_SetArcGyroKd((float)atof(argv[1]));
  cprintf("arcgkd = %d (x10)  (duty per deg/s of rotation-rate error)\r\n",
          (int)(Control_GetArcGyroKd() * 10.0f));
}

/* arctrans [mm] : smooth-turn transition length - the floor distance each omega
   ramp spans entering/leaving the curve (alpha is derived from it per start, so
   the turn's drawn path is the same at every speed). Bare = show. */
static void cmd_arctrans(int argc, char **argv)
{
  if (argc >= 2) Control_SetArcTrans((float)atof(argv[1]));
  cprintf("arctrans = %d mm  (omega ramp floor length; 2..80)\r\n",
          (int)Control_GetArcTrans());
}

/* dfilt [alpha] : heading-PID derivative low-pass (EMA coeff 0..1). Lower =
   smoother / less motor buzz but laggier damping; 1.0 = raw gyro D. Bare = show. */
static void cmd_dfilt(int argc, char **argv)
{
  if (argc >= 2) Control_SetHeadDFilt((float)atof(argv[1]));
  cprintf("dfilt = %ld (x1000)  (heading D-term LPF; lower = smoother)\r\n",
          (long)(Control_GetHeadDFilt() * 1000.0f));
}

/* tcfg <rate dps> [accel dps/s^2] : pivot slew rate + accel. 'tcfg' alone shows
   them. The heading PID (tune via 'pid head') does the rotating; these shape how
   fast the setpoint leads it into / out of the turn. */
static void cmd_tcfg(int argc, char **argv)
{
  if (argc < 2)
  {
    float r, a; Control_GetTurn(&r, &a);
    cprintf("tcfg rate = %ld  accel = %ld  (deg/s, deg/s^2)\r\n", (long)r, (long)a);
    puts_("usage: tcfg <rate dps> [accel dps/s^2]   (e.g. 'tcfg 300' or 'tcfg 300 1500')\r\n");
    return;
  }
  float r = strtof(argv[1], NULL);
  float a = (argc >= 3) ? strtof(argv[2], NULL) : -1.0f;   /* sentinel: keep accel */
  Control_SetTurn(r, a);
  Control_GetTurn(&r, &a);
  cprintf("tcfg rate = %ld  accel = %ld  (deg/s, deg/s^2)\r\n", (long)r, (long)a);
}

/* gyroscale [<lsb/dps> | cal <actual_deg>] : gyro turn-scale calibration so a
   commanded pivot lands at the true angle. 'gyroscale' alone shows it.
   Flow: 'turn 360', read how far it ACTUALLY rotated, then 'gyroscale cal 352'.
   'gyroscale 16.4' sets the value directly. Once happy, bake it into the
   GYRO_LSB_PER_DPS default in control.c so it survives a power cycle. */
static void cmd_gyroscale(int argc, char **argv)
{
  if (argc < 2)
  {
    cprintf("gyroscale = %ld (x1000 LSB/dps)\r\n",
            (long)(Control_GetGyroScale() * 1000.0f));
    puts_("usage: gyroscale <lsb/dps> | gyroscale cal <actual_deg>\r\n");
    puts_("  cal: after 'turn 360', enter the angle it really turned\r\n");
    return;
  }

  if (ci_eq(argv[1], "cal"))
  {
    if (argc < 3) { puts_("usage: gyroscale cal <actual_deg>\r\n"); return; }
    float actual = strtof(argv[2], NULL);
    if (Control_TurnCalFromActual(actual))
      cprintf("gyroscale cal OK -> %ld (x1000 LSB/dps). Re-test 'turn 360'.\r\n",
              (long)(Control_GetGyroScale() * 1000.0f));
    else
      puts_("gyroscale cal: need a prior 'turn' and a plausible actual angle\r\n");
    return;
  }

  Control_SetGyroScale(strtof(argv[1], NULL));
  cprintf("gyroscale -> %ld (x1000 LSB/dps)\r\n",
          (long)(Control_GetGyroScale() * 1000.0f));
}

/* dump : replay the last offline run's buffered log (SOLVE,/SIR, lines, same
   format as a live solve) so the app rebuilds the discovered map + raw IR and
   overlays them on the maze you drew. Run it after reconnecting. RAM-only, so
   dump before powering the robot off. */
/* The dump is a big burst (hundreds of lines). The STM32 TX ring spin-waits so
   it never drops, but the ESP32->WiFi bridge has a finite buffer and DROPS/garbles
   bytes if fed a sustained full-rate stream (that's what truncated a run to look
   "stuck" and merged TRACE lines). So we pace it: a short delay every few lines
   gives the bridge time to flush. Slower fetch, intact data. */
#define DUMP_GAP_MS     2U      /* pause this long ...                            */
#define DUMP_GAP_EVERY  3U      /* ... every this many lines                      */
/* Run-index -> human label for the launcher session (Search/Fast/Fastest). Any
   higher index (shouldn't happen) prints "run". */
static const char *dump_run_label(int r)
{
  switch (r) { case 0: return "search"; case 1: return "fast"; case 2: return "fastest"; }
  return "run";
}

static void cmd_dump(int argc, char **argv)
{
  (void)argc; (void)argv;
  unsigned emitted = 0;
  #define DUMP_PACE() do { if (++emitted % DUMP_GAP_EVERY == 0) HAL_Delay(DUMP_GAP_MS); } while (0)

  int n     = Solver_LogCount();
  int tn    = Solver_TraceCount();
  int maxr  = Solver_MaxRun();                 /* -1 when nothing was logged        */
  int nruns = (maxr < 0) ? 1 : (maxr + 1);     /* a standalone run is just run 0    */

  /* DUMP,begin,<nruns> - the number of RUNS in this fetch (was the log count; the
     per-run counts now ride on each DUMP,run marker below). */
  cprintf("DUMP,begin,%d\r\n", nruns);

  for (int r = 0; r < nruns; r++)
  {
    /* Count this run's entries so the app knows what to expect (and can flag a
       truncated transfer per run). Entries are untagged (run 0) on a standalone
       fetch, so run 0 sweeps them all. */
    int nl = 0, nt = 0;
    for (int i = 0; i < n;  i++) if (Solver_GetLogRun(i)   == r || (maxr < 0)) nl++;
    for (int i = 0; i < tn; i++) if (Solver_GetTraceRun(i) == r || (maxr < 0)) nt++;

    /* The DUMP,run / DUMP,trace markers are STRUCTURAL - if the ESP bridge drops one
       the app can't split the runs (a dropped DUMP,run merges that run into the
       previous file, seen 2026-07-01). They land right after the previous run's
       trace burst when the bridge buffer is most stressed, so give it a moment to
       flush before AND after each marker. */
    /* A lone standalone run tagged 0 is "search" by index, but a long-press
       from-memory replay is a FAST run - relabel it so the fetched file is correct. */
    const char *lbl = (r == 0 && nruns == 1 && Solver_StandaloneFastLogged())
                      ? "fast" : dump_run_label(r);
    HAL_Delay(DUMP_GAP_MS);
    cprintf("DUMP,run,%d,%s,%d,%d\r\n", r, lbl, nl, nt);
    HAL_Delay(DUMP_GAP_MS);

    for (int i = 0; i < n; i++)
    {
      if (!(Solver_GetLogRun(i) == r || (maxr < 0))) continue;
      int x, y, facing, fld, ph, wb, ir[6], tlf, trf;
      if (!Solver_GetLogEntry(i, &x, &y, &facing, &fld, &ph, &wb, ir, &tlf, &trf))
        break;
      cprintf("SOLVE,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", x, y, facing, fld, ph,
              (wb & 0x1) ? 1 : 0, (wb & 0x2) ? 1 : 0,      /* wallN, wallE */
              (wb & 0x4) ? 1 : 0, (wb & 0x8) ? 1 : 0);     /* wallS, wallW */
      DUMP_PACE();
      cprintf("SIR,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", x, y,
              ir[0], ir[1], ir[2], ir[3], ir[4], ir[5], tlf, trf);
      DUMP_PACE();
    }

    /* Continuous trace: encoders, heading, gyro-Z, accel-Z, 6 IR, cell, flood, vbat. */
    HAL_Delay(DUMP_GAP_MS);
    cprintf("DUMP,trace,%d\r\n", nt);
    HAL_Delay(DUMP_GAP_MS);
    for (int i = 0; i < tn; i++)
    {
      if (!(Solver_GetTraceRun(i) == r || (maxr < 0))) continue;
      uint32_t t; int32_t pl, pr; int hd, gz, az, s[6], x, y, fld, vb;
      if (!Solver_GetTraceEntry(i, &t, &pl, &pr, &hd, &gz, &az, s, &x, &y, &fld, &vb))
        break;
      cprintf("TRACE,%lu,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
              (unsigned long)t, (long)pl, (long)pr, hd, gz, az,
              s[0], s[1], s[2], s[3], s[4], s[5], x, y, fld, vb);
      DUMP_PACE();
    }
  }
  cprintf("DUMP,end\r\n> ");
  #undef DUMP_PACE
}

/* follow [on|off] : IR corridor centring on forward runs. 'follow' alone shows
   state + gain. Needs 'ircal side' done; with no walls it's plain heading-hold. */
static void cmd_follow(int argc, char **argv)
{
  if (argc < 2)
  {
    cprintf("follow %s   gain = %ld (x1000 deg/count)\r\n",
            Control_GetFollow() ? "ON" : "off",
            (long)(Control_GetCenterGain() * 1000.0f));
    puts_("usage: follow <on|off>   (corridor centring; tune gain with 'fcfg')\r\n");
    return;
  }
  if (ci_eq(argv[1], "on"))
  {
    Control_SetFollow(true);
    puts_("follow ON - corridor centring active on forward runs\r\n");
  }
  else if (ci_eq(argv[1], "off"))
  {
    Control_SetFollow(false);
    puts_("follow off - plain heading-hold\r\n");
  }
  else puts_("usage: follow <on|off>\r\n");
}

/* fcfg <gain> : centring gain (heading-offset deg per IR lateral count). 'fcfg'
   alone shows it. Start ~0.05 and raise until it centres crisply without weaving;
   a negative value flips the steer direction if it heads INTO the wall. */
static void cmd_fcfg(int argc, char **argv)
{
  if (argc < 2)
  {
    cprintf("fcfg gain = %ld  damp = %ld (x1000; deg per lateral count[/s])\r\n",
            (long)(Control_GetCenterGain() * 1000.0f),
            (long)(Control_GetCenterDamp() * 1000.0f));
    puts_("usage: fcfg <gain> [damp]   (e.g. 'fcfg 0.05 0.0015'; gain<0 flips; "
          "raise damp to kill the weave, 0 = plain P)\r\n");
    return;
  }
  Control_SetCenterGain(strtof(argv[1], NULL));
  if (argc >= 3) Control_SetCenterDamp(strtof(argv[2], NULL));
  cprintf("fcfg gain = %ld  damp = %ld (x1000)\r\n",
          (long)(Control_GetCenterGain() * 1000.0f),
          (long)(Control_GetCenterDamp() * 1000.0f));
}

/* speed <cps> : change the target of an in-progress run */
static void cmd_speed(int argc, char **argv)
{
  if (argc < 2) { puts_("usage: speed <counts/s>\r\n"); return; }
  if (!Control_IsActive()) { puts_("speed: not running (use 'drive <cps>')\r\n"); return; }
  int cps = atoi(argv[1]);
  Control_SetSpeed(cps);
  cprintf("speed -> %d counts/s\r\n", cps);
}

/* pid <loop> <kp> <ki> <kd> | pid  (show all) */
static void cmd_pid(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("PID gains:\r\n");
    print_gains("lvel", CTRL_LOOP_LVEL);
    print_gains("rvel", CTRL_LOOP_RVEL);
    print_gains("head", CTRL_LOOP_HEAD);
    cprintf("  ff   vel feedforward = %ld (x1000 duty/cps)\r\n",
            (long)(Control_GetVelFF() * 1000.0f));
    puts_("usage: pid <vel|lvel|rvel|head> <kp> <ki> <kd>\r\n");
    return;
  }
  int loop = parse_loop(argv[1]);
  if (loop < 0) { puts_("pid: loop is vel|lvel|rvel|head\r\n"); return; }
  if (argc < 5) { puts_("usage: pid <loop> <kp> <ki> <kd>\r\n"); return; }

  float kp = strtof(argv[2], NULL);
  float ki = strtof(argv[3], NULL);
  float kd = strtof(argv[4], NULL);
  Control_SetGains((CtrlLoop)loop, kp, ki, kd);
  cprintf("pid %s set:\r\n", argv[1]);
  print_gains(argv[1], (loop == CTRL_LOOP_VEL) ? CTRL_LOOP_LVEL : (CtrlLoop)loop);
}

/* ff <value> : velocity feedforward gain (duty per cps); 'ff' alone shows it */
static void cmd_ff(int argc, char **argv)
{
  if (argc < 2)
  {
    cprintf("vel feedforward = %ld (x1000 duty/cps)\r\n",
            (long)(Control_GetVelFF() * 1000.0f));
    puts_("usage: ff <value>   (e.g. 'ff 0.2')\r\n");
    return;
  }
  float k = strtof(argv[1], NULL);
  Control_SetVelFF(k);
  cprintf("vel ff -> %ld (x1000 duty/cps)\r\n", (long)(k * 1000.0f));
}

/* autotune [cps] : relay (Astrom-Hagglund) auto-tune of the velocity loop.
   Robot wobbles forward while it measures; gains apply + echo when it finishes. */
static void cmd_autotune(int argc, char **argv)
{
  if (Control_AutotuneActive()) { puts_("autotune: already running ('stop' to abort)\r\n"); return; }
  int cps = (argc >= 2) ? atoi(argv[1]) : 3000;
  if (Control_AutotuneStart(cps))
    cprintf("AUTOTUNE: relay test @ %d cps - keep it on the floor with space ahead.\r\n"
            "  measuring the wobble... 'stop' to abort; gains apply when done.\r\n", cps);
  else
    cprintf("BATTERY %s - autotune disabled. Charge the pack.\r\n",
            Battery_StateName(Battery_State()));
}

/* accel <cps/s> [decel] : setpoint ramp rates. 'accel' alone shows them.
   One value sets both; two sets accel and decel separately. */
static void cmd_accel(int argc, char **argv)
{
  if (argc < 2)
  {
    float a, d;
    Control_GetAccel(&a, &d);
    cprintf("accel = %ld  decel = %ld  (cps/s)\r\n", (long)a, (long)d);
    puts_("usage: accel <cps/s> [decel]   (e.g. 'accel 40000' or 'accel 60000 30000')\r\n");
    return;
  }
  float a = strtof(argv[1], NULL);
  float d = (argc >= 3) ? strtof(argv[2], NULL) : a;
  Control_SetAccel(a, d);
  Control_GetAccel(&a, &d);
  cprintf("accel = %ld  decel = %ld  (cps/s)\r\n", (long)a, (long)d);
}

/* brake <cps> : active reverse-brake floor for distance moves. While a move is
   braking and still rolling, the setpoint is driven this far below zero so the
   motors reverse-torque for a real stop instead of coasting. 0 disables it. The
   fast run arms its own value; this command is for bench tuning. */
static void cmd_brake(int argc, char **argv)
{
  if (argc < 2)
  {
    cprintf("brake = %ld cps  (0 = off)\r\n", (long)Control_GetBrakeReverse());
    puts_("usage: brake <cps>   (reverse-torque brake floor; e.g. 'brake 6000', 'brake 0')\r\n");
    return;
  }
  Control_SetBrakeReverse(strtof(argv[1], NULL));
  cprintf("brake = %ld cps\r\n", (long)Control_GetBrakeReverse());
}

/* vfilt <window_ms> [alpha] : velocity-feedback filter (smoothness vs lag).
   'vfilt' alone shows the current settings. */
static void cmd_vfilt(int argc, char **argv)
{
  if (argc < 2)
  {
    int w; float a;
    Control_GetVelFilter(&w, &a);
    cprintf("vfilt window = %d ms  alpha = %ld (x1000)\r\n", w, (long)(a * 1000.0f));
    puts_("usage: vfilt <window 2..50 ms> [alpha 0..1]   (smaller alpha = smoother)\r\n");
    return;
  }
  int   w = atoi(argv[1]);
  float a = -1.0f;                       /* sentinel: keep current alpha */
  if (argc >= 3) a = strtof(argv[2], NULL);
  Control_SetVelFilter(w, a);
  int wc; float ac;
  Control_GetVelFilter(&wc, &ac);
  cprintf("vfilt window = %d ms  alpha = %ld (x1000)\r\n", wc, (long)(ac * 1000.0f));
}

/* Echo the full live tuning set in the same formats the app already parses
   (gains, ff, accel, vfilt) so a profile switch / load syncs the UI spinboxes. */
static void echo_params(void)
{
  print_gains("lvel", CTRL_LOOP_LVEL);
  print_gains("rvel", CTRL_LOOP_RVEL);
  print_gains("head", CTRL_LOOP_HEAD);
  cprintf("  ff   vel feedforward = %ld (x1000 duty/cps)\r\n",
          (long)(Control_GetVelFF() * 1000.0f));
  {
    float a, d; Control_GetAccel(&a, &d);
    cprintf("accel = %ld  decel = %ld  (cps/s)\r\n", (long)a, (long)d);
  }
  {
    int w; float al; Control_GetVelFilter(&w, &al);
    cprintf("vfilt window = %d ms  alpha = %ld (x1000)\r\n", w, (long)(al * 1000.0f));
  }
  {
    float r, a; Control_GetTurn(&r, &a);
    cprintf("tcfg rate = %ld  accel = %ld  (deg/s, deg/s^2)\r\n", (long)r, (long)a);
  }
  {
    float rad, ent, ex; Control_GetArcGeom(&rad, &ent, &ex);
    cprintf("turns = %s  arc R=%ld entry=%ld exit=%ld mm\r\n",
            Control_GetTurnMode() ? "SMOOTH" : "pivot",
            (long)rad, (long)ent, (long)ex);
  }
}

/* profile [name|n] : show / switch the active tuning profile (applies it live).
   Names: search | fast | fastest (or 0..2). */
static void cmd_profile(int argc, char **argv)
{
  if (argc < 2)
  {
    int act = Settings_Active();
    cprintf("PROFILE,%d,%s\r\n", act, Settings_ProfileName(act));
    puts_("profiles:\r\n");
    for (int i = 0; i < SETTINGS_NUM_PROFILES; i++)
      cprintf("  %d  %-8s%s\r\n", i, Settings_ProfileName(i),
              i == act ? "  <- active" : "");
    puts_("usage: profile <search|fast|fastest|0..2>\r\n");
    return;
  }
  int idx = Settings_Parse(argv[1]);
  if (idx < 0) { puts_("profile: name is search|fast|fastest (or 0..2)\r\n"); return; }
  Settings_SetActive(idx);
  cprintf("PROFILE,%d,%s\r\n", idx, Settings_ProfileName(idx));
  cprintf("profile -> %d (%s), applied:\r\n", idx, Settings_ProfileName(idx));
  echo_params();
}

/* save [name|n] : snapshot the current live tuning into a profile and commit
   ALL profiles to flash. Defaults to the active profile. Robot must be stopped
   (flash erase pauses the CPU). */
static void cmd_save(int argc, char **argv)
{
  if (Control_IsActive())
  {
    puts_("save: stop the robot first - a flash write briefly pauses the CPU\r\n");
    return;
  }
  int idx = (argc >= 2) ? Settings_Parse(argv[1]) : Settings_Active();
  if (idx < 0) { puts_("save: name is search|fast|fastest (or 0..2)\r\n"); return; }
  Settings_Capture(idx);
  if (Settings_Save())
    cprintf("SAVE,ok,%d,%s\r\nsaved current values to profile %d (%s) in flash\r\n",
            idx, Settings_ProfileName(idx), idx, Settings_ProfileName(idx));
  else
    puts_("SAVE,fail\r\nsave: flash write failed\r\n");
}

/* load [name|n] : push a stored profile's values back into the live controller
   (and echo them). Defaults to the active profile. */
static void cmd_load(int argc, char **argv)
{
  int idx = (argc >= 2) ? Settings_Parse(argv[1]) : Settings_Active();
  if (idx < 0) { puts_("load: name is search|fast|fastest (or 0..2)\r\n"); return; }
  Settings_Apply(idx);
  cprintf("LOAD,%d,%s\r\nloaded profile %d (%s):\r\n",
          idx, Settings_ProfileName(idx), idx, Settings_ProfileName(idx));
  echo_params();
}

/* tlm <loop> [on|off] : stream "TLM,t,setpoint,measured,output" for a loop */
static void cmd_tlm(int argc, char **argv)
{
  if (argc < 2)
  {
    puts_("usage: tlm <vel|lvel|rvel|head> [on|off]\r\n");
    return;
  }
  if (ci_eq(argv[1], "off"))
  {
    Control_SetTelem(CTRL_LOOP_LVEL, false);
    puts_("telemetry off\r\n");
    return;
  }
  int loop = parse_loop(argv[1]);
  if (loop < 0) { puts_("tlm: loop is vel|lvel|rvel|head (or 'off')\r\n"); return; }
  bool on = !(argc >= 3 && ci_eq(argv[2], "off"));
  Control_SetTelem((CtrlLoop)loop, on);
  cprintf("telemetry %s for %s loop\r\n", on ? "ON" : "off", argv[1]);
}

/* W25Q32JW OTA-staging flash on QUADSPI1. Bring-up + diagnostics:
     qspi [id]            read + interpret the JEDEC id (bus-alive proof)
     qspi status          dump Status Register 1 (WIP/WEL)
     qspi read <addr> [n] hex-dump n bytes from hex addr (n<=256, default 64)
     qspi test            DESTRUCTIVE erase/program/verify round-trip on the
                          last 4 KB sector (scratch, never an OTA slot) */
static void cmd_qspi(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "id";

  if (ci_eq(sub, "id"))
  {
    QSpiFlash_ID id;
    if (!QSpiFlash_ReadID(&id)) { puts_("qspi: ID read FAILED (bus error)\r\n"); return; }
    cprintf("QSPI id: mfr=0x%02X type=0x%02X cap=0x%02X",
            id.mfr, id.mem_type, id.capacity);
    if (id.mfr == QSPIFLASH_MFR_WINBOND && id.capacity == QSPIFLASH_CAP_32MBIT)
      puts_("  -> Winbond W25Q32 (4 MB) OK\r\n");
    else
      puts_("  -> UNRECOGNISED (expected EF/60/16)\r\n");
  }
  else if (ci_eq(sub, "status"))
  {
    uint8_t sr1;
    if (!QSpiFlash_ReadStatus(&sr1)) { puts_("qspi: status read FAILED\r\n"); return; }
    cprintf("QSPI SR1=0x%02X (WIP=%d WEL=%d)\r\n",
            sr1, (int)(sr1 & 1u), (int)((sr1 >> 1) & 1u));
  }
  else if (ci_eq(sub, "read"))
  {
    if (argc < 3) { puts_("usage: qspi read <hexaddr> [n]\r\n"); return; }
    uint32_t addr = (uint32_t)strtoul(argv[2], NULL, 16);
    uint32_t n    = (argc >= 4) ? (uint32_t)strtoul(argv[3], NULL, 0) : 64u;
    if (n > 256u) n = 256u;
    uint8_t buf[256];
    if (!QSpiFlash_Read(addr, buf, n)) { puts_("qspi: read FAILED (range?)\r\n"); return; }
    for (uint32_t i = 0; i < n; i += 16u)
    {
      cprintf("%06lX:", (unsigned long)(addr + i));
      for (uint32_t j = 0; j < 16u && (i + j) < n; j++) cprintf(" %02X", buf[i + j]);
      puts_("\r\n");
    }
  }
  else if (ci_eq(sub, "test"))
  {
    /* Round-trip on the LAST sector - scratch space, never an OTA image slot. */
    const uint32_t addr = QSPIFLASH_CHIP_SIZE - QSPIFLASH_SECTOR_SIZE;
    uint8_t buf[64], pat[64];

    cprintf("QSPI self-test @0x%06lX (erase+program+verify)...\r\n",
            (unsigned long)addr);
    if (!QSpiFlash_EraseSector(addr)) { puts_("  erase FAILED\r\n"); return; }
    if (!QSpiFlash_Read(addr, buf, sizeof buf)) { puts_("  read-after-erase FAILED\r\n"); return; }
    for (int i = 0; i < 64; i++)
      if (buf[i] != 0xFF) { cprintf("  erase-verify FAILED @%d=0x%02X\r\n", i, buf[i]); return; }

    for (int i = 0; i < 64; i++) pat[i] = (uint8_t)(i * 7 + 3);
    if (!QSpiFlash_Write(addr, pat, sizeof pat)) { puts_("  program FAILED\r\n"); return; }
    if (!QSpiFlash_Read(addr, buf, sizeof buf)) { puts_("  read-back FAILED\r\n"); return; }
    for (int i = 0; i < 64; i++)
      if (buf[i] != pat[i]) { cprintf("  verify FAILED @%d got=0x%02X want=0x%02X\r\n", i, buf[i], pat[i]); return; }

    puts_("  PASS - erase/program/read round-trip OK\r\n");
  }
  else
  {
    puts_("usage: qspi [id|status|read <addr> [n]|test]\r\n");
  }
}

/* ====================== dispatcher ====================================== */

static void dispatch(char *s)
{
  char *argv[6];
  int argc = 0;
  char *p = strtok(s, " \t");
  while (p && argc < 6) { argv[argc++] = p; p = strtok(NULL, " \t"); }

  if (argc == 0)                              /* bare Enter */
  {
    if (mode != MODE_IDLE) stop_streams();
    else                   print_menu();
    return;
  }

  const char *cmd = argv[0];

  /* any command other than a no-op cancels an active stream first ('rate'
     adjusts the live stream, so it must NOT cancel it) */
  if (mode != MODE_IDLE && !ci_eq(cmd, "stop") && !ci_eq(cmd, "rate"))
    mode = MODE_IDLE;

  if (ci_eq(cmd, "help") || ci_eq(cmd, "debug") || ci_eq(cmd, "menu") || ci_eq(cmd, "?"))
  {
    print_menu();
  }
  else if (ci_eq(cmd, "stop") || ci_eq(cmd, "s"))
  {
    stop_streams();
    bench_active = false;                                  /* drop any benchturn */
    griptest_abort(NULL);                                  /* drop any grip test */
    spintest_abort(NULL);                                  /* drop any spin test */
    Solver_Abort();                                        /* drop any solve run */
    Control_NavAbort();                                    /* drop any path run */
    Control_Stop();                                        /* end any PID run */
    Motor_Coast(MOTOR_1);
    Motor_Coast(MOTOR_2);
    __HAL_TIM_SET_COMPARE(C->htim_fan, TIM_CHANNEL_3, 0);   /* fan off too */
    puts_("STOP: control off, motors coasting, vacuum off\r\n> ");
  }
  else if (ci_eq(cmd, "1") || ci_eq(cmd, "encoder") || ci_eq(cmd, "enc"))
  {
    start_stream(MODE_ENC, 100, "encoder");
  }
  else if (ci_eq(cmd, "2") || ci_eq(cmd, "imu"))
  {
    start_stream(MODE_IMU, 50, "imu");
  }
  else if (ci_eq(cmd, "3") || ci_eq(cmd, "sensors") || ci_eq(cmd, "ir"))
  {
    start_stream(MODE_IR, 200, "sensors");
  }
  else if (ci_eq(cmd, "walls") || ci_eq(cmd, "wall"))
  {
    start_stream(MODE_WALLS, 100, "walls");
  }
  else if (ci_eq(cmd, "irset"))
  {
    cmd_irset(argc, argv);
  }
  else if (ci_eq(cmd, "irasync"))
  {
    cmd_irasync(argc, argv);
  }
  else if (ci_eq(cmd, "irraw"))
  {
    cmd_irraw(argc, argv);
  }
  else if (ci_eq(cmd, "ircal"))
  {
    cmd_ircal(argc, argv);
  }
  else if (ci_eq(cmd, "ircsave"))
  {
    cmd_ircsave(argc, argv);
  }
  else if (ci_eq(cmd, "ircshow"))
  {
    cmd_ircshow(argc, argv);
  }
  else if (ci_eq(cmd, "4") || ci_eq(cmd, "motor"))
  {
    cmd_motor(argc, argv);
  }
  else if (ci_eq(cmd, "griptest") || ci_eq(cmd, "grip"))
  {
    cmd_griptest(argc, argv);
  }
  else if (ci_eq(cmd, "spintest") || ci_eq(cmd, "spin"))
  {
    cmd_spintest(argc, argv);
  }
  else if (ci_eq(cmd, "5") || ci_eq(cmd, "vacuum") || ci_eq(cmd, "fan"))
  {
    cmd_vacuum(argc, argv);
  }
  else if (ci_eq(cmd, "6") || ci_eq(cmd, "battery") || ci_eq(cmd, "batt"))
  {
    start_stream(MODE_BATT, 1000, "battery");
  }
  else if (ci_eq(cmd, "7") || ci_eq(cmd, "buzzer"))
  {
    cmd_buzzer(argc, argv);
  }
  else if (ci_eq(cmd, "rate"))
  {
    cmd_rate(argc, argv);
  }
  else if (ci_eq(cmd, "drive") || ci_eq(cmd, "go"))
  {
    cmd_drive(argc, argv);
  }
  else if (ci_eq(cmd, "speed"))
  {
    cmd_speed(argc, argv);
  }
  else if (ci_eq(cmd, "move") || ci_eq(cmd, "mv"))
  {
    cmd_move(argc, argv);
  }
  else if (ci_eq(cmd, "arc"))
  {
    cmd_arc(argc, argv);
  }
  else if (ci_eq(cmd, "flowturn") || ci_eq(cmd, "ft"))
  {
    cmd_flowturn(argc, argv);
  }
  else if (ci_eq(cmd, "benchturn") || ci_eq(cmd, "bt"))
  {
    cmd_benchturn(argc, argv);
  }
  else if (ci_eq(cmd, "snappy"))
  {
    /* toggle the fast-run snappy pivot release (loose tol, early release) so a
       benchturn can reproduce the fast-run turn on the bench. */
    if (argc >= 2)
      Control_SetPivotSnappy(ci_eq(argv[1], "on") || ci_eq(argv[1], "1"));
    cprintf("snappy pivots %s\r\n", Control_GetPivotSnappy() ? "ON" : "off");
  }
  else if (ci_eq(cmd, "arcff"))
  {
    cmd_arcff(argc, argv);
  }
  else if (ci_eq(cmd, "arckp"))
  {
    cmd_arckp(argc, argv);
  }
  else if (ci_eq(cmd, "arckd"))
  {
    cmd_arckd(argc, argv);
  }
  else if (ci_eq(cmd, "arcgkd"))
  {
    cmd_arcgkd(argc, argv);
  }
  else if (ci_eq(cmd, "arctrans"))
  {
    cmd_arctrans(argc, argv);
  }
  else if (ci_eq(cmd, "dfilt"))
  {
    cmd_dfilt(argc, argv);
  }
  else if (ci_eq(cmd, "turn") || ci_eq(cmd, "pivot"))
  {
    cmd_turn(argc, argv);
  }
  else if (ci_eq(cmd, "advance") || ci_eq(cmd, "adv"))
  {
    cmd_advance(argc, argv);
  }
  else if (ci_eq(cmd, "cell"))
  {
    cmd_cell(argc, argv);
  }
  else if (ci_eq(cmd, "path"))
  {
    cmd_path(argc, argv);
  }
  else if (ci_eq(cmd, "solve"))
  {
    cmd_solve(argc, argv);
  }
  else if (ci_eq(cmd, "fast"))
  {
    cmd_fast(argc, argv);
  }
  else if (ci_eq(cmd, "mem"))
  {
    cmd_mem(argc, argv);
  }
  else if (ci_eq(cmd, "qspi"))
  {
    cmd_qspi(argc, argv);
  }
  else if (ci_eq(cmd, "sim"))
  {
    cmd_sim(argc, argv);
  }
  else if (ci_eq(cmd, "dump"))
  {
    cmd_dump(argc, argv);
  }
  else if (ci_eq(cmd, "maze"))
  {
    cmd_maze(argc, argv);
  }
  else if (ci_eq(cmd, "goal"))
  {
    cmd_goal(argc, argv);
  }
  else if (ci_eq(cmd, "start"))
  {
    cmd_start(argc, argv);
  }
  else if (ci_eq(cmd, "align"))
  {
    cmd_align(argc, argv);
  }
  else if (ci_eq(cmd, "acfg"))
  {
    cmd_acfg(argc, argv);
  }
  else if (ci_eq(cmd, "frontcal") || ci_eq(cmd, "fcal"))
  {
    cmd_frontcal(argc, argv);
  }
  else if (ci_eq(cmd, "tcfg"))
  {
    cmd_tcfg(argc, argv);
  }
  else if (ci_eq(cmd, "tcost"))
  {
    cmd_tcost(argc, argv);
  }
  else if (ci_eq(cmd, "gyroscale") || ci_eq(cmd, "gscale"))
  {
    cmd_gyroscale(argc, argv);
  }
  else if (ci_eq(cmd, "follow"))
  {
    cmd_follow(argc, argv);
  }
  else if (ci_eq(cmd, "fcfg"))
  {
    cmd_fcfg(argc, argv);
  }
  else if (ci_eq(cmd, "pid"))
  {
    cmd_pid(argc, argv);
  }
  else if (ci_eq(cmd, "ff"))
  {
    cmd_ff(argc, argv);
  }
  else if (ci_eq(cmd, "autotune") || ci_eq(cmd, "at"))
  {
    cmd_autotune(argc, argv);
  }
  else if (ci_eq(cmd, "accel"))
  {
    cmd_accel(argc, argv);
  }
  else if (ci_eq(cmd, "brake"))
  {
    cmd_brake(argc, argv);
  }
  else if (ci_eq(cmd, "vfilt"))
  {
    cmd_vfilt(argc, argv);
  }
  else if (ci_eq(cmd, "profile") || ci_eq(cmd, "prof"))
  {
    cmd_profile(argc, argv);
  }
  else if (ci_eq(cmd, "save"))
  {
    cmd_save(argc, argv);
  }
  else if (ci_eq(cmd, "load"))
  {
    cmd_load(argc, argv);
  }
  else if (ci_eq(cmd, "tlm"))
  {
    cmd_tlm(argc, argv);
  }
  else
  {
    cprintf("unknown: '%s' (type 'help')\r\n", cmd);
  }

  if (mode == MODE_IDLE) puts_("> ");
}

/* ====================== public API ====================================== */

void Console_Init(const ConsoleCtx *ctx)
{
  C = ctx;
  mode = MODE_IDLE;
  line_len = 0;
  last_term = 0;

  /* Bring USART1 up on interrupts with empty TX/RX rings. */
  UARTx = C->huart->Instance;
  txr_head = txr_tail = 0;
  rxr_head = rxr_tail = 0;
  __HAL_UART_ENABLE_IT(C->huart, UART_IT_RXNE);   /* fire on each byte received */
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);        /* below SysTick, above app */
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* IR sensor table (emitter GPIO + ADC channel per receiver). */
  ir_sensors[0] = (IR_Sensor){ "L_LM", LEFT_LM_EMMITER_GPIO_Port,     LEFT_LM_EMMITER_Pin,     C->hadc1, ADC_CHANNEL_1 };
  ir_sensors[1] = (IR_Sensor){ "L_M",  LEFT_M_EMMITER_GPIO_Port,      LEFT_M_EMMITER_Pin,      C->hadc1, ADC_CHANNEL_9 };
  ir_sensors[2] = (IR_Sensor){ "L_F",  LEFT_FRONT_EMMITER_GPIO_Port,  LEFT_FRONT_EMMITER_Pin,  C->hadc1, ADC_CHANNEL_8 };
  ir_sensors[3] = (IR_Sensor){ "R_F",  RIGHT_FRONT_EMMITER_GPIO_Port, RIGHT_FRONT_EMMITER_Pin, C->hadc1, ADC_CHANNEL_7 };
  ir_sensors[4] = (IR_Sensor){ "R_M",  RIGHT_M_EMMITER_GPIO_Port,     RIGHT_M_EMMITER_Pin,     C->hadc1, ADC_CHANNEL_4 };
  ir_sensors[5] = (IR_Sensor){ "R_RM", RIGHT_RM_EMMITER_GPIO_Port,    RIGHT_RM_EMMITER_Pin,    C->hadc4, ADC_CHANNEL_3 };

  /* Encoders: both on SPI3, separate CS lines. */
  AS5047P_Init(&encL, C->hspi_enc, ENCODERL_CS_GPIO_Port, ENCODERL_CS_Pin);
  AS5047P_Init(&encR, C->hspi_enc, ENCODERR_CS_GPIO_Port, ENCODERR_CS_Pin);

  /* Calibrate both ADCs so the IR + motor-sense reads are accurate even if the
     boot self-test is later stripped out. (Re-cal is safe; ADCs idle here.) */
  HAL_ADCEx_Calibration_Start(C->hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(C->hadc4, ADC_SINGLE_ENDED);

  /* ABI quadrature counters. */
  HAL_TIM_Encoder_Start(C->htim_encL, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(C->htim_encR, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(C->htim_encL, 0);
  __HAL_TIM_SET_COUNTER(C->htim_encR, 0);

  /* Motor bench driver: DWT timing + ADC calibration, motors coasting. */
  Motor_TestInit(C->hadc4);

  /* Closed-loop motion controller: takes over TIM3 as hardware motor PWM and
     arms the 1 kHz control loop (idle until 'drive'). After this the motors
     are PWM-driven, so 'stop'/coast actually cut drive. */
  ControlCtx cc = {
    .htim_mot  = C->htim_mot,
    .htim_encL = C->htim_encL,
    .htim_encR = C->htim_encR,
    .hspi_imu  = C->hspi_imu,
  };
  Control_Init(&cc);

  /* Load saved tuning profiles from flash (applies the active one over the
     baked-in defaults). Must come after Control_Init. */
  Settings_Init();

  /* Autonomous maze solver: clear its map + park the mouse at the start cell. */
  Solver_Reset();

  /* Vacuum PWM ready at 0 %. */
  HAL_TIM_PWM_Start(C->htim_fan, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(C->htim_fan, TIM_CHANNEL_3, 0);

  /* Probe the W25Q32JW OTA-staging flash on QUADSPI1 (binds the driver handle).
     Non-fatal: a missing/failed chip just means OTA staging is unavailable. */
  qspi_ok = QSpiFlash_Init(C->hqspi);
  cprintf("QSPI flash (OTA staging): %s\r\n",
          qspi_ok ? "W25Q32JW OK" : "NOT DETECTED (run 'qspi id')");

  print_menu();
}

void Console_Task(void)
{
  if (poll_line())
    dispatch(line);

  if (bench_active)                 /* drive the turn-accuracy bench sequence */
    bench_task();

  /* Drain any control-loop telemetry the ISR buffered and print it as
     "TLM,t,setpoint,measured,output" (the Python app's tlm parser). Bounded
     per pass so a backlog can't starve the rest of the loop. */
  {
    uint32_t t; int32_t sp, meas, outv;
    for (int i = 0; i < 16 && Control_PopTelem(&t, &sp, &meas, &outv); i++)
      cprintf("TLM,%lu,%ld,%ld,%ld\r\n",
              (unsigned long)t, (long)sp, (long)meas, (long)outv);
  }

  /* Auto-tune result: apply it to the velocity loop and echo so the app's gain
     parser updates the spinboxes. Floats shown x1000 (no %f in newlib-nano). */
  {
    bool ok; float kp, ki, kd, ku, tu;
    if (Control_AutotunePopResult(&ok, &kp, &ki, &kd, &ku, &tu))
    {
      if (ok)
      {
        Control_SetGains(CTRL_LOOP_VEL, kp, ki, kd);
        cprintf("ATUNE,ok,%ld,%ld,%ld,%ld,%ld\r\n",
                (long)(kp * 1000.0f), (long)(ki * 1000.0f), (long)(kd * 1000.0f),
                (long)(ku * 1000.0f), (long)tu);
        cprintf("AUTOTUNE done: Ku=%ld Tu=%ld ms (x1000) -> applied to vel loop\r\n",
                (long)(ku * 1000.0f), (long)tu);
        print_gains("lvel", CTRL_LOOP_LVEL);
        print_gains("rvel", CTRL_LOOP_RVEL);
        puts_("> ");
      }
      else
      {
        puts_("ATUNE,fail\r\n"
              "AUTOTUNE failed: no clean oscillation - set a rough 'ff' first or "
              "try a higher 'autotune <cps>'\r\n> ");
      }
    }
  }

  /* Pump the vacuum grip/downforce tests (fan-duty sweeps) if running. */
  griptest_task();
  spintest_task();

  /* Pump the nav sequencer (the `path` queue): auto-starts the next step when
     the current one finishes. While a sequence runs this drains the move/turn
     done flags, so the per-move prints below stay quiet and we report at path
     level instead. */
  Control_NavTask();
  {
    int idx, total, type, arg;
    if (Control_NavPopStepStart(&idx, &total, &type, &arg))
    {
      if (type == 0)
        cprintf("PATH %d/%d: advance %d cell%s\r\n",
                idx + 1, total, arg, (arg == 1) ? "" : "s");
      else
        cprintf("PATH %d/%d: turn %d deg\r\n", idx + 1, total, arg);
    }
    bool ok;
    if (Control_NavPopDone(&ok))
      cprintf("PATH: %s\r\n> ", ok ? "sequence done" : "aborted");
  }

  /* Pump the autonomous flood-fill solver. Like the nav sequencer it owns the
     move/turn done flags while it runs (it's pumped before the per-move prints
     below, and they're gated on !Solver_Active()), so we report at solve level:
     "SOLVE,x,y,facing,flood,phase" per cell, "SOLVE: done/aborted" at the end. */
  Solver_Task();
  {
    int x, y, facing, fld, ph, wb;
    if (Solver_PopStep(&x, &y, &facing, &fld, &ph, &wb))
    {
      cprintf("SOLVE,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", x, y, facing, fld, ph,
              (wb & 0x1) ? 1 : 0, (wb & 0x2) ? 1 : 0,        /* wallN, wallE */
              (wb & 0x4) ? 1 : 0, (wb & 0x8) ? 1 : 0);       /* wallS, wallW */
      /* raw IR at this sense: L_LM,L_M,L_F,R_F,R_M,R_RM then thrL_F,thrR_F */
      int ir[6], tlf, trf; Solver_GetSenseIR(ir, &tlf, &trf);
      cprintf("SIR,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", x, y,
              ir[0], ir[1], ir[2], ir[3], ir[4], ir[5], tlf, trf);
    }
    int fdeg, fr, fcps;
    if (Solver_PopFlowStart(&fdeg, &fr, &fcps))      /* fast-run smooth turn marker */
      cprintf("FLOW,%d,%d,%d\r\n", fdeg, fr, fcps);
    bool ok;
    if (Solver_PopDone(&ok))
      cprintf("SOLVE: %s\r\n> ", ok ? "done - back at start" : "aborted");
    int calmm;
    if (Solver_PopCalDone(&ok, &calmm))
    {
      if (ok) cprintf("FRONTCAL: centre->wall = %d mm (recenter set)\r\n> ", calmm);
      else    cprintf("FRONTCAL: no wall hit (%d mm) - place at a centre facing a wall\r\n> ",
                      calmm);
    }
  }

  /* Cell-counted advance reached a cell centre: latch + report the walls there
     (the per-cell decision point the maze solver consumes). */
  {
    int32_t idx;
    if (Control_PopCellMark(&idx))
      cprintf("CELL,%ld,%d,%d,%d\r\n", (long)idx,
              Walls_Left() ? 1 : 0, Walls_Right() ? 1 : 0, Walls_Front() ? 1 : 0);
  }

  /* Distance-limited move finished: announce arrival. (Quiet during a solve -
     the solver consumes these per-cell.) */
  if (!Solver_Active())
  {
    int32_t mm;
    if (Control_PopMoveDone(&mm))
      cprintf("MOVE: done, travelled %ld mm\r\n> ", (long)mm);
  }

  /* Pivot turn finished: announce the heading actually reached + the structured
     TURNDIAG line (idx -1 = a standalone 'turn', not part of a benchturn run). */
  if (!Solver_Active() && !bench_active)
  {
    int32_t deg;
    if (Control_PopTurnDone(&deg))
    {
      int32_t cmd, ach, pk, ms, rs, pl, pr;
      Control_GetLastTurnDiag(&cmd, &ach, &pk, &ms, &rs, &pl, &pr);
      cprintf("TURN: done, heading %ld deg\r\n", (long)deg);
      cprintf("TURNDIAG,-1,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n> ",
              (long)cmd, (long)ach, (long)(ach - cmd), (long)pk, (long)ms, (long)rs,
              (long)pl, (long)pr);
    }
  }

  /* Standalone front-align finished (the solver consumes its own). */
  if (!Solver_Active())
  {
    bool aok;
    if (Control_PopAlignDone(&aok))
      cprintf("ALIGN: squared\r\n> ");
  }

  if (mode != MODE_IDLE)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last_sample) >= stream_interval)
    {
      last_sample = now;
      switch (mode)
      {
        case MODE_IMU:  sample_imu();  break;
        case MODE_ENC:  sample_enc();  break;
        case MODE_IR:   sample_ir();   break;
        case MODE_WALLS:sample_walls();break;
        case MODE_BATT: sample_batt(); break;
        default: break;
      }
    }
  }
}
