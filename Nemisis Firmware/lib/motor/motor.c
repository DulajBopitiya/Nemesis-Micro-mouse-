/**
 ******************************************************************************
 * @file    motor.c
 * @brief   Bench-test driver for the two DRV8874PWPR H-bridges. See motor.h.
 ******************************************************************************
 */
#include "motor.h"
#include "dbg.h"

/* ===========================================================================
 *  Current-sense scaling  --  values taken from the Nemisis schematic.
 *
 *  DRV8874 mirrors the motor current onto the IPROPI pin:
 *      I_IPROPI = I_OUT * A_IPROPI          (A_IPROPI = 455 uA per A, datasheet)
 *      V_SENSE  = I_IPROPI * R_IPROPI       (voltage the ADC actually reads)
 *  =>  I_OUT[mA] = V_SENSE[mV] / (R_IPROPI[ohm] * A_IPROPI[uA/A] / 1e6)
 *
 *  R27 on the board = 2.49 k (IPROPI -> GND). With A_IPROPI = 455 uA/A this
 *  gives ~1.133 V per amp of motor current (full 3.3 V scale ~ 2.9 A).
 * ========================================================================= */
#define DRV8874_R_IPROPI_OHMS     2490.0f   /* R27, IPROPI -> GND resistor     */
#define DRV8874_A_IPROPI_UA_PER_A 455.0f    /* DRV8874 mirror ratio (datasheet)*/

#define ADC_VREF_MV               3300U     /* ADC reference (3.3 V rail)      */
#define ADC_FULL_SCALE            4095U     /* 12-bit                          */
#define SENSE_AVG_SAMPLES         8U        /* averaging per current read      */

/* Software-PWM parameters (bit-banged). 1 kHz period = 1000 us. */
#define SOFT_PWM_PERIOD_US        1000U

/* --- Microsecond timing via the DWT cycle counter (as in the buzzer) ------ */
static void dwt_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks) { }
}

/* --- pin lookup ----------------------------------------------------------- */
typedef struct { GPIO_TypeDef *port; uint16_t pin; } gpio_t;

static void motor_pins(MotorId m, gpio_t *in1, gpio_t *in2)
{
  if (m == MOTOR_1)
  {
    in1->port = M1_IN1_GPIO_Port; in1->pin = M1_IN1_Pin;
    in2->port = M1_IN2_GPIO_Port; in2->pin = M1_IN2_Pin;
  }
  else
  {
    in1->port = M2_IN1_GPIO_Port; in1->pin = M2_IN1_Pin;
    in2->port = M2_IN2_GPIO_Port; in2->pin = M2_IN2_Pin;
  }
}

static uint32_t sense_channel(MotorId m)
{
  /* PB15 = ADC4_IN5 (M1) , PB14 = ADC4_IN4 (M2) */
  return (m == MOTOR_1) ? ADC_CHANNEL_5 : ADC_CHANNEL_4;
}

/* --- hardware PWM (TIM3) --------------------------------------------------- */
/* IN-pin -> TIM3 channel map (see motor.h):
 *   M1: IN1=PC8=CH3 , IN2=PC7=CH2     M2: IN1=PC6=CH1 , IN2=PB7=CH4         */
static TIM_HandleTypeDef *s_htim;      /* TIM3, or NULL until Motor_PWM_Init */
static uint8_t            s_pwm_ready;

static void motor_channels(MotorId m, uint32_t *ch_in1, uint32_t *ch_in2)
{
  if (m == MOTOR_1) { *ch_in1 = TIM_CHANNEL_3; *ch_in2 = TIM_CHANNEL_2; }
  else              { *ch_in1 = TIM_CHANNEL_1; *ch_in2 = TIM_CHANNEL_4; }
}

void Motor_PWM_Init(TIM_HandleTypeDef *htim3)
{
  s_htim = htim3;

  /* Force a clean 20 kHz period regardless of what CubeMX generated (it left
     prescaler 0 / period 65535 -> ~2.6 kHz, audible). */
  __HAL_TIM_SET_PRESCALER(htim3, 0U);
  __HAL_TIM_SET_AUTORELOAD(htim3, (uint32_t)MOTOR_PWM_MAX - 1U);

  /* CubeMX generated CH2 (M1_IN2) as OCMODE_TIMING, not PWM - put it back. */
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(htim3, &oc, TIM_CHANNEL_2);

  /* Start all four channels at 0 % (both inputs low -> coast). */
  HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, 0);

  s_pwm_ready = 1;
}

void Motor_SetDuty(MotorId m, int duty)
{
  if (!s_pwm_ready) return;
  if (duty >  MOTOR_PWM_MAX) duty =  MOTOR_PWM_MAX;
  if (duty < -MOTOR_PWM_MAX) duty = -MOTOR_PWM_MAX;

  uint32_t ch1, ch2;
  motor_channels(m, &ch1, &ch2);     /* ch1 = IN1, ch2 = IN2 */

  if (duty >= 0)                     /* forward: PWM on IN1, IN2 low */
  {
    __HAL_TIM_SET_COMPARE(s_htim, ch1, (uint32_t)duty);
    __HAL_TIM_SET_COMPARE(s_htim, ch2, 0U);
  }
  else                               /* reverse: PWM on IN2, IN1 low */
  {
    __HAL_TIM_SET_COMPARE(s_htim, ch1, 0U);
    __HAL_TIM_SET_COMPARE(s_htim, ch2, (uint32_t)(-duty));
  }
}

/* --- low-level drive ------------------------------------------------------ */
/* Legacy GPIO path - only used before Motor_PWM_Init() (e.g. the pin-probe
   diagnostic). Once PWM is up the pins are timer AF and these are no-ops, so
   the public Coast/Forward/etc. below route through the timer instead. */
static void drive(MotorId m, GPIO_PinState s1, GPIO_PinState s2)
{
  gpio_t in1, in2;
  motor_pins(m, &in1, &in2);
  HAL_GPIO_WritePin(in1.port, in1.pin, s1);
  HAL_GPIO_WritePin(in2.port, in2.pin, s2);
}

void Motor_Coast(MotorId m)
{
  if (s_pwm_ready) Motor_SetDuty(m, 0);
  else             drive(m, GPIO_PIN_RESET, GPIO_PIN_RESET);
}

void Motor_Brake(MotorId m)
{
  if (s_pwm_ready)                   /* both inputs high -> low-side brake */
  {
    uint32_t ch1, ch2; motor_channels(m, &ch1, &ch2);
    __HAL_TIM_SET_COMPARE(s_htim, ch1, (uint32_t)MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(s_htim, ch2, (uint32_t)MOTOR_PWM_MAX);
  }
  else drive(m, GPIO_PIN_SET, GPIO_PIN_SET);
}

void Motor_Forward(MotorId m)
{
  if (s_pwm_ready) Motor_SetDuty(m,  MOTOR_PWM_MAX);
  else             drive(m, GPIO_PIN_SET, GPIO_PIN_RESET);
}

void Motor_Reverse(MotorId m)
{
  if (s_pwm_ready) Motor_SetDuty(m, -MOTOR_PWM_MAX);
  else             drive(m, GPIO_PIN_RESET, GPIO_PIN_SET);
}

/* Blocking convenience for the console "motor" command: drive one motor at a
   percentage for `ms`, then coast. Now backed by hardware PWM (the old bit-bang
   can't work - the pins are timer AF). speed: -100..+100 %. */
void Motor_SoftPWM(MotorId m, int speed, uint32_t ms)
{
  if (speed >  100) speed =  100;
  if (speed < -100) speed = -100;
  Motor_SetDuty(m, speed * MOTOR_PWM_MAX / 100);
  HAL_Delay(ms);
  Motor_SetDuty(m, 0);
}

/* --- current sense -------------------------------------------------------- */
uint16_t Motor_ReadSenseRaw(ADC_HandleTypeDef *hadc, MotorId m)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel      = sense_channel(m);
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;  /* long-ish for the source Z */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    return 0;

  uint32_t acc = 0;
  for (uint32_t i = 0; i < SENSE_AVG_SAMPLES; i++)
  {
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
      acc += HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
  }
  return (uint16_t)(acc / SENSE_AVG_SAMPLES);
}

uint32_t Motor_ReadSense_mV(ADC_HandleTypeDef *hadc, MotorId m)
{
  uint32_t raw = Motor_ReadSenseRaw(hadc, m);
  return (raw * ADC_VREF_MV) / ADC_FULL_SCALE;
}

int32_t Motor_ReadCurrent_mA(ADC_HandleTypeDef *hadc, MotorId m)
{
  float mv    = (float)Motor_ReadSense_mV(hadc, m);
  float denom = DRV8874_R_IPROPI_OHMS * (DRV8874_A_IPROPI_UA_PER_A / 1e6f);
  return (int32_t)(mv / denom);   /* mV / (ohm * A/A) -> mA */
}

/* --- init + full test sequence -------------------------------------------- */
void Motor_TestInit(ADC_HandleTypeDef *hadc)
{
  dwt_delay_init();
  Motor_Coast(MOTOR_1);
  Motor_Coast(MOTOR_2);

  /* Single-ended ADC calibration (MX_ADC4_Init does not do this). */
  if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK)
    LOG("MOTOR: ADC calibration FAILED\r\n");
}

static void test_one_motor(ADC_HandleTypeDef *hadc, MotorId m)
{
  const char *name = (m == MOTOR_1) ? "M1" : "M2";

  LOG("\r\n--- %s test (PWM-mode IN1/IN2) -------------------------\r\n", name);

  /* baseline: motor coasting, should read ~0 mA */
  Motor_Coast(m);
  delay_us(50000U);
  LOG("%s coast   : sense=%lu mV  ~%ld mA (baseline)\r\n",
      name,
      (unsigned long)Motor_ReadSense_mV(hadc, m),
      (long)Motor_ReadCurrent_mA(hadc, m));

  /* forward ramp 0 -> 100 % so you can hear/see it accelerate */
  LOG("%s forward ramp 0..100%%\r\n", name);
  for (int duty = 10; duty <= 100; duty += 10)
    Motor_SoftPWM(m, duty, 150);          /* 150 ms per step */

  /* hold full forward (steady drive) and measure current */
  Motor_Forward(m);
  delay_us(200000U);
  LOG("%s forward : sense=%lu mV  ~%ld mA\r\n",
      name,
      (unsigned long)Motor_ReadSense_mV(hadc, m),
      (long)Motor_ReadCurrent_mA(hadc, m));

  Motor_Coast(m);
  delay_us(400000U);

  /* reverse ramp */
  LOG("%s reverse ramp 0..-100%%\r\n", name);
  for (int duty = 10; duty <= 100; duty += 10)
    Motor_SoftPWM(m, -duty, 150);

  Motor_Reverse(m);
  delay_us(200000U);
  LOG("%s reverse : sense=%lu mV  ~%ld mA\r\n",
      name,
      (unsigned long)Motor_ReadSense_mV(hadc, m),
      (long)Motor_ReadCurrent_mA(hadc, m));

  /* brake then coast */
  LOG("%s brake\r\n", name);
  Motor_Brake(m);
  delay_us(300000U);
  Motor_Coast(m);
  delay_us(300000U);

  LOG("%s done.\r\n", name);
}

void Motor_TestRun(ADC_HandleTypeDef *hadc)
{
  LOG("\r\n======== DRV8874 motor test ========\r\n");
  test_one_motor(hadc, MOTOR_1);
  test_one_motor(hadc, MOTOR_2);
  Motor_Coast(MOTOR_1);
  Motor_Coast(MOTOR_2);
  LOG("======== motor test complete =======\r\n");
}

/* --- drive BOTH motors simultaneously (hardware PWM, shared duty) --------- */
void Motor_DriveBoth(int dir1, int dir2, int duty, uint32_t ms)
{
  if (duty > 100) duty = 100;
  if (duty <   0) duty =   0;
  int d = duty * MOTOR_PWM_MAX / 100;

  Motor_SetDuty(MOTOR_1, (dir1 > 0) ? d : (dir1 < 0) ? -d : 0);
  Motor_SetDuty(MOTOR_2, (dir2 > 0) ? d : (dir2 < 0) ? -d : 0);
  if (ms) HAL_Delay(ms);
  /* leave both coasting */
  Motor_SetDuty(MOTOR_1, 0);
  Motor_SetDuty(MOTOR_2, 0);
}

/* --- one-shot movement demo ---------------------------------------------- */
void Motor_DriveSequence(ADC_HandleTypeDef *hadc)
{
  const int      DUTY     = 20;    /* % cruise - moderate so pivots aren't violent */
  const int      STEP     = 2;     /* duty increment per ramp tick                 */
  const uint32_t RAMP_MS  = 500U;  /* time to ease up to / down from cruise        */
  const uint32_t CRUISE_MS= 900U;  /* steady run at cruise duty                    */
  const uint32_t PAUSE    = 700U;  /* coast between phases                         */

  /* per-tick dwell so the whole ramp takes ~RAMP_MS (steps = DUTY/STEP) */
  const uint32_t tick_ms = RAMP_MS / ((uint32_t)(DUTY / STEP) + 1U);

  LOG("\r\n======== motor movement demo (cruise=%d%%, ramped) ========\r\n", DUTY);
  LOG("M1 = PC8/PC7 , M2 = PC6/PB7. Sequence: fwd, rev, pivot, pivot, stop.\r\n");

  struct { const char *name; int d1; int d2; } phase[] = {
    { "BOTH FORWARD",        +1, +1 },
    { "BOTH REVERSE",        -1, -1 },
    { "PIVOT  (M1 fwd/M2 rev)", +1, -1 },
    { "PIVOT  (M1 rev/M2 fwd)", -1, +1 },
  };

  for (unsigned p = 0; p < sizeof(phase)/sizeof(phase[0]); p++)
  {
    int d1 = phase[p].d1, d2 = phase[p].d2;
    LOG("-> %s\r\n", phase[p].name);

    /* accelerate: ease 0 -> DUTY so the start isn't a jerk */
    for (int duty = STEP; duty <= DUTY; duty += STEP)
      Motor_DriveBoth(d1, d2, duty, tick_ms);

    /* cruise at DUTY, then sample current (brief steady drive for a clean read) */
    Motor_DriveBoth(d1, d2, DUTY, CRUISE_MS);
    drive(MOTOR_1, d1 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET,
                   d1 >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    drive(MOTOR_2, d2 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET,
                   d2 >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    delay_us(60000U);
    LOG("   current: M1 ~%ld mA , M2 ~%ld mA\r\n",
        (long)Motor_ReadCurrent_mA(hadc, MOTOR_1),
        (long)Motor_ReadCurrent_mA(hadc, MOTOR_2));

    /* decelerate: ease DUTY -> 0 so the STOP is gentle, not a hard cut */
    for (int duty = DUTY; duty >= 0; duty -= STEP)
      Motor_DriveBoth(d1, d2, duty, tick_ms);

    Motor_Coast(MOTOR_1);
    Motor_Coast(MOTOR_2);
    HAL_Delay(PAUSE);
  }

  Motor_Coast(MOTOR_1);
  Motor_Coast(MOTOR_2);
  LOG("======== demo complete - motors stopped ========\r\n");
}

/* Read the actual pin level back from the GPIO input register. Even when a pin
 * is an output, IDR reflects the real voltage on the pad - so if we command a
 * pin HIGH and it reads back LOW, the MCU can't drive it (short / heavy load /
 * misconfig). If it reads HIGH, the MCU side is good and the fault is downstream
 * (broken trace to the DRV, or the DRV itself). */
static int pin_rb(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1 : 0;
}

void Motor_DiagLoop(ADC_HandleTypeDef *hadc)
{
  gpio_t m1a, m1b, m2a, m2b;
  motor_pins(MOTOR_1, &m1a, &m1b);
  motor_pins(MOTOR_2, &m2a, &m2b);

  LOG("\r\n==== DRV8874 STATIC pin diagnostic (loops forever) ====\r\n");
  LOG("Use the meter in DC VOLTS (not ohms!) - the board is powered.\r\n");
  LOG("'rb=' is the MCU pin read back from its own register:\r\n");
  LOG("  commanded HIGH but rb=0 -> MCU pin can't drive (short/load)\r\n");
  LOG("  commanded HIGH and rb=1 -> MCU is fine, check DRV input pin & trace\r\n\r\n");

  for (;;)
  {
    /* state 0: everything off */
    Motor_Coast(MOTOR_1);
    Motor_Coast(MOTOR_2);
    LOG("[ALL OFF] IN low. Meter VM(p11)=batt, nSLEEP(p3)=3v3, nFAULT(p4)=3v3. "
        "rb PC8=%d PC7=%d PC6=%d PB7=%d | sense M1=%lu M2=%lu mV\r\n",
        pin_rb(m1a.port,m1a.pin), pin_rb(m1b.port,m1b.pin),
        pin_rb(m2a.port,m2a.pin), pin_rb(m2b.port,m2b.pin),
        (unsigned long)Motor_ReadSense_mV(hadc, MOTOR_1),
        (unsigned long)Motor_ReadSense_mV(hadc, MOTOR_2));
    HAL_Delay(4000);

    /* --- M1: drive forward then reverse. With NO motor connected, OUT1/OUT2
       should swing fully VM<->0 in BOTH directions. A burnt FET = an output
       stuck at VM, stuck at 0, or floating. --------------------------------- */
    Motor_Forward(MOTOR_1);
    LOG("[M1 FWD] PC8=1 PC7=0 (rb %d/%d). Expect OUT1~VM, OUT2~0. "
        "sense=%lu mV (~%ld mA)\r\n",
        pin_rb(m1a.port,m1a.pin), pin_rb(m1b.port,m1b.pin),
        (unsigned long)Motor_ReadSense_mV(hadc, MOTOR_1),
        (long)Motor_ReadCurrent_mA(hadc, MOTOR_1));
    HAL_Delay(4000);
    Motor_Reverse(MOTOR_1);
    LOG("[M1 REV] PC8=0 PC7=1 (rb %d/%d). Expect OUT1~0, OUT2~VM. "
        "sense=%lu mV (~%ld mA)\r\n",
        pin_rb(m1a.port,m1a.pin), pin_rb(m1b.port,m1b.pin),
        (unsigned long)Motor_ReadSense_mV(hadc, MOTOR_1),
        (long)Motor_ReadCurrent_mA(hadc, MOTOR_1));
    HAL_Delay(4000);
    Motor_Coast(MOTOR_1);

    /* --- M2: same forward/reverse swing test ------------------------------ */
    Motor_Forward(MOTOR_2);
    LOG("[M2 FWD] PC6=1 PB7=0 (rb %d/%d). Expect OUT1~VM, OUT2~0. "
        "sense=%lu mV (~%ld mA)\r\n",
        pin_rb(m2a.port,m2a.pin), pin_rb(m2b.port,m2b.pin),
        (unsigned long)Motor_ReadSense_mV(hadc, MOTOR_2),
        (long)Motor_ReadCurrent_mA(hadc, MOTOR_2));
    HAL_Delay(4000);
    Motor_Reverse(MOTOR_2);
    LOG("[M2 REV] PC6=0 PB7=1 (rb %d/%d). Expect OUT1~0, OUT2~VM. "
        "sense=%lu mV (~%ld mA)\r\n",
        pin_rb(m2a.port,m2a.pin), pin_rb(m2b.port,m2b.pin),
        (unsigned long)Motor_ReadSense_mV(hadc, MOTOR_2),
        (long)Motor_ReadCurrent_mA(hadc, MOTOR_2));
    HAL_Delay(4000);
    Motor_Coast(MOTOR_2);
  }
}
