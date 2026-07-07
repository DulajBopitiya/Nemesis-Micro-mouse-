/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include "buzzer.h"
#include "dbg.h"
#include "icm42688.h"
#include "max17049.h"
#include "motor.h"
#include "as5047p.h"
#include "battery.h"
#include "console.h"
#include "sensors.h"
#include "launcher.h"
#include "control.h"
#include "solver.h"
#include "mazestore.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc4;

I2C_HandleTypeDef hi2c3;

QSPI_HandleTypeDef hqspi1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim15_ch1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM15_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC4_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_QUADSPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ====================== IR reflectance sensors ========================
   6 emitter/receiver pairs. Naming: side (LEFT/RIGHT) + position
   (LM = left-most, M = middle, FRONT). Emitters are GPIO outputs that drive
   an IR LED through a MOSFET (active-HIGH). Receivers are phototransistors
   read by an ADC.

   Receiver -> ADC mapping (from main.h pins / CubeMX MSP comments):
     RIGHT_RM    PB12 -> ADC4_IN3   (hadc4)
     LEFT_LM     PA0  -> ADC1_IN1   \
     RIGHT_M     PA3  -> ADC1_IN4    |
     RIGHT_FRONT PC1  -> ADC1_IN7    >  ADC1 (hadc1), configured in CubeMX
     LEFT_FRONT  PC2  -> ADC1_IN8    |
     LEFT_M      PC3  -> ADC1_IN9   /
   ====================================================================== */

#define IR_AVG_SAMPLES  16U     /* conversions averaged per phase (noise floor) */

/* Average N single conversions of one channel on the given ADC. Reconfigures
   the channel each call so one ADC can be shared across several inputs. */
static uint16_t ADC_ReadAvg(ADC_HandleTypeDef *adc, uint32_t channel, uint32_t n)
{
  ADC_ChannelConfTypeDef c = {0};
  c.Channel      = channel;
  c.Rank         = ADC_REGULAR_RANK_1;
  c.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;  /* high-Z phototransistor node */
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

/* One IR sensor: its emitter GPIO + the ADC + channel its receiver sits on. */
typedef struct
{
  const char        *name;
  GPIO_TypeDef      *em_port;
  uint16_t           em_pin;
  ADC_HandleTypeDef *adc;
  uint32_t           channel;
} IR_Sensor;

#define IR_COUNT 6
static const IR_Sensor ir_sensors[IR_COUNT] =
{
  { "L_LM", LEFT_LM_EMMITER_GPIO_Port,     LEFT_LM_EMMITER_Pin,     &hadc1, ADC_CHANNEL_1 }, /* recv PA0=ADC1_IN1 */
  { "L_M",  LEFT_M_EMMITER_GPIO_Port,      LEFT_M_EMMITER_Pin,      &hadc1, ADC_CHANNEL_9 }, /* recv PC3=ADC1_IN9 */
  { "L_F",  LEFT_FRONT_EMMITER_GPIO_Port,  LEFT_FRONT_EMMITER_Pin,  &hadc1, ADC_CHANNEL_8 }, /* recv PC2=ADC1_IN8 */
  { "R_F",  RIGHT_FRONT_EMMITER_GPIO_Port, RIGHT_FRONT_EMMITER_Pin, &hadc1, ADC_CHANNEL_7 }, /* recv PC1=ADC1_IN7 */
  { "R_M",  RIGHT_M_EMMITER_GPIO_Port,     RIGHT_M_EMMITER_Pin,     &hadc1, ADC_CHANNEL_4 }, /* recv PA3=ADC1_IN4 */
  { "R_RM", RIGHT_RM_EMMITER_GPIO_Port,    RIGHT_RM_EMMITER_Pin,    &hadc4,    ADC_CHANNEL_3 }, /* recv PB12=ADC4_IN3 */
};

/* Reflective read of one sensor: emitter OFF (ambient) then ON (ambient + IR);
   reflected = lit - ambient. Only one emitter is ever on at a time, so there's
   no optical crosstalk between sensors. Emitter assumed active-HIGH. */
static void IR_Read(const IR_Sensor *s, uint16_t *amb, uint16_t *lit)
{
  HAL_GPIO_WritePin(s->em_port, s->em_pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  *amb = ADC_ReadAvg(s->adc, s->channel, IR_AVG_SAMPLES);

  HAL_GPIO_WritePin(s->em_port, s->em_pin, GPIO_PIN_SET);
  HAL_Delay(1);                                   /* let LED + receiver settle */
  *lit = ADC_ReadAvg(s->adc, s->channel, IR_AVG_SAMPLES);

  HAL_GPIO_WritePin(s->em_port, s->em_pin, GPIO_PIN_RESET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t last_led = 0;     /* heartbeat LED scheduler */
  uint8_t  led_step = 0;
  uint32_t last_sens = 0;    /* IR sensor sweep scheduler (~100 Hz) */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM15_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_ADC4_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_QUADSPI1_Init();
  /* USER CODE BEGIN 2 */
  DBG_Init();
  LOG("\r\n=== Nemisis boot ===\r\n");
  LOG("build: " __DATE__ " " __TIME__ "\r\n");   /* prove which binary is running */

  /* --- Persistent maze / speed-run memory (internal flash) --------------
     Reads the reset cause FIRST (before anything clears the flags): a cold
     power-on / battery swap KEEPS the last saved run, a warm NRST-button reset
     ERASES it. Then loads any saved map so a long-press fast run is available
     after a power cycle. Must run before the launcher/solver use the map. */
  MazeStore_Init();

  /* Recompute SystemCoreClock straight from the RCC registers, then dump the
     actual PLL config so we can see what the silicon is really doing. */
  SystemCoreClockUpdate();
  LOG("SYSCLK = %lu Hz  (HSE_VALUE=%lu)\r\n",
      (unsigned long)SystemCoreClock, (unsigned long)HSE_VALUE);
  LOG("PLLCFGR=0x%08lX  PLLM=%lu PLLN=%lu PLLR=%lu\r\n",
      (unsigned long)RCC->PLLCFGR,
      (unsigned long)(((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U),
      (unsigned long)((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos),
      (unsigned long)((((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1U) * 2U));
  LOG("CR=0x%08lX  HSERDY=%lu PLLRDY=%lu  CFGR_SWS=%lu (3=PLL)\r\n",
      (unsigned long)RCC->CR,
      (unsigned long)((RCC->CR & RCC_CR_HSERDY) ? 1U : 0U),
      (unsigned long)((RCC->CR & RCC_CR_PLLRDY) ? 1U : 0U),
      (unsigned long)((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos));

  Buzzer_Init();
  WS2812_Init(&htim15, TIM_CHANNEL_1);

  /* cool power-on chime */
  Buzzer_StartupSound();

  /* --- ICM-42688-P IMU bring-up check (SPI1) ---------------------------- */
  bool imu_ok = ICM42688_Init(&hspi1);       /* soft reset + WHO_AM_I + power on */
  uint8_t imu_who = ICM42688_ReadReg(&hspi1, 0x75);  /* read it back for the log */
  LOG("IMU WHO_AM_I = 0x%02X (expect 0x%02X) -> %s\r\n",
      imu_who, ICM42688_WHOAMI,
      imu_ok ? "OK" : "FAILED (check wiring/CS/SPI cfg)");

  /* --- I2C3 bus scan (diagnostic): list every 7-bit address that ACKs --- */
  LOG("I2C3 scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 128; a++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(a << 1), 2, 5) == HAL_OK)
    {
      LOG(" 0x%02X", a);
      found++;
    }
  }
  LOG("  (%u device(s))\r\n", found);

  /* --- MAX17049 fuel gauge bring-up check (I2C3) ------------------------ */
  bool fuel_ok = MAX17049_Probe(&hi2c3);
  if (fuel_ok)
  {
    MAX17049_Status s;
    MAX17049_ReadStatus(&hi2c3, &s);
    LOG("FUEL ver=0x%04X  pack=%ld mV  soc=%ld.%02ld %%  rate=%ld m%%/hr -> OK\r\n",
        s.version, (long)(s.pack_v * 1000.0f),
        (long)s.soc, (long)((s.soc - (long)s.soc) * 100.0f),
        (long)(s.crate * 1000.0f));
  }
  else
  {
    LOG("FUEL gauge: NO ACK (check I2C3 wiring / pull-ups / battery)\r\n");
  }

  /* --- AS5047P encoders bring-up check (shared SPI3, separate CS) -------
     Both encoders sit on SPI3; left CS=PA4, right CS=PB2. Each read pulses
     only its own CS, so they coexist on the one bus. */
  AS5047P_Handle encL, encR;
  AS5047P_Init(&encL, &hspi3, ENCODERL_CS_GPIO_Port, ENCODERL_CS_Pin);
  AS5047P_Init(&encR, &hspi3, ENCODERR_CS_GPIO_Port, ENCODERR_CS_Pin);
  bool encL_ok = AS5047P_Probe(&encL);
  bool encR_ok = AS5047P_Probe(&encR);
  LOG("ENCODERL raw=%u -> %s\r\n", AS5047P_ReadAngleRaw(&encL),
      encL_ok ? "OK" : "bad frame (check CS PA4 / MISO / power)");
  LOG("ENCODERR raw=%u -> %s\r\n", AS5047P_ReadAngleRaw(&encR),
      encR_ok ? "OK" : "bad frame (check CS PB2 / MISO / power)");

  /* ABI quadrature cross-check: left A/B (PC0/PA9) -> TIM1, right A/B
     (PA1/PA5) -> TIM2. Start both counters from zero. */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  /* --- All 6 IR reflectance sensors bring-up --------------------------
     Calibrate both ADCs (ADC4 for RIGHT_RM, ADC1 for the other five), then do
     one reflective read of each. With nothing in front 'refl' is small; bring
     a hand/wall close to a given sensor and only that one's 'refl' should jump. */
  HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  for (int i = 0; i < IR_COUNT; i++)
  {
    uint16_t amb, lit;
    IR_Read(&ir_sensors[i], &amb, &lit);
    int delta = (int)lit - (int)amb;
    LOG("IR %-4s: ambient=%4u lit=%4u reflected=%5d -> %s\r\n",
        ir_sensors[i].name, amb, lit, delta,
        (delta > 20 || delta < -20) ? "responding" : "weak/none (check wiring/polarity)");
  }

  /* --- Vacuum fan bring-up (PB9 = TIM8_CH3 PWM, MOSFET-driven) ----------
     TIM8 is an advanced timer, so HAL_TIM_PWM_Start also flips the main output
     enable (MOE) for us. Period=65535, so CCR = duty% * 65535 / 100. Start at
     0% (fan off) and ramp it in the loop below so you can hear/feel each step.
     WARNING: secure the robot - the fan pulls air hard at 100%. */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
  LOG("VACUUM FAN on PB9 (TIM8_CH3) ready - ramping 0/25/50/75/100%% in loop\r\n");

  LOG("init done\r\n");

  /* --- DRV8874 motor movement test (M1: PC8/PC7, M2: PC6/PB7) -----------
     Runs ONCE at boot: both forward, both reverse, pivot one way, pivot the
     other, then stops. WARNING: the robot WILL drive - put it on a stand or
     in clear space first. 3 s grace delay below so you can let go. Watch RTT
     for the per-phase current sense. Set this #if to 0 once verified. */
/* Motors are now driven interactively from the console ('motor ...' /
   'motor test'); the old auto-drive-at-boot is disabled for safety. */
#if 0
  Motor_TestInit(&hadc4);
  LOG("MOTOR demo starting in 3 s - clear the bench / put on stand...\r\n");
  HAL_Delay(3000);
  Motor_DriveSequence(&hadc4);
#endif

  /* --- Interactive debug console (RTT + USART1->ESP32) ------------------
     Type 'help' over J-Link RTT Viewer (or the ESP32/WiFi link) to drive any
     subsystem on demand. This replaces the old fixed demo loop. */
  ConsoleCtx con = {
    .hspi_imu  = &hspi1,
    .hspi_enc  = &hspi3,
    .hi2c_fuel = &hi2c3,
    .hadc1     = &hadc1,
    .hadc4     = &hadc4,
    .htim_fan  = &htim8,
    .htim_mot  = &htim3,
    .htim_encL = &htim1,
    .htim_encR = &htim2,
    .huart     = &huart1,
    .hqspi     = &hqspi1,
  };
  Console_Init(&con);

  /* --- Battery safety monitor (MAX17049 on I2C3) -----------------------
     Watches pack voltage; blocks + kills motors/vacuum when CRITICAL and
     signals state on the RGB LED + buzzer. Voltage-based, not SOC-based.
     Must come after WS2812/Buzzer init and the fan PWM start. */
  Battery_Init(&hi2c3, &htim8, TIM_CHANNEL_3);
  LOG("BATTERY monitor armed (crit<%umV low<%umV) - state=%s\r\n",
      (unsigned)BATTERY_CRIT_MV, (unsigned)BATTERY_LOW_MV,
      Battery_StateName(Battery_State()));

  /* --- IR wall sensors (6x reflectance on ADC1/ADC4) -------------------
     Acquisition layer for wall detection + corridor centring. Swept from the
     main loop below (~100 Hz); ADCs are already calibrated in Console_Init. */
  Sensors_Init(&hadc1, &hadc4);
  Sensors_CalLoad();                 /* restore wall-sensor calibration from flash */
  LOG("SENSORS armed (6x IR, ~100 Hz sweep) - 'walls' to view\r\n");

  /* --- Offline race-start launcher (button PD2 + front-sensor gesture) ---
     Press the button to go fully offline (mutes the WiFi link so it can't jitter
     the loop), wave a hand over the two front sensors to arm, 3 s countdown, then
     it runs the solve with no app attached. Button again = cancel/abort. */
  Launcher_Init();
  LOG("LAUNCHER ready - press button for offline run (hand over front IR to go)\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Non-blocking: parse any command from RTT/USART1 and stream live samples
       for whichever subsystem is active. All interactive debug lives here. */
    Console_Task();

    /* IR wall-sensor sweep at ~200 Hz (every 5 ms). Polling for now; the 1 kHz
       control loop preempts it so its timing is unaffected. (Restored to 200 Hz
       for lower centring lag now that the boot hang - a UART TX-ring deadlock,
       unrelated to the sweep rate - is fixed. ~4 mm lag at mapping speed. The
       real low-lag fix is still the DMA transport swap for the speed run - see
       the navigator-cell-motion / ir-wall-sensors notes / fast-run-dma reminder.) */
    /* Speed-adaptive IR: faster sweep (fewer samples) + more often when moving
       fast, to cut the position lag that makes fast runs drift; clean + 5 ms when
       slow/stationary. */
    float wcps = Control_GetWheelSpeedCps();
    Sensors_AdaptTiming(wcps);
    /* Drop to the 3 ms (effectively back-to-back) sweep rate from 6000 cps so the
       8000 cps SEARCH speed also gets the fresher reads on a cell approach - cuts
       front-wall detection latency (~5->3 ms) so the front-stop brakes sooner and
       stops bumping walls. Per-read timing is unchanged (AdaptTiming stays off),
       so the readings still match the stationary calibration. */
    uint32_t sweep_ms = (wcps > 6000.0f) ? 3U : 5U;
    if ((HAL_GetTick() - last_sens) >= sweep_ms)
    {
      last_sens = HAL_GetTick();
      Sensors_Update();               /* blocking sweep (no-op when async is on)   */
    }
    /* Non-blocking background sweep: pumped EVERY pass (ungated). Owns the sweep
       when 'irasync on', else returns immediately. This is the DMA-alternative -
       it frees the ~3.4 ms the blocking sweep spins without any DMA/timer/IRQ. */
    Sensors_Pump();

    /* Battery safety: poll the gauge, run the cut-off FSM, drive the LED/buzzer
       warning. Owns the RGB LED whenever the pack isn't healthy. */
    Battery_Task();

    /* Offline race-start sequencer: button -> mute WiFi -> hand gesture ->
       countdown -> solve. Owns the LEDs while armed/counting/running. */
    Launcher_Task();

    /* Heartbeat: cycle the RGB LED dimly once a second so you can see the
       firmware is alive without spamming the console. Skipped while the battery
       monitor owns the LED (LOW/CRITICAL) or the offline launcher owns it. */
    if (Battery_State() == BATT_OK && !Launcher_Active() &&
        (HAL_GetTick() - last_led) >= 1000U)
    {
      last_led = HAL_GetTick();
      switch (led_step)
      {
        case 0:  WS2812_SetAll(8, 0, 0); break;   /* red   */
        case 1:  WS2812_SetAll(0, 8, 0); break;   /* green */
        default: WS2812_SetAll(0, 0, 8); break;   /* blue  */
      }
      WS2812_Show();
      led_step = (led_step + 1U) % 3U;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  /* HARDWARE OVERSAMPLER (IR-acquisition Increment A, added by hand - re-apply if
     you regen from CubeMX). 8x average with a 3-bit right shift keeps the 12-bit
     scale, so one conversion returns the mean of 8 samples in hardware. Replaces
     the 8x software poll loop in sensors.c (adc_read_avg) -> same averaged value,
     far less CPU per sweep. Also averages the DRV8874 current sense on ADC4 (a
     free win, scale unchanged). */
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  /* HARDWARE OVERSAMPLER - see the ADC1 note above (IR Increment A, re-apply after
     a CubeMX regen). R_RM lives on ADC4; the motor current sense shares it and just
     gets a cleaner averaged reading (same scale). */
  hadc4.Init.OversamplingMode = ENABLE;
  hadc4.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc4.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc4.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc4.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x30D29DE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief QUADSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI1_Init(void)
{

  /* USER CODE BEGIN QUADSPI1_Init 0 */

  /* USER CODE END QUADSPI1_Init 0 */

  /* USER CODE BEGIN QUADSPI1_Init 1 */

  /* USER CODE END QUADSPI1_Init 1 */
  /* QUADSPI1 parameter configuration*/
  hqspi1.Instance = QUADSPI;
  hqspi1.Init.ClockPrescaler = 3;
  hqspi1.Init.FifoThreshold = 4;
  hqspi1.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi1.Init.FlashSize = 21;
  hqspi1.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
  hqspi1.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi1.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi1.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI1_Init 2 */

  /* USER CODE END QUADSPI1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 2-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEFT_FRONT_EMMITER_Pin|LEFT_M_EMMITER_Pin|LEFT_LM_EMMITER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENCODERL_CS_Pin|RIGHT_M_EMMITER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENCODERR_CS_Pin|RIGHT_RM_EMMITER_Pin|RIGHT_FRONT_EMMITER_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEFT_FRONT_EMMITER_Pin LEFT_M_EMMITER_Pin LEFT_LM_EMMITER_Pin */
  GPIO_InitStruct.Pin = LEFT_FRONT_EMMITER_Pin|LEFT_M_EMMITER_Pin|LEFT_LM_EMMITER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODERL_CS_Pin RIGHT_M_EMMITER_Pin */
  GPIO_InitStruct.Pin = ENCODERL_CS_Pin|RIGHT_M_EMMITER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODERR_CS_Pin RIGHT_RM_EMMITER_Pin RIGHT_FRONT_EMMITER_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = ENCODERR_CS_Pin|RIGHT_RM_EMMITER_Pin|RIGHT_FRONT_EMMITER_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RIGHT_RM_RECEIVER_Pin */
  GPIO_InitStruct.Pin = RIGHT_RM_RECEIVER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RIGHT_RM_RECEIVER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
