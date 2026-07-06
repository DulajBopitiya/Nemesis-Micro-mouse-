/**
 ******************************************************************************
 * @file    motor.h
 * @brief   Bench-test driver for the two TI DRV8874PWPR H-bridges.
 *
 * Pin map (from Core/Inc/main.h, set by CubeMX):
 *
 *   Motor 1 :  M1_IN1 = PC8 ,  M1_IN2 = PC7 ,  current sense = PB15 (ADC4_IN5)
 *   Motor 2 :  M2_IN1 = PC6 ,  M2_IN2 = PB7 ,  current sense = PB14 (ADC4_IN4)
 *
 * IMPORTANT - control mode
 *   This driver assumes the DRV8874 is strapped into PWM mode (PMODE pin), so
 *   IN1/IN2 are the two independent half-bridge inputs:
 *
 *       IN1   IN2    OUT1  OUT2   result
 *        0     0      Hi-Z  Hi-Z   coast  (fast decay / freewheel)
 *        1     0      H     L      forward
 *        0     1      L     H      reverse
 *        1     1      L     L      brake  (low-side, slow decay)
 *
 *   If your board instead straps PH/EN mode, IN1 = EN (PWM speed) and
 *   IN2 = PH (direction) - tell me and I'll swap the truth table.
 *
 * IMPORTANT - no hardware PWM
 *   CubeMX assigned the four IN pins as plain GPIO push-pull outputs, NOT timer
 *   channels. Speed control here is done by *software* PWM (bit-banged at 1 kHz
 *   using the DWT cycle counter). It is blocking and meant for bench testing.
 *   For real running you'll want to remap the IN pins onto timer PWM channels
 *   in CubeMX.
 ******************************************************************************
 */
#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"          /* HAL + the M1_/M2_ pin macros */
#include <stdint.h>

typedef enum
{
  MOTOR_1 = 0,             /* PC8/PC7 , sense PB15 (ADC4_IN5) */
  MOTOR_2 = 1              /* PC6/PB7 , sense PB14 (ADC4_IN4) */
} MotorId;

/** Initialise: starts DWT timing, calibrates the ADC, coasts both motors.
 *  Pass the ADC4 handle (declared in main.c as `hadc4`). */
void Motor_TestInit(ADC_HandleTypeDef *hadc);

/* === hardware PWM drive (the real, non-blocking path) =====================
 *  After the CubeMX remap the four IN pins are TIM3 channels, not GPIO:
 *      M1_IN1 = PC8 = TIM3_CH3      M1_IN2 = PC7 = TIM3_CH2
 *      M2_IN1 = PC6 = TIM3_CH1      M2_IN2 = PB7 = TIM3_CH4
 *  Motor_PWM_Init() takes over TIM3: it forces a clean ~20 kHz period, fixes
 *  the channel that CubeMX generated as plain "output compare/timing" back to
 *  PWM, and starts all four channels coasting (0 % duty).
 *
 *  Once this is called, Motor_Coast/Forward/Reverse/Brake and the *_SoftPWM /
 *  DriveBoth helpers all route through the timer instead of bit-banging GPIO,
 *  so they keep working from the console AND the 'stop' command genuinely cuts
 *  drive (vital: with AF pins, a GPIO write no longer stops a motor).
 *
 *  Drive is sign-magnitude / fast-decay: for forward we PWM IN1 and hold IN2
 *  low; for reverse we PWM IN2 and hold IN1 low; 0 = both low = coast. */
#define MOTOR_PWM_MAX   8000        /* TIM3 ARR top -> 160 MHz / 8000 = 20 kHz */

/** Bind TIM3, fix its config, and start both motors coasting. Call once. */
void Motor_PWM_Init(TIM_HandleTypeDef *htim3);

/** Set a motor's signed duty directly: -MOTOR_PWM_MAX..+MOTOR_PWM_MAX.
 *  >0 forward, <0 reverse, 0 coast. Non-blocking - this is what the control
 *  loop calls every tick. Out-of-range values are clamped. */
void Motor_SetDuty(MotorId m, int duty);

/* --- low-level helpers (full drive, no PWM) ------------------------------- */
void Motor_Coast(MotorId m);    /* IN1=0 IN2=0 */
void Motor_Brake(MotorId m);    /* IN1=1 IN2=1 */
void Motor_Forward(MotorId m);  /* IN1=1 IN2=0 - full speed */
void Motor_Reverse(MotorId m);  /* IN1=0 IN2=1 - full speed */

/** Software-PWM drive, blocking for `ms` milliseconds.
 *  speed: -100..+100 (% duty). >0 = forward, <0 = reverse, 0 = coast. */
void Motor_SoftPWM(MotorId m, int speed, uint32_t ms);

/* --- current sense -------------------------------------------------------- */
/** Raw 12-bit ADC reading of the motor's IPROPI sense voltage (averaged). */
uint16_t Motor_ReadSenseRaw(ADC_HandleTypeDef *hadc, MotorId m);
/** Sense voltage in millivolts. */
uint32_t Motor_ReadSense_mV(ADC_HandleTypeDef *hadc, MotorId m);
/** Estimated motor current in mA (see scaling macros in motor.c - CALIBRATE!). */
int32_t  Motor_ReadCurrent_mA(ADC_HandleTypeDef *hadc, MotorId m);

/** Full automated test sequence (coast/fwd ramp/reverse/brake) for both
 *  motors, logging current sense over RTT at each step. Blocking. */
void Motor_TestRun(ADC_HandleTypeDef *hadc);

/** Software-PWM BOTH motors at once (same duty, independent direction) for
 *  `ms` milliseconds, blocking. dir1/dir2: +1 forward, -1 reverse, 0 coast.
 *  duty: 0..100 %. Lets you drive both wheels together for straight/pivot. */
void Motor_DriveBoth(int dir1, int dir2, int duty, uint32_t ms);

/** Movement demo: both forward, both reverse, pivot one way, pivot the other,
 *  then stop (coast). Logs current sense per phase. Blocking, runs once.
 *  Put the robot on a stand the first time. */
void Motor_DriveSequence(ADC_HandleTypeDef *hadc);

/** Diagnostic: holds each drive state STATICALLY (~4 s) and loops forever, so
 *  you can probe pins with a multimeter. Never returns. Use when the motors
 *  don't move to find out where the signal/power chain breaks. */
void Motor_DiagLoop(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
