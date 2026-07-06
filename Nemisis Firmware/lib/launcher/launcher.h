/**
 ******************************************************************************
 * @file    launcher.h
 * @brief   Offline race-start sequencer for the Nemisis micromouse.
 *
 * Lets the mouse run a full solve with NO app / WiFi attached, so the comms
 * bridge can't add latency or jitter to the main loop (the reason a connected
 * run sometimes misbehaves). Driven entirely from hardware:
 *
 *   1. Press the BUTTON (PD2)        -> go OFFLINE: the USART1/WiFi link is muted
 *                                       (Console_SetMuted), any active run is
 *                                       stopped, LEDs turn amber + a chirp.
 *   2. Wave a hand over the two front
 *      sensors (L_F + R_F)           -> ARM: a low->high cover is the deliberate
 *                                       trigger (a static front wall won't do it).
 *   3. ~3 s countdown                -> LEDs count down blue with a beep/second.
 *   4. GO                            -> Solver_Start() runs the maze fully offline.
 *
 * Press the button again at any point to cancel / abort and bring the link back.
 *
 * NOTE: there is no WiFi/BLE enable line wired to the STM32, so "offline" means
 * the STM32 stops talking to the ESP32 (mutes USART1). To truly power down the
 * ESP radios you'd need an enable GPIO or an ESP-side command - a later add.
 * Data logging is also deferred (hook it into LA_RUN when wanted).
 *
 * Usage (main.c): Launcher_Init() once after Sensors_Init(); Launcher_Task()
 * every superloop pass; gate the heartbeat LED on !Launcher_Active().
 ******************************************************************************
 */
#ifndef LAUNCHER_H
#define LAUNCHER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/** Configure the button input and reset the sequencer. Call once at boot, after
 *  Sensors_Init() (the hand gesture reads the front IR). */
void Launcher_Init(void);

/** Pump the offline-start state machine. Call every main-loop pass. Non-blocking
 *  except for the brief feedback tones, which only play while the robot is
 *  stationary (arming / countdown / finish). */
void Launcher_Task(void);

/** True whenever the offline sequencer owns the robot (armed, counting down, or
 *  running) - used by main.c to yield the heartbeat LED. */
bool Launcher_Active(void);

#ifdef __cplusplus
}
#endif

#endif /* LAUNCHER_H */
