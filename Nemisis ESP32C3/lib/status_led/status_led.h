/**
 ******************************************************************************
 * @file    status_led.h
 * @brief   Onboard WS2812 RGB status indicator for the Nemisis ESP32-C3 bridge.
 *
 * Drives the single addressable RGB LED (WS2812B-compatible, same family as the
 * ones on the STM32 board) to show the bridge's live state at a glance:
 *
 *   amber  blink    booting / joining WiFi
 *   magenta fade    BOOT-button window open (press BOOT now to open WiFi setup)
 *   cyan   blink    WiFi setup portal active (waiting for you to pick a router)
 *   blue   pulse    on the network, waiting for the Python app to connect
 *   green  steady   app (TCP client) connected, link idle
 *   green  shimmer  app connected AND data is streaming through the bridge
 *   red    blink    error (WiFi lost)
 *   orange double-pip  mouse is OFFLINE (comms muted for an autonomous run);
 *                      overrides the states above until it comes back online
 *
 * Uses the Arduino-ESP32 core's built-in RMT helper (neopixelWrite), so there's
 * no external library dependency. Non-blocking: call task() every loop().
 *
 * The LED data line is on GPIO2 on this board. GPIO2 is a boot strapping pin on
 * the ESP32-C3 (sampled at reset); the WS2812 input is high-impedance so it
 * doesn't disturb boot, but don't add a hard pull-down here.
 ******************************************************************************
 */
#pragma once
#include <Arduino.h>

namespace StatusLed {

/* WiFi-level state of the bridge. */
enum class Net : uint8_t {
  Booting,        /* powering up / trying to join stored WiFi     */
  ButtonWindow,   /* post-boot window: press BOOT to reconfigure   */
  StaConnected,   /* joined WiFi (Station mode)                   */
  Provisioning,   /* hosting the WiFi setup captive portal        */
  Error,          /* WiFi lost after coming up                    */
};

/** Bind the LED to a GPIO and set master brightness (0..255, kept low so the
 *  onboard LED isn't blinding). Call once in setup(). */
void begin(int pin, uint8_t brightness = 64);

/** Report the current WiFi-level state. */
void setNet(Net n);

/** Report whether a TCP debug client (the app) is connected. */
void setClient(bool connected);

/** Mouse went offline (comms muted for a jitter-free autonomous run) or came
 *  back online. While offline the LED shows a distinctive orange double-pip
 *  heartbeat that OVERRIDES the net/client states, so you can see at a glance
 *  the mouse is running on its own. Driven by the STM32's ##OFFLINE## /
 *  ##ONLINE## markers (see main.cpp). */
void setOffline(bool offline);

/** Call whenever bytes were bridged in either direction; drives the "streaming"
 *  shimmer. Cheap — just stamps a timestamp. */
void notifyActivity();

/** Non-blocking animator. Call every loop() pass. */
void task();

} // namespace StatusLed
