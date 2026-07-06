/**
 ******************************************************************************
 * @file    wifi_provision.h
 * @brief   Runtime WiFi provisioning for the Nemisis ESP32-C3 bridge.
 *
 * Lets the mouse join ANY local router without reflashing the firmware.
 * Credentials (SSID/password) live in NVS (Preferences, namespace "wifi"),
 * not in source code. Three ways to (re)configure them:
 *
 *   1. First boot ever (NVS empty): main.cpp seeds a compiled-in default so
 *      the bench setup keeps working out of the box.
 *   2. Power up, then PRESS the board's BOOT button within a few seconds:
 *      forces the portal even if stored credentials are still valid (e.g.
 *      moving to a new venue). NB: don't HOLD BOOT through reset - GPIO9 is a
 *      strapping pin, so that just drops the C3 into serial-download mode and
 *      the firmware never runs. See main.cpp's BOOT_BUTTON_PIN note.
 *   3. From the Python app, while still connected to the OLD network: send
 *      the '##WIFI_SETUP##' command over the existing debug link. See
 *      main.cpp's loop() for where that's intercepted.
 *
 * The portal itself is a classic "WiFiManager"-style captive portal: the C3
 * hosts an open AP, a DNS server that answers every query with its own IP
 * (so phones pop the "sign in to network" prompt automatically), and a tiny
 * web page listing nearby networks to choose from (or type one manually).
 * Saving reboots the board, which then joins the new network normally.
 ******************************************************************************
 */
#pragma once
#include <Arduino.h>

namespace WifiProvision {

struct Credentials {
  String ssid;
  String pass;
};

/** Open the "wifi" NVS namespace. Call once in setup() before anything else
 *  in this namespace. */
void begin();

/** Load stored STA credentials into `out`. Returns true if a non-empty SSID
 *  was found (false on a genuinely first-ever boot). */
bool load(Credentials &out);

/** Persist STA credentials, overwriting any previous value. */
void save(const String &ssid, const String &pass);

/** Erase stored STA credentials (forces a first-boot condition next time). */
void clearCredentials();

/** One-shot "please reopen the portal on next boot" flag. Used by the
 *  in-band '##WIFI_SETUP##' command so the request survives the reboot it
 *  triggers. Set it, then ESP.restart(); on the next boot call
 *  consumePortalRequest() once (it clears itself so a later power-cycle
 *  doesn't get stuck re-provisioning). */
void requestPortalOnNextBoot();
bool consumePortalRequest();

/** Hosts the setup AP + captive portal + web page until the user saves new
 *  credentials, then persists them and calls ESP.restart(). Never returns.
 *  apPass may be "" / nullptr for an open network (simplest for onboarding). */
void runPortal(const char *apSsid, const char *apPass);

/** Non-blocking: drain the USB serial monitor for line-based provisioning
 *  commands (SSID=/PASS=/SHOW/SAVE) and act on them. Call every loop() pass so
 *  WiFi can be (re)configured over USB at ANY time - no AP/phone/DHCP needed,
 *  which is the reliable fallback when the captive portal misbehaves. On SAVE
 *  it persists to NVS and reboots. Safe to call from both the normal bridge
 *  loop and the portal loop; it only reads USB serial input, nothing else uses
 *  it. */
void serialTask();

} // namespace WifiProvision
