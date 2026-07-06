/**
 ******************************************************************************
 * @file    status_led.cpp
 * @brief   WS2812 status indicator - see status_led.h.
 ******************************************************************************
 */
#include "status_led.h"

namespace StatusLed {
namespace {

int      s_pin     = -1;
uint8_t  s_bright  = 64;        /* master brightness 0..255 */
Net      s_net     = Net::Booting;
bool     s_client  = false;
bool     s_offline = false;
uint32_t s_lastActivity = 0;

/* Cache the last colour actually pushed so we only hit the RMT when it
   changes - steady states then cost nothing per loop. */
int s_lastR = -1, s_lastG = -1, s_lastB = -1;

const uint32_t ACTIVITY_HOLD_MS = 150;   /* keep "streaming" this long after data */

inline uint8_t scale(uint8_t v)
{
  return (uint8_t)(((uint16_t)v * s_bright) / 255U);
}

void show(uint8_t r, uint8_t g, uint8_t b)
{
  int rr = scale(r), gg = scale(g), bb = scale(b);
  if (rr == s_lastR && gg == s_lastG && bb == s_lastB) return;
  s_lastR = rr; s_lastG = gg; s_lastB = bb;
  if (s_pin >= 0) neopixelWrite((uint8_t)s_pin, (uint8_t)rr, (uint8_t)gg, (uint8_t)bb);
}

} // namespace

void begin(int pin, uint8_t brightness)
{
  s_pin    = pin;
  s_bright = brightness;
  s_lastR  = s_lastG = s_lastB = -1;   /* force the first show() to write */
  show(0, 0, 0);
}

void setNet(Net n)            { s_net = n; }
void setClient(bool connected){ s_client = connected; }
void setOffline(bool offline) { s_offline = offline; }
void notifyActivity()         { s_lastActivity = millis(); }

void task()
{
  uint32_t now = millis();
  bool fast = (now / 60)  & 1U;   /* ~8 Hz shimmer for data activity */
  bool slow = (now / 500) & 1U;   /* 1 Hz blink/pulse for waiting     */

  /* OFFLINE overrides everything: the mouse muted comms for an autonomous run.
     Orange "heartbeat" - two quick pips then a rest - reads clearly as "busy,
     running on its own" and is unlike any other state here. */
  if (s_offline)
  {
    uint32_t ph = now % 1500;                    /* ~0.67 Hz heartbeat */
    bool pip = (ph < 90) || (ph >= 210 && ph < 300);   /* blip .. blip .... */
    show(pip ? 255 : 0, pip ? 60 : 0, 0);        /* orange */
    return;
  }

  /* Connected to the app takes priority over the WiFi-level state. */
  if (s_client)
  {
    bool streaming = (now - s_lastActivity) < ACTIVITY_HOLD_MS;
    if (streaming)
      show(0, fast ? 220 : 70, fast ? 90 : 25);   /* green/cyan shimmer */
    else
      show(0, 90, 0);                              /* steady green       */
    return;
  }

  switch (s_net)
  {
    case Net::Booting:                             /* amber blink */
      show(slow ? 90 : 0, slow ? 55 : 0, 0);
      break;
    case Net::ButtonWindow:                        /* magenta breathing fade */
    {
      /* Smooth triangle breathe (~1.4 s period) so it's obviously different
         from every blink state - this is your "press BOOT now" cue. */
      const uint32_t period = 1400;
      uint32_t ph = now % period;
      uint16_t level = (ph < period / 2)                 /* 0..255 ramp up/down */
                         ? (uint16_t)(ph * 255 / (period / 2))
                         : (uint16_t)((period - ph) * 255 / (period / 2));
      uint8_t r = (uint8_t)((uint16_t)150 * level / 255);
      uint8_t b = (uint8_t)((uint16_t)150 * level / 255);
      show(r, 0, b);
      break;
    }
    case Net::StaConnected:                        /* blue pulse: on net, waiting */
      show(0, 0, slow ? 110 : 10);
      break;
    case Net::Provisioning:                        /* cyan blink: setup portal up */
      show(0, slow ? 130 : 0, slow ? 130 : 0);
      break;
    case Net::Error:                               /* red blink */
      show(slow ? 140 : 0, 0, 0);
      break;
  }
}

} // namespace StatusLed
