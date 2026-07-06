/**
 ******************************************************************************
 * @file    main.cpp
 * @brief   Nemisis ESP32-C3 WiFi <-> UART transparent bridge.
 *
 * Sits between the STM32G474 debug console (on its USART1) and a WiFi TCP
 * client (the Python debug app, ../NEMISIS PYTHON DEBUG APPLICATION). Whatever
 * the PC sends over TCP is forwarded byte-for-byte to the STM32, and everything
 * the STM32 prints is forwarded back. The STM32 firmware can't tell whether
 * commands arrive from the J-Link RTT terminal or from here - same text
 * protocol either way (see lib/console in the STM32 firmware).
 *
 *  WIFI CREDENTIALS: no reflash needed to join a different router.
 *    Station-mode SSID/password live in NVS (lib/wifi_provision), not in this
 *    source file. Three ways to (re)configure them:
 *      1. First boot ever (fresh board): DEFAULT_STA_SSID/PASS below are
 *         seeded into NVS once, so the bench setup works out of the box.
 *      2. Power up normally, then PRESS the board's BOOT button (GPIO9) within
 *         a few seconds: forces the WiFi setup portal even if the stored
 *         network still works - use this at a new venue/competition to point
 *         the mouse at a different router. (Do NOT hold BOOT during power-up:
 *         GPIO9 is a strapping pin, so holding it at reset drops the C3 into
 *         serial-download mode and the firmware never runs - no portal, no AP.)
 *      3. From the Python app, while still connected over the OLD network:
 *         send the command "##WIFI_SETUP##" - the bridge intercepts it (it's
 *         never forwarded to the STM32), reboots, and opens the portal.
 *    The portal itself: the C3 hosts an open AP ("NEMISIS-SETUP") + a captive
 *    DNS trick, so a phone/laptop that joins it gets a "sign in to network"
 *    prompt with a page listing nearby WiFi networks to pick from. Saving
 *    reboots the board, which joins the new network and resumes bridging.
 *
 *  WIRING (ESP32-C3  <->  STM32G474 USART1, 115200 8N1):
 *      ESP RX = GPIO8   <----  STM32 USART1 TX (PC4)
 *      ESP TX = GPIO10  ---->  STM32 USART1 RX (PC5)
 *      GND    <-------------->  GND   (common ground REQUIRED)
 *
 *  NOTE: Serial (USB-CDC) is the status/monitor port; Serial1 is the STM32 link.
 *  GPIO8 is a boot strapping pin on the C3 (must be HIGH at reset for a normal
 *  flash boot). It's the UART RX here, driven by the STM32 TX which idles HIGH,
 *  so boot is normally fine; if the board ever refuses to boot with the STM32
 *  attached, add a pull-up on GPIO8 or move RX to another free GPIO and update
 *  UART_RX_PIN below.
 *
 *  LINK RELIABILITY (2026-07-06): traced intermittent log/telemetry loss to
 *  this bridge, not the STM32 or the Python app. Two fixes:
 *    1. WiFiClient::write() can silently accept FEWER bytes than asked when
 *       the TCP send buffer is momentarily full (weak 2.4GHz link) - the old
 *       code ignored the return value and threw the remainder away. Now
 *       writeAllToClient() retries until every byte is sent, and only gives
 *       up (dropping the client) after CLIENT_WRITE_TIMEOUT_MS of total
 *       stall, so a genuinely dead client can't wedge the bridge forever.
 *    2. Serial1's default RX ring is tiny (~256 B, ~22 ms at 115200 baud) -
 *       any brief stall while (1) was blocking could overflow it before this
 *       loop got back around to draining it. Bumped to UART_RX_BUF_SZ bytes
 *       (~700 ms of headroom) via setRxBufferSize() before LINK.begin().
 ******************************************************************************
 */
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "status_led.h"
#include "wifi_provision.h"

/* ----------------------------- config ----------------------------------- */
/* Seed values used ONLY the very first time the board boots (NVS empty).
   After that, WiFi credentials live in NVS and are changed via the portal -
   editing these and reflashing is no longer required for a network change. */
static const char *DEFAULT_STA_SSID = "Dialog 5G 753";
static const char *DEFAULT_STA_PASS = "335e3359";

/* mDNS hostname: reach the bridge at  nemisis.local:3333  on networks that
   support mDNS (avoids caring about the DHCP-assigned IP). */
static const char *MDNS_HOST = "nemisis";

/* WiFi setup portal's own access point (what you join to reconfigure). Open
   network keeps onboarding simple; give it a password below if that's a
   concern in your environment. */
static const char *PORTAL_AP_SSID = "NEMISIS-SETUP";
static const char *PORTAL_AP_PASS = "";                 /* "" = open */

static const uint16_t TCP_PORT = 3333;
static const uint32_t STA_TIMEOUT_MS = 12000;   /* how long to try the stored WiFi */

static const int      UART_RX_PIN = 8;          /* <- STM32 TX (PC4), wired GPIO8  */
static const int      UART_TX_PIN = 10;         /* -> STM32 RX (PC5), wired GPIO10 */
static const uint32_t UART_BAUD   = 115200;
static const size_t   UART_RX_BUF_SZ = 8192;    /* cushion vs a stalled WiFi write */

/* How long a TCP write may stall (send buffer full) before we give up on that
   chunk and drop the client - bounded so a dead/vanished client can't wedge
   the whole bridge (and, transitively, back-pressure the STM32's UART TX
   ring) forever. */
static const uint32_t CLIENT_WRITE_TIMEOUT_MS = 500;

static const int      LED_PIN     = 2;          /* onboard WS2812 RGB status LED   */

/* ESP32-C3-DevKitM-1's BOOT button (GPIO9).
 *
 * IMPORTANT: GPIO9 is a BOOT STRAPPING pin. If it is held LOW at the moment of
 * reset, the ROM forces serial-download (flash) mode and the sketch never runs
 * - so "hold BOOT while power-cycling" can NEVER open the portal (the chip is
 * sitting in the bootloader, no firmware, no AP). Instead we boot normally
 * (GPIO9 idles HIGH via its pull-up) and then watch for a BOOT *press* during a
 * short window right after startup: power on, THEN tap/hold BOOT within a few
 * seconds to reconfigure WiFi. */
static const int      BOOT_BUTTON_PIN = 9;

/* How long after boot to watch for a BOOT-button press before giving up and
   joining the stored WiFi. The RGB LED breathes MAGENTA for this whole window
   (StatusLed::Net::ButtonWindow) so you can clearly see when a press will
   trigger the setup portal. */
static const uint32_t BOOT_BTN_WINDOW_MS = 5000;

/* Sent by the Python app over the existing TCP link to remotely trigger the
   portal (e.g. moving venues, current router unreachable from the app's
   side but the mouse can still hear it). Intercepted here, never forwarded
   to the STM32 console. Chosen to be nothing a console command would ever
   contain by accident. */
static const char *WIFI_SETUP_MAGIC = "##WIFI_SETUP##";

/* BENCH TEST HOOK (no maze / no STM32 run needed): the app's "Fake dump (ESP)"
   toggle sends these over the TCP link. The bridge SYNTHESISES a DUMP block and
   streams it straight to the app, so the log-fetch reliability + auto-refetch
   path can be exercised on the bench. Like WIFI_SETUP, these are swallowed here
   and never forwarded to the STM32 console.
     ##DUMPTEST##on / ##DUMPTEST##off  - enable / disable the mode
     ##DUMPTEST##       (bare)         - emit a fake dump: FIRST one after each
                                         enable arrives SHORT on purpose (drops
                                         ~1/3 of the rows) so the app detects the
                                         shortfall and auto re-fetches; the retry
                                         arrives in full -> COMPLETE.
     ##DUMPTEST##short / ##DUMPTEST##full - force a short / full dump (type in the
                                         app's command box to test the give-up vs
                                         clean-fetch paths directly). */
static const char *DUMPTEST_MAGIC = "##DUMPTEST##";
static const int   DUMPTEST_ROWS  = 200;        /* TRACE rows a full fake run announces */
static bool        g_dumpTestMode = false;      /* toggled by ##DUMPTEST##on/off */
static int         g_dumpTestAttempt = 0;       /* per-enable counter: 1st = short */

/* Markers the STM32 emits over USART1 (see Console_SetMuted in the STM32
   firmware's lib/console) right before it mutes comms for an offline autonomous
   run, and again when it comes back online. The bridge watches the STM32->PC
   stream for these and drives the status LED (orange heartbeat = offline). They
   still pass through to the app too - harmless, and a nice log cue. */
static const char *OFFLINE_MARK = "##OFFLINE##";
static const char *ONLINE_MARK  = "##ONLINE##";

#define LINK Serial1                            /* HardwareSerial to the STM32 */

/* ----------------------------- state ------------------------------------ */
WiFiServer server(TCP_PORT);
WiFiClient client;

static uint8_t buf[1024];
static String  g_staSsid;                       /* for printBanner */
static uint32_t g_droppedBytes = 0;             /* lifetime count, for diagnosis */

/* Write ALL of `len` bytes to the TCP client, retrying while its send buffer
   is momentarily full instead of silently truncating (see file header). Only
   gives up - and reports the client as dead - if it can't accept another
   byte for CLIENT_WRITE_TIMEOUT_MS straight. Returns false if the client
   should be dropped (caller then calls client.stop()). */
static bool writeAllToClient(WiFiClient &c, const uint8_t *data, int len)
{
  int written = 0;
  uint32_t stallStart = 0;
  while (written < len)
  {
    if (!c.connected())
    {
      g_droppedBytes += (uint32_t)(len - written);
      return false;
    }
    int n = c.write(data + written, len - written);
    if (n > 0)
    {
      written += n;
      stallStart = 0;
    }
    else
    {
      if (stallStart == 0) stallStart = millis();
      if (millis() - stallStart > CLIENT_WRITE_TIMEOUT_MS)
      {
        g_droppedBytes += (uint32_t)(len - written);
        Serial.printf("WARN: client write stalled %lu ms - dropping %d bytes, "
                      "closing link (total dropped so far: %lu)\n",
                      (unsigned long)CLIENT_WRITE_TIMEOUT_MS, len - written,
                      (unsigned long)g_droppedBytes);
        return false;
      }
      delay(1);   /* brief backoff, keep polling the same client */
    }
  }
  return true;
}

/* Send one already-formatted line to the client via the retrying writer. */
static void fakeSendLine(WiFiClient &c, const char *s)
{
  writeAllToClient(c, (const uint8_t *)s, (int)strlen(s));
}

/* Synthesise one buffered-run replay and stream it to the app - identical shape
   to what the STM32 emits for a real 'dump' (DUMP,begin / DUMP,run / TRACE... /
   DUMP,end), so the app parses and per-run row-counts it exactly the same way.
   `mode`: 0 = auto (first call after enable is SHORT, next is full - proves the
   auto-refetch recovery), 1 = force SHORT, 2 = force full. */
static void sendFakeDump(WiFiClient &c, int mode)
{
  if (!(c && c.connected())) return;

  const int ntrace = DUMPTEST_ROWS;
  bool shortSend;
  if (mode == 1)      shortSend = true;
  else if (mode == 2) shortSend = false;
  else                { g_dumpTestAttempt++; shortSend = (g_dumpTestAttempt == 1); }

  int rows = shortSend ? (ntrace * 2 / 3) : ntrace;   /* drop ~1/3 on a short send */

  char line[160];
  fakeSendLine(c, "DUMP,begin,1\r\n");
  snprintf(line, sizeof line, "DUMP,run,0,faketest,0,%d\r\n", ntrace);
  fakeSendLine(c, line);

  for (int i = 0; i < rows; i++)
  {
    int t   = i * 5;                 /* t_ms       */
    int pos = i * 8;                 /* posL/posR  */
    int head = (i % 40) - 20;        /* head_ddeg  */
    int accel = 1000;                /* accelz_mg  */
    int s0 = 100 + (i % 50), s1 = 110 + (i % 40), s2 = 120 + (i % 30);
    int s3 = 130 + (i % 30), s4 = 140 + (i % 40), s5 = 150 + (i % 50);
    int x = (i / 8) % 16, y = (i / 8) % 16;
    int vbat = 7400 - (i / 4);       /* vbat_mv, gently sagging */
    snprintf(line, sizeof line,
             "TRACE,%d,%d,%d,%d,0,%d,%d,%d,%d,%d,%d,%d,%d,%d,0,%d\r\n",
             t, pos, pos, head, accel, s0, s1, s2, s3, s4, s5, x, y, vbat);
    fakeSendLine(c, line);
    yield();                         /* let the WiFi/lwIP task breathe mid-burst */
  }

  fakeSendLine(c, "DUMP,end\r\n");
  Serial.printf("fake dump sent: %d/%d TRACE rows (%s)\n",
                rows, ntrace, shortSend ? "SHORT - app should auto re-fetch" : "full");
  if (mode == 0 && !shortSend) g_dumpTestAttempt = 0;   /* auto cycle complete */
}

/* Try to join a WiFi network. Returns true on success. */
static bool joinWifi(const char *ssid, const char *pass)
{
  Serial.printf("Joining WiFi \"%s\" ...\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                          /* lower latency for tuning */
  WiFi.begin(ssid, pass);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < STA_TIMEOUT_MS)
  {
    /* keep the boot/connecting blink alive while we wait */
    uint32_t step = millis();
    while (millis() - step < 250) { StatusLed::task(); delay(5); }
    Serial.print('.');
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

/* True if a raw byte buffer contains the given ASCII needle (whole buffer,
   doesn't handle a needle split across two TCP reads - fine for a short,
   manually-triggered control command). */
static bool bufContains(const uint8_t *data, int len, const char *needle)
{
  size_t nlen = strlen(needle);
  if (nlen == 0 || (size_t)len < nlen) return false;
  for (int i = 0; i <= len - (int)nlen; i++)
  {
    if (memcmp(data + i, needle, nlen) == 0) return true;
  }
  return false;
}

/* Watch the STM32->PC byte stream for the ##OFFLINE##/##ONLINE## markers and
   drive the status LED. Carries a few bytes between calls so a marker split
   across two UART reads is still caught. */
static void scanOfflineMarkers(const uint8_t *data, int len)
{
  static char carry[16];              /* tail of the previous chunk           */
  static int  carryLen = 0;
  static char scan[1088];             /* carry + this chunk, for the search   */

  int n = 0;
  for (int i = 0; i < carryLen && n < (int)sizeof(scan); i++) scan[n++] = carry[i];
  for (int i = 0; i < len && n < (int)sizeof(scan); i++)      scan[n++] = (char)data[i];

  /* If both appear in one window, honour whichever comes last. */
  int offAt = -1, onAt = -1;
  for (int i = 0; i + (int)strlen(OFFLINE_MARK) <= n; i++)
    if (memcmp(scan + i, OFFLINE_MARK, strlen(OFFLINE_MARK)) == 0) offAt = i;
  for (int i = 0; i + (int)strlen(ONLINE_MARK) <= n; i++)
    if (memcmp(scan + i, ONLINE_MARK, strlen(ONLINE_MARK)) == 0) onAt = i;

  if (offAt >= 0 || onAt >= 0)
  {
    bool offline = offAt > onAt;      /* later marker wins */
    StatusLed::setOffline(offline);
    Serial.printf("STM32 marker: mouse is %s\n", offline ? "OFFLINE (comms muted)"
                                                         : "ONLINE");
  }

  /* keep the last (maxNeedle-1) bytes so a boundary-split marker still matches */
  int keep = n < 15 ? n : 15;
  for (int i = 0; i < keep; i++) carry[i] = scan[n - keep + i];
  carryLen = keep;
}

static void printBanner()
{
  Serial.println();
  Serial.println(F("=== NEMISIS ESP32-C3 WiFi bridge ==="));
  Serial.println(F("MODE    : Station (joined WiFi)"));
  Serial.printf ("SSID    : %s   RSSI %d dBm\n", g_staSsid.c_str(), WiFi.RSSI());
  Serial.println(F("CONNECT the Python app to EITHER of these:"));
  Serial.printf ("   IP   : %s : %u\n", WiFi.localIP().toString().c_str(), TCP_PORT);
  Serial.printf ("   name : %s.local : %u\n", MDNS_HOST, TCP_PORT);
  Serial.printf("UART    : %lu 8N1  RX=GPIO%d (from STM32 TX)  TX=GPIO%d (to STM32 RX)\n",
                (unsigned long)UART_BAUD, UART_RX_PIN, UART_TX_PIN);
  Serial.println(F("To join a DIFFERENT router (no reflash): power up then press BOOT"));
  Serial.println(F("within 3s, or send '##WIFI_SETUP##' from the app, to reopen the portal."));
  Serial.println(F("Waiting for a TCP client..."));
}

void setup()
{
  Serial.begin(115200);                         /* USB-CDC: bridge status     */
  delay(300);

  StatusLed::begin(LED_PIN);                     /* RGB status indicator       */
  StatusLed::setNet(StatusLed::Net::Booting);

  LINK.setRxBufferSize(UART_RX_BUF_SZ);          /* must be set before begin() */
  LINK.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  WifiProvision::begin();

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  bool forcedByApp = WifiProvision::consumePortalRequest();

  /* Post-boot BOOT-button window (see BOOT_BUTTON_PIN note above: the button
     CANNOT be sampled at reset without dropping the C3 into download mode).
     Give the user a few seconds after power-up to press it. */
  bool buttonHeld = false;
  if (!forcedByApp)
  {
    Serial.printf("Press BOOT now (%lus, LED breathing magenta) to open the "
                  "WiFi setup portal...\n",
                  (unsigned long)(BOOT_BTN_WINDOW_MS / 1000));
    Serial.printf("BOOT(GPIO%d) idle level = %d (expect 1 released, 0 pressed)\n",
                  BOOT_BUTTON_PIN, digitalRead(BOOT_BUTTON_PIN));
    StatusLed::setNet(StatusLed::Net::ButtonWindow);   /* magenta fade cue */
    uint32_t start = millis();
    int lastLevel = -1;
    while (millis() - start < BOOT_BTN_WINDOW_MS)
    {
      int lvl = digitalRead(BOOT_BUTTON_PIN);
      if (lvl != lastLevel)                            /* log every transition */
      {
        Serial.printf("  BOOT level -> %d\n", lvl);
        lastLevel = lvl;
      }
      if (lvl == LOW) { buttonHeld = true; break; }
      StatusLed::task();
      delay(5);
    }
  }

  WifiProvision::Credentials creds;
  bool haveCreds = WifiProvision::load(creds);
  if (!haveCreds)
  {
    /* Genuinely first boot: seed the compiled-in defaults so this still
       works out of the box, while remaining fully reconfigurable later. */
    WifiProvision::save(DEFAULT_STA_SSID, DEFAULT_STA_PASS);
    WifiProvision::load(creds);
  }

  bool joined = false;
  if (buttonHeld)
  {
    Serial.println(F("BOOT button pressed - opening WiFi setup portal."));
  }
  else if (forcedByApp)
  {
    Serial.println(F("WiFi setup requested from the app - opening portal."));
  }
  else
  {
    joined = joinWifi(creds.ssid.c_str(), creds.pass.c_str());
    if (!joined)
    {
      Serial.println(F("Could not join the stored WiFi - falling back to setup portal."));
    }
  }

  if (!joined)
  {
    WifiProvision::runPortal(PORTAL_AP_SSID, PORTAL_AP_PASS);   /* never returns */
  }

  g_staSsid = creds.ssid;
  StatusLed::setNet(StatusLed::Net::StaConnected);

  if (MDNS.begin(MDNS_HOST))                     /* advertise nemisis.local */
  {
    MDNS.addService("nemisis", "tcp", TCP_PORT);
  }

  server.begin();
  server.setNoDelay(true);                       /* low latency for tuning */

  printBanner();
}

void loop()
{
  bool busy = false;

  /* Accept a new client; reject extras (single-session bridge). */
  if (server.hasClient())
  {
    if (client && client.connected())
    {
      WiFiClient extra = server.available();
      extra.println("busy: another debug client is connected");
      extra.stop();
    }
    else
    {
      client = server.available();
      client.setNoDelay(true);
      StatusLed::setClient(true);
      Serial.printf("client connected: %s\n", client.remoteIP().toString().c_str());
      client.println("== connected to NEMISIS bridge ==");
      LINK.write('\r');                          /* nudge STM32 to print menu */
    }
  }

  /* PC -> STM32 */
  if (client && client.connected())
  {
    int n = client.available();
    while (n > 0)
    {
      int chunk = client.read(buf, n > (int)sizeof(buf) ? (int)sizeof(buf) : n);
      if (chunk <= 0) break;

      /* Remote "reconfigure WiFi" trigger from the app - swallowed here, the
         STM32 console never sees it. */
      if (bufContains(buf, chunk, WIFI_SETUP_MAGIC))
      {
        client.println("Rebooting into WiFi setup mode - join \"NEMISIS-SETUP\" "
                        "to choose a different network.");
        delay(50);
        client.stop();
        WifiProvision::requestPortalOnNextBoot();
        delay(50);
        ESP.restart();
      }

      /* Bench fake-dump hook - swallowed here, never reaches the STM32. Check the
         longer variants (on/off/short/full) before the bare trigger. */
      if (bufContains(buf, chunk, DUMPTEST_MAGIC))
      {
        if (bufContains(buf, chunk, "##DUMPTEST##off"))
        {
          g_dumpTestMode = false;
          client.println("[esp] fake-dump test mode OFF - Fetch pulls the real run");
          Serial.println("fake-dump test mode OFF");
        }
        else if (bufContains(buf, chunk, "##DUMPTEST##on"))
        {
          g_dumpTestMode = true;
          g_dumpTestAttempt = 0;                 /* fresh cycle: next fetch is short */
          client.println("[esp] fake-dump test mode ON - click Fetch for synthetic data");
          Serial.println("fake-dump test mode ON");
        }
        else if (bufContains(buf, chunk, "##DUMPTEST##short"))
        {
          sendFakeDump(client, 1);
        }
        else if (bufContains(buf, chunk, "##DUMPTEST##full"))
        {
          sendFakeDump(client, 2);
        }
        else                                     /* bare trigger = the Fetch button */
        {
          if (g_dumpTestMode) sendFakeDump(client, 0);
          else client.println("[esp] fake-dump mode is OFF - enable it first "
                              "(##DUMPTEST##on, or the app's 'Fake dump' toggle)");
        }
        n -= chunk;
        busy = true;
        continue;                                /* skip forwarding to the STM32 */
      }

      LINK.write(buf, chunk);
      n -= chunk;
      busy = true;
    }
  }

  /* STM32 -> PC (also mirror to USB serial so you can watch without the app).
     writeAllToClient() retries instead of silently truncating on a stalled
     WiFi write - see the file header for why that mattered. */
  int m = LINK.available();
  while (m > 0)
  {
    int chunk = LINK.read(buf, m > (int)sizeof(buf) ? (int)sizeof(buf) : m);
    if (chunk <= 0) break;
    scanOfflineMarkers(buf, chunk);            /* drive offline/online LED cue */
    if (client && client.connected())
    {
      if (!writeAllToClient(client, buf, chunk))
      {
        client.stop();
        StatusLed::setClient(false);
      }
    }
    Serial.write(buf, chunk);
    m -= chunk;
    busy = true;
  }

  if (client && !client.connected())
  {
    Serial.println("client disconnected");
    client.stop();
    StatusLed::setClient(false);
  }

  /* Let WiFi be (re)configured over USB serial at any time, even while bridging
     - type SSID=.../PASS=.../SAVE in the monitor (or the Python app's WiFi
     setup does it for you). Only reads USB serial input, which the bridge
     otherwise ignores. */
  WifiProvision::serialTask();

  if (busy) StatusLed::notifyActivity();   /* drive the streaming shimmer */
  StatusLed::task();                       /* animate the status LED      */

  /* ESP32-C3 is single-core: when there's nothing to move, yield so the WiFi /
     lwIP task can run. Without this the network stack starves and TCP delivery
     becomes bursty ("works then stops"). When data IS flowing we don't delay,
     so latency stays low. */
  if (!busy) delay(2);
}
