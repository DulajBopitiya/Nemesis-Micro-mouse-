/**
 ******************************************************************************
 * @file    wifi_provision.cpp
 * @brief   Captive-portal WiFi provisioning - see wifi_provision.h.
 ******************************************************************************
 */
#include "wifi_provision.h"

#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

#include "status_led.h"

namespace WifiProvision {
namespace {

Preferences prefs;
const char *NS_WIFI = "wifi";

const byte    DNS_PORT = 53;
DNSServer     dnsServer;
WebServer     webServer(80);

bool   s_saved = false;
String s_pendingSsid;
String s_pendingPass;

/* Pre-rendered <option> list of nearby networks. Built ONCE at portal startup
   (before any client associates) so serving the page never triggers a blocking
   channel-hopping scan that would drop the client mid-request. */
String s_networkOptions;

String htmlEscape(const String &s)
{
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++)
  {
    char c = s[i];
    switch (c)
    {
      case '&':  out += "&amp;";  break;
      case '"':  out += "&quot;"; break;
      case '<':  out += "&lt;";   break;
      case '>':  out += "&gt;";   break;
      default:   out += c;
    }
  }
  return out;
}

/* Scan nearby 2.4GHz networks into s_networkOptions. MUST be called before the
   softAP has associated clients - scanNetworks() is blocking and hops the
   radio's channel, which would drop any connected portal client. */
void scanNetworksIntoCache()
{
  int n = WiFi.scanNetworks();   /* blocking ~1-2s; done once, up front */
  String opts;
  if (n <= 0)
  {
    opts = "<option value=\"\">(no networks seen - type the name manually)</option>";
  }
  else
  {
    for (int i = 0; i < n; i++)
    {
      String ss = WiFi.SSID(i);
      if (ss.length() == 0) continue;
      bool open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
      opts += "<option value=\"" + htmlEscape(ss) + "\">" + htmlEscape(ss) +
              " (" + String(WiFi.RSSI(i)) + " dBm" + (open ? ", open" : "") + ")</option>";
    }
  }
  WiFi.scanDelete();
  s_networkOptions = opts;
  Serial.printf("portal: cached %d nearby network(s) for the setup page\n",
                n < 0 ? 0 : n);
}

String buildSetupPage()
{
  const String &opts = s_networkOptions;   /* pre-scanned; no scan during request */

  String html;
  html += F("<!DOCTYPE html><html><head><meta name='viewport' "
            "content='width=device-width,initial-scale=1'>"
            "<title>NEMISIS WiFi setup</title><style>"
            "body{font-family:sans-serif;background:#101317;color:#e6e6e6;"
            "padding:24px;max-width:420px;margin:auto}"
            "h1{font-size:20px;line-height:1.3}"
            "label{display:block;margin-top:12px;font-size:13px;color:#9aa5b1}"
            "select,input{width:100%;padding:9px;margin-top:4px;box-sizing:border-box;"
            "background:#1b1f24;color:#fff;border:1px solid #444;border-radius:4px;"
            "font-size:15px}"
            "button{width:100%;margin-top:18px;padding:11px;background:#3aa76d;"
            "color:#fff;border:none;border-radius:4px;font-size:16px}"
            "</style></head><body>");
  html += F("<h1>NEMISIS &middot; connect the mouse to your WiFi</h1>");
  html += F("<form method='POST' action='/save'>");
  html += F("<label>Nearby networks</label>"
            "<select onchange=\"document.getElementById('ssid').value=this.value\">");
  html += "<option value=\"\">-- pick one --</option>" + opts + "</select>";
  html += F("<label>SSID</label>"
            "<input id='ssid' name='ssid' maxlength='32' autocomplete='off' "
            "placeholder='or type it here'>");
  html += F("<label>Password</label>"
            "<input name='pass' type='password' maxlength='63' autocomplete='off' "
            "placeholder='leave blank if open'>");
  html += F("<button type='submit'>Save &amp; connect</button>");
  html += F("</form></body></html>");
  return html;
}

String resultPage(const String &ssid)
{
  String html;
  html += F("<!DOCTYPE html><html><head><meta name='viewport' "
            "content='width=device-width,initial-scale=1'>"
            "<style>body{font-family:sans-serif;background:#101317;color:#e6e6e6;"
            "padding:24px;max-width:420px;margin:auto}</style></head><body>");
  html += "<h2>Saved.</h2><p>Rebooting and joining <b>" + htmlEscape(ssid) + "</b>...</p>";
  html += F("<p>Reconnect your phone/PC to your normal WiFi, then reconnect the "
            "NEMISIS app to the mouse. Check your router's client list for its "
            "new address, or try <b>nemisis.local</b>.</p>");
  html += F("</body></html>");
  return html;
}

void handleRoot()
{
  Serial.printf("HTTP GET %s  (from %s)\n",
                webServer.uri().c_str(),
                webServer.client().remoteIP().toString().c_str());
  webServer.send(200, "text/html", buildSetupPage());
}

void handleSave()
{
  String ssid = webServer.hasArg("ssid") ? webServer.arg("ssid") : "";
  ssid.trim();
  if (ssid.length() == 0)
  {
    webServer.send(200, "text/html",
      "<html><body style='font-family:sans-serif;background:#101317;color:#e6e6e6;"
      "padding:24px'><p>Please choose or type a network name.</p>"
      "<a href='/' style='color:#7fd0ff'>Back</a></body></html>");
    return;
  }
  s_pendingSsid = ssid;
  s_pendingPass = webServer.hasArg("pass") ? webServer.arg("pass") : "";
  webServer.send(200, "text/html", resultPage(ssid));
  s_saved = true;   /* portal loop reboots shortly after this reply is flushed */
}

/* Captive-portal trick: ANY unknown path/host gets the setup page instead of
   a 404, which is what makes phones/laptops pop the "sign in to network"
   prompt automatically. */
void handleNotFound()
{
  Serial.printf("HTTP (captive) %s%s -> serving setup page\n",
                webServer.hostHeader().c_str(), webServer.uri().c_str());
  handleRoot();
}

/* --- Serial (USB monitor) provisioning: a rock-solid fallback for when the
   captive portal won't cooperate. You type the credentials into the same
   serial monitor that shows these logs; no AP / phone / DHCP involved.

   Commands (one per line, press Enter):
     SSID=<your network name>     (everything after '=' is the SSID; spaces OK)
     PASS=<your password>         (leave blank for an open network)
     SHOW                         (echo what's staged)
     SAVE                         (write to NVS and reboot into that network)
--- */
void handleSerialLine(String line)
{
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("SSID="))
  {
    s_pendingSsid = line.substring(5);
    s_pendingSsid.trim();
    Serial.printf("  staged SSID = \"%s\"\n", s_pendingSsid.c_str());
  }
  else if (line.startsWith("PASS="))
  {
    s_pendingPass = line.substring(5);      /* keep as-is (spaces/case matter) */
    Serial.printf("  staged PASS = (%u chars)\n", (unsigned)s_pendingPass.length());
  }
  else if (line.equalsIgnoreCase("SHOW"))
  {
    Serial.printf("  pending: SSID=\"%s\"  PASS=\"%s\"\n",
                  s_pendingSsid.c_str(), s_pendingPass.c_str());
  }
  else if (line.equalsIgnoreCase("SAVE"))
  {
    if (s_pendingSsid.length() == 0)
    {
      Serial.println(F("  no SSID staged - send  SSID=YourNetwork  first"));
      return;
    }
    save(s_pendingSsid, s_pendingPass);
    Serial.printf("  Saved WiFi \"%s\" to NVS - rebooting to join it...\n",
                  s_pendingSsid.c_str());
    delay(300);
    ESP.restart();
  }
  else
  {
    Serial.println(F("  serial WiFi setup - commands (one per line):"));
    Serial.println(F("    SSID=<name>   PASS=<pw>   SHOW   SAVE"));
  }
}

/* Non-blocking: accumulate a line from the USB serial monitor and dispatch it. */
void pollSerialProvisioning()
{
  static String line;
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (line.length()) { handleSerialLine(line); line = ""; }
    }
    else if (line.length() < 160)
    {
      line += c;
    }
  }
}

} // namespace

void begin() { prefs.begin(NS_WIFI, false); }

bool load(Credentials &out)
{
  out.ssid = prefs.getString("ssid", "");
  out.pass = prefs.getString("pass", "");
  return out.ssid.length() > 0;
}

void save(const String &ssid, const String &pass)
{
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
}

void clearCredentials()
{
  prefs.remove("ssid");
  prefs.remove("pass");
}

void requestPortalOnNextBoot() { prefs.putBool("forceportal", true); }

bool consumePortalRequest()
{
  bool v = prefs.getBool("forceportal", false);
  if (v) prefs.putBool("forceportal", false);
  return v;
}

void runPortal(const char *apSsid, const char *apPass)
{
  StatusLed::setNet(StatusLed::Net::Provisioning);

  /* Clean slate: fully tear down any STA attempt AND erase its stored config so
     the core's auto-reconnect doesn't keep hopping the shared radio's channel
     in the background - that churn is what makes the softAP beacon unstable /
     invisible on phones and laptops. (true,true = disconnect + wifioff + erase) */
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(200);

  /* AP_STA so the setup page's scanNetworks() has an STA interface to scan
     with. Crucially we've just ERASED the stored STA creds above, so the STA
     side stays idle on the AP channel (no auto-reconnect channel-hopping) -
     that's what keeps the beacon stable, not the mode itself. */
  WiFi.mode(WIFI_AP_STA);
  delay(100);

  /* Scan for nearby networks NOW, while the AP has no clients yet, and cache
     the result. Doing this during a page request instead would channel-hop the
     radio and drop the client mid-load (the "connects but page never opens"
     symptom). */
  scanNetworksIntoCache();

  /* Pin an explicit AP subnet + (re)start the DHCP server. Without this a
     client can associate (stations=1) yet never get an IP lease, so it can't
     route to 192.168.4.1 at all - the browser just shows "can't reach page"
     (Chrome dinosaur). softAPConfig() must be called BEFORE softAP(). */
  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGw(192, 168, 4, 1);
  IPAddress apMask(255, 255, 255, 0);
  bool cfgOk = WiFi.softAPConfig(apIP, apGw, apMask);

  /* Explicit channel 1, not hidden, up to 4 clients. Letting the channel
     default can inherit a stale/invalid STA channel after a failed join. */
  bool apOk = WiFi.softAP(apSsid, (apPass && apPass[0]) ? apPass : nullptr,
                          /*channel=*/1, /*hidden=*/0, /*max_conn=*/4);
  delay(100);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);   /* max beacon strength for visibility */
  apIP = WiFi.softAPIP();
  Serial.printf("softAPConfig %s\n", cfgOk ? "OK" : "FAILED");

  Serial.printf("softAP(\"%s\") %s  IP=%s  MAC=%s  ch=%d\n",
                apSsid, apOk ? "OK" : "FAILED",
                apIP.toString().c_str(), WiFi.softAPmacAddress().c_str(),
                WiFi.channel());
  if (!apOk)
    Serial.println(F("softAP failed to start - check power/brownout; the AP "
                     "won't be visible. Try a cleaner 5V supply / USB port."));

  dnsServer.start(DNS_PORT, "*", apIP);

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/save", HTTP_POST, handleSave);
  webServer.onNotFound(handleNotFound);
  webServer.begin();

  Serial.println();
  Serial.println(F("=== NEMISIS WiFi setup portal active ==="));
  Serial.printf("Join WiFi \"%s\"%s from your phone or laptop, then open http://%s/\n",
                apSsid, (apPass && apPass[0]) ? "" : " (open network)",
                apIP.toString().c_str());
  Serial.println(F("(most phones/laptops pop the sign-in page automatically)"));
  Serial.println();
  Serial.println(F("OR set WiFi over THIS serial monitor (most reliable) - type:"));
  Serial.println(F("    SSID=YourNetworkName"));
  Serial.println(F("    PASS=YourPassword"));
  Serial.println(F("    SAVE"));

  s_saved = false;
  uint32_t savedAt = 0;
  uint32_t lastBeat = 0;
  for (;;)
  {
    dnsServer.processNextRequest();
    webServer.handleClient();
    pollSerialProvisioning();          /* also accept SSID=/PASS=/SAVE over USB */
    StatusLed::task();

    /* Heartbeat: proves the AP loop is alive and shows if any device has
       associated (softAPgetStationNum > 0 means a phone/laptop joined). */
    if (millis() - lastBeat > 3000)
    {
      lastBeat = millis();
      Serial.printf("portal alive: mode=%d  ch=%d  stations=%u\n",
                    (int)WiFi.getMode(), WiFi.channel(),
                    WiFi.softAPgetStationNum());
    }

    if (s_saved && savedAt == 0) savedAt = millis();
    /* give the HTTP response time to actually reach the browser before we
       tear the AP down and reboot into the new network. */
    if (savedAt != 0 && (millis() - savedAt) > 1500)
    {
      save(s_pendingSsid, s_pendingPass);
      Serial.printf("Saved WiFi credentials for \"%s\" - rebooting...\n",
                    s_pendingSsid.c_str());
      delay(200);
      ESP.restart();
    }
    delay(2);
  }
}

void serialTask() { pollSerialProvisioning(); }

} // namespace WifiProvision
