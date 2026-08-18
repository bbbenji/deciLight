#include "web_control.h"

#if FEATURE_WIFI

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>

#include "ota.h"
#include "settings.h"
#include "signal_light.h"
#include "web_page.h"

namespace web_control {
namespace {

WebServer server(WEB_SERVER_PORT);

bool accessPointMode = false;
bool running = false;
char addressText[16] = "";

// Newest measurement, refreshed from loop() and read by the status endpoint.
sound_level::Reading latest = {0.0f, sound_level::Quality::BelowNoiseFloor};

const char* modeName(signal_light::Mode mode) {
  switch (mode) {
    case signal_light::Mode::Auto:   return "auto";
    case signal_light::Mode::Manual: return "manual";
    default:                         return "off";
  }
}

const char* zoneName(signal_light::Zone zone) {
  switch (zone) {
    case signal_light::Zone::Quiet: return "quiet";
    case signal_light::Zone::Warn:  return "warn";
    case signal_light::Zone::Loud:  return "loud";
    default:                        return "unknown";
  }
}

const char* qualityName(sound_level::Quality quality) {
  switch (quality) {
    case sound_level::Quality::Overload:        return "overload";
    case sound_level::Quality::BelowNoiseFloor: return "quiet";
    default:                                    return "ok";
  }
}

// Hand-rolled rather than pulling in a JSON library: the document is fixed,
// small, and every value is a number or a known-safe identifier. The only
// caller-supplied string is the SSID, which is escaped below.
String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    const char c = in[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c >= 0x20) {
      out += c;
    }
  }
  return out;
}

void sendState() {
  const Settings& s = settings::get();
  const String ssid = accessPointMode ? String(WIFI_AP_SSID) : WiFi.SSID();
  const IPAddress ip = accessPointMode ? WiFi.softAPIP() : WiFi.localIP();

  char color[8];
  snprintf(color, sizeof(color), "%06lX",
           static_cast<unsigned long>(signal_light::manualColor() & 0xFFFFFF));

  String out;
  out.reserve(320);
  out += "{\"name\":\"" PRODUCT_NAME_JSON "\"";
  out += ",\"version\":\"" FIRMWARE_VERSION_JSON "\"";
  out += ",\"db\":";      out += String(latest.leqDb, 1);
  out += ",\"units\":\"" DB_UNITS "\"";
  out += ",\"quality\":\""; out += qualityName(latest.quality);
  out += "\",\"mode\":\"";  out += modeName(signal_light::mode());
  out += "\",\"zone\":\"";  out += zoneName(signal_light::zone());
  out += "\",\"color\":\""; out += color;
  out += "\",\"dbMin\":";      out += s.dbMin;
  out += ",\"dbMax\":";        out += s.dbMax;
  out += ",\"brightness\":";   out += s.brightness;
  out += ",\"net\":\"";        out += accessPointMode ? "ap" : "sta";
  out += "\",\"ssid\":\"";     out += jsonEscape(ssid);
  out += "\",\"ip\":\"";       out += ip.toString();
#if FEATURE_OTA
  out += "\",\"ota\":";       out += ota::available() ? "true" : "false";
  out += ",\"otaUser\":\"";   out += ota::username();
  out += "\"}";
#else
  out += "\",\"ota\":false,\"otaUser\":\"\"}";
#endif

  server.send(200, "application/json", out);
}

void handleRoot() { server.send_P(200, "text/html", WEB_PAGE); }

// Deliberately separate from /api/state: answering "what is running on that
// unit" should not require pulling a live measurement, and after an
// over-the-air update this is the thing worth checking.
void sendVersion() {
  server.send(200, "application/json",
              "{\"name\":\"" PRODUCT_NAME_JSON "\",\"version\":\"" FIRMWARE_VERSION_JSON "\"}");
}

// Thresholds and brightness. Every value is clamped by the settings module,
// so a malformed or hostile request cannot produce an unusable device.
void handleSet() {
  if (server.hasArg("dbMin")) settings::setDbMin(server.arg("dbMin").toInt());
  if (server.hasArg("dbMax")) settings::setDbMax(server.arg("dbMax").toInt());
  if (server.hasArg("brightness")) {
    settings::setBrightness(server.arg("brightness").toInt());
    signal_light::setBrightness(settings::get().brightness);
  }
  sendState();
}

void handleMode() {
  const String mode = server.arg("mode");
  if (mode == "auto") {
    signal_light::setMode(signal_light::Mode::Auto);
  } else if (mode == "off") {
    signal_light::setMode(signal_light::Mode::Off);
  } else if (mode == "manual") {
    // strtoul over the raw hex, so an unparseable colour lands on 0 (black)
    // rather than anything undefined.
    const uint32_t rgb = strtoul(server.arg("color").c_str(), nullptr, 16) & 0xFFFFFF;
    signal_light::setManualColor(rgb);
  } else {
    server.send(400, "text/plain", "unknown mode");
    return;
  }
  sendState();
}

// Credentials are written straight to NVS and take effect on restart, which
// avoids having to tear down and rebuild the network stack underneath a live
// request.
void handleWifi() {
  if (!server.hasArg("ssid")) {
    server.send(400, "text/plain", "ssid required");
    return;
  }
  settings::setWifiCredentials(server.arg("ssid").c_str(), server.arg("pass").c_str());
  server.send(200, "application/json", "{\"restarting\":true}");
  server.client().flush();
  delay(200);  // let the response reach the browser before the radio drops
  ESP.restart();
}

bool connectToStoredNetwork() {
  const char* ssid = settings::wifiSsid();
  if (ssid[0] == '\0') return false;

  Serial.printf("wifi: joining %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.begin(ssid, settings::wifiPassword());

  const uint32_t startedMs = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startedMs >= WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println(F("wifi: timed out"));
      WiFi.disconnect(true);
      return false;
    }
    delay(100);
  }
  Serial.print(F("wifi: connected, http://"));
  Serial.println(WiFi.localIP());
  return true;
}

bool startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.setHostname(WIFI_HOSTNAME);
  // An empty password yields an open network, which softAP expects as nullptr.
  const char* password = WIFI_AP_PASSWORD[0] ? WIFI_AP_PASSWORD : nullptr;
  if (!WiFi.softAP(WIFI_AP_SSID, password)) {
    Serial.println(F("wifi: could not start the access point"));
    return false;
  }
  Serial.print(F("wifi: access point "));
  Serial.print(WIFI_AP_SSID);
  Serial.print(F(", http://"));
  Serial.println(WiFi.softAPIP());
  return true;
}

}  // namespace

bool begin() {
  accessPointMode = !connectToStoredNetwork();
  if (accessPointMode && !startAccessPoint()) return false;

  if (MDNS.begin(WIFI_HOSTNAME)) {
    MDNS.addService("http", "tcp", WEB_SERVER_PORT);
    Serial.printf("wifi: also at http://%s.local/\n", WIFI_HOSTNAME);
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, sendState);
  server.on("/api/version", HTTP_GET, sendVersion);
  server.on("/api/set", HTTP_POST, handleSet);
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/wifi", HTTP_POST, handleWifi);
#if FEATURE_OTA
  ota::registerRoutes(server);
#endif
  // Anything else redirects to the page, so a captive-portal probe or a
  // mistyped path still lands somewhere useful.
  server.onNotFound(handleRoot);
  server.begin();

  const IPAddress ip = accessPointMode ? WiFi.softAPIP() : WiFi.localIP();
  strncpy(addressText, ip.toString().c_str(), sizeof(addressText) - 1);
  addressText[sizeof(addressText) - 1] = '\0';

  running = true;
  return true;
}

const char* address() { return addressText; }

void tick() {
  if (running) server.handleClient();
}

void publishLevel(const sound_level::Reading& reading) { latest = reading; }

}  // namespace web_control

#endif  // FEATURE_WIFI
