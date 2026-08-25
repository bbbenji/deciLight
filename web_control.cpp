#include "web_control.h"

#if FEATURE_WIFI

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>

#include "display.h"
#include "group_sync.h"
#include "ota.h"
#include "remote_control.h"
#include "settings.h"
#include "signal_light.h"
#include "web_page.h"

namespace web_control {
namespace {

WebServer server(WEB_SERVER_PORT);

bool accessPointMode = false;
bool running = false;
char addressText[16] = "";

// Networking is a small state machine rather than a one-shot at boot, so a
// unit that comes up before the building's router still ends up on it.
enum class Net : uint8_t {
  Connecting,  // station attempt in flight
  Connected,   // on the stored network
  Dropped,     // was connected, waiting to see if it returns
  Ap,          // serving our own access point
};
Net netState = Net::Ap;
uint32_t stateSinceMs = 0;

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

// Hand-rolled rather than pulling in a JSON library: the document is fixed
// and small. Every value goes through one of the appenders below, each of
// which opens and closes its own quoting. An earlier version let one field's
// closing quote live in the next field's literal, which reads compactly right
// up until someone inserts a number in the middle and silently invalidates
// the whole document.
void appendEscaped(String& out, const char* value) {
  for (const char* c = value; *c != '\0'; c++) {
    if (*c == '"' || *c == '\\') {
      out += '\\';
      out += *c;
    } else if (uint8_t(*c) >= 0x20) {
      out += *c;
    }
  }
}

void appendKey(String& out, const char* key) {
  if (out.length() > 1) out += ',';
  out += '"';
  out += key;
  out += "\":";
}

void appendStr(String& out, const char* key, const char* value) {
  appendKey(out, key);
  out += '"';
  appendEscaped(out, value);
  out += '"';
}

void appendInt(String& out, const char* key, long value) {
  appendKey(out, key);
  out += value;
}

void appendBool(String& out, const char* key, bool value) {
  appendKey(out, key);
  out += value ? "true" : "false";
}

void appendFixed(String& out, const char* key, float value, int decimals) {
  appendKey(out, key);
  out += String(value, decimals);
}

// For a value that is itself a JSON document - the peer roster is an array of
// objects, each assembled with these same appenders, so nothing anywhere is
// quoted by hand.
void appendRaw(String& out, const char* key, const String& value) {
  appendKey(out, key);
  out += value;
}

void sendState() {
  const Settings& s = settings::get();
  const String ssid = accessPointMode ? String(WIFI_AP_SSID) : WiFi.SSID();
  const IPAddress ip = accessPointMode ? WiFi.softAPIP() : WiFi.localIP();

  char color[8];
  snprintf(color, sizeof(color), "%06lX",
           static_cast<unsigned long>(signal_light::manualColor() & 0xFFFFFF));

  String out;
  out.reserve(384);
  out += '{';

  appendStr(out, "name", PRODUCT_NAME);
  appendStr(out, "version", FIRMWARE_VERSION);
  appendFixed(out, "db", latest.leqDb, 1);
  appendStr(out, "units", DB_UNITS);
  appendStr(out, "quality", qualityName(latest.quality));
  appendStr(out, "mode", modeName(signal_light::mode()));
  appendStr(out, "zone", zoneName(signal_light::zone()));
  appendStr(out, "color", color);
  appendInt(out, "dbMin", s.dbMin);
  appendInt(out, "dbMax", s.dbMax);
  appendInt(out, "brightness", s.brightness);
  appendInt(out, "displayBrightness", s.displayBrightness);
  appendStr(out, "group", settings::groupName());
  appendBool(out, "groupLevel", s.groupLevel);
  appendInt(out, "zones", s.zones);
  appendInt(out, "combine", s.combine);
  appendInt(out, "inactiveLevel", s.inactiveLevel);
  appendInt(out, "peers", group_sync::peerCount());
  appendInt(out, "channel", group_sync::channel());
  appendBool(out, "groupActive", group_sync::active());
  appendStr(out, "unit", group_sync::localName());
  appendInt(out, "coverage", group_sync::zoneCoverage());
  appendInt(out, "overlap", group_sync::zoneOverlap());

  // The roster is the one array in the document. Each entry is built with
  // the same appenders, starting from its own opening brace, so appendKey's
  // comma handling works inside it unchanged.
  group_sync::PeerInfo peers[GROUP_MAX_PEERS];
  const uint8_t n = group_sync::peerList(peers, GROUP_MAX_PEERS);

  String roster;
  roster.reserve(32 + n * 64);
  roster += '[';
  for (uint8_t i = 0; i < n; i++) {
    if (i) roster += ',';
    String entry;
    entry += '{';
    appendStr(entry, "name", peers[i].name);
    appendFixed(entry, "db", peers[i].levelDb, 1);
    appendInt(entry, "zones", peers[i].zones);
    appendBool(entry, "follows", peers[i].followsGroup);
    appendInt(entry, "ageMs", peers[i].ageMs);
    entry += '}';
    roster += entry;
  }
  roster += ']';
  appendRaw(out, "roster", roster);
  appendStr(out, "net", accessPointMode ? "ap" : "sta");
  appendStr(out, "ssid", ssid.c_str());
  appendStr(out, "ip", ip.toString().c_str());
  appendStr(out, "test", signal_light::selfTestLabel());
  appendStr(out, "irCode",
            remote_control::haveLastCode() ? remote_control::lastCodeHex() : "");
  appendStr(out, "irProtocol",
            remote_control::haveLastCode() ? remote_control::lastProtocol() : "");
  appendBool(out, "irMapped", remote_control::lastCodeMapped());
  appendInt(out, "irAgeMs", remote_control::lastCodeAgeMs());
#if FEATURE_OTA
  appendBool(out, "ota", ota::available());
  appendStr(out, "otaUser", ota::username());
#else
  appendBool(out, "ota", false);
  appendStr(out, "otaUser", "");
#endif

  out += '}';

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
  if (server.hasArg("dbMin") || server.hasArg("dbMax") || server.hasArg("brightness")) {
    group_sync::publishSettings();
  }
  if (server.hasArg("displayBrightness")) {
    settings::setDisplayBrightness(server.arg("displayBrightness").toInt());
    display::setBrightness(settings::get().displayBrightness);
  }
  sendState();
}

// Group membership and how this unit behaves within it. Changing the name
// needs the radio re-initialised, which is simplest to do by restarting.
void handleGroup() {
  bool restart = false;

  if (server.hasArg("group")) {
    if (strcmp(server.arg("group").c_str(), settings::groupName()) != 0) {
      settings::setGroupName(server.arg("group").c_str());
      restart = true;
    }
  }
  if (server.hasArg("groupLevel")) {
    settings::setGroupLevel(server.arg("groupLevel").toInt() != 0);
  }
  if (server.hasArg("unit")) settings::setUnitName(server.arg("unit").c_str());
  if (server.hasArg("zones")) settings::setZones(server.arg("zones").toInt());
  if (server.hasArg("combine")) {
    settings::setCombine(server.arg("combine").toInt());
    // Group-wide, so the rest of the group is told rather than left to
    // disagree about how readings are combined.
    group_sync::publishSettings();
  }
  if (server.hasArg("inactiveLevel")) {
    settings::setInactiveLevel(server.arg("inactiveLevel").toInt());
  }

  const Settings& s = settings::get();
  signal_light::setZones(s.zones, s.inactiveLevel);

  if (restart) {
    // Flush the new name before the reset, or it would be lost with the
    // deferred write still pending.
    settings::flush();
    server.send(200, "application/json", "{\"restarting\":true}");
    server.client().flush();
    delay(200);
    ESP.restart();
    return;
  }
  sendState();
}

// Starts the LED self test. Deliberately its own route rather than a mode, so
// it cannot be reached by accident and always leaves the light as it found it.
void handleTest() {
  signal_light::startSelfTest();
  signal_light::tick();
  group_sync::publishSelfTest();
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
  group_sync::publishMode();
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

void rememberAddress(const IPAddress& ip) {
  strncpy(addressText, ip.toString().c_str(), sizeof(addressText) - 1);
  addressText[sizeof(addressText) - 1] = '\0';
}

void enterState(Net next) {
  netState = next;
  stateSinceMs = millis();
}

// Kicks off a station attempt without waiting for it. Callers poll the state
// machine; nothing here may block, because tick() runs it.
bool startStationAttempt() {
  const char* ssid = settings::wifiSsid();
  if (ssid[0] == '\0') return false;

  Serial.printf("wifi: joining %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  // Power save parks the radio between beacons, which silently drops incoming
  // ESP-NOW packets. This is a mains-powered device, so the trade is easy.
  WiFi.setSleep(false);
  WiFi.begin(ssid, settings::wifiPassword());
  enterState(Net::Connecting);
  return true;
}

bool startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.setHostname(WIFI_HOSTNAME);
  // An empty password yields an open network, which softAP expects as nullptr.
  const char* password = WIFI_AP_PASSWORD[0] ? WIFI_AP_PASSWORD : nullptr;
  WiFi.setSleep(false);
  // Pinned rather than left to the default so that several units falling back
  // to their own access points still share a channel, which is what lets them
  // hear each other over ESP-NOW.
  if (!WiFi.softAP(WIFI_AP_SSID, password, WIFI_AP_CHANNEL)) {
    Serial.println(F("wifi: could not start the access point"));
    return false;
  }
  accessPointMode = true;
  rememberAddress(WiFi.softAPIP());
  enterState(Net::Ap);

  // Same reason as onStationUp(): the ESP-NOW peer is bound to an interface
  // when it is added, so changing mode leaves it pointing at one that is no
  // longer carrying traffic. Both directions of the switch rebuild it.
  group_sync::begin();

  Serial.print(F("wifi: access point "));
  Serial.print(WIFI_AP_SSID);
  Serial.print(F(", http://"));
  Serial.println(WiFi.softAPIP());
  return true;
}

void onStationUp() {
  accessPointMode = false;
  rememberAddress(WiFi.localIP());
  enterState(Net::Connected);

  Serial.print(F("wifi: connected, http://"));
  Serial.println(addressText);

  MDNS.end();
  if (MDNS.begin(WIFI_HOSTNAME)) MDNS.addService("http", "tcp", WEB_SERVER_PORT);

  // The channel almost certainly changed, which invalidates the ESP-NOW
  // broadcast peer, so the group is rebuilt on the new one.
  group_sync::begin();
}

// Called from tick(). Every branch returns promptly; none of them wait.
void maintainNetwork() {
  const uint32_t now = millis();

  switch (netState) {
    case Net::Connecting:
      if (WiFi.status() == WL_CONNECTED) {
        onStationUp();
      } else if (now - stateSinceMs >= WIFI_CONNECT_TIMEOUT_MS) {
        Serial.println(F("wifi: timed out, falling back to the access point"));
        WiFi.disconnect(true);
        startAccessPoint();
      }
      break;

    case Net::Connected:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("wifi: connection dropped"));
        enterState(Net::Dropped);
      } else if (WiFi.localIP().toString() != String(addressText)) {
        // A lease change would otherwise leave the screen and the web page
        // advertising an address that no longer reaches the unit.
        rememberAddress(WiFi.localIP());
        Serial.print(F("wifi: address is now "));
        Serial.println(addressText);
      }
      break;

    case Net::Dropped:
      // The core retries on its own; this is the point at which we stop
      // believing it and make the unit reachable some other way.
      if (WiFi.status() == WL_CONNECTED) {
        onStationUp();
      } else if (now - stateSinceMs >= WIFI_FALLBACK_AFTER_MS) {
        startAccessPoint();
      }
      break;

    case Net::Ap:
      if (settings::wifiSsid()[0] == '\0') break;
      if (now - stateSinceMs < WIFI_RETRY_INTERVAL_MS) break;
      // Retrying tears the access point down, so not while someone is on it.
      if (WiFi.softAPgetStationNum() > 0) {
        enterState(Net::Ap);  // reset the timer; try again once they leave
        break;
      }
      startStationAttempt();
      break;
  }
}

}  // namespace

bool begin() {
  // Establish a known state rather than relying on static initialisation, so
  // begin() is a real reset point.
  running = false;
  addressText[0] = '\0';
  latest = {0.0f, sound_level::Quality::BelowNoiseFloor};

  // Blocking here is fine: nothing else is running yet, and a unit that
  // reaches its network at boot should be on it before the first frame.
  if (startStationAttempt()) {
    while (netState == Net::Connecting) {
      maintainNetwork();
      if (netState == Net::Connecting) delay(100);
    }
  } else if (!startAccessPoint()) {
    return false;
  }
  if (netState == Net::Ap && addressText[0] == '\0' && !startAccessPoint()) return false;

  if (MDNS.begin(WIFI_HOSTNAME)) {
    MDNS.addService("http", "tcp", WEB_SERVER_PORT);
    Serial.printf("wifi: also at http://%s.local/\n", WIFI_HOSTNAME);
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, sendState);
  server.on("/api/version", HTTP_GET, sendVersion);
  server.on("/api/set", HTTP_POST, handleSet);
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/test", HTTP_POST, handleTest);
  server.on("/api/group", HTTP_POST, handleGroup);
  server.on("/api/wifi", HTTP_POST, handleWifi);
#if FEATURE_OTA
  ota::registerRoutes(server);
#endif
  // Anything else redirects to the page, so a captive-portal probe or a
  // mistyped path still lands somewhere useful.
  server.onNotFound(handleRoot);
  server.begin();

  running = true;
  return true;
}

const char* address() { return addressText; }

void tick() {
  if (!running) return;
  server.handleClient();
  maintainNetwork();
}

void publishLevel(const sound_level::Reading& reading) { latest = reading; }

}  // namespace web_control

#endif  // FEATURE_WIFI
