#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>

#include "config.h"

namespace settings {
namespace {

// Namespace and keys are unchanged from the first firmware so units that are
// already in the field keep their stored thresholds across an update.
constexpr char kNamespace[]  = "traffic";
constexpr char kKeyDbMin[]   = "dB_min";
constexpr char kKeyDbMax[]   = "dB_max";
constexpr char kKeyBright[]  = "bright";
constexpr char kKeySsid[]    = "wifi_ssid";
constexpr char kKeyPass[]    = "wifi_pass";

// Quiet period after the last change before anything is written to flash.
constexpr uint32_t kFlushDelayMs = 3000;

Preferences prefs;

// Sized to the 802.11 maxima: 32-character SSID, 63-character WPA2 passphrase.
char wifiSsidBuf[33] = {0};
char wifiPassBuf[64] = {0};

Settings current;
Settings stored;
bool dirty = false;
uint32_t dirtySinceMs = 0;

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

void markDirty() {
  dirty = true;
  dirtySinceMs = millis();
}

}  // namespace

void begin() {
  // Opened once and left open; closing and reopening per access costs several
  // milliseconds and gains nothing.
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    Serial.println(F("settings: NVS unavailable, using defaults for this session"));
    current = {DB_MIN_DEFAULT, DB_MAX_DEFAULT, LED_BRIGHTNESS_DEFAULT};
    stored = current;
    return;
  }

  current.dbMin = clampInt(prefs.getUInt(kKeyDbMin, DB_MIN_DEFAULT), DB_LIMIT_LOW, DB_LIMIT_HIGH);
  current.dbMax = clampInt(prefs.getUInt(kKeyDbMax, DB_MAX_DEFAULT), DB_LIMIT_LOW, DB_LIMIT_HIGH);
  current.brightness = clampInt(prefs.getUInt(kKeyBright, LED_BRIGHTNESS_DEFAULT),
                                LED_BRIGHTNESS_MIN, LED_BRIGHTNESS_MAX);

  // A unit flashed with an older build may hold a pair that violates the span.
  if (current.dbMax < current.dbMin + DB_MIN_SPAN) {
    current.dbMax = clampInt(current.dbMin + DB_MIN_SPAN, DB_LIMIT_LOW, DB_LIMIT_HIGH);
    current.dbMin = clampInt(current.dbMax - DB_MIN_SPAN, DB_LIMIT_LOW, DB_LIMIT_HIGH);
  }

  prefs.getString(kKeySsid, wifiSsidBuf, sizeof(wifiSsidBuf));
  prefs.getString(kKeyPass, wifiPassBuf, sizeof(wifiPassBuf));

  stored = current;
}

const Settings& get() { return current; }

void setDbMin(int value) {
  const uint8_t next = clampInt(value, DB_LIMIT_LOW, current.dbMax - DB_MIN_SPAN);
  if (next == current.dbMin) return;
  current.dbMin = next;
  markDirty();
}

void setDbMax(int value) {
  const uint8_t next = clampInt(value, current.dbMin + DB_MIN_SPAN, DB_LIMIT_HIGH);
  if (next == current.dbMax) return;
  current.dbMax = next;
  markDirty();
}

void setBrightness(int value) {
  const uint8_t next = clampInt(value, LED_BRIGHTNESS_MIN, LED_BRIGHTNESS_MAX);
  if (next == current.brightness) return;
  current.brightness = next;
  markDirty();
}

void adjustDbMin(int delta) { setDbMin(current.dbMin + delta); }
void adjustDbMax(int delta) { setDbMax(current.dbMax + delta); }
void adjustBrightness(int delta) { setBrightness(current.brightness + delta); }

const char* wifiSsid() { return wifiSsidBuf; }
const char* wifiPassword() { return wifiPassBuf; }

void setWifiCredentials(const char* ssid, const char* password) {
  strncpy(wifiSsidBuf, ssid, sizeof(wifiSsidBuf) - 1);
  wifiSsidBuf[sizeof(wifiSsidBuf) - 1] = 0;
  strncpy(wifiPassBuf, password, sizeof(wifiPassBuf) - 1);
  wifiPassBuf[sizeof(wifiPassBuf) - 1] = 0;
  prefs.putString(kKeySsid, wifiSsidBuf);
  prefs.putString(kKeyPass, wifiPassBuf);
}

void tick() {
  if (!dirty) return;
  // Unsigned arithmetic, so this stays correct across the millis() rollover.
  if (millis() - dirtySinceMs < kFlushDelayMs) return;

  if (current.dbMin != stored.dbMin) prefs.putUInt(kKeyDbMin, current.dbMin);
  if (current.dbMax != stored.dbMax) prefs.putUInt(kKeyDbMax, current.dbMax);
  if (current.brightness != stored.brightness) prefs.putUInt(kKeyBright, current.brightness);

  stored = current;
  dirty = false;
}

}  // namespace settings
