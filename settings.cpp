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
constexpr char kKeyScreen[]  = "screen_bri";
constexpr char kKeyGroup[]   = "group";
constexpr char kKeyUnit[]    = "unit";
constexpr char kKeyGrpLevel[] = "grp_level";
constexpr char kKeyZones[]   = "zones";
constexpr char kKeyCombine[] = "combine";
constexpr char kKeyInactive[] = "inactive";
constexpr char kKeySsid[]    = "wifi_ssid";
constexpr char kKeyPass[]    = "wifi_pass";

// Quiet period after the last change before anything is written to flash.
constexpr uint32_t kFlushDelayMs = 3000;

Preferences prefs;

// Sized to the 802.11 maxima: 32-character SSID, 63-character WPA2 passphrase.
char wifiSsidBuf[33] = {0};
char wifiPassBuf[64] = {0};

// Kept out of Settings so the flush can compare it with a strcmp rather than
// the field-by-field equality the numeric settings use.
char groupNameBuf[17] = {0};
char groupNameStored[17] = {0};
char unitNameBuf[GROUP_NAME_MAX + 1] = {0};
char unitNameStored[GROUP_NAME_MAX + 1] = {0};

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

// Only fields that actually differ are written, so a burst of edits that ends
// where it started costs nothing.
void writeNow() {
  if (current.dbMin != stored.dbMin) prefs.putUInt(kKeyDbMin, current.dbMin);
  if (current.dbMax != stored.dbMax) prefs.putUInt(kKeyDbMax, current.dbMax);
  if (current.brightness != stored.brightness) prefs.putUInt(kKeyBright, current.brightness);
  if (current.displayBrightness != stored.displayBrightness)
    prefs.putUInt(kKeyScreen, current.displayBrightness);
  if (current.groupLevel != stored.groupLevel)
    prefs.putUInt(kKeyGrpLevel, current.groupLevel ? 1 : 0);
  if (current.zones != stored.zones) prefs.putUInt(kKeyZones, current.zones);
  if (current.combine != stored.combine) prefs.putUInt(kKeyCombine, current.combine);
  if (current.inactiveLevel != stored.inactiveLevel)
    prefs.putUInt(kKeyInactive, current.inactiveLevel);
  if (strcmp(groupNameBuf, groupNameStored) != 0) {
    prefs.putString(kKeyGroup, groupNameBuf);
    strncpy(groupNameStored, groupNameBuf, sizeof(groupNameStored) - 1);
  }
  if (strcmp(unitNameBuf, unitNameStored) != 0) {
    prefs.putString(kKeyUnit, unitNameBuf);
    strncpy(unitNameStored, unitNameBuf, sizeof(unitNameStored) - 1);
  }

  stored = current;
  dirty = false;
}

}  // namespace

void begin() {
  // Establish a known state rather than relying on static initialisation, so
  // begin() is a real reset point. The credential buffers matter in
  // particular: getString leaves them untouched when the key is absent, so
  // without clearing them first a missing key would read as whatever was
  // there before.
  dirty = false;
  dirtySinceMs = 0;
  wifiSsidBuf[0] = '\0';
  wifiPassBuf[0] = '\0';
  groupNameBuf[0] = '\0';
  groupNameStored[0] = '\0';
  unitNameBuf[0] = '\0';
  unitNameStored[0] = '\0';

  // Opened once and left open; closing and reopening per access costs several
  // milliseconds and gains nothing.
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    Serial.println(F("settings: NVS unavailable, using defaults for this session"));
    current = {DB_MIN_DEFAULT,   DB_MAX_DEFAULT,
               LED_BRIGHTNESS_DEFAULT, DISPLAY_BRIGHTNESS_DEFAULT,
               false,               ZONE_MASK_ALL,
               COMBINE_DEFAULT,     ZONE_INACTIVE_LEVEL_DEFAULT};
    stored = current;
    dirty = false;
    return;
  }

  current.dbMin = clampInt(prefs.getUInt(kKeyDbMin, DB_MIN_DEFAULT), DB_LIMIT_LOW, DB_LIMIT_HIGH);
  current.dbMax = clampInt(prefs.getUInt(kKeyDbMax, DB_MAX_DEFAULT), DB_LIMIT_LOW, DB_LIMIT_HIGH);
  current.brightness = clampInt(prefs.getUInt(kKeyBright, LED_BRIGHTNESS_DEFAULT),
                                LED_BRIGHTNESS_MIN, LED_BRIGHTNESS_MAX);
  current.displayBrightness =
      clampInt(prefs.getUInt(kKeyScreen, DISPLAY_BRIGHTNESS_DEFAULT),
               DISPLAY_BRIGHTNESS_MIN, DISPLAY_BRIGHTNESS_MAX);

  // A unit flashed with an older build may hold a pair that violates the span.
  if (current.dbMax < current.dbMin + DB_MIN_SPAN) {
    current.dbMax = clampInt(current.dbMin + DB_MIN_SPAN, DB_LIMIT_LOW, DB_LIMIT_HIGH);
    current.dbMin = clampInt(current.dbMax - DB_MIN_SPAN, DB_LIMIT_LOW, DB_LIMIT_HIGH);
  }

  current.groupLevel = prefs.getUInt(kKeyGrpLevel, 0) != 0;
  current.zones = clampInt(prefs.getUInt(kKeyZones, ZONE_MASK_ALL), 0, ZONE_MASK_ALL);
  current.combine = clampInt(prefs.getUInt(kKeyCombine, COMBINE_DEFAULT),
                             COMBINE_LOUDEST, COMBINE_AVERAGE);
  current.inactiveLevel = clampInt(prefs.getUInt(kKeyInactive, ZONE_INACTIVE_LEVEL_DEFAULT),
                                   0, LED_BRIGHTNESS_MAX);

  // A unit whose mask ended up empty would never light at all, which reads as
  // a dead unit rather than a configuration mistake.
  if (current.zones == 0) current.zones = ZONE_MASK_ALL;

  prefs.getString(kKeySsid, wifiSsidBuf, sizeof(wifiSsidBuf));
  prefs.getString(kKeyPass, wifiPassBuf, sizeof(wifiPassBuf));
  prefs.getString(kKeyGroup, groupNameBuf, sizeof(groupNameBuf));
  strncpy(groupNameStored, groupNameBuf, sizeof(groupNameStored) - 1);
  prefs.getString(kKeyUnit, unitNameBuf, sizeof(unitNameBuf));
  strncpy(unitNameStored, unitNameBuf, sizeof(unitNameStored) - 1);

  stored = current;
  dirty = false;
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

void setDisplayBrightness(int value) {
  const uint8_t next = clampInt(value, DISPLAY_BRIGHTNESS_MIN, DISPLAY_BRIGHTNESS_MAX);
  if (next == current.displayBrightness) return;
  current.displayBrightness = next;
  markDirty();
}

void setGroupLevel(bool enabled) {
  if (enabled == current.groupLevel) return;
  current.groupLevel = enabled;
  markDirty();
}

void setZones(int mask) {
  const uint8_t next = clampInt(mask, 0, ZONE_MASK_ALL);
  // Refuse to leave a unit with nothing to light; that looks like a fault.
  if (next == 0 || next == current.zones) return;
  current.zones = next;
  markDirty();
}

void setCombine(int mode) {
  const uint8_t next = clampInt(mode, COMBINE_LOUDEST, COMBINE_AVERAGE);
  if (next == current.combine) return;
  current.combine = next;
  markDirty();
}

void setInactiveLevel(int value) {
  const uint8_t next = clampInt(value, 0, LED_BRIGHTNESS_MAX);
  if (next == current.inactiveLevel) return;
  current.inactiveLevel = next;
  markDirty();
}

const char* groupName() { return groupNameBuf; }

const char* unitName() { return unitNameBuf; }

void setUnitName(const char* name) {
  strncpy(unitNameBuf, name ? name : "", sizeof(unitNameBuf) - 1);
  unitNameBuf[sizeof(unitNameBuf) - 1] = '\0';
  markDirty();
}

void setGroupName(const char* name) {
  strncpy(groupNameBuf, name ? name : "", sizeof(groupNameBuf) - 1);
  groupNameBuf[sizeof(groupNameBuf) - 1] = '\0';
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

void flush() {
  if (dirty) writeNow();
}

void tick() {
  if (!dirty) return;
  // Unsigned arithmetic, so this stays correct across the millis() rollover.
  if (millis() - dirtySinceMs < kFlushDelayMs) return;
  writeNow();
}

}  // namespace settings
