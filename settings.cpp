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

// Quiet period after the last change before anything is written to flash.
constexpr uint32_t kFlushDelayMs = 3000;

Preferences prefs;
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

  stored = current;
}

const Settings& get() { return current; }

void adjustDbMin(int delta) {
  const int upper = current.dbMax - DB_MIN_SPAN;
  const uint8_t next = clampInt(current.dbMin + delta, DB_LIMIT_LOW, upper);
  if (next == current.dbMin) return;
  current.dbMin = next;
  markDirty();
}

void adjustDbMax(int delta) {
  const int lower = current.dbMin + DB_MIN_SPAN;
  const uint8_t next = clampInt(current.dbMax + delta, lower, DB_LIMIT_HIGH);
  if (next == current.dbMax) return;
  current.dbMax = next;
  markDirty();
}

void adjustBrightness(int delta) {
  const uint8_t next =
      clampInt(current.brightness + delta, LED_BRIGHTNESS_MIN, LED_BRIGHTNESS_MAX);
  if (next == current.brightness) return;
  current.brightness = next;
  markDirty();
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
