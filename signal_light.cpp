#include "signal_light.h"

#include <Arduino.h>
#include <FastLED.h>

#include "config.h"

namespace signal_light {
namespace {

CRGB leds[LED_COUNT];

Mode currentMode = Mode::Auto;
Zone currentZone = Zone::Unknown;

uint32_t manualColor_ = COLOR_QUIET;

// Running average of the measured level. NAN until the first measurement, so
// the filter starts from a real reading rather than easing up from zero.
float smoothedDb = NAN;

// Set whenever the desired output changes; cleared by tick() after a show().
bool dirty = true;

// Non-zero while an acknowledgement flash is running.
uint32_t flashUntilMs = 0;
constexpr uint32_t kFlashDurationMs = 120;

CRGB zoneColor(Zone zone) {
  switch (zone) {
    case Zone::Quiet: return CRGB(COLOR_QUIET);
    case Zone::Warn:  return CRGB(COLOR_WARN);
    case Zone::Loud:  return CRGB(COLOR_LOUD);
    default:          return CRGB::Black;
  }
}

// Picks the zone for a level, requiring it to move DB_HYSTERESIS past a
// threshold before leaving the zone it is in. From Unknown - the first
// measurement after boot - the thresholds apply plainly so the light shows
// something truthful straight away.
Zone zoneFor(float db, float dbMin, float dbMax, Zone from) {
  const float h = DB_HYSTERESIS;
  switch (from) {
    case Zone::Quiet:
      if (db > dbMax + h) return Zone::Loud;
      if (db > dbMin + h) return Zone::Warn;
      return Zone::Quiet;
    case Zone::Warn:
      if (db > dbMax + h) return Zone::Loud;
      if (db < dbMin - h) return Zone::Quiet;
      return Zone::Warn;
    case Zone::Loud:
      if (db < dbMin - h) return Zone::Quiet;
      if (db < dbMax - h) return Zone::Warn;
      return Zone::Loud;
    default:
      if (db > dbMax) return Zone::Loud;
      if (db > dbMin) return Zone::Warn;
      return Zone::Quiet;
  }
}

// The colour the LEDs should be showing right now, given mode and flash state.
CRGB targetColor() {
  if (flashUntilMs != 0) return CRGB::Black;
  switch (currentMode) {
    case Mode::Off:    return CRGB::Black;
    case Mode::Manual: return CRGB(manualColor_);
    default:           return zoneColor(currentZone);
  }
}

}  // namespace

void begin(uint8_t brightness) {
  FastLED.addLeds<NEOPIXEL, PIN_LED_DATA>(leds, LED_COUNT);
  FastLED.setDither(false);
  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(brightness);
  FastLED.setMaxPowerInVoltsAndMilliamps(LED_PSU_VOLTS, LED_PSU_MILLIAMPS);
  if (LED_POWER_INDICATOR_PIN >= 0) set_max_power_indicator_LED(LED_POWER_INDICATOR_PIN);

  // Establish power-on state explicitly rather than relying on static
  // initialisation, so begin() is a real reset point.
  currentMode = Mode::Auto;
  currentZone = Zone::Unknown;
  smoothedDb = NAN;
  flashUntilMs = 0;

  fill_solid(leds, LED_COUNT, CRGB::Black);
  FastLED.show();
  dirty = true;
}

Mode mode() { return currentMode; }
Zone zone() { return currentZone; }
uint32_t manualColor() { return manualColor_; }
float smoothedLevel() { return smoothedDb; }

void setMode(Mode next) {
  if (next == currentMode) return;
  currentMode = next;
  // Re-derive the zone from the next measurement rather than resuming with a
  // stale one, and let the smoothing settle again from a real reading.
  if (next == Mode::Auto) {
    currentZone = Zone::Unknown;
    smoothedDb = NAN;
  }
  dirty = true;
}

void setManualColor(uint32_t rgb) {
  manualColor_ = rgb;
  currentMode = Mode::Manual;
  dirty = true;
}

void setBrightness(uint8_t brightness) {
  FastLED.setBrightness(brightness);
  dirty = true;
}

void updateLevel(float leqDb, uint8_t dbMin, uint8_t dbMax) {
  if (currentMode != Mode::Auto) return;

  smoothedDb = isnan(smoothedDb) ? leqDb
                                 : smoothedDb + DB_SMOOTHING * (leqDb - smoothedDb);

  const Zone next = zoneFor(smoothedDb, dbMin, dbMax, currentZone);
  if (next == currentZone) return;
  currentZone = next;
  dirty = true;
}

void flashAck() {
  flashUntilMs = millis() + kFlashDurationMs;
  if (flashUntilMs == 0) flashUntilMs = 1;  // 0 is the "not flashing" marker
  dirty = true;
}

void tick() {
  // Signed comparison, so this survives the millis() rollover.
  if (flashUntilMs != 0 && static_cast<int32_t>(millis() - flashUntilMs) >= 0) {
    flashUntilMs = 0;
    dirty = true;
  }
  if (!dirty) return;

  // FastLED.show() disables interrupts for the length of the NeoPixel frame,
  // which is long enough to cost the IR receiver an edge. Writing only on an
  // actual change keeps that window rare instead of several times a second.
  fill_solid(leds, LED_COUNT, targetColor());
  FastLED.show();
  dirty = false;
}

}  // namespace signal_light
