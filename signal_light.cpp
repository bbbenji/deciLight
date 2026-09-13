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

uint8_t zoneMask = ZONE_MASK_ALL;
uint8_t inactiveLevel = 0;

// Running average of the measured level. NAN until the first measurement, so
// the filter starts from a real reading rather than easing up from zero.
float smoothedDb = NAN;

// Set whenever the desired output changes; cleared by tick() after a show().
bool dirty = true;

// Non-zero while an acknowledgement flash is running.
uint32_t flashUntilMs = 0;
constexpr uint32_t kFlashDurationMs = 120;

// The self-test sequence. White is last because it is the worst case for the
// power budget, so if the supply is marginal that is where it shows.
struct TestStep {
  uint32_t rgb;
  uint32_t ms;
  const char* label;
};
const TestStep kSelfTest[] = {
    {0xFF0000, 1200, "red"},
    {0x00FF00, 1200, "green"},
    {0x0000FF, 1200, "blue"},
    {0xFFFFFF, 1200, "white"},
    {0x000000, 400, "off"},
};
constexpr uint8_t kSelfTestSteps = sizeof(kSelfTest) / sizeof(kSelfTest[0]);

bool testActive = false;
uint8_t testStep = 0;
uint32_t testStepUntilMs = 0;

uint8_t maskBit(Zone zone) {
  switch (zone) {
    case Zone::Quiet: return ZONE_MASK_QUIET;
    case Zone::Warn:  return ZONE_MASK_WARN;
    case Zone::Loud:  return ZONE_MASK_LOUD;
    default:          return 0;
  }
}

// Scaled by hand rather than with FastLED's nscale8 so the arithmetic is the
// same on the host, where the tests check it.
CRGB dimmed(uint32_t rgb, uint8_t level) {
  const uint32_t r = ((rgb >> 16) & 0xFF) * level / 255;
  const uint32_t g = ((rgb >> 8) & 0xFF) * level / 255;
  const uint32_t b = (rgb & 0xFF) * level / 255;
  return CRGB((r << 16) | (g << 8) | b);
}

// The colour a unit stands for: the most severe zone it covers. A unit
// holding quiet and warn is a yellow lamp that also shows green, so yellow is
// what it dims to.
uint32_t lampColor() {
  if (zoneMask & ZONE_MASK_LOUD) return COLOR_LOUD;
  if (zoneMask & ZONE_MASK_WARN) return COLOR_WARN;
  return COLOR_QUIET;
}

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
  // Outranks everything else: the point of the test is that what is on the
  // ring is known, not derived.
  if (testActive) return CRGB(kSelfTest[testStep].rgb);
  if (flashUntilMs != 0) return CRGB::Black;
  switch (currentMode) {
    case Mode::Off:    return CRGB::Black;
    case Mode::Manual: return CRGB(manualColor_);
    default: {
      const uint8_t bit = maskBit(currentZone);
      // A zone this unit does not cover means another unit in the stack is
      // showing it, so this one steps back.
      if (bit != 0 && (zoneMask & bit) == 0) {
        return inactiveLevel == 0 ? CRGB(CRGB::Black) : dimmed(lampColor(), inactiveLevel);
      }
      return zoneColor(currentZone);
    }
  }
}


bool bootActive = false;
uint32_t bootUntilMs = 0;
bool updatingActive = false;

CRGB hsvToRgb(uint8_t h, uint8_t s, uint8_t v) {
  uint8_t region = h / 43;
  uint8_t remainder = (h - (region * 43)) * 6;
  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
    case 0:  return CRGB((uint32_t(v) << 16) | (uint32_t(t) << 8) | p);
    case 1:  return CRGB((uint32_t(q) << 16) | (uint32_t(v) << 8) | p);
    case 2:  return CRGB((uint32_t(p) << 16) | (uint32_t(v) << 8) | t);
    case 3:  return CRGB((uint32_t(p) << 16) | (uint32_t(q) << 8) | v);
    case 4:  return CRGB((uint32_t(t) << 16) | (uint32_t(p) << 8) | v);
    default: return CRGB((uint32_t(v) << 16) | (uint32_t(p) << 8) | q);
  }
}

uint8_t sin8(uint8_t theta) {
  const float rad = (float(theta) / 255.0f) * 6.2831853f;
  return uint8_t((sinf(rad) + 1.0f) * 127.5f);
}

void fillGeminiRainbow(CRGB* targetLeds, int count, uint32_t now) {
  if (count <= 0) return;
  // Base hue advances smoothly without wrapping jumps
  const uint8_t baseHue = uint8_t(now / 8);
  targetLeds[0] = hsvToRgb(baseHue, 255, 255);

  const int outerCount = count > 1 ? count - 1 : 1;
  // Rotation phase advances continuously in uint8_t modulo arithmetic
  const uint8_t rotPhase = uint8_t(now / 4);

  for (int i = 1; i < count; i++) {
    // Spatial angle around outer ring (0 to 255)
    uint8_t ringAngle = uint8_t((i - 1) * 256 / outerCount);
    // Two opposing color poles (180 degrees apart in position)
    uint8_t wave = sin8(ringAngle + rotPhase);
    uint8_t hueOffset = (wave * 120) / 255;
    uint8_t hue = baseHue + hueOffset;
    targetLeds[i] = hsvToRgb(hue, 255, 255);
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
  testActive = false;
  testStep = 0;
  zoneMask = ZONE_MASK_ALL;
  inactiveLevel = 0;
  bootActive = false;
  bootUntilMs = 0;
  updatingActive = false;

  fill_solid(leds, LED_COUNT, CRGB::Black);
  FastLED.show();
  dirty = true;
}

Mode mode() { return currentMode; }
Zone zone() { return currentZone; }
uint32_t manualColor() { return manualColor_; }
float smoothedLevel() { return smoothedDb; }


void startBootEffect(uint32_t durationMs) {
  bootActive = true;
  bootUntilMs = millis() + durationMs;
  dirty = true;
}

void setUpdatingEffect(bool active) {
  updatingActive = active;
  dirty = true;
}

bool isBooting() { return bootActive; }
bool isUpdating() { return updatingActive; }

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

void setZones(uint8_t mask, uint8_t inactive) {
  // An empty mask would leave the unit permanently dark, which reads as a
  // fault rather than a setting.
  const uint8_t next = (mask & ZONE_MASK_ALL) == 0 ? ZONE_MASK_ALL : uint8_t(mask & ZONE_MASK_ALL);
  if (next == zoneMask && inactive == inactiveLevel) return;
  zoneMask = next;
  inactiveLevel = inactive;
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

void startSelfTest() {
  // The mode is deliberately left alone. Only what reaches the ring is
  // overridden, so a colour or mode chosen while the test is running still
  // takes effect the moment it ends, rather than being reverted.
  testActive = true;
  testStep = 0;
  testStepUntilMs = millis() + kSelfTest[0].ms;
  dirty = true;
}

bool selfTestRunning() { return testActive; }

const char* selfTestLabel() { return testActive ? kSelfTest[testStep].label : ""; }

void flashAck() {
  flashUntilMs = millis() + kFlashDurationMs;
  if (flashUntilMs == 0) flashUntilMs = 1;  // 0 is the "not flashing" marker
  dirty = true;
}

void tick() {
  const uint32_t now = millis();

  if (bootActive) {
    if (static_cast<int32_t>(now - bootUntilMs) >= 0) {
      bootActive = false;
    } else {
      dirty = true;
    }
  }
  if (updatingActive) {
    dirty = true;
  }

  // Signed comparisons throughout, so these survive the millis() rollover.
  if (testActive && static_cast<int32_t>(now - testStepUntilMs) >= 0) {
    testStep++;
    if (testStep >= kSelfTestSteps) {
      testActive = false;
    } else {
      testStepUntilMs = now + kSelfTest[testStep].ms;
    }
    dirty = true;
  }

  if (flashUntilMs != 0 && static_cast<int32_t>(now - flashUntilMs) >= 0) {
    flashUntilMs = 0;
    dirty = true;
  }
  if (!dirty) return;

  if (bootActive || updatingActive) {
    fillGeminiRainbow(leds, LED_COUNT, now);
  } else {
    fill_solid(leds, LED_COUNT, targetColor());
  }
  FastLED.show();
  dirty = false;
}

}  // namespace signal_light
