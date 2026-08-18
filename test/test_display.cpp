/*
 * display: what reaches the panel, and how often.
 *
 * Pushing a frame blocks loop() for around 22ms while 1KB goes out over I2C,
 * so the interesting property is not what it draws but how rarely. These
 * tests pin down that a level wobbling below the resolution of the screen
 * costs nothing, and that a genuine change is still rate limited.
 */

#include "harness.h"

#include <string.h>

#include <Adafruit_SSD1306.h>

#include "../config.h"
#include "../display.h"
#include "../settings.h"
#include "../signal_light.h"
#include "fakes.h"

namespace {

Settings makeSettings(uint8_t lo = 40, uint8_t hi = 60, uint8_t bright = 255) {
  Settings s{};
  s.dbMin = lo;
  s.dbMax = hi;
  s.brightness = bright;
  s.displayBrightness = DISPLAY_BRIGHTNESS_DEFAULT;
  return s;
}

void show(float db, const Settings& s,
          signal_light::Mode mode = signal_light::Mode::Auto,
          sound_level::Quality q = sound_level::Quality::Ok) {
  display::update(db, q, s, mode, signal_light::Zone::Quiet);
  display::tick();
}

// Enough simulated time for the rate limiter to allow another frame.
void waitOutInterval() { fakes::advanceMillis(DISPLAY_MIN_INTERVAL_MS + 1); }

bool drew(const char* needle) { return strstr(fakes::displayText(), needle) != nullptr; }

}  // namespace

void test_display() {
  SUITE("display");
  const Settings s = makeSettings();

  CASE("a unit built without a panel carries on regardless");
  fakes::reset();
  fakes::setPanelPresent(false);
  CHECK(!display::begin(), "begin() claimed a panel that is not there");
  CHECK(!display::present(), "present() is true with no panel");
  for (int i = 0; i < 20; i++) {
    waitOutInterval();
    show(40.0f + i, s);
  }
  CHECK(fakes::displayFrames() == 0, "drew %d frame(s) with no panel", fakes::displayFrames());

  CASE("a fitted panel is found and drawn on");
  fakes::reset();
  fakes::setPanelPresent(true);
  CHECK(display::begin(), "begin() did not find the panel");
  CHECK(display::present(), "present() is false with a panel fitted");
  waitOutInterval();
  show(52.0f, s);
  CHECK(fakes::displayFrames() >= 1, "nothing was drawn");

  CASE("the frame carries the level, the window and the mode");
  CHECK(drew("52"), "level missing from '%s'", fakes::displayText());
  CHECK(drew(DB_UNITS), "units missing from '%s'", fakes::displayText());
  CHECK(drew("40") && drew("60"), "threshold window missing from '%s'", fakes::displayText());
  CHECK(drew("AUTO"), "mode missing from '%s'", fakes::displayText());

  CASE("an unchanged reading is not redrawn");
  int before = fakes::displayFrames();
  for (int i = 0; i < 50; i++) {
    waitOutInterval();
    show(52.0f, s);
  }
  CHECK(fakes::displayFrames() == before, "%d redundant frame(s)",
        fakes::displayFrames() - before);

  // The screen shows whole decibels and the bar is 128px across a 80dB range,
  // so a fraction of a decibel is often invisible. Redrawing for it would be
  // 22ms of blocked loop for no visible benefit.
  CASE("a change too small to see is not redrawn");
  before = fakes::displayFrames();
  for (int i = 0; i < 20; i++) {
    waitOutInterval();
    show(52.0f + i * 0.001f, s);
  }
  CHECK(fakes::displayFrames() == before, "%d frame(s) for an invisible change",
        fakes::displayFrames() - before);

  CASE("a visible change is drawn");
  waitOutInterval();
  show(70.0f, s);
  CHECK(fakes::displayFrames() == before + 1, "expected one frame, got %d",
        fakes::displayFrames() - before);
  CHECK(drew("70"), "new level missing from '%s'", fakes::displayText());

  CASE("changes are rate limited rather than drawn as fast as they arrive");
  before = fakes::displayFrames();
  for (int i = 0; i < 40; i++) {
    fakes::advanceMillis(10);  // 40 distinct levels over 400ms
    show(40.0f + i, s);
  }
  const int frames = fakes::displayFrames() - before;
  CHECK(frames <= 2, "drew %d frames in 400ms, expected at most 2", frames);
  CHECK(frames >= 1, "rate limiting starved the display entirely");

  CASE("a pending change is drawn once the interval passes");
  before = fakes::displayFrames();
  waitOutInterval();
  display::tick();
  CHECK(fakes::displayFrames() == before + 1, "the staged frame was never drawn");

  CASE("a threshold change is drawn even when the level has not moved");
  waitOutInterval();
  show(70.0f, makeSettings(45, 75));
  CHECK(drew("45") && drew("75"), "new window missing from '%s'", fakes::displayText());

  CASE("mode changes reach the panel");
  waitOutInterval();
  show(70.0f, s, signal_light::Mode::Off);
  CHECK(drew("OFF"), "mode missing from '%s'", fakes::displayText());
  waitOutInterval();
  show(70.0f, s, signal_light::Mode::Manual);
  CHECK(drew("MANUAL"), "mode missing from '%s'", fakes::displayText());

  CASE("microphone range warnings are surfaced");
  waitOutInterval();
  show(116.0f, s, signal_light::Mode::Auto, sound_level::Quality::Overload);
  CHECK(drew("OVER"), "overload not shown in '%s'", fakes::displayText());
  waitOutInterval();
  show(29.0f, s, signal_light::Mode::Auto, sound_level::Quality::BelowNoiseFloor);
  CHECK(drew("QUIET"), "noise floor not shown in '%s'", fakes::displayText());

  CASE("the splash carries the product name and firmware version");
  fakes::reset();
  fakes::setPanelPresent(true);
  display::begin();
  const int beforeSplash = fakes::displayFrames();
  display::splash();
  CHECK(fakes::displayFrames() == beforeSplash + 1, "the splash was not drawn");
  CHECK(drew(PRODUCT_NAME), "product name missing from '%s'", fakes::displayText());
  CHECK(drew(FIRMWARE_VERSION), "version missing from '%s'", fakes::displayText());

  // A measurement arrives within milliseconds of boot, and without a hold it
  // would wipe the splash before anyone could read it.
  CASE("measurements do not wipe the splash while it is held");
  int splashFrames = fakes::displayFrames();
  for (uint32_t elapsed = 100; elapsed < DISPLAY_SPLASH_MS; elapsed += 100) {
    fakes::advanceMillis(100);
    show(55.0f + (elapsed / 100), s);
  }
  CHECK(fakes::displayFrames() == splashFrames, "%d frame(s) drew over the splash",
        fakes::displayFrames() - splashFrames);
  CHECK(drew(PRODUCT_NAME), "the splash was overwritten: '%s'", fakes::displayText());

  CASE("the screen is handed over once the splash has been held");
  fakes::advanceMillis(DISPLAY_SPLASH_MS);
  show(55.0f, s);
  CHECK(fakes::displayFrames() == splashFrames + 1, "expected one frame, got %d",
        fakes::displayFrames() - splashFrames);
  CHECK(!drew(PRODUCT_NAME), "still showing the splash: '%s'", fakes::displayText());
  CHECK(drew("55"), "level missing after the splash: '%s'", fakes::displayText());

  CASE("splash on a unit with no panel does nothing");
  fakes::reset();
  fakes::setPanelPresent(false);
  display::begin();
  display::splash();
  CHECK(fakes::displayFrames() == 0, "drew a splash with no panel");

  CASE("brightness reaches the panel as a contrast level");
  fakes::reset();
  fakes::setPanelPresent(true);
  display::begin();
  display::setBrightness(90);
  CHECK(fakes::displayCommandCount(SSD1306_DISPLAYON) >= 1, "panel was not switched on");
  CHECK(fakes::lastContrast() > 0 && fakes::lastContrast() < 90,
        "contrast %u should be curved below the setting", fakes::lastContrast());

  // A linear slider spends most of its travel in a range that all looks much
  // the same. The curve is what makes the bottom of it useful.
  CASE("the scale is curved, not linear");
  display::setBrightness(DISPLAY_BRIGHTNESS_MAX);
  CHECK(fakes::lastContrast() == DISPLAY_BRIGHTNESS_MAX, "full scale gave %u, want %u",
        fakes::lastContrast(), DISPLAY_BRIGHTNESS_MAX);
  display::setBrightness(128);
  const uint8_t half = fakes::lastContrast();
  CHECK(half > 40 && half < 80, "half travel gave contrast %u, expected roughly a fifth", half);
  // Contrast zero is off, not dim, so no non-zero setting may reach it.
  CASE("no setting above zero leaves the panel dark");
  for (int level = 1; level <= 255; level++) {
    display::setBrightness(uint8_t(level));
    if (fakes::lastContrast() < DISPLAY_CONTRAST_MIN) {
      CHECK(false, "level %d gave contrast %u, below the visible floor", level,
            fakes::lastContrast());
      break;
    }
  }

  // Contrast bottoms out while the panel is still clearly lit, so the low end
  // shortens the pre-charge period as well to get below that floor.
  CASE("pre-charge ramps across the bottom and meets the driver default");
  display::setBrightness(1);
  const uint8_t lowest = fakes::lastCommandValue(SSD1306_SETPRECHARGE);
  CHECK(lowest < DISPLAY_PRECHARGE_NORMAL, "bottom pre-charge 0x%02X is not shortened",
        lowest);
  display::setBrightness(DISPLAY_BRIGHTNESS_MAX);
  CHECK(fakes::lastCommandValue(SSD1306_SETPRECHARGE) == DISPLAY_PRECHARGE_NORMAL,
        "pre-charge 0x%02X at full, want the driver default 0x%02X",
        fakes::lastCommandValue(SSD1306_SETPRECHARGE), DISPLAY_PRECHARGE_NORMAL);

  // A step here would read as the brightness lurching partway along the slider.
  CASE("pre-charge never steps by more than one phase between adjacent levels");
  uint8_t previous = 0;
  for (int level = 1; level <= DISPLAY_DIM_BELOW + 2; level++) {
    display::setBrightness(uint8_t(level));
    const uint8_t phase2 = fakes::lastCommandValue(SSD1306_SETPRECHARGE) >> 4;
    if (level > 1 && (phase2 < previous || phase2 - previous > 1)) {
      CHECK(false, "phase 2 jumped %u -> %u at level %d", previous, phase2, level);
      break;
    }
    previous = phase2;
  }
  CHECK(previous == (DISPLAY_PRECHARGE_NORMAL >> 4),
        "ramp ended at phase 2 = %u, want %u", previous, DISPLAY_PRECHARGE_NORMAL >> 4);

  CASE("brightness is not lost when set again");
  display::setBrightness(90);

  // Zero powers the panel down, which also means there is no point spending
  // 22ms of blocked loop() pushing frames nobody can see.
  CASE("zero powers the panel down and stops frames");
  fakes::advanceMillis(DISPLAY_SPLASH_MS + 1);
  display::setBrightness(0);
  CHECK(fakes::displayCommandCount(SSD1306_DISPLAYOFF) == 1, "panel was not switched off");
  before = fakes::displayFrames();
  for (int i = 0; i < 20; i++) {
    waitOutInterval();
    show(40.0f + i, s);
  }
  CHECK(fakes::displayFrames() == before, "%d frame(s) drawn to a powered-down panel",
        fakes::displayFrames() - before);

  CASE("turning it back on redraws what is current");
  display::setBrightness(200);
  CHECK(fakes::lastContrast() > 0, "panel was not given a contrast on wake");
  waitOutInterval();
  display::tick();
  CHECK(fakes::displayFrames() == before + 1, "the panel was not redrawn on wake");

  // The panel loses its contents when powered down. If nothing has changed in
  // the meantime, the staged frame still matches what was last drawn, so
  // without an explicit invalidation the screen would come back blank and
  // stay that way until the level happened to move.
  CASE("waking an idle panel redraws it even though nothing changed");
  fakes::reset();
  fakes::setPanelPresent(true);
  display::begin();
  display::setBrightness(200);
  fakes::advanceMillis(DISPLAY_SPLASH_MS + 1);
  show(52.0f, s);
  waitOutInterval();
  show(52.0f, s);
  const int idle = fakes::displayFrames();
  display::setBrightness(0);
  display::setBrightness(200);   // nothing measured in between
  waitOutInterval();
  display::tick();
  CHECK(fakes::displayFrames() == idle + 1, "the panel was not redrawn on wake");

  CASE("brightness on a unit with no panel is harmless");
  fakes::reset();
  fakes::setPanelPresent(false);
  display::begin();
  display::setBrightness(120);
  CHECK(fakes::displayCommandCount(SSD1306_DISPLAYON) == 0, "talked to a panel that is absent");

  CASE("the status line is shown and can be changed");
  fakes::reset();
  fakes::setPanelPresent(true);
  display::begin();
  waitOutInterval();
  fakes::advanceMillis(DISPLAY_SPLASH_MS);
  display::setStatus("192.168.4.1");
  waitOutInterval();
  show(50.0f, s);
  CHECK(drew("192.168.4.1"), "status missing from '%s'", fakes::displayText());
}
