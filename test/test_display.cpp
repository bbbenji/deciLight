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

#include "../config.h"
#include "../display.h"
#include "../settings.h"
#include "../signal_light.h"
#include "fakes.h"

namespace {

Settings makeSettings(uint8_t lo = 40, uint8_t hi = 60, uint8_t bright = 255) {
  Settings s;
  s.dbMin = lo;
  s.dbMax = hi;
  s.brightness = bright;
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

  CASE("the status line is shown and can be changed");
  display::setStatus("192.168.4.1");
  waitOutInterval();
  show(50.0f, s);
  CHECK(drew("192.168.4.1"), "status missing from '%s'", fakes::displayText());
}
