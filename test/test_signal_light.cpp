/*
 * signal_light: colour mapping, dampening, and how often the ring is rewritten.
 *
 * The dampening tests are the point of this file. FastLED.show() disables
 * interrupts for the length of a NeoPixel frame, so a light that changes
 * colour on every measurement both looks bad and costs the IR receiver edges.
 */

#include "harness.h"

#include <string.h>

#include <FastLED.h>

#include "../config.h"
#include "../signal_light.h"
#include "fakes.h"

namespace {

// Drives enough measurements at one level for the smoothing filter to settle.
void settleAt(float db, uint8_t dbMin, uint8_t dbMax) {
  for (int i = 0; i < 40; i++) signal_light::updateLevel(db, dbMin, dbMax);
  signal_light::tick();
}

const char* colorName(uint32_t c) {
  switch (c) {
    case COLOR_QUIET: return "green";
    case COLOR_WARN:  return "yellow";
    case COLOR_LOUD:  return "red";
    case 0:           return "black";
    default:          return "other";
  }
}

constexpr uint8_t kMin = 40;
constexpr uint8_t kMax = 60;

void start() {
  fakes::reset();
  signal_light::begin(LED_BRIGHTNESS_DEFAULT);
  signal_light::setMode(signal_light::Mode::Auto);
}

}  // namespace

void test_signal_light() {
  SUITE("signal_light");

  CASE("level maps to the expected colour in each band");
  start();
  settleAt(30, kMin, kMax);
  CHECK(fakes::lastColor() == COLOR_QUIET, "quiet room is %s", colorName(fakes::lastColor()));
  settleAt(50, kMin, kMax);
  CHECK(fakes::lastColor() == COLOR_WARN, "mid room is %s", colorName(fakes::lastColor()));
  settleAt(90, kMin, kMax);
  CHECK(fakes::lastColor() == COLOR_LOUD, "loud room is %s", colorName(fakes::lastColor()));

  CASE("hysteresis: dithering across a threshold never changes the colour");
  start();
  settleAt(30, kMin, kMax);
  int shows = fakes::showCount();
  for (int i = 0; i < 200; i++) {
    signal_light::updateLevel(i % 2 ? kMin + 0.5f : kMin - 0.5f, kMin, kMax);
    signal_light::tick();
  }
  CHECK(fakes::showCount() == shows, "ring was rewritten %d time(s) while dithering",
        fakes::showCount() - shows);

  CASE("hysteresis: the same dither on the upper threshold is also stable");
  start();
  settleAt(90, kMin, kMax);
  shows = fakes::showCount();
  for (int i = 0; i < 200; i++) {
    signal_light::updateLevel(i % 2 ? kMax + 0.5f : kMax - 0.5f, kMin, kMax);
    signal_light::tick();
  }
  CHECK(fakes::showCount() == shows, "ring was rewritten %d time(s) while dithering",
        fakes::showCount() - shows);

  CASE("a clean sweep up passes through exactly green, yellow, red");
  start();
  settleAt(30, kMin, kMax);
  uint32_t seen[8];
  int n = 0;
  uint32_t last = fakes::lastColor();
  for (float db = 30; db <= 100; db += 0.5f) {
    settleAt(db, kMin, kMax);
    if (fakes::lastColor() != last && n < 8) seen[n++] = last = fakes::lastColor();
  }
  CHECK(n == 2 && seen[0] == COLOR_WARN && seen[1] == COLOR_LOUD,
        "%d transition(s), first was %s", n, n ? colorName(seen[0]) : "none");

  CASE("and back down again");
  n = 0;
  for (float db = 100; db >= 30; db -= 0.5f) {
    settleAt(db, kMin, kMax);
    if (fakes::lastColor() != last && n < 8) seen[n++] = last = fakes::lastColor();
  }
  CHECK(n == 2 && seen[0] == COLOR_WARN && seen[1] == COLOR_QUIET,
        "%d transition(s), first was %s", n, n ? colorName(seen[0]) : "none");

  CASE("the ring is only rewritten when the colour actually changes");
  start();
  settleAt(30, kMin, kMax);
  shows = fakes::showCount();
  for (int i = 0; i < 100; i++) {
    signal_light::updateLevel(30.0f, kMin, kMax);
    signal_light::tick();
  }
  CHECK(fakes::showCount() == shows, "%d redundant show() call(s)", fakes::showCount() - shows);

  CASE("smoothing damps a single-measurement spike");
  start();
  settleAt(30, kMin, kMax);
  signal_light::updateLevel(95.0f, kMin, kMax);  // one loud burst
  signal_light::tick();
  CHECK(fakes::lastColor() != COLOR_LOUD,
        "one spike drove the light straight to %s", colorName(fakes::lastColor()));

  CASE("a sustained loud level still gets through");
  settleAt(95.0f, kMin, kMax);
  CHECK(fakes::lastColor() == COLOR_LOUD, "sustained noise shows %s",
        colorName(fakes::lastColor()));

  CASE("manual mode holds its colour against incoming measurements");
  start();
  signal_light::setManualColor(CRGB::Blue);
  signal_light::tick();
  settleAt(95.0f, kMin, kMax);
  CHECK(fakes::lastColor() == uint32_t(CRGB::Blue), "manual colour was overwritten");
  CHECK(signal_light::mode() == signal_light::Mode::Manual, "mode is not Manual");

  CASE("off stays dark while measurement continues");
  start();
  signal_light::setMode(signal_light::Mode::Off);
  signal_light::tick();
  settleAt(95.0f, kMin, kMax);
  CHECK(fakes::lastColor() == 0, "light lit while off: %s", colorName(fakes::lastColor()));

  CASE("returning to auto re-derives the zone from fresh measurements");
  signal_light::setMode(signal_light::Mode::Auto);
  settleAt(30.0f, kMin, kMax);
  CHECK(fakes::lastColor() == COLOR_QUIET, "resumed as %s", colorName(fakes::lastColor()));

  CASE("the acknowledgement flash blanks briefly and restores itself");
  start();
  settleAt(30, kMin, kMax);
  signal_light::flashAck();
  signal_light::tick();
  CHECK(fakes::lastColor() == 0, "flash did not blank the ring");
  fakes::advanceMillis(50);
  signal_light::tick();
  CHECK(fakes::lastColor() == 0, "flash ended too early");
  fakes::advanceMillis(200);
  signal_light::tick();
  CHECK(fakes::lastColor() == COLOR_QUIET, "flash did not restore the signal colour, got %s",
        colorName(fakes::lastColor()));

  CASE("brightness is forwarded to the driver");
  start();
  signal_light::setBrightness(77);
  signal_light::tick();
  CHECK(fakes::brightness() == 77, "brightness %u, want 77", fakes::brightness());

  // A narrow window is reachable from the remote, and naive hysteresis would
  // let the two thresholds' guard bands overlap and strand the light.
  // The sequence exists so a freshly wired ring can be compared against what
  // it is meant to be showing, so the colours must be exactly those and the
  // labels must agree with them.
  CASE("the self test walks red, green, blue, white");
  start();
  settleAt(30, kMin, kMax);
  signal_light::startSelfTest();
  signal_light::tick();
  CHECK(signal_light::selfTestRunning(), "the test did not start");

  const char* wantLabel[] = {"red", "green", "blue", "white", "off"};
  const uint32_t wantColor[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF, 0x000000};
  for (int i = 0; i < 5; i++) {
    CHECK(fakes::lastColor() == wantColor[i], "step %d showed 0x%06X, want 0x%06X", i,
          fakes::lastColor(), wantColor[i]);
    CHECK(strcmp(signal_light::selfTestLabel(), wantLabel[i]) == 0,
          "step %d is labelled '%s', want '%s'", i, signal_light::selfTestLabel(),
          wantLabel[i]);
    fakes::advanceMillis(1500);
    signal_light::tick();
  }
  CHECK(!signal_light::selfTestRunning(), "the test never finished");

  CASE("measurements cannot take the ring over mid-test");
  start();
  settleAt(30, kMin, kMax);
  signal_light::startSelfTest();
  signal_light::tick();
  for (int i = 0; i < 20; i++) {
    signal_light::updateLevel(95.0f, kMin, kMax);  // loud enough to force red
    signal_light::tick();
  }
  CHECK(fakes::lastColor() == 0xFF0000 && signal_light::selfTestRunning(),
        "a measurement disturbed the test");

  CASE("the mode carries on underneath the test");
  start();
  signal_light::setManualColor(CRGB::Blue);
  signal_light::tick();
  signal_light::startSelfTest();
  for (int i = 0; i < 6; i++) {
    fakes::advanceMillis(1500);
    signal_light::tick();
  }
  CHECK(signal_light::mode() == signal_light::Mode::Manual, "mode was disturbed");
  CHECK(fakes::lastColor() == uint32_t(CRGB::Blue), "colour did not resume: 0x%06X",
        fakes::lastColor());

  CASE("a colour chosen during the test survives it");
  start();
  settleAt(30, kMin, kMax);
  signal_light::startSelfTest();
  signal_light::tick();
  signal_light::setManualColor(CRGB::Cyan);   // picked mid-test
  for (int i = 0; i < 6; i++) {
    fakes::advanceMillis(1500);
    signal_light::tick();
  }
  CHECK(fakes::lastColor() == uint32_t(CRGB::Cyan), "the mid-test choice was reverted: 0x%06X",
        fakes::lastColor());

  CASE("a test started from auto returns to following the level");
  start();
  settleAt(30, kMin, kMax);
  signal_light::startSelfTest();
  for (int i = 0; i < 6; i++) {
    fakes::advanceMillis(1500);
    signal_light::tick();
  }
  CHECK(signal_light::mode() == signal_light::Mode::Auto, "mode was disturbed");
  settleAt(95.0f, kMin, kMax);
  CHECK(fakes::lastColor() == COLOR_LOUD, "did not resume following the level: %s",
        colorName(fakes::lastColor()));

  CASE("a minimum-width threshold window still resolves to a colour");
  start();
  const uint8_t narrowMin = 50, narrowMax = 50 + DB_MIN_SPAN;
  settleAt(30, narrowMin, narrowMax);
  CHECK(fakes::lastColor() == COLOR_QUIET, "below a narrow window: %s",
        colorName(fakes::lastColor()));
  settleAt(90, narrowMin, narrowMax);
  CHECK(fakes::lastColor() == COLOR_LOUD, "above a narrow window: %s",
        colorName(fakes::lastColor()));
}
