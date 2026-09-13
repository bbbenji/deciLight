/*
 * remote_control: the key map and what each key does.
 *
 * The map is a 24-entry table of raw hex codes, which is exactly the shape of
 * thing where a copy-paste slip goes unnoticed. Asserting every key
 * individually catches a duplicated code, because lookup() returns the first
 * match and the shadowed entry would fire the wrong action.
 */

#include "harness.h"

#include <string.h>

#include <FastLED.h>

#include "../config.h"
#include "../remote_control.h"
#include "../settings.h"
#include "../signal_light.h"
#include "fakes.h"

namespace {

// NEC codes, in the physical layout of the 24-key remote.
constexpr uint64_t kBrightUp    = 0xF700FF;
constexpr uint64_t kBrightDown  = 0xF7807F;
constexpr uint64_t kOff         = 0xF740BF;
constexpr uint64_t kOn          = 0xF7C03F;
constexpr uint64_t kFlash       = 0xF7D02F;  // repurposed: lower threshold up
constexpr uint64_t kStrobe      = 0xF7F00F;  // repurposed: lower threshold down
constexpr uint64_t kFade        = 0xF7C837;  // repurposed: upper threshold up
constexpr uint64_t kSmooth      = 0xF7E817;  // repurposed: upper threshold down
constexpr uint64_t kNecRepeat   = 0xFFFFFFFFFFFFFFFFULL;

struct ColorKey {
  uint64_t code;
  uint32_t color;
  const char* label;
};

const ColorKey kColorKeys[] = {
    {0xF720DF, CRGB::Red,          "Red"},
    {0xF7A05F, CRGB::Green,        "Green"},
    {0xF7609F, CRGB::Blue,         "Blue"},
    {0xF7E01F, CRGB::White,        "White"},
    {0xF710EF, CRGB::Tomato,       "Tomato"},
    {0xF7906F, CRGB::LightGreen,   "LightGreen"},
    {0xF750AF, CRGB::SkyBlue,      "SkyBlue"},
    {0xF730CF, CRGB::OrangeRed,    "OrangeRed"},
    {0xF7B04F, CRGB::Cyan,         "Cyan"},
    {0xF7708F, CRGB::Purple,       "Purple"},
    {0xF708F7, CRGB::Orange,       "Orange"},
    {0xF78877, CRGB::Turquoise,    "Turquoise"},
    {0xF748B7, CRGB::MediumPurple, "MediumPurple"},
    {0xF728D7, CRGB::Yellow,       "Yellow"},
    {0xF7A857, CRGB::DarkCyan,     "DarkCyan"},
    {0xF76897, CRGB::Plum,         "Plum"},
};

void press(uint64_t code) {
  fakes::receiveIr(code);
  remote_control::poll();
  signal_light::tick();
}

void start() {
  fakes::reset();
  settings::begin();
  signal_light::begin(LED_BRIGHTNESS_DEFAULT);
  signal_light::setMode(signal_light::Mode::Auto);
  remote_control::begin();
}

}  // namespace

void test_remote_control() {
  SUITE("remote_control");
  const Settings& s = settings::get();

  CASE("every colour key sets its own colour and leaves auto mode");
  for (const ColorKey& key : kColorKeys) {
    start();
    press(key.code);
    CHECK(fakes::lastColor() == key.color, "%s gave 0x%06X, want 0x%06X", key.label,
          fakes::lastColor(), key.color);
    CHECK(signal_light::mode() == signal_light::Mode::Manual, "%s did not switch to manual",
          key.label);
  }

  CASE("on and off switch mode without disturbing the thresholds");
  start();
  const uint8_t min0 = s.dbMin, max0 = s.dbMax;
  press(kOff);
  CHECK(signal_light::mode() == signal_light::Mode::Off, "Off did not switch mode");
  CHECK(fakes::lastColor() == 0, "Off left the ring lit");
  press(kOn);
  CHECK(signal_light::mode() == signal_light::Mode::Auto, "On did not return to auto");
  CHECK(s.dbMin == min0 && s.dbMax == max0, "power keys moved the thresholds");

  CASE("brightness keys step and are forwarded to the driver");
  start();
  const uint8_t bright0 = s.brightness;
  press(kBrightDown);
  CHECK(s.brightness < bright0, "Bright- did not reduce brightness");
  CHECK(fakes::brightness() == s.brightness, "driver brightness %u, settings %u",
        fakes::brightness(), s.brightness);
  press(kBrightUp);
  CHECK(s.brightness == bright0, "Bright+ did not undo Bright-");

  CASE("the four effect keys move the thresholds they are documented to move");
  start();
  press(kFlash);
  CHECK(s.dbMin == min0 + 1, "Flash gave dbMin %u, want %u", s.dbMin, min0 + 1);
  press(kStrobe);
  CHECK(s.dbMin == min0, "Strobe did not undo Flash: %u", s.dbMin);
  press(kFade);
  CHECK(s.dbMax == max0 + 1, "Fade gave dbMax %u, want %u", s.dbMax, max0 + 1);
  press(kSmooth);
  CHECK(s.dbMax == max0, "Smooth did not undo Fade: %u", s.dbMax);

  // The original firmware only applied threshold changes in auto mode, which
  // made three of these four keys silently dead while a colour was held.
  CASE("threshold keys work regardless of mode");
  start();
  press(kOff);
  press(kFlash);
  CHECK(s.dbMin == min0 + 1, "Flash was ignored while off");

  CASE("holding a threshold key repeats, subject to the repeat interval");
  start();
  press(kFlash);
  const uint8_t after1 = s.dbMin;
  press(kNecRepeat);  // too soon
  CHECK(s.dbMin == after1, "repeat fired without waiting for the interval");
  fakes::advanceMillis(500);
  press(kNecRepeat);
  CHECK(s.dbMin == after1 + 1, "held key did not repeat: %u", s.dbMin);

  CASE("holding a colour key does not repeat");
  start();
  press(kColorKeys[0].code);
  const int shows = fakes::showCount();
  for (int i = 0; i < 5; i++) {
    fakes::advanceMillis(500);
    press(kNecRepeat);
  }
  CHECK(fakes::showCount() == shows, "a held colour key rewrote the ring %d time(s)",
        fakes::showCount() - shows);

  CASE("a repeat with no preceding key press is ignored");
  start();
  const uint8_t before = s.dbMin;
  fakes::advanceMillis(500);
  press(kNecRepeat);
  CHECK(s.dbMin == before, "a stray repeat changed dbMin");

  CASE("an unmapped code changes nothing");
  start();
  const uint8_t m0 = s.dbMin, x0 = s.dbMax, b0 = s.brightness;
  press(0xDEADBEEF);
  CHECK(s.dbMin == m0 && s.dbMax == x0 && s.brightness == b0,
        "an unmapped code altered settings");
  CHECK(signal_light::mode() == signal_light::Mode::Auto, "an unmapped code changed the mode");

  CASE("an unmapped code also clears the repeat target");
  start();
  press(kFlash);
  const uint8_t held = s.dbMin;
  press(0xDEADBEEF);
  fakes::advanceMillis(500);
  press(kNecRepeat);
  CHECK(s.dbMin == held, "repeat still fired after an unmapped code");

  // Surfaced over the network so a receiver can be checked, and an unfamiliar
  // remote mapped, without a serial cable.
  // begin() is the module's reset point. Leaving state behind it is benign on
  // a board that boots once, but it is the same slip that left signal_light
  // holding a stale flash timer and display talking to an absent panel.
  CASE("begin() clears any code recorded before it");
  start();
  press(kFlash);
  CHECK(remote_control::haveLastCode(), "precondition failed: nothing was recorded");
  remote_control::begin();
  CHECK(!remote_control::haveLastCode(), "a stale code survived begin()");
  CHECK(remote_control::lastCodeHex()[0] == '\0', "begin() left '%s' behind",
        remote_control::lastCodeHex());
  CHECK(remote_control::lastProtocol()[0] == '\0', "begin() left a stale protocol behind");
  CHECK(!remote_control::lastCodeMapped(), "begin() left the mapped flag set");

  CASE("begin() also clears the hold-to-repeat target");
  start();
  press(kFlash);
  const uint8_t heldBefore = s.dbMin;
  remote_control::begin();
  fakes::advanceMillis(500);
  press(kNecRepeat);
  CHECK(s.dbMin == heldBefore, "a repeat fired against a key press from before begin()");

  CASE("a mapped key is recorded, formatted and flagged as mapped");
  start();
  press(kFlash);
  CHECK(remote_control::haveLastCode(), "nothing was recorded");
  CHECK(strcmp(remote_control::lastCodeHex(), "0xF7D02F") == 0, "recorded '%s', want 0xF7D02F",
        remote_control::lastCodeHex());
  CHECK(remote_control::lastCodeMapped(), "a mapped key was flagged unmapped");

  CASE("an unmapped code is recorded too, and flagged as unmapped");
  press(0xDEADBEEF);
  CHECK(strcmp(remote_control::lastCodeHex(), "0xDEADBEEF") == 0, "recorded '%s'",
        remote_control::lastCodeHex());
  CHECK(!remote_control::lastCodeMapped(), "an unmapped code was flagged mapped");

  CASE("hold-down repeats do not overwrite the code of interest");
  start();
  press(kFlash);
  for (int i = 0; i < 5; i++) {
    fakes::advanceMillis(500);
    press(kNecRepeat);
  }
  CHECK(strcmp(remote_control::lastCodeHex(), "0xF7D02F") == 0,
        "a repeat overwrote the recorded code: '%s'", remote_control::lastCodeHex());

  CASE("age is reported from when the code arrived");
  start();
  press(kColorKeys[0].code);
  CHECK(remote_control::lastCodeAgeMs() == 0, "age %u immediately after receipt",
        remote_control::lastCodeAgeMs());
  fakes::advanceMillis(2500);
  CHECK(remote_control::lastCodeAgeMs() == 2500, "age %u after 2500ms",
        remote_control::lastCodeAgeMs());

  CASE("holding a threshold key cannot push a value past its limit");
  start();
  press(kStrobe);
  for (int i = 0; i < 500; i++) {
    fakes::advanceMillis(500);
    press(kNecRepeat);
  }
  CHECK(s.dbMin == DB_LIMIT_LOW, "held Strobe drove dbMin to %u, want %u", s.dbMin,
        DB_LIMIT_LOW);
}
