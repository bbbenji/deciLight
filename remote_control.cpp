#include "remote_control.h"

#include <Arduino.h>
#include <FastLED.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>

#include "config.h"
#include "settings.h"
#include "signal_light.h"

namespace remote_control {
namespace {

IRrecv irrecv(PIN_IR_RECV);
decode_results results;

// NEC sends this instead of the code while a key is held down.
constexpr uint64_t kNecRepeat = 0xFFFFFFFFFFFFFFFFULL;

// Repeats arrive about every 110ms, which is faster than anyone wants to step
// a threshold. Rate-limit them to something readable.
constexpr uint32_t kRepeatIntervalMs = 200;

enum class Action : uint8_t {
  SetColor,
  PowerOn,
  PowerOff,
  BrightnessUp,
  BrightnessDown,
  DbMinUp,
  DbMinDown,
  DbMaxUp,
  DbMaxDown,
};

struct KeyMapping {
  uint32_t code;
  Action action;
  uint32_t color;      // only meaningful for SetColor
  bool holdToRepeat;   // whether holding the key keeps applying the action
};

// Layout of the remote, row by row. The four "effect" keys along the bottom
// right are repurposed as threshold controls, which is what the labels in the
// comments refer to.
const KeyMapping kKeyMap[] = {
    {0xF700FF, Action::BrightnessUp,   0,                  true},   // Bright+
    {0xF7807F, Action::BrightnessDown, 0,                  true},   // Bright-
    {0xF740BF, Action::PowerOff,       0,                  false},  // Off
    {0xF7C03F, Action::PowerOn,        0,                  false},  // On

    {0xF720DF, Action::SetColor, CRGB::Red,          false},
    {0xF7A05F, Action::SetColor, CRGB::Green,        false},
    {0xF7609F, Action::SetColor, CRGB::Blue,         false},
    {0xF7E01F, Action::SetColor, CRGB::White,        false},

    {0xF710EF, Action::SetColor, CRGB::Tomato,       false},
    {0xF7906F, Action::SetColor, CRGB::LightGreen,   false},
    {0xF750AF, Action::SetColor, CRGB::SkyBlue,      false},
    {0xF7D02F, Action::DbMinUp,  0,                  true},   // Flash

    {0xF730CF, Action::SetColor, CRGB::OrangeRed,    false},
    {0xF7B04F, Action::SetColor, CRGB::Cyan,         false},
    {0xF7708F, Action::SetColor, CRGB::Purple,       false},
    {0xF7F00F, Action::DbMinDown, 0,                 true},   // Strobe

    {0xF708F7, Action::SetColor, CRGB::Orange,       false},
    {0xF78877, Action::SetColor, CRGB::Turquoise,    false},
    {0xF748B7, Action::SetColor, CRGB::MediumPurple, false},
    {0xF7C837, Action::DbMaxUp,  0,                  true},   // Fade

    {0xF728D7, Action::SetColor, CRGB::Yellow,       false},
    {0xF7A857, Action::SetColor, CRGB::DarkCyan,     false},
    {0xF76897, Action::SetColor, CRGB::Plum,         false},
    {0xF7E817, Action::DbMaxDown, 0,                 true},   // Smooth
};

const KeyMapping* lastKey = nullptr;
uint32_t lastAppliedMs = 0;

// Diagnostics for the most recent code, mapped or not.
bool haveLast = false;
char lastHex[19] = "";
char lastProto[12] = "";
bool lastMapped = false;
uint32_t lastSeenMs = 0;

// Written by hand because printf on this platform cannot be relied on for
// 64-bit conversions, and the IR library's helpers all return Strings.
void formatHex(uint64_t value, char* out, size_t size) {
  char digits[17];
  uint8_t n = 0;
  if (value == 0) digits[n++] = '0';
  while (value != 0 && n < sizeof(digits)) {
    const uint8_t nibble = value & 0xF;
    digits[n++] = nibble < 10 ? char('0' + nibble) : char('A' + nibble - 10);
    value >>= 4;
  }
  size_t i = 0;
  if (size > 2) {
    out[i++] = '0';
    out[i++] = 'x';
  }
  while (n > 0 && i + 1 < size) out[i++] = digits[--n];
  out[i] = '\0';
}

const KeyMapping* lookup(uint64_t code) {
  for (const KeyMapping& key : kKeyMap) {
    if (key.code == code) return &key;
  }
  return nullptr;
}

// Acknowledges a threshold change and reports the new window on serial. The
// LEDs are the only display this thing has, so a blink is the whole UI.
void reportThresholds() {
  const Settings& s = settings::get();
  Serial.printf("thresholds: %u - %u " DB_UNITS "\n", s.dbMin, s.dbMax);
  if (signal_light::mode() != signal_light::Mode::Off) signal_light::flashAck();
}

void apply(const KeyMapping& key) {
  switch (key.action) {
    case Action::SetColor:
      signal_light::setManualColor(key.color);
      break;

    case Action::PowerOn:
      signal_light::setMode(signal_light::Mode::Auto);
      break;

    case Action::PowerOff:
      signal_light::setMode(signal_light::Mode::Off);
      break;

    case Action::BrightnessUp:
    case Action::BrightnessDown: {
      const int delta =
          (key.action == Action::BrightnessUp) ? LED_BRIGHTNESS_STEP : -LED_BRIGHTNESS_STEP;
      settings::adjustBrightness(delta);
      signal_light::setBrightness(settings::get().brightness);
      break;
    }

    case Action::DbMinUp:   settings::adjustDbMin(+1);  reportThresholds(); break;
    case Action::DbMinDown: settings::adjustDbMin(-1);  reportThresholds(); break;
    case Action::DbMaxUp:   settings::adjustDbMax(+1);  reportThresholds(); break;
    case Action::DbMaxDown: settings::adjustDbMax(-1);  reportThresholds(); break;
  }
}

}  // namespace

void begin() {
  // Establish a known state rather than relying on static initialisation, so
  // begin() is a real reset point - the same contract the other modules keep.
  lastKey = nullptr;
  lastAppliedMs = 0;
  haveLast = false;
  lastHex[0] = '\0';
  lastProto[0] = '\0';
  lastMapped = false;
  lastSeenMs = 0;

  irrecv.enableIRIn();
}

bool haveLastCode() { return haveLast; }
const char* lastCodeHex() { return lastHex; }
const char* lastProtocol() { return lastProto; }
bool lastCodeMapped() { return lastMapped; }
uint32_t lastCodeAgeMs() { return haveLast ? millis() - lastSeenMs : 0; }

void poll() {
  if (!irrecv.decode(&results)) return;

  // results is our own struct and is only overwritten by the next decode(),
  // so the receiver can go back to listening before any work happens here.
  const uint64_t code = results.value;
  irrecv.resume();

  if (code == kNecRepeat) {
    if (lastKey == nullptr || !lastKey->holdToRepeat) return;
    if (millis() - lastAppliedMs < kRepeatIntervalMs) return;
    lastAppliedMs = millis();
    apply(*lastKey);
    return;
  }

  const KeyMapping* key = lookup(code);

  formatHex(code, lastHex, sizeof(lastHex));
  strncpy(lastProto, typeToString(results.decode_type).c_str(), sizeof(lastProto) - 1);
  lastProto[sizeof(lastProto) - 1] = '\0';
  lastMapped = key != nullptr;
  lastSeenMs = millis();
  haveLast = true;

  if (key == nullptr) {
    lastKey = nullptr;
    // Logged rather than swallowed, so a different remote can be mapped by
    // watching the serial console.
    Serial.print(F("remote: unmapped "));
    Serial.print(typeToString(results.decode_type));
    Serial.print(' ');
    Serial.println(resultToHexidecimal(&results));
    return;
  }

  lastKey = key;
  lastAppliedMs = millis();
  apply(*key);
}

}  // namespace remote_control
