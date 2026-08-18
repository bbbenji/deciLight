#include "fakes.h"

#include <Arduino.h>
#include <FastLED.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <Preferences.h>

#include <deque>
#include <map>
#include <string>

SerialStub Serial;
CFastLED FastLED;

namespace {

unsigned long nowMs = 0;

uint32_t ledColor = 0;
int shows = 0;
uint8_t ledBrightness = 0;

std::map<std::string, uint32_t> nvsUInt;
std::map<std::string, std::string> nvsStr;
int writes = 0;

std::deque<uint64_t> irQueue;

}  // namespace

namespace fakes {

void reset() {
  nowMs = 0;
  ledColor = 0;
  shows = 0;
  ledBrightness = 0;
  nvsUInt.clear();
  nvsStr.clear();
  writes = 0;
  irQueue.clear();
}

void advanceMillis(uint32_t ms) { nowMs += ms; }

uint32_t lastColor() { return ledColor; }
int showCount() { return shows; }
uint8_t brightness() { return ledBrightness; }

int nvsWrites() { return writes; }
void seedUInt(const char* key, uint32_t value) { nvsUInt[key] = value; }
uint32_t storedUInt(const char* key, uint32_t fallback) {
  auto it = nvsUInt.find(key);
  return it == nvsUInt.end() ? fallback : it->second;
}

void receiveIr(uint64_t code) { irQueue.push_back(code); }

}  // namespace fakes

// --- Arduino core ---

unsigned long millis() { return nowMs; }
void delay(unsigned long ms) { nowMs += ms; }
bool setCpuFrequencyMhz(uint32_t) { return true; }

// FreeRTOS and I2S are deliberately left undefined. Only sound_level.cpp
// needs them, and that module is syntax-checked rather than linked, so a fake
// here would be dead weight that drifts out of step with the real driver.

// --- FastLED ---

void fill_solid(CRGB* leds, int count, const CRGB& color) {
  for (int i = 0; i < count; i++) leds[i] = color;
  ledColor = color.packed();
}
void set_max_power_indicator_LED(uint8_t) {}
void CFastLED::setDither(bool) {}
void CFastLED::setCorrection(uint32_t) {}
void CFastLED::setBrightness(uint8_t value) { ledBrightness = value; }
void CFastLED::setMaxPowerInVoltsAndMilliamps(uint8_t, uint16_t) {}
void CFastLED::show() { shows++; }

// --- Preferences ---

bool Preferences::begin(const char*, bool) { return true; }
void Preferences::end() {}

uint32_t Preferences::getUInt(const char* key, uint32_t defaultValue) {
  auto it = nvsUInt.find(key);
  return it == nvsUInt.end() ? defaultValue : it->second;
}
size_t Preferences::putUInt(const char* key, uint32_t value) {
  writes++;
  nvsUInt[key] = value;
  return sizeof(uint32_t);
}
size_t Preferences::getString(const char* key, char* out, size_t max) {
  auto it = nvsStr.find(key);
  if (it == nvsStr.end() || max == 0) {
    if (max) out[0] = '\0';
    return 0;
  }
  const size_t n = it->second.copy(out, max - 1);
  out[n] = '\0';
  return n;
}
size_t Preferences::putString(const char* key, const char* value) {
  writes++;
  nvsStr[key] = value;
  return strlen(value);
}

// --- IR ---

void IRrecv::enableIRIn() {}
void IRrecv::resume() {}
bool IRrecv::decode(decode_results* results) {
  if (irQueue.empty()) return false;
  results->decode_type = NEC;
  results->value = irQueue.front();
  irQueue.pop_front();
  return true;
}

String typeToString(const decode_type_t, const bool) { return "NEC"; }
String resultToHexidecimal(const decode_results* const) { return "0x0"; }
