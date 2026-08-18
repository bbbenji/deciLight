#include "fakes.h"

#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <FastLED.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <Preferences.h>
#include <Wire.h>

#include <deque>
#include <map>
#include <string>

SerialStub Serial;
CFastLED FastLED;
TwoWire Wire;
WiFiClass WiFi;

namespace {

unsigned long nowMs = 0;

uint32_t ledColor = 0;
int shows = 0;
uint8_t ledBrightness = 0;

std::map<std::string, uint32_t> nvsUInt;
std::map<std::string, std::string> nvsStr;
int writes = 0;

std::deque<uint64_t> irQueue;

bool panelPresent = true;
int frames = 0;
std::string frameText;    // text drawn in the frame being composed
std::string lastFrame;    // text of the most recently pushed frame
std::map<uint8_t,int> panelCommands;
std::map<uint8_t,uint8_t> panelValues;
uint8_t pendingCommand = 0;  // command still awaiting its parameter byte

esp_now_recv_cb_t recvCb = nullptr;
int packetsSent = 0;
uint8_t lastPacket[64];
int lastPacketLen = 0;
uint8_t radioChannel = 1;
int wifiMode = WIFI_MODE_STA;

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
  panelPresent = true;
  frames = 0;
  frameText.clear();
  lastFrame.clear();
  panelCommands.clear();
  panelValues.clear();
  pendingCommand = 0;
  recvCb = nullptr;
  packetsSent = 0;
  lastPacketLen = 0;
  radioChannel = 1;
  wifiMode = WIFI_MODE_STA;
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

int groupPacketsSent() { return packetsSent; }
int groupPacketLength() { return lastPacketLen; }
const uint8_t* groupPacket() { return lastPacket; }
void deliverGroupPacket(const uint8_t* mac, const uint8_t* data, int len) {
  if (recvCb) recvCb(mac, data, len);
}
void setChannel(uint8_t channel) { radioChannel = channel; }
void setWifiMode(int mode) { wifiMode = mode; }

void setPanelPresent(bool present) { panelPresent = present; }
int displayFrames() { return frames; }
const char* displayText() { return lastFrame.c_str(); }
int displayCommandCount(uint8_t c) { return panelCommands.count(c) ? panelCommands[c] : 0; }
uint8_t lastContrast() { return lastCommandValue(SSD1306_SETCONTRAST); }
uint8_t lastCommandValue(uint8_t c) {
  return panelValues.count(c) ? panelValues[c] : 0;
}

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
  // Faithful to the real library: a missing key returns 0 and leaves the
  // caller's buffer exactly as it was. Clearing it here would be friendlier
  // and would hide the class of bug where a caller assumes otherwise.
  auto it = nvsStr.find(key);
  if (it == nvsStr.end() || max == 0) return 0;

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

// --- I2C ---

void TwoWire::begin(int, int) {}
void TwoWire::setClock(uint32_t) {}

// --- SSD1306 ---

bool Adafruit_SSD1306::begin(uint8_t, uint8_t, bool, bool) { return panelPresent; }

void Adafruit_SSD1306::clearDisplay() { frameText.clear(); }
void Adafruit_SSD1306::ssd1306_command(uint8_t c) {
  // Some commands are followed by a parameter byte written separately.
  if (pendingCommand != 0) {
    panelValues[pendingCommand] = c;
    pendingCommand = 0;
    return;
  }
  panelCommands[c]++;
  if (c == SSD1306_SETCONTRAST || c == SSD1306_SETPRECHARGE) pendingCommand = c;
}
void Adafruit_SSD1306::display() {
  frames++;
  lastFrame = frameText;
}

void Adafruit_SSD1306::setTextColor(uint16_t) {}
void Adafruit_SSD1306::setTextSize(uint8_t) {}
void Adafruit_SSD1306::setCursor(int16_t, int16_t) { frameText += " "; }

void Adafruit_SSD1306::print(const char* s) { frameText += s; }
void Adafruit_SSD1306::print(char c) { frameText += c; }
void Adafruit_SSD1306::print(int v) { frameText += std::to_string(v); }
void Adafruit_SSD1306::print(unsigned int v) { frameText += std::to_string(v); }
void Adafruit_SSD1306::print(long v) { frameText += std::to_string(v); }
void Adafruit_SSD1306::print(unsigned long v) { frameText += std::to_string(v); }
void Adafruit_SSD1306::print(unsigned char v) { frameText += std::to_string((unsigned)v); }
void Adafruit_SSD1306::print(double v, int) { frameText += std::to_string(v); }

void Adafruit_SSD1306::drawRect(int16_t, int16_t, int16_t, int16_t, uint16_t) {}
void Adafruit_SSD1306::fillRect(int16_t, int16_t, int16_t, int16_t, uint16_t) {}
void Adafruit_SSD1306::drawFastVLine(int16_t, int16_t, int16_t, uint16_t) {}
void Adafruit_SSD1306::drawFastHLine(int16_t, int16_t, int16_t, uint16_t) {}

// --- WiFi / ESP-NOW ---

int WiFiClass::getMode() { return wifiMode; }

esp_err_t esp_wifi_get_channel(uint8_t* primary, wifi_second_chan_t* second) {
  if (primary) *primary = radioChannel;
  if (second) *second = 0;
  return ESP_OK;
}

esp_err_t esp_now_init() { return ESP_OK; }
esp_err_t esp_now_deinit() {
  recvCb = nullptr;
  return ESP_OK;
}
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb) {
  recvCb = cb;
  return ESP_OK;
}
esp_err_t esp_now_add_peer(const esp_now_peer_info_t*) { return ESP_OK; }
esp_err_t esp_now_send(const uint8_t*, const uint8_t* data, size_t len) {
  packetsSent++;
  lastPacketLen = int(len < sizeof(lastPacket) ? len : sizeof(lastPacket));
  memcpy(lastPacket, data, size_t(lastPacketLen));
  return ESP_OK;
}
