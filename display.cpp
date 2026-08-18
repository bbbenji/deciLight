#include "display.h"

#if FEATURE_DISPLAY

#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

namespace display {
namespace {

Adafruit_SSD1306 panel(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, /*rst_pin=*/-1);

bool fitted = false;
char status[22] = "";

// Everything that ends up on screen, reduced to values that can be compared.
// Redrawing is decided by comparing two of these rather than by guessing from
// the inputs, so a change that is not visible - a fraction of a decibel that
// rounds to the same integer - costs nothing.
struct Frame {
  int16_t db;
  uint8_t dbMin;
  uint8_t dbMax;
  uint8_t mode;
  uint8_t quality;
  uint8_t needleX;
  char status[22];

  bool operator==(const Frame& other) const {
    return db == other.db && dbMin == other.dbMin && dbMax == other.dbMax &&
           mode == other.mode && quality == other.quality && needleX == other.needleX &&
           strncmp(status, other.status, sizeof(status)) == 0;
  }
};

Frame staged = {};
Frame shown = {};
bool dirty = false;
uint32_t lastDrawMs = 0;

// Bar geometry. The scale spans the same range the thresholds are allowed to
// take, so a marker can never sit off the end of it.
constexpr int16_t BAR_X = 0;
constexpr int16_t BAR_Y = 44;
constexpr int16_t BAR_W = DISPLAY_WIDTH;
constexpr int16_t BAR_H = 10;

int16_t xForDb(float db) {
  const float span = float(DB_LIMIT_HIGH - DB_LIMIT_LOW);
  const float t = (db - float(DB_LIMIT_LOW)) / span;
  const int16_t x = int16_t(t * (BAR_W - 1) + 0.5f);
  return x < 0 ? 0 : (x > BAR_W - 1 ? BAR_W - 1 : x);
}

const char* modeText(signal_light::Mode mode) {
  switch (mode) {
    case signal_light::Mode::Auto:   return "AUTO";
    case signal_light::Mode::Manual: return "MANUAL";
    default:                         return "OFF";
  }
}

// Bottom right. The microphone's limits are worth calling out, because a
// reading pinned at either end usually means a wiring problem rather than a
// very quiet or very loud room.
const char* qualityText(sound_level::Quality quality) {
  switch (quality) {
    case sound_level::Quality::Overload:        return "OVER";
    case sound_level::Quality::BelowNoiseFloor: return "QUIET";
    default:                                    return "";
  }
}

void draw(const Frame& frame) {
  panel.clearDisplay();
  panel.setTextColor(SSD1306_WHITE);

  // Top row: mode on the left, network status on the right.
  panel.setTextSize(1);
  panel.setCursor(0, 0);
  panel.print(frame.mode == uint8_t(signal_light::Mode::Auto)     ? "AUTO"
              : frame.mode == uint8_t(signal_light::Mode::Manual) ? "MANUAL"
                                                                  : "OFF");
  if (frame.status[0] != '\0') {
    const int16_t w = int16_t(strlen(frame.status)) * 6;
    panel.setCursor(DISPLAY_WIDTH - w, 0);
    panel.print(frame.status);
  }

  // The level itself, as large as will fit.
  panel.setTextSize(3);
  panel.setCursor(0, 14);
  panel.print(frame.db);
  panel.setTextSize(1);
  panel.print(" ");
  panel.print(DB_UNITS);

  // Threshold markers sit above the bar so the fill cannot hide them.
  panel.drawFastVLine(xForDb(frame.dbMin), BAR_Y - 4, 3, SSD1306_WHITE);
  panel.drawFastVLine(xForDb(frame.dbMax), BAR_Y - 4, 3, SSD1306_WHITE);

  panel.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, SSD1306_WHITE);
  if (frame.needleX > 1) {
    panel.fillRect(BAR_X + 1, BAR_Y + 2, frame.needleX - 1, BAR_H - 4, SSD1306_WHITE);
  }

  // Bottom row: the window on the left, any warning on the right.
  panel.setTextSize(1);
  panel.setCursor(0, 56);
  panel.print(frame.dbMin);
  panel.print(" - ");
  panel.print(frame.dbMax);
  const char* note = qualityText(sound_level::Quality(frame.quality));
  if (note[0] != '\0') {
    const int16_t w = int16_t(strlen(note)) * 6;
    panel.setCursor(DISPLAY_WIDTH - w, 56);
    panel.print(note);
  }

  panel.display();
}

}  // namespace

bool begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  for (uint8_t address : DISPLAY_ADDRESSES) {
    // periphBegin is false because Wire is already up on our own pins; letting
    // the library call Wire.begin() would put it back on the defaults.
    if (panel.begin(SSD1306_SWITCHCAPVCC, address, /*reset=*/false, /*periphBegin=*/false)) {
      Serial.printf("display: SSD1306 at 0x%02X\n", address);
      fitted = true;
      panel.clearDisplay();
      panel.display();
      dirty = true;
      return true;
    }
  }

  Serial.println(F("display: none found, continuing without one"));
  return false;
}

bool present() { return fitted; }

void setStatus(const char* text) {
  strncpy(status, text ? text : "", sizeof(status) - 1);
  status[sizeof(status) - 1] = '\0';
}

void update(float leqDb, sound_level::Quality quality, const Settings& settings,
            signal_light::Mode mode, signal_light::Zone /*zone*/) {
  if (!fitted) return;

  Frame frame = {};
  frame.db = int16_t(leqDb + 0.5f);
  frame.dbMin = settings.dbMin;
  frame.dbMax = settings.dbMax;
  frame.mode = uint8_t(mode);
  frame.quality = uint8_t(quality);
  frame.needleX = uint8_t(xForDb(leqDb));
  strncpy(frame.status, status, sizeof(frame.status) - 1);

  if (frame == staged) return;
  staged = frame;
  dirty = true;
}

void tick() {
  if (!fitted || !dirty) return;
  if (staged == shown) {
    dirty = false;
    return;
  }
  // Unsigned arithmetic, so this survives the millis() rollover.
  if (millis() - lastDrawMs < DISPLAY_MIN_INTERVAL_MS) return;

  draw(staged);
  shown = staged;
  dirty = false;
  lastDrawMs = millis();
}

}  // namespace display

#endif  // FEATURE_DISPLAY
