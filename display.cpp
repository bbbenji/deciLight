#include "display.h"

#if FEATURE_DISPLAY

#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

namespace display {
namespace {

Adafruit_SSD1306 panel(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, /*rst_pin=*/-1);

bool fitted = false;
uint8_t brightness = DISPLAY_BRIGHTNESS_DEFAULT;
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

// Non-zero while the splash screen is being held. Measurements stage frames
// underneath it as normal; they just are not sent until it expires.
uint32_t splashUntilMs = 0;

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

// Maps the stored setting onto the panel's contrast register. See the gamma
// note in config.h for why this is not the identity.
uint8_t contrastFor(uint8_t level) {
  const float t = float(level) / float(DISPLAY_BRIGHTNESS_MAX);
  const float scaled = powf(t, DISPLAY_BRIGHTNESS_GAMMA) * float(DISPLAY_BRIGHTNESS_MAX);
  const uint8_t contrast = uint8_t(scaled + 0.5f);
  return contrast < DISPLAY_CONTRAST_MIN ? DISPLAY_CONTRAST_MIN : contrast;
}

// Pre-charge is the only handle left once contrast is at its floor. The
// phase-2 period (the high nibble) is ramped across the bottom of the slider
// so it meets the driver's default exactly at DISPLAY_DIM_BELOW; a step there
// would show up as the brightness jumping partway along the travel.
uint8_t prechargeFor(uint8_t level) {
  if (level >= DISPLAY_DIM_BELOW) return DISPLAY_PRECHARGE_NORMAL;
  const uint8_t phase2 = 1 + uint8_t((uint16_t(level) * 14) / DISPLAY_DIM_BELOW);
  return uint8_t((phase2 << 4) | 0x01);  // phase 1 stays at 1, as the driver sets it
}

// The built-in font advances 6px per character at size 1, and scales with it.
void printCentred(const char* text, uint8_t size, int16_t y) {
  const int16_t width = int16_t(strlen(text)) * 6 * size;
  int16_t x = (DISPLAY_WIDTH - width) / 2;
  if (x < 0) x = 0;
  panel.setTextSize(size);
  panel.setCursor(x, y);
  panel.print(text);
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
  // Establish a known state rather than relying on static initialisation, so
  // begin() is a real reset point and a second call cannot inherit a panel
  // that is no longer there.
  fitted = false;
  brightness = DISPLAY_BRIGHTNESS_DEFAULT;
  staged = Frame{};
  shown = Frame{};
  dirty = false;
  lastDrawMs = 0;
  splashUntilMs = 0;
  status[0] = '\0';

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

void setBrightness(uint8_t level) {
  if (!fitted) return;
  const bool wasOff = brightness == 0;
  brightness = level;

  if (level == 0) {
    panel.ssd1306_command(SSD1306_DISPLAYOFF);
    return;
  }

  panel.ssd1306_command(SSD1306_DISPLAYON);
  panel.ssd1306_command(SSD1306_SETPRECHARGE);
  panel.ssd1306_command(prechargeFor(level));
  panel.ssd1306_command(SSD1306_SETCONTRAST);
  panel.ssd1306_command(contrastFor(level));

  if (wasOff) {
    // The panel forgets what was on it, and staged may already equal shown,
    // so force the current frame back out rather than waiting for a change.
    shown = Frame{};
    dirty = true;
  }
}

void splash() {
  if (!fitted) return;

  panel.clearDisplay();
  panel.setTextColor(SSD1306_WHITE);
  printCentred(PRODUCT_NAME, 2, 18);
  printCentred(FIRMWARE_VERSION, 1, 42);
  panel.display();

  splashUntilMs = millis() + DISPLAY_SPLASH_MS;
  if (splashUntilMs == 0) splashUntilMs = 1;  // 0 is the "not showing" marker
  lastDrawMs = millis();
}

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
  if (!fitted || brightness == 0 || !dirty) return;

  // Signed comparison, so this survives the millis() rollover.
  if (splashUntilMs != 0) {
    if (int32_t(millis() - splashUntilMs) < 0) return;
    splashUntilMs = 0;
  }

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
