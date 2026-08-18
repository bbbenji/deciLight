/*
 * Host stub: FastLED
 *
 * Records what the firmware asked for instead of driving any LEDs. The colour
 * constants carry FastLED's real values, so a test asserting on CRGB::Green
 * catches the same 0x008000-versus-0x00FF00 distinction the hardware would.
 */

#ifndef DECILIGHT_TEST_FASTLED_H
#define DECILIGHT_TEST_FASTLED_H

#include <Arduino.h>

#define NEOPIXEL 1
#define TypicalLEDStrip 0xFFB0F0

struct CRGB {
  uint8_t r = 0, g = 0, b = 0;

  enum HTMLColorCode : uint32_t {
    Black = 0x000000, Red = 0xFF0000, Green = 0x008000, Blue = 0x0000FF,
    White = 0xFFFFFF, Tomato = 0xFF6347, LightGreen = 0x90EE90, SkyBlue = 0x87CEEB,
    OrangeRed = 0xFF4500, Cyan = 0x00FFFF, Purple = 0x800080, Orange = 0xFFA500,
    Turquoise = 0x40E0D0, MediumPurple = 0x9370DB, Yellow = 0xFFFF00,
    DarkCyan = 0x008B8B, Plum = 0xDDA0DD,
  };

  CRGB() {}
  CRGB(uint32_t packed) : r(packed >> 16), g(packed >> 8), b(packed) {}

  uint32_t packed() const {
    return (uint32_t(r) << 16) | (uint32_t(g) << 8) | b;
  }
};

void fill_solid(CRGB* leds, int count, const CRGB& color);
void set_max_power_indicator_LED(uint8_t pin);

struct CFastLED {
  template <int Type, uint8_t Pin>
  void addLeds(CRGB*, int) {}
  void setDither(bool);
  void setCorrection(uint32_t);
  void setBrightness(uint8_t);
  void setMaxPowerInVoltsAndMilliamps(uint8_t, uint16_t);
  void show();
};
extern CFastLED FastLED;

#endif  // DECILIGHT_TEST_FASTLED_H
