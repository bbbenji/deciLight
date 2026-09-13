/*
 * Host stub: Adafruit_SSD1306
 *
 * Renders into a string instead of a panel, so tests can assert on what the
 * firmware asked to be shown and on how often it pushed a frame. Whether
 * begin() finds a panel is under the test's control - see fakes.h.
 */

#ifndef DECILIGHT_TEST_ADAFRUIT_SSD1306_H
#define DECILIGHT_TEST_ADAFRUIT_SSD1306_H

#include <Arduino.h>
#include <Wire.h>

#define SSD1306_BLACK 0
#define SSD1306_WHITE 1
#define SSD1306_SWITCHCAPVCC 0x02

// Command bytes, matching the values in the real driver.
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETPRECHARGE 0xD9

class Adafruit_SSD1306 {
 public:
  Adafruit_SSD1306(uint8_t w, uint8_t h, TwoWire* twi = &Wire, int8_t rst = -1)
      : width_(w), height_(h) {}

  bool begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC, uint8_t addr = 0, bool reset = true,
             bool periphBegin = true);

  void clearDisplay();
  void display();
  void ssd1306_command(uint8_t c);

  void setTextColor(uint16_t c);
  void setTextSize(uint8_t s);
  void setCursor(int16_t x, int16_t y);

  // Arduino's Print prints integers as decimal text, including uint8_t, which
  // this project relies on when showing the thresholds.
  void print(const char* s);
  void print(char c);
  void print(int v);
  void print(unsigned int v);
  void print(long v);
  void print(unsigned long v);
  void print(unsigned char v);
  void print(double v, int digits = 2);

  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c);
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c);

  int16_t width() const { return width_; }
  int16_t height() const { return height_; }

 private:
  uint8_t width_, height_;
};

#endif  // DECILIGHT_TEST_ADAFRUIT_SSD1306_H
