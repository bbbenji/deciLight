/*
 * Host stub: Arduino I2C
 *
 * Records the pins it was handed and nothing else. No bus is simulated.
 */

#ifndef DECILIGHT_TEST_WIRE_H
#define DECILIGHT_TEST_WIRE_H

#include <Arduino.h>

class TwoWire {
 public:
  void begin(int sda, int scl);
  void setClock(uint32_t hz);
};
extern TwoWire Wire;

#endif  // DECILIGHT_TEST_WIRE_H
