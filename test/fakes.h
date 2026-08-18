/*
 * Control surface for the host stubs.
 *
 * Lets a test move the clock, inspect what the firmware did to the LEDs, count
 * flash writes, and feed IR key presses in.
 */

#ifndef DECILIGHT_TEST_FAKES_H
#define DECILIGHT_TEST_FAKES_H

#include <stdint.h>

namespace fakes {

// Clears every fake back to power-on state: clock at zero, NVS empty, LED
// record blank, IR queue drained.
void reset();

// --- clock ---
void advanceMillis(uint32_t ms);

// --- LEDs ---
uint32_t lastColor();   // 0xRRGGBB most recently written to the ring
int showCount();        // how many times FastLED.show() was called
uint8_t brightness();   // last value passed to FastLED.setBrightness()

// --- NVS ---
int nvsWrites();                                  // writes since reset()
void seedUInt(const char* key, uint32_t value);   // pretend a stored value exists
uint32_t storedUInt(const char* key, uint32_t fallback);

// --- IR ---
void receiveIr(uint64_t code);  // queue one decoded code for the next poll()

}  // namespace fakes

#endif  // DECILIGHT_TEST_FAKES_H
