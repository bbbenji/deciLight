/*
 * deciLight - persisted user settings
 *
 * Values live in RAM and are written back to NVS lazily, so holding a button
 * on the remote costs one flash write instead of one per repeat.
 */

#ifndef DECILIGHT_SETTINGS_H
#define DECILIGHT_SETTINGS_H

#include <stdint.h>

struct Settings {
  uint8_t dbMin;       // below this the light is green
  uint8_t dbMax;       // above this the light is red
  uint8_t brightness;  // 0-255, applied globally by FastLED
};

namespace settings {

// Loads stored values, falling back to the defaults in config.h.
void begin();

const Settings& get();

// All setters clamp to the limits in config.h and keep dbMin + DB_MIN_SPAN
// <= dbMax, so the remote can never wrap a value or close the window.
void adjustDbMin(int delta);
void adjustDbMax(int delta);
void adjustBrightness(int delta);

// Call from loop(). Writes pending changes once they have settled.
void tick();

}  // namespace settings

#endif  // DECILIGHT_SETTINGS_H
