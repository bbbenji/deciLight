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
  uint8_t brightness;         // 0-255, applied globally by FastLED
  uint8_t displayBrightness;  // 0-255 panel contrast; 0 powers the screen down
};

namespace settings {

// Loads stored values, falling back to the defaults in config.h.
void begin();

const Settings& get();

// Absolute setters. All of them clamp to the limits in config.h and keep
// dbMin + DB_MIN_SPAN <= dbMax, so no caller - remote, web or otherwise - can
// wrap a value or close the window.
void setDbMin(int value);
void setDbMax(int value);
void setBrightness(int value);
void setDisplayBrightness(int value);

// Relative equivalents, for the remote's step keys.
void adjustDbMin(int delta);
void adjustDbMax(int delta);
void adjustBrightness(int delta);

// WiFi station credentials. An empty SSID means "no network configured", in
// which case the firmware brings up its own access point instead.
const char* wifiSsid();
const char* wifiPassword();

// Written to NVS immediately rather than lazily - the caller is expected to
// restart so the new credentials take effect.
void setWifiCredentials(const char* ssid, const char* password);

// Call from loop(). Writes pending changes once they have settled.
void tick();

}  // namespace settings

#endif  // DECILIGHT_SETTINGS_H
