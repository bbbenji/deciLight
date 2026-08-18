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

  // Group behaviour. See the zone-mask note in config.h for how these
  // compose into a stack, a mirrored room, or a unit working alone.
  bool    groupLevel;     // light from the group's level rather than our own
  uint8_t zones;          // ZONE_MASK_* bits this unit lights for
  uint8_t combine;        // COMBINE_LOUDEST or COMBINE_AVERAGE
  uint8_t inactiveLevel;  // shown when the group is in a zone we do not cover
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
void setGroupLevel(bool enabled);
void setZones(int mask);
void setCombine(int mode);
void setInactiveLevel(int value);

// Group name. Empty means this unit does not sync with anything. Units join
// a group by being given the same name; there is no pairing step.
const char* groupName();
void setGroupName(const char* name);

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

// Writes any pending change immediately. For the paths that restart the unit,
// where waiting for the settle would lose it.
void flush();

}  // namespace settings

#endif  // DECILIGHT_SETTINGS_H
