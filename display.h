/*
 * deciLight - optional SSD1306 status screen
 *
 * Shows the measured level, the threshold window and the operating mode on a
 * 128x64 OLED. The panel is probed at boot; if nothing answers, every call
 * below becomes a no-op and the light works exactly as it did before, so one
 * firmware serves units built with and without a screen.
 *
 * Pushing a frame blocks loop() for roughly 22ms, so the module keeps the
 * frame it last drew and sends a new one only when something visible has
 * actually changed - and never more often than DISPLAY_MIN_INTERVAL_MS.
 */

#ifndef DECILIGHT_DISPLAY_H
#define DECILIGHT_DISPLAY_H

#include <stdint.h>

#include "config.h"
#include "settings.h"
#include "signal_light.h"
#include "sound_level.h"

namespace display {

#if FEATURE_DISPLAY

// Brings up I2C and looks for a panel at each of DISPLAY_ADDRESSES. False
// means no screen is fitted, which is not an error.
bool begin();

bool present();

// One line of context for the top right - an IP address, or a short note when
// there is no network. Copied, so the caller need not keep the string alive.
void setStatus(const char* text);

// Stages what should be on screen. Cheap: it only builds a frame description
// and compares it with the one already displayed.
void update(float leqDb, sound_level::Quality quality, const Settings& settings,
            signal_light::Mode mode, signal_light::Zone zone);

// Call from loop(). Sends the staged frame if it differs from what is showing
// and the minimum interval has passed.
void tick();

#else

inline bool begin() { return false; }
inline bool present() { return false; }
inline void setStatus(const char*) {}
inline void update(float, sound_level::Quality, const Settings&, signal_light::Mode,
                   signal_light::Zone) {}
inline void tick() {}

#endif  // FEATURE_DISPLAY

}  // namespace display

#endif  // DECILIGHT_DISPLAY_H
