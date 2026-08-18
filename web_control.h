/*
 * deciLight - WiFi and the web control interface
 *
 * Brings up networking and serves a small self-contained page that shows the
 * live level and adjusts everything the IR remote can, from any phone or
 * laptop browser. No app, and no internet connection required.
 *
 * The whole module compiles away when FEATURE_WIFI is 0.
 */

#ifndef DECILIGHT_WEB_CONTROL_H
#define DECILIGHT_WEB_CONTROL_H

#include <stdint.h>

#include "config.h"
#include "sound_level.h"

namespace web_control {

#if FEATURE_WIFI

// Joins the stored network, or starts the fallback access point, then starts
// the HTTP server. False means networking is unavailable; the light carries
// on working from the remote either way.
bool begin();

// Call from loop(). Services at most one HTTP request.
void tick();

// Hands the newest measurement over for the status endpoint to report.
void publishLevel(const sound_level::Reading& reading);

#else

inline bool begin() { return false; }
inline void tick() {}
inline void publishLevel(const sound_level::Reading&) {}

#endif  // FEATURE_WIFI

}  // namespace web_control

#endif  // DECILIGHT_WEB_CONTROL_H
