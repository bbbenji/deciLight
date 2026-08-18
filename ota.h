/*
 * deciLight - over-the-air firmware updates
 *
 * Registers an upload endpoint on the existing web server, so a mounted unit
 * can be reflashed from a browser rather than taken down for a USB cable.
 *
 * Disabled unless OTA_PASSWORD is set - see the security note in config.h.
 */

#ifndef DECILIGHT_OTA_H
#define DECILIGHT_OTA_H

#include "config.h"

#if FEATURE_WIFI && FEATURE_OTA

class WebServer;

namespace ota {

// Adds the update route to an already-constructed server. Returns false when
// OTA is unavailable because no password is configured, in which case the
// route still exists but rejects everything.
bool registerRoutes(WebServer& server);

// Whether a password is configured. Reported by the status endpoint so the
// page can hide the upload form when it would only ever fail.
bool available();

}  // namespace ota

#endif  // FEATURE_WIFI && FEATURE_OTA

#endif  // DECILIGHT_OTA_H
