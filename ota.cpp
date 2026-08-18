#include "ota.h"

#if FEATURE_WIFI && FEATURE_OTA

#include <Arduino.h>
#include <Update.h>
#include <WebServer.h>

#include "signal_light.h"

namespace ota {
namespace {

// Set at the start of an upload and read by the completion handler, which the
// web server calls separately once the body has been consumed.
bool authorized = false;
bool wrote = false;

bool passwordConfigured() { return OTA_PASSWORD[0] != '\0'; }

void handleUpload(WebServer& server) {
  HTTPUpload& upload = server.upload();

  switch (upload.status) {
    case UPLOAD_FILE_START: {
      authorized = false;
      wrote = false;

      if (!passwordConfigured()) {
        Serial.println(F("ota: refused, no OTA_PASSWORD is set"));
        return;
      }
      // Checked on the first chunk rather than in the completion handler, so
      // an unauthenticated client cannot stream a whole firmware image at us
      // before being turned away.
      if (!server.authenticate(OTA_USERNAME, OTA_PASSWORD)) {
        Serial.println(F("ota: refused, bad credentials"));
        return;
      }
      authorized = true;

      Serial.printf("ota: receiving %s\n", upload.filename.c_str());
      // Hold a visible colour for the duration. The loop stops servicing
      // measurements while the upload streams, so the signal would otherwise
      // freeze on whatever it happened to be showing.
      signal_light::setManualColor(COLOR_UPDATING);
      signal_light::tick();

      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
        authorized = false;
      }
      break;
    }

    case UPLOAD_FILE_WRITE:
      if (!authorized) return;
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
        authorized = false;
        return;
      }
      wrote = true;
      break;

    case UPLOAD_FILE_END:
      if (!authorized) return;
      // true finalises and marks the new image bootable.
      if (Update.end(true)) {
        Serial.printf("ota: wrote %u bytes, restarting\n", upload.totalSize);
      } else {
        Update.printError(Serial);
        authorized = false;
      }
      break;

    case UPLOAD_FILE_ABORTED:
      Serial.println(F("ota: upload aborted"));
      Update.abort();
      authorized = false;
      break;

    default:
      break;
  }
}

void handleResult(WebServer& server) {
  if (!passwordConfigured()) {
    server.send(503, "application/json",
                "{\"error\":\"OTA is disabled until OTA_PASSWORD is set\"}");
    return;
  }
  if (!authorized || !wrote || Update.hasError()) {
    signal_light::setManualColor(COLOR_FAILED);
    signal_light::tick();
    server.send(authorized ? 400 : 401, "application/json",
                authorized ? "{\"error\":\"update failed\"}"
                           : "{\"error\":\"unauthorized\"}");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true,\"restarting\":true}");
  server.client().flush();
  delay(200);  // let the response reach the browser before the reset
  ESP.restart();
}

}  // namespace

bool available() { return passwordConfigured(); }

bool registerRoutes(WebServer& server) {
  server.on(
      "/api/update", HTTP_POST,
      [&server]() { handleResult(server); },
      [&server]() { handleUpload(server); });

  if (!passwordConfigured()) {
    Serial.println(F("ota: disabled, set OTA_PASSWORD in config.h to enable"));
    return false;
  }
  Serial.println(F("ota: update endpoint ready at /api/update"));
  return true;
}

}  // namespace ota

#endif  // FEATURE_WIFI && FEATURE_OTA
