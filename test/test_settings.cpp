/*
 * settings: clamping, invariants, and how often it writes to flash.
 *
 * The clamping matters because both the remote and the web interface drive
 * these setters, and an unclamped value would either wrap a uint8_t or close
 * the threshold window to nothing.
 */

#include "harness.h"

#include <string.h>

#include <string>

#include "../config.h"
#include "../settings.h"
#include "fakes.h"

void test_settings() {
  SUITE("settings");
  const Settings& s = settings::get();

  CASE("defaults apply when nothing is stored");
  fakes::reset();
  settings::begin();
  CHECK(s.dbMin == DB_MIN_DEFAULT, "dbMin %u, want %u", s.dbMin, DB_MIN_DEFAULT);
  CHECK(s.dbMax == DB_MAX_DEFAULT, "dbMax %u, want %u", s.dbMax, DB_MAX_DEFAULT);
  CHECK(s.brightness == LED_BRIGHTNESS_DEFAULT, "brightness %u", s.brightness);

  CASE("stored values are loaded and clamped to the configured limits");
  fakes::reset();
  fakes::seedUInt("dB_min", 5);           // below DB_LIMIT_LOW
  fakes::seedUInt("dB_max", 9999);        // above DB_LIMIT_HIGH
  fakes::seedUInt("bright", 0);           // below LED_BRIGHTNESS_MIN
  settings::begin();
  CHECK(s.dbMin == DB_LIMIT_LOW, "dbMin %u, want %u", s.dbMin, DB_LIMIT_LOW);
  CHECK(s.dbMax == DB_LIMIT_HIGH, "dbMax %u, want %u", s.dbMax, DB_LIMIT_HIGH);
  CHECK(s.brightness == LED_BRIGHTNESS_MIN, "brightness %u", s.brightness);

  CASE("an inverted pair stored by an older build is repaired");
  fakes::reset();
  fakes::seedUInt("dB_min", 80);
  fakes::seedUInt("dB_max", 40);
  settings::begin();
  CHECK(s.dbMin + DB_MIN_SPAN <= s.dbMax, "window still inverted: %u / %u", s.dbMin, s.dbMax);

  CASE("thresholds cannot be pushed past their limits or wrapped");
  fakes::reset();
  settings::begin();
  for (int i = 0; i < 500; i++) settings::adjustDbMin(-1);
  CHECK(s.dbMin == DB_LIMIT_LOW, "dbMin floor %u, want %u", s.dbMin, DB_LIMIT_LOW);
  for (int i = 0; i < 500; i++) settings::adjustDbMax(+1);
  CHECK(s.dbMax == DB_LIMIT_HIGH, "dbMax ceiling %u, want %u", s.dbMax, DB_LIMIT_HIGH);

  CASE("the two thresholds cannot cross");
  for (int i = 0; i < 500; i++) settings::adjustDbMin(+1);
  CHECK(s.dbMin == s.dbMax - DB_MIN_SPAN, "dbMin %u ran into dbMax %u", s.dbMin, s.dbMax);
  for (int i = 0; i < 500; i++) settings::adjustDbMax(-1);
  CHECK(s.dbMax == s.dbMin + DB_MIN_SPAN, "dbMax %u ran into dbMin %u", s.dbMax, s.dbMin);

  CASE("absolute setters clamp the same way as the relative ones");
  fakes::reset();
  settings::begin();
  settings::setDbMin(-40);
  CHECK(s.dbMin == DB_LIMIT_LOW, "setDbMin(-40) gave %u", s.dbMin);
  settings::setDbMax(10000);
  CHECK(s.dbMax == DB_LIMIT_HIGH, "setDbMax(10000) gave %u", s.dbMax);
  settings::setBrightness(1000);
  CHECK(s.brightness == LED_BRIGHTNESS_MAX, "setBrightness(1000) gave %u", s.brightness);

  CASE("brightness stays inside its range");
  fakes::reset();
  settings::begin();
  for (int i = 0; i < 20; i++) settings::adjustBrightness(-LED_BRIGHTNESS_STEP);
  CHECK(s.brightness == LED_BRIGHTNESS_MIN, "brightness floor %u", s.brightness);
  for (int i = 0; i < 20; i++) settings::adjustBrightness(+LED_BRIGHTNESS_STEP);
  CHECK(s.brightness == LED_BRIGHTNESS_MAX, "brightness ceiling %u", s.brightness);

  // A held-down remote key produces a stream of adjustments. Writing each one
  // through to flash would burn NVS for no reason, so writes are deferred and
  // coalesced.
  CASE("a burst of edits costs one deferred write, not one per edit");
  fakes::reset();
  settings::begin();
  for (int i = 0; i < 30; i++) settings::adjustDbMin(+1);
  CHECK(fakes::nvsWrites() == 0, "wrote %d time(s) mid-burst", fakes::nvsWrites());
  settings::tick();
  CHECK(fakes::nvsWrites() == 0, "flushed before the quiet period elapsed");
  fakes::advanceMillis(5000);
  settings::tick();
  CHECK(fakes::nvsWrites() == 1, "expected 1 write, got %d", fakes::nvsWrites());
  CHECK(fakes::storedUInt("dB_min", 0) == s.dbMin, "stored value does not match RAM");

  CASE("a settled flush does not repeat");
  fakes::advanceMillis(60000);
  settings::tick();
  settings::tick();
  CHECK(fakes::nvsWrites() == 1, "wrote again with nothing dirty: %d", fakes::nvsWrites());

  CASE("only fields that actually changed are written");
  fakes::reset();
  settings::begin();
  settings::adjustDbMin(+3);
  settings::adjustDbMin(-3);  // back where it started
  settings::adjustDbMax(+1);
  fakes::advanceMillis(5000);
  settings::tick();
  CHECK(fakes::nvsWrites() == 1, "expected only dbMax to be written, got %d writes",
        fakes::nvsWrites());

  CASE("screen brightness clamps and persists like the others");
  fakes::reset();
  settings::begin();
  CHECK(s.displayBrightness == DISPLAY_BRIGHTNESS_DEFAULT, "default %u, want %u",
        s.displayBrightness, DISPLAY_BRIGHTNESS_DEFAULT);
  settings::setDisplayBrightness(9999);
  CHECK(s.displayBrightness == DISPLAY_BRIGHTNESS_MAX, "ceiling %u", s.displayBrightness);
  settings::setDisplayBrightness(-5);
  CHECK(s.displayBrightness == DISPLAY_BRIGHTNESS_MIN, "floor %u", s.displayBrightness);
  fakes::advanceMillis(5000);
  settings::tick();
  CHECK(fakes::storedUInt("screen_bri", 999) == s.displayBrightness,
        "not written to NVS");

  CASE("wifi credentials round-trip and are written immediately");
  fakes::reset();
  settings::begin();
  const int before = fakes::nvsWrites();
  settings::setWifiCredentials("some-network", "hunter2");
  CHECK(fakes::nvsWrites() > before, "credentials were not written");
  CHECK(strcmp(settings::wifiSsid(), "some-network") == 0, "ssid round-trip failed: %s",
        settings::wifiSsid());
  CHECK(strcmp(settings::wifiPassword(), "hunter2") == 0, "password round-trip failed");

  // getString does not touch the buffer when the key is missing, so without
  // an explicit clear a unit that had credentials removed would keep serving
  // the old ones.
  CASE("begin() clears credentials that are no longer stored");
  fakes::reset();
  settings::begin();
  settings::setWifiCredentials("old-network", "old-password");
  CHECK(strlen(settings::wifiSsid()) > 0, "precondition failed: nothing was set");
  fakes::reset();          // NVS now empty, as after a wipe
  settings::begin();
  CHECK(settings::wifiSsid()[0] == '\0', "stale ssid survived begin(): '%s'",
        settings::wifiSsid());
  CHECK(settings::wifiPassword()[0] == '\0', "stale password survived begin()");

  CASE("an over-long ssid is truncated rather than overflowing");
  fakes::reset();
  settings::begin();
  const char* huge =
      "0123456789012345678901234567890123456789012345678901234567890123456789";
  settings::setWifiCredentials(huge, huge);
  CHECK(strlen(settings::wifiSsid()) <= 32, "ssid length %zu", strlen(settings::wifiSsid()));
  CHECK(strlen(settings::wifiPassword()) <= 63, "password length %zu",
        strlen(settings::wifiPassword()));
}
