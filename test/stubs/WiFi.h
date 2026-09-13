/*
 * Host stub: Arduino WiFi
 *
 * Only what group_sync needs. web_control is not part of the host build, so
 * the rest of the class is deliberately absent rather than half-faked.
 */
#ifndef DECILIGHT_TEST_WIFI_H
#define DECILIGHT_TEST_WIFI_H

#include <Arduino.h>

#define WIFI_MODE_STA 1
#define WIFI_MODE_AP 2

class WiFiClass {
 public:
  int getMode();
  uint8_t* macAddress(uint8_t* mac);
};
extern WiFiClass WiFi;

#endif  // DECILIGHT_TEST_WIFI_H
