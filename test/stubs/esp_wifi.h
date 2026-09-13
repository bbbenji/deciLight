/* Host stub: the two pieces of esp_wifi the group module touches. */
#ifndef DECILIGHT_TEST_ESP_WIFI_H
#define DECILIGHT_TEST_ESP_WIFI_H

#include <Arduino.h>

#define WIFI_IF_STA 0
#define WIFI_IF_AP 1

typedef int wifi_second_chan_t;
esp_err_t esp_wifi_get_channel(uint8_t* primary, wifi_second_chan_t* second);

#endif  // DECILIGHT_TEST_ESP_WIFI_H
