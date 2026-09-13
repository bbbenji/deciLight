/*
 * Host stub: ESP-NOW
 *
 * Captures what the firmware transmits and lets a test hand packets back to
 * the registered callback, so the wire format is exercised end to end without
 * a radio.
 */

#ifndef DECILIGHT_TEST_ESP_NOW_H
#define DECILIGHT_TEST_ESP_NOW_H

#include <Arduino.h>

#define ESP_NOW_MAX_DATA_LEN 250

typedef void (*esp_now_recv_cb_t)(const uint8_t* mac, const uint8_t* data, int len);

typedef struct {
  uint8_t peer_addr[6];
  uint8_t channel;
  bool encrypt;
  int ifidx;
} esp_now_peer_info_t;

esp_err_t esp_now_init();
esp_err_t esp_now_deinit();
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t* peer);
esp_err_t esp_now_send(const uint8_t* mac, const uint8_t* data, size_t len);

#endif  // DECILIGHT_TEST_ESP_NOW_H
