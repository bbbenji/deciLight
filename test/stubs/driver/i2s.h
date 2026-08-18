/*
 * Host stub: ESP-IDF legacy I2S driver
 *
 * Declaration-only. Present so sound_level.cpp passes a syntax check on the
 * host; the sampling path itself is not exercised, since its inner loop is
 * hand-written Xtensa assembly.
 */

#ifndef DECILIGHT_TEST_DRIVER_I2S_H
#define DECILIGHT_TEST_DRIVER_I2S_H

#include <Arduino.h>

typedef enum { I2S_NUM_0 = 0, I2S_NUM_1 = 1 } i2s_port_t;
typedef enum { I2S_MODE_MASTER = 1, I2S_MODE_RX = 16 } i2s_mode_t;
typedef enum { I2S_BITS_PER_SAMPLE_32BIT = 32 } i2s_bits_per_sample_t;
typedef enum { I2S_CHANNEL_FMT_ONLY_RIGHT = 0, I2S_CHANNEL_FMT_ONLY_LEFT = 1 } i2s_channel_fmt_t;
typedef enum { I2S_COMM_FORMAT_STAND_I2S = 0x01 } i2s_comm_format_t;
typedef enum { I2S_MCLK_MULTIPLE_DEFAULT = 0 } i2s_mclk_multiple_t;
typedef enum { I2S_BITS_PER_CHAN_DEFAULT = 0 } i2s_bits_per_chan_t;

#define I2S_PIN_NO_CHANGE (-1)

typedef struct {
  i2s_mode_t mode;
  int sample_rate;
  i2s_bits_per_sample_t bits_per_sample;
  i2s_channel_fmt_t channel_format;
  i2s_comm_format_t communication_format;
  int intr_alloc_flags;
  int dma_buf_count;
  int dma_buf_len;
  bool use_apll;
  bool tx_desc_auto_clear;
  int fixed_mclk;
  i2s_mclk_multiple_t mclk_multiple;
  i2s_bits_per_chan_t bits_per_chan;
} i2s_config_t;

typedef struct {
  int mck_io_num;
  int bck_io_num;
  int ws_io_num;
  int data_out_num;
  int data_in_num;
} i2s_pin_config_t;

esp_err_t i2s_driver_install(i2s_port_t port, const i2s_config_t* config, int queueSize, void* queue);
esp_err_t i2s_driver_uninstall(i2s_port_t port);
esp_err_t i2s_set_pin(i2s_port_t port, const i2s_pin_config_t* pins);
esp_err_t i2s_read(i2s_port_t port, void* dest, size_t bytes, size_t* read, TickType_t wait);

#endif  // DECILIGHT_TEST_DRIVER_I2S_H
