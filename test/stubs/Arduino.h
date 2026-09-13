/*
 * Host stub: Arduino core
 *
 * Enough of the Arduino and FreeRTOS surface for the firmware's logic modules
 * to compile and run on a development machine. Nothing here emulates hardware
 * - see test/README.md for what that means for the tests.
 */

#ifndef DECILIGHT_TEST_ARDUINO_H
#define DECILIGHT_TEST_ARDUINO_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <string>

#define F(x) (x)
#define PROGMEM

typedef std::string String;

// Swallows output. Tests assert on state, not on log lines.
struct SerialStub {
  void begin(unsigned long) {}
  void print(const char*) {}
  void print(char) {}
  void print(const String&) {}
  void println(const char*) {}
  void println(const String&) {}
  void printf(const char*, ...) {}
  operator bool() const { return true; }
};
extern SerialStub Serial;

// Test-controlled clock. Never advances on its own, so timing-dependent
// behaviour is deterministic rather than dependent on how fast the host runs.
unsigned long millis();
void delay(unsigned long);

bool setCpuFrequencyMhz(uint32_t);

// FreeRTOS surface used by sound_level.cpp. Declared so that module compiles;
// the sampling task is not exercised on the host.
typedef void* QueueHandle_t;
typedef uint32_t TickType_t;
typedef unsigned UBaseType_t;
typedef int BaseType_t;

#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY 0xffffffffUL
#define pdMS_TO_TICKS(x) ((TickType_t)(x))

QueueHandle_t xQueueCreate(unsigned length, unsigned itemSize);
BaseType_t xQueueSend(QueueHandle_t queue, const void* item, TickType_t wait);
BaseType_t xQueueReceive(QueueHandle_t queue, void* item, TickType_t wait);
BaseType_t xTaskCreate(void (*fn)(void*), const char* name, uint32_t stack, void* arg,
                       UBaseType_t priority, void* handle);
TickType_t xTaskGetTickCount();

// FreeRTOS critical sections are no-ops here: the tests are single threaded,
// and the point of the locking in the firmware is the WiFi task, which does
// not exist on the host.
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) ((void)0)
#define portEXIT_CRITICAL(mux) ((void)0)

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_INTR_FLAG_LEVEL1 (1 << 1)

#endif  // DECILIGHT_TEST_ARDUINO_H
