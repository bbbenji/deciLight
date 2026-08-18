/*
 * deciLight - build-time configuration
 *
 * Everything that varies between builds, hardware revisions or personal
 * taste lives here. No logic, no state - just values.
 */

#ifndef DECILIGHT_CONFIG_H
#define DECILIGHT_CONFIG_H

#include <stdint.h>

// -----------------------------------------------------------------------------
// Pins (wire colours are for the reference build, see pins.txt)
// -----------------------------------------------------------------------------
constexpr uint8_t PIN_LED_DATA = 2;   // NeoPixel Jewel data in  (Y)
constexpr uint8_t PIN_IR_RECV  = 4;   // IR receiver signal      (Y)
constexpr uint8_t PIN_I2S_SCK  = 14;  // I2S bit clock           (G)
constexpr uint8_t PIN_I2S_WS   = 15;  // I2S word select         (B)
constexpr uint8_t PIN_I2S_SD   = 32;  // I2S serial data         (Y)

// SCK and WS must be output-capable; SD may be an input-only pin (36-39).

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200;

// The ESP32 runs this workload comfortably at 80MHz, which keeps the board
// cool and cuts power draw. FastLED drives the NeoPixels over RMT (clocked
// from the fixed 80MHz APB, not the CPU) and I2S uses the APLL, so neither
// is affected by the lower core clock. 80MHz is also the floor for the WiFi
// radio, so do not go below it with FEATURE_WIFI enabled.
constexpr uint32_t CPU_FREQ_MHZ = 80;

// Let the 5V rail settle before pulling current through the LEDs.
constexpr uint32_t STARTUP_DELAY_MS = 1000;

// Print each measurement to the serial console.
constexpr bool SERIAL_LOG_LEVEL = true;

// -----------------------------------------------------------------------------
// LEDs
// -----------------------------------------------------------------------------
constexpr uint16_t LED_COUNT = 7;   // NeoPixel Jewel

constexpr uint8_t LED_BRIGHTNESS_DEFAULT = 255;
constexpr uint8_t LED_BRIGHTNESS_MIN     = 10;
constexpr uint8_t LED_BRIGHTNESS_MAX     = 255;
constexpr uint8_t LED_BRIGHTNESS_STEP    = 51;  // 5 presses from min to max

// Power budget handed to FastLED, which dims globally rather than browning
// out the regulator. Sized for a 5V 2A supply with headroom for the ESP32.
constexpr uint8_t  LED_PSU_VOLTS      = 5;
constexpr uint16_t LED_PSU_MILLIAMPS  = 420;

// GPIO blinked when FastLED has to throttle for the power budget.
// Set to a pin number to enable; -1 leaves every GPIO alone.
constexpr int8_t LED_POWER_INDICATOR_PIN = -1;

// Signal colours, 0xRRGGBB.
constexpr uint32_t COLOR_QUIET = 0x00FF00;  // below dB_min
constexpr uint32_t COLOR_WARN  = 0xFFFF00;  // between dB_min and dB_max
constexpr uint32_t COLOR_LOUD  = 0xFF0000;  // above dB_max

// -----------------------------------------------------------------------------
// Thresholds
// -----------------------------------------------------------------------------
constexpr uint8_t DB_MIN_DEFAULT = 40;
constexpr uint8_t DB_MAX_DEFAULT = 60;

// Hard limits the remote cannot push the thresholds past, plus the smallest
// allowed window between them. Guards against wrap-around and unusable setups.
constexpr uint8_t DB_LIMIT_LOW  = 30;
constexpr uint8_t DB_LIMIT_HIGH = 110;
constexpr uint8_t DB_MIN_SPAN   = 2;

// -----------------------------------------------------------------------------
// Dampening
//
// Two independent knobs that together stop the light strobing when the room
// sits right on a threshold:
//
//   SMOOTHING  weight given to each new reading in the running average.
//              1.0 reacts instantly, 0.1 crawls. 0.35 settles in about a
//              second at the default measurement period.
//   HYSTERESIS how far past a threshold the level must go before the colour
//              changes, in dB. Applied in both directions.
// -----------------------------------------------------------------------------
constexpr float DB_SMOOTHING  = 0.35f;
constexpr float DB_HYSTERESIS = 1.5f;

// -----------------------------------------------------------------------------
// Microphone
//
// Values below are from the datasheet of the fitted microphone. MIC_EQUALIZER
// and MIC_WEIGHTING must name filters defined in sound_level.cpp.
// -----------------------------------------------------------------------------
#define MIC_EQUALIZER INMP441      // 'None' disables equalisation
#define MIC_WEIGHTING A_weighting  // 'C_weighting' or 'None' (Z-weighting)
#define DB_UNITS      "dBA"        // match the weighting above

constexpr double MIC_SENSITIVITY = -26.0;   // dBFS produced at MIC_REF_DB
constexpr double MIC_REF_DB      = 94.0;    // dB the sensitivity is quoted at
constexpr double MIC_OVERLOAD_DB = 116.0;   // acoustic overload point
constexpr double MIC_NOISE_DB    = 29.0;    // noise floor
constexpr int    MIC_BITS        = 24;      // valid bits in the I2S frame

// Sine-wave RMS vs. dBFS. Nudge this to calibrate against a reference meter.
constexpr double MIC_OFFSET_DB = 3.0103;

// -----------------------------------------------------------------------------
// WiFi control
//
// Set to 0 to build without networking. That saves roughly 700KB of flash and
// 45KB of RAM, and removes the WiFi radio's power draw, at the cost of the
// web interface.
//
// On boot the firmware joins the network stored in NVS, if there is one, and
// otherwise brings up its own access point. A classroom unit that moves
// between rooms can therefore be controlled with no network at all: join the
// deciLight access point from a phone and open http://192.168.4.1/.
// -----------------------------------------------------------------------------
#define FEATURE_WIFI 1

// Also the mDNS name, so the unit answers to http://decilight.local/ on
// networks whose clients support it.
constexpr char WIFI_HOSTNAME[] = "decilight";

// Fallback access point. The password must be at least 8 characters, or empty
// for an open network.
constexpr char WIFI_AP_SSID[]     = "deciLight";
constexpr char WIFI_AP_PASSWORD[] = "decilight";

// How long to wait for the stored network before giving up and starting the
// access point instead.
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

constexpr uint16_t WEB_SERVER_PORT = 80;

// -----------------------------------------------------------------------------
// Sampling
// -----------------------------------------------------------------------------
constexpr uint32_t SAMPLE_RATE = 48000;  // fixed by the IIR filter design
constexpr uint32_t SAMPLE_BITS = 32;

// Sample blocks handed from the I2S task to the main loop. 1/8s is a good
// trade between filter efficiency and how promptly overloads are noticed.
constexpr uint32_t SAMPLES_PER_BLOCK = SAMPLE_RATE / 8;  // 6000 = 125ms

// Averaging window for the reported Leq, rounded up to whole blocks.
// 250ms means every reported value averages two blocks.
constexpr uint32_t LEQ_PERIOD_MS   = 250;
constexpr uint32_t LEQ_BLOCK_COUNT =
    (LEQ_PERIOD_MS * SAMPLE_RATE / 1000 + SAMPLES_PER_BLOCK - 1) / SAMPLES_PER_BLOCK;

// How long the main loop parks on the measurement queue before servicing the
// remote again. Caps worst-case IR latency without spinning.
constexpr uint32_t LOOP_POLL_MS = 20;

#endif  // DECILIGHT_CONFIG_H
