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
constexpr uint8_t PIN_LED_DATA = 2; // NeoPixel Jewel data in  (Y)
constexpr uint8_t PIN_IR_RECV = 4;  // IR receiver signal      (Y)
constexpr uint8_t PIN_I2S_SCK = 14; // I2S bit clock           (G)
constexpr uint8_t PIN_I2S_WS = 15;  // I2S word select         (B)
constexpr uint8_t PIN_I2S_SD = 32;  // I2S serial data         (Y)
constexpr uint8_t PIN_I2C_SDA = 21; // OLED SDA
constexpr uint8_t PIN_I2C_SCL = 22; // OLED SCL

// SCK and WS must be output-capable; SD may be an input-only pin (36-39).

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200;

// Shown on the splash screen at boot and printed to the serial console.
// Bump it when you flash something you want to be able to identify later -
// with OTA in the picture, "which build is actually on that unit" stops being
// a rhetorical question.
#define FIRMWARE_VERSION_JSON "2.0.0"
constexpr char FIRMWARE_VERSION[] = FIRMWARE_VERSION_JSON;

// The name on the splash screen. Lower-case "d" to match how the project
// spells itself everywhere else.
#define PRODUCT_NAME_JSON "deciLight"
constexpr char PRODUCT_NAME[] = PRODUCT_NAME_JSON;

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
constexpr uint16_t LED_COUNT = 7; // NeoPixel Jewel

constexpr uint8_t LED_BRIGHTNESS_DEFAULT = 255;
constexpr uint8_t LED_BRIGHTNESS_MIN = 10;
constexpr uint8_t LED_BRIGHTNESS_MAX = 255;
constexpr uint8_t LED_BRIGHTNESS_STEP = 51; // 5 presses from min to max

// Power budget handed to FastLED, which dims globally rather than browning
// out the regulator. Sized for a 5V 2A supply with headroom for the ESP32.
constexpr uint8_t LED_PSU_VOLTS = 5;
constexpr uint16_t LED_PSU_MILLIAMPS = 420;

// GPIO blinked when FastLED has to throttle for the power budget.
// Set to a pin number to enable; -1 leaves every GPIO alone.
constexpr int8_t LED_POWER_INDICATOR_PIN = -1;

// Signal colours, 0xRRGGBB.
constexpr uint32_t COLOR_QUIET = 0x00FF00; // below dB_min
constexpr uint32_t COLOR_WARN = 0xFFFF00;  // between dB_min and dB_max
constexpr uint32_t COLOR_LOUD = 0xFF0000;  // above dB_max

// -----------------------------------------------------------------------------
// Display
//
// An optional SSD1306 128x64 OLED on the I2C pins above, showing the current
// level, the threshold window and the operating mode. Entirely optional: the
// panel is probed at boot and everything below is skipped if nothing answers,
// so one firmware serves units built with and without a screen.
// -----------------------------------------------------------------------------
#ifndef FEATURE_DISPLAY
#define FEATURE_DISPLAY 1
#endif

constexpr uint8_t DISPLAY_WIDTH = 128;
constexpr uint8_t DISPLAY_HEIGHT = 64;

// Nearly all of these modules answer at 0x3C; a few are strapped to 0x3D.
// Both are probed, in this order.
constexpr uint8_t DISPLAY_ADDRESSES[] = {0x3C, 0x3D};

// Pushing a full frame is about 1KB over I2C - roughly 22ms at 400kHz, during
// which loop() is blocked. The frame is only sent when something visible has
// actually changed, and never more often than this.
constexpr uint32_t DISPLAY_MIN_INTERVAL_MS = 250;

// How long the splash screen is held before measurements take the screen
// over. Nothing waits on this - the rest of setup() carries on underneath,
// and on a unit joining a stored WiFi network the splash is usually still up
// well past this anyway.
constexpr uint32_t DISPLAY_SPLASH_MS = 2000;

// Panel contrast. 207 is 0xCF, which is what the SSD1306 driver sets for a
// charge-pump supply, so the default leaves the panel exactly as the library
// would. Zero is not merely the dimmest setting - it powers the panel down,
// which is the useful thing for a room the light stays in overnight.
constexpr uint8_t DISPLAY_BRIGHTNESS_DEFAULT = 207;
constexpr uint8_t DISPLAY_BRIGHTNESS_MIN = 0;
constexpr uint8_t DISPLAY_BRIGHTNESS_MAX = 255;

// The contrast register is roughly linear in drive current while perception
// is roughly logarithmic, so a linear slider spends most of its travel in a
// range that all looks equally bright. The setting is curved before it
// reaches the panel: at 2.2, half travel gives about a fifth of full drive
// and the bottom of the slider is genuinely dim rather than nominally so.
constexpr float DISPLAY_BRIGHTNESS_GAMMA = 2.2f;

// Contrast zero is not the dimmest setting, it is off: the panel produces no
// visible output at all. Any non-zero brightness is therefore floored here,
// so every position on the slider above zero shows something. Raise it if a
// particular panel needs more before it lights.
constexpr uint8_t DISPLAY_CONTRAST_MIN = 1;

// Reaching below the contrast floor means shortening the pre-charge period
// instead. Below DISPLAY_DIM_BELOW the phase-2 period is ramped from its
// shortest up to the driver's own 0xF1, rather than stepped - a step there
// produces a visible jump in brightness partway along the slider.
constexpr uint8_t DISPLAY_DIM_BELOW = 26;
constexpr uint8_t DISPLAY_PRECHARGE_NORMAL = 0xF1;

// -----------------------------------------------------------------------------
// Thresholds
// -----------------------------------------------------------------------------
constexpr uint8_t DB_MIN_DEFAULT = 40;
constexpr uint8_t DB_MAX_DEFAULT = 60;

// Hard limits the remote cannot push the thresholds past, plus the smallest
// allowed window between them. Guards against wrap-around and unusable setups.
constexpr uint8_t DB_LIMIT_LOW = 30;
constexpr uint8_t DB_LIMIT_HIGH = 110;
constexpr uint8_t DB_MIN_SPAN = 2;

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
constexpr float DB_SMOOTHING = 0.35f;
constexpr float DB_HYSTERESIS = 1.5f;

// How a group's readings become one number. Units stacked together hear the
// same sound, so averaging cancels per-microphone variation; units spread
// around a room hear different things, and there the loudest corner is the
// signal worth showing.
constexpr uint8_t COMBINE_LOUDEST = 0;
constexpr uint8_t COMBINE_AVERAGE = 1;
constexpr uint8_t COMBINE_DEFAULT = COMBINE_LOUDEST;

// Which zones a unit lights. A lone light covers all three and behaves as it
// always has; a stacked one covers a single zone and stays dark otherwise,
// which is what makes three units read as one traffic signal. A two-unit
// stack works too, with one of them covering a pair.
constexpr uint8_t ZONE_MASK_QUIET = 1 << 0;
constexpr uint8_t ZONE_MASK_WARN  = 1 << 1;
constexpr uint8_t ZONE_MASK_LOUD  = 1 << 2;
constexpr uint8_t ZONE_MASK_ALL   = ZONE_MASK_QUIET | ZONE_MASK_WARN | ZONE_MASK_LOUD;

// What a unit shows when the group is in a zone it does not cover. Zero is a
// dark lamp, like a real traffic signal; a low value leaves it glowing faintly
// so the stack still reads as a traffic light even when only one lamp is
// active, and a dead unit is distinguishable from an unlit one.
constexpr uint8_t ZONE_INACTIVE_LEVEL_DEFAULT = 0;

// -----------------------------------------------------------------------------
// Microphone
//
// Values below are from the datasheet of the fitted microphone. MIC_EQUALIZER
// and MIC_WEIGHTING must name filters defined in sound_level.cpp.
// -----------------------------------------------------------------------------
#define MIC_EQUALIZER INMP441     // 'None' disables equalisation
#define MIC_WEIGHTING A_weighting // 'C_weighting' or 'None' (Z-weighting)
#define DB_UNITS "dBA"            // match the weighting above

constexpr double MIC_SENSITIVITY = -26.0; // dBFS produced at MIC_REF_DB
constexpr double MIC_REF_DB = 94.0;       // dB the sensitivity is quoted at
constexpr double MIC_OVERLOAD_DB = 116.0; // acoustic overload point
constexpr double MIC_NOISE_DB = 29.0;     // noise floor
constexpr int MIC_BITS = 24;              // valid bits in the I2S frame

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
// Guarded so a build system can override it with -DFEATURE_WIFI=0 without
// editing this file. CI builds both variants that way.
#ifndef FEATURE_WIFI
#define FEATURE_WIFI 1
#endif

// Also the mDNS name, so the unit answers to http://decilight.local/ on
// networks whose clients support it.
constexpr char WIFI_HOSTNAME[] = "decilight";

// Fixed so that units which all fall back to their own access point end up on
// the same radio channel. ESP-NOW only reaches peers sharing a channel, and in
// station mode the channel is dictated by whichever router was joined - so
// this is what lets a group of un-networked units find each other at all.
constexpr uint8_t WIFI_AP_CHANNEL = 1;

// Fallback access point. The password must be at least 8 characters, or empty
// for an open network.
constexpr char WIFI_AP_SSID[] = "deciLight";
constexpr char WIFI_AP_PASSWORD[] = "decilight";

// How long to wait for the stored network before giving up and starting the
// access point instead.
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

// A unit that boots faster than the building's router - after a power cut,
// say - would otherwise sit in access-point fallback until someone power
// cycled it, unreachable at its usual address and, because access-point mode
// pins the channel, dropped out of its group as well. So the stored network
// is retried, but only while nobody is using the fallback access point:
// tearing it down under someone mid-configuration would be worse than
// waiting.
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 60000;

// How long a dropped station connection is given to come back on its own
// before falling back to the access point. The core retries by itself; this
// is the point at which we stop believing it will succeed.
constexpr uint32_t WIFI_FALLBACK_AFTER_MS = 30000;

constexpr uint16_t WEB_SERVER_PORT = 80;

// -----------------------------------------------------------------------------
// Groups (ESP-NOW)
//
// Units on the same radio channel that share a group name act as one light.
// Each broadcasts its measured level a few times a second; each decides what
// to show from the group's level rather than its own.
//
// There is no pairing step and no peer list to maintain: transport is
// broadcast, and the group name is what separates one set of lights from
// another. That also means group membership is a convention rather than a
// secret - anything in radio range running this firmware with the same name
// joins in.
//
// Empty group name means sync is off, which is the default. A light that
// starts talking to the neighbours out of the box would be a surprise.
// -----------------------------------------------------------------------------
#ifndef FEATURE_ESPNOW
#define FEATURE_ESPNOW 1
#endif

constexpr char GROUP_NAME_DEFAULT[] = "";

// How often a unit tells the group what it is hearing. Four times a second
// matches the measurement period, so nothing is ever more than one reading
// out of date.
constexpr uint32_t GROUP_BROADCAST_MS = 250;

// A peer that has gone quiet for this long stops counting towards the group
// level. Without it, a unit switched off mid-shout would hold the whole room
// red indefinitely.
constexpr uint32_t GROUP_PEER_TIMEOUT_MS = 3000;

// Well under the radio's own limit of 20; a classroom needs a handful.
constexpr uint8_t GROUP_MAX_PEERS = 8;

// Bumped only when the wire format changes incompatibly. Messages carrying
// anything else are ignored, so a half-updated group degrades to units
// working alone rather than to nonsense.
constexpr uint8_t GROUP_PROTOCOL_VERSION = 2;

// -----------------------------------------------------------------------------
// Over-the-air updates
//
// Lets a mounted unit be reflashed from a browser instead of coming off the
// wall for a USB cable. Requires FEATURE_WIFI.
//
// SECURITY: an OTA endpoint accepts arbitrary code. Changing the light's
// colour over the network is harmless; replacing its firmware is not, and the
// access point password below is published in this repository. So OTA is
// disabled until OTA_PASSWORD is set to something, and refuses every request
// while it is empty. Set it before relying on this, and prefer a value you do
// not use anywhere else - it travels as base64 over plain HTTP.
// -----------------------------------------------------------------------------
#ifndef FEATURE_OTA
#define FEATURE_OTA 1
#endif

constexpr char OTA_USERNAME[] = "decilight";
constexpr char OTA_PASSWORD[] = "decilight"; // empty disables OTA entirely

// Held on the ring while an update is being written, so a room can see why the
// light stopped responding. The unit restarts on its own afterwards.
constexpr uint32_t COLOR_UPDATING = 0x0000FF;
constexpr uint32_t COLOR_FAILED = 0xFF00FF;

// -----------------------------------------------------------------------------
// Sampling
// -----------------------------------------------------------------------------
constexpr uint32_t SAMPLE_RATE = 48000; // fixed by the IIR filter design
constexpr uint32_t SAMPLE_BITS = 32;

// Sample blocks handed from the I2S task to the main loop. 1/8s is a good
// trade between filter efficiency and how promptly overloads are noticed.
constexpr uint32_t SAMPLES_PER_BLOCK = SAMPLE_RATE / 8; // 6000 = 125ms

// Averaging window for the reported Leq, rounded up to whole blocks.
// 250ms means every reported value averages two blocks.
constexpr uint32_t LEQ_PERIOD_MS = 250;
constexpr uint32_t LEQ_BLOCK_COUNT =
    (LEQ_PERIOD_MS * SAMPLE_RATE / 1000 + SAMPLES_PER_BLOCK - 1) /
    SAMPLES_PER_BLOCK;

// How long the main loop parks on the measurement queue before servicing the
// remote again. Caps worst-case IR latency without spinning.
constexpr uint32_t LOOP_POLL_MS = 20;

#endif // DECILIGHT_CONFIG_H
