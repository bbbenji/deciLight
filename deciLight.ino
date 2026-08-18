/*
 * deciLight - visual noise monitor
 *
 * A traffic-signal light that colours itself from the ambient sound level:
 * green while the room is below dB_min, yellow up to dB_max, red above it.
 * Thresholds, colours and brightness are set from an IR remote and survive a
 * power cycle.
 *
 * https://github.com/bbbenji/deciLight
 *
 * Sound level measurement is derived from esp32-i2s-slm by Ivan Kostoski
 * (c)2019, https://github.com/ikostoski/esp32-i2s-slm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Layout
 *
 *   config.h          every tunable value, no logic
 *   settings.*        thresholds and brightness, persisted to NVS
 *   sound_level.*     I2S sampling, IIR filtering, Leq in dB
 *   signal_light.*    LED state, colour mapping, smoothing and hysteresis
 *   remote_control.*  IR key map and what each key does
 *   web_control.*     WiFi bring-up and the HTTP control interface
 *   web_page.h        the page served to the browser
 *   sos-iir-filter.h  filter kernel, upstream, do not include twice
 *
 * This file only wires them together.
 */

#include "config.h"
#include "remote_control.h"
#include "settings.h"
#include "signal_light.h"
#include "sound_level.h"
#include "web_control.h"

// True once the microphone is running. When it is not, the remote still works
// so the light can be used as a plain lamp.
static bool micReady = false;

void setup() {
  // Drop the core clock before anything else starts depending on the timing.
  // FastLED drives the NeoPixels over RMT and I2S runs off the APLL, so both
  // are clocked independently of the CPU.
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  Serial.begin(SERIAL_BAUD);

  // Give the 5V rail a moment before the LEDs start drawing from it.
  delay(STARTUP_DELAY_MS);

  settings::begin();
  signal_light::begin(settings::get().brightness);
  remote_control::begin();

  micReady = sound_level::begin();
  if (!micReady) {
    Serial.println(F("deciLight: no microphone, running in remote-only mode"));
    signal_light::setMode(signal_light::Mode::Off);
  }

  // Networking comes up last: it is the slowest step and the only optional
  // one, so everything else is already serving the room while it connects.
  web_control::begin();

  const Settings& s = settings::get();
  Serial.printf("deciLight ready, thresholds %u - %u " DB_UNITS "\n", s.dbMin, s.dbMax);
}

void loop() {
  remote_control::poll();
  web_control::tick();
  signal_light::tick();
  settings::tick();

  if (!micReady) {
    delay(LOOP_POLL_MS);
    return;
  }

  // Parks on the measurement queue for a short while, which paces the loop
  // without spinning and keeps worst-case remote latency at LOOP_POLL_MS.
  sound_level::Reading reading;
  if (!sound_level::read(reading, LOOP_POLL_MS)) return;

  if (SERIAL_LOG_LEVEL) {
    Serial.printf("%.1f " DB_UNITS "%s\n", reading.leqDb,
                  reading.quality == sound_level::Quality::Overload      ? " (overload)"
                  : reading.quality == sound_level::Quality::BelowNoiseFloor ? " (below noise floor)"
                                                                            : "");
  }

  web_control::publishLevel(reading);

  const Settings& s = settings::get();
  signal_light::updateLevel(reading.leqDb, s.dbMin, s.dbMax);
}
