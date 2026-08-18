# deciLight - Visual Noise Monitor & Educational Tool

deciLight is a WIP traffic signal-inspired lighting system, designed to dynamically respond to ambient sound levels. This modular, stackable light features a unique capability to change colors—from red to yellow to green—based on the decibel levels in its surrounding environment. Users can set and adjust sensitivity thresholds remotely using an infrared (IR) remote control, which also allows for manual color and brightness changes, offering versatility and convenience.

### Classroom Noise Management

In the classroom, deciLight can serve as an effective noise monitor, signaling to students when the noise level has become too high (red), when it's close (yellow), or when the classroom environment is at an acceptable sound level (green). This visual cue helps in self-regulation, as students can adjust their volume without direct intervention from the teacher, fostering a sense of responsibility and self-awareness among the pupils. By setting specific decibel thresholds via the IR remote, educators can customize the sensitivity of deciLight to suit the needs of different activities, whether it's quiet reading time or a lively group discussion.

### Educational Games and Activities

Beyond its utility as a noise monitor, deciLight's color-changing feature can be integrated into various games and activities that engage students and facilitate learning. For instance:

- **Green Light, Red Light Game:** Leveraging deciLight's ability to change colors, teachers can conduct the classic "Green Light, Red Light" game, where students move when the light is green and stop when it's red. This can be a fun, physically active break between lessons or used as a tool for teaching self-control and listening skills.
- **Sound Level Challenges:** Teachers can create challenges for students to maintain a certain noise level (green) during group work or activities, rewarding them when they successfully stay within the acceptable range. This encourages teamwork and collective effort to achieve a common goal.
- **Interactive Storytelling:** Incorporating deciLight into storytelling can make reading sessions more interactive. For example, the light could change colors to reflect the mood or action within the story, or students could be asked to modulate their voice levels to keep the light green, enhancing engagement and comprehension.

### Bill of Materials (BOM):

- **ESP32 Microcontroller:** The brain of deciLight, offering WiFi and Bluetooth capabilities for future expansions and updates.
- **NeoPixel Jewel:** Provides bright, customizable colors for the light signal, ensuring vivid visibility.
- **IR Receiver:** Enables remote control functionality, allowing users to adjust settings and change colors from a distance.
- **I2S MEMS Microphone:** Senses ambient sound levels to trigger color changes based on predefined decibel thresholds.
- **Screws for Assembly:** m3x5 screws (4), m3-3 screws (2), and m1.5x3 screws (4-8) for secure assembly and mounting.
- **IR LED Remote:** Offers a user-friendly interface for adjusting deciLight settings and colors remotely.
- **5V 2A USB Power Supply:** Ensures reliable power delivery to the deciLight. A high-quality supply is recommended for optimal performance.
- **Wire:** Necessary for connections and assembly.

### Future Enhancements:

- **Networked Synchronization:** The ability to pair multiple deciLights (ESP-NOW?), creating a cohesive and synchronized lighting experience across multiple units.
- **External Display:** Show operational modes and noise thresholds in real-time.
- **WiFi/Bluetooth control:** Adjust settings and modes via a mobile device.
- **Multi-device control:** Control multiple deciLights via single IR remote.

### Assembly & Printing Tips:

- The construction of deciLight is designed to be straightforward and user-friendly.
- The ESP32 mount is adjustable, catering to various models of the ESP32. If the mounting holes differ, customizing your mounting plate might be necessary.
- To enhance light reflection and efficiency, applying aluminum tape to the reflector is recommended.
- The components, including the reflector, lens, and visor, are designed for a friction fit, simplifying the assembly process.
- Print the reflector in vase-mode.
- Print the lens in clear PETG with grid infill.
- Print the feet in TPU and attach them with double sided tape.

### Schematic:


![Screenshot from 2024-02-13 23-50-47](https://github.com/bbbenji/deciLight/assets/1678118/5957b364-939a-45fc-963b-7a0aaaa96e0c)

### Firmware:

The firmware is open source and lives at [https://github.com/bbbenji/deciLight](https://github.com/bbbenji/deciLight). Contributions and customisations are very welcome.

For alternative functionality, consider flashing deciLight with WLED, offering sound-reactive animations at the sacrifice of precise decibel readings.

#### Layout

The sketch is split by responsibility, so that adding a feature usually means touching one file. `deciLight.ino` itself is only wiring.

| File | Responsibility |
| --- | --- |
| `deciLight.ino` | `setup()` and `loop()`, nothing else |
| `config.h` | Every tunable value - pins, microphone datasheet figures, thresholds, colours, timing. No logic |
| `settings.{h,cpp}` | Thresholds and brightness, clamped to safe ranges and persisted to NVS |
| `sound_level.{h,cpp}` | I2S sampling task, IIR filtering, Leq in dB |
| `signal_light.{h,cpp}` | LED state, colour mapping, smoothing and hysteresis |
| `remote_control.{h,cpp}` | IR key map and what each key does |
| `sos-iir-filter.h` | Second-Order Sections filter kernel, with a hand-written Xtensa assembly inner loop. Upstream code from [esp32-i2s-slm](https://github.com/ikostoski/esp32-i2s-slm), unmodified |
| `math/*.m` | GNU Octave scripts that generate the equaliser coefficients for each supported microphone |

`sos-iir-filter.h` emits its filter kernel as file-scope assembly, so it *defines* symbols rather than declaring them. It must be included from exactly one translation unit - currently `sound_level.cpp`. Including it anywhere else will fail at link time with duplicate definitions.

#### How it works

Two FreeRTOS tasks, connected by a queue:

1. **Sampling task** (high priority). Reads 125ms blocks of audio from the I2S microphone at 48kHz, runs each block through the microphone equaliser and the A-weighting filter, and pushes the resulting sums of squares onto a queue. It does the minimum possible per block - no divisions, no logarithms.
2. **Main loop.** Pulls blocks off the queue, converts them to a decibel value averaged over 250ms, feeds that through a smoothing filter, and maps the result to a colour. In between it services the IR receiver and flushes any pending settings to flash.

The split matters because the FPU-heavy filtering can then be scheduled independently of the LED and remote work. The sample rate is fixed at 48kHz by the design of the IIR filters - changing it invalidates the coefficients.

Colour is chosen with hysteresis rather than a bare comparison, so a room sitting exactly on a threshold does not strobe between two colours. `DB_SMOOTHING` sets how quickly the light reacts, `DB_HYSTERESIS` sets how far past a threshold the level must travel before the colour changes.

#### Building

Requires the ESP32 core and two libraries. Verified against ESP32 core 2.0.5, FastLED 3.10.5 and IRremoteESP8266 2.9.0.

```sh
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@2.0.5

arduino-cli lib install FastLED
arduino-cli lib install IRremoteESP8266

arduino-cli compile --fqbn esp32:esp32:firebeetle32 .
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:firebeetle32 .
```

A current build uses about 63% of program storage and 15% of dynamic memory, leaving plenty of room for the networking features on the roadmap.

The sketch also opens directly in the Arduino IDE, and `.vscode/` carries a working configuration for the VS Code Arduino extension. Note that the `.ino` filename has to match the folder name, which is why it is `deciLight.ino`.

#### Configuring

Almost everything worth changing is a named constant in `config.h`:

| Constant | Purpose |
| --- | --- |
| `PIN_*` | Pin assignment, matching `pins.txt` |
| `DB_MIN_DEFAULT`, `DB_MAX_DEFAULT` | Thresholds a factory-fresh unit starts with |
| `DB_LIMIT_LOW`, `DB_LIMIT_HIGH` | How far the remote may push the thresholds |
| `DB_SMOOTHING` | How quickly the light reacts. 1.0 is instant, lower is calmer |
| `DB_HYSTERESIS` | dB of overshoot needed before the colour changes |
| `COLOR_QUIET`, `COLOR_WARN`, `COLOR_LOUD` | The three signal colours, `0xRRGGBB` |
| `LED_COUNT`, `LED_BRIGHTNESS_*` | LED ring size and brightness range |
| `LED_PSU_VOLTS`, `LED_PSU_MILLIAMPS` | Power budget FastLED dims against, rather than browning out the regulator |
| `MIC_*` | Microphone datasheet figures. `MIC_OFFSET_DB` is the linear calibration against a reference meter |
| `MIC_EQUALIZER`, `MIC_WEIGHTING` | Which filters to apply. Set the weighting to `C_weighting` or `None`, and update `DB_UNITS` to match |

Fitting a different microphone means setting the `MIC_*` values from its datasheet and pointing `MIC_EQUALIZER` at the matching filter. Coefficients for the ICS-43432, ICS-43434, IM69D130 and SPH0645LM4H-B are derived in `math/`.

#### Remote control

Mapped for the 24-key NEC remote sold with cheap LED strips. Holding a key repeats it.

| Key | Action |
| --- | --- |
| On | Return to automatic mode, colour follows the sound level |
| Off | LEDs dark. Measurement continues |
| Bright +/- | Brightness, in five steps. Persists across a power cycle |
| Any colour key | Hold that colour, leaving automatic mode |
| Flash / Strobe | Lower threshold up / down |
| Fade / Smooth | Upper threshold up / down |

Threshold changes blink the ring to confirm and print the new window to the serial console at 115200 baud. Thresholds are clamped so they can never cross or wrap, and are written to flash a few seconds after the last press, so holding a key costs one flash write rather than dozens.

An unrecognised key prints its protocol and hex code to the serial console, which is enough to map a different remote: add the code to `kKeyMap` in `remote_control.cpp`.
