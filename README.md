# deciLight - Visual Noise Monitor & Educational Tool

[![Build firmware](https://github.com/bbbenji/deciLight/actions/workflows/build.yml/badge.svg)](https://github.com/bbbenji/deciLight/actions/workflows/build.yml)

deciLight is a traffic signal-inspired lighting system that responds to ambient sound. It measures a properly A-weighted sound level with an I2S microphone and colours itself from it—green while a room is quiet, yellow as it rises, red once it is too loud.

Thresholds, colours and brightness can be set from an infrared remote, or from any phone or laptop browser without installing anything. An optional screen shows the live level, and units can be linked so that several lights act as one.

- **Real measurement.** A-weighted LAeq from a digital microphone, filtered on a dedicated core, not a bare analogue reading.
- **Two ways to control it.** A 24-key IR remote, and a self-contained web interface the unit serves itself.
- **Optional status screen.** A 128x64 OLED showing level, thresholds, mode and the unit's address.
- **Groups.** Give two units the same group name and they act as one light—including three stacked as a working traffic signal, one lamp each.
- **Updates over the air.** Reflash a mounted unit from the browser instead of taking it down for a cable.
- **Works with no network at all.** A unit that cannot find WiFi brings up its own access point, so it is usable in any room.

### Classroom Noise Management

In the classroom, deciLight can serve as an effective noise monitor, signaling to students when the noise level has become too high (red), when it's close (yellow), or when the classroom environment is at an acceptable sound level (green). This visual cue helps in self-regulation, as students can adjust their volume without direct intervention from the teacher, fostering a sense of responsibility and self-awareness among the pupils. By setting specific decibel thresholds—from the IR remote or a phone—educators can customize the sensitivity of deciLight to suit the needs of different activities, whether it's quiet reading time or a lively group discussion.

### Educational Games and Activities

Beyond its utility as a noise monitor, deciLight's color-changing feature can be integrated into various games and activities that engage students and facilitate learning. For instance:

- **Green Light, Red Light Game:** Leveraging deciLight's ability to change colors, teachers can conduct the classic "Green Light, Red Light" game, where students move when the light is green and stop when it's red. This can be a fun, physically active break between lessons or used as a tool for teaching self-control and listening skills.
- **Sound Level Challenges:** Teachers can create challenges for students to maintain a certain noise level (green) during group work or activities, rewarding them when they successfully stay within the acceptable range. This encourages teamwork and collective effort to achieve a common goal.
- **Interactive Storytelling:** Incorporating deciLight into storytelling can make reading sessions more interactive. For example, the light could change colors to reflect the mood or action within the story, or students could be asked to modulate their voice levels to keep the light green, enhancing engagement and comprehension.

### Bill of Materials (BOM):

- **ESP32 Microcontroller:** The brain of deciLight. Either a FireBeetle ESP32 or any generic ESP-WROOM-32 development board; its WiFi carries both the web interface and the link between units.
- **NeoPixel Jewel:** Provides bright, customizable colors for the light signal, ensuring vivid visibility.
- **IR Receiver:** Enables remote control functionality, allowing users to adjust settings and change colors from a distance.
- **I2S MEMS Microphone:** Senses ambient sound levels to trigger color changes based on predefined decibel thresholds. An INMP441 or ICS-43434; filter coefficients for several others are included.
- **0.96" SSD1306 OLED (optional):** A 128x64 I2C module showing the live level, thresholds and mode. The firmware detects whether one is fitted, so the same build serves units with and without.
- **Screws for Assembly:** m3x5 screws (4), m3-3 screws (2), and m1.5x3 screws (4-8) for secure assembly and mounting.
- **IR LED Remote:** Offers a user-friendly interface for adjusting deciLight settings and colors remotely.
- **5V 2A USB Power Supply:** Ensures reliable power delivery to the deciLight. A high-quality supply is recommended for optimal performance.
- **Wire:** Necessary for connections and assembly.

### Future Enhancements:

Networked synchronisation, the external display and controlling several units from one remote have all landed—see [Groups](#groups) and [Status screen](#status-screen). What is still open:

- **Authenticated group traffic:** membership is currently a convention rather than a secret. Anything in radio range running this firmware with the same group name joins in.
- **Quiet hours:** put the light and screen to sleep outside teaching time rather than leaving them lit overnight.
- **Calibration helper:** `MIC_OFFSET_DB` is adjusted by hand against a reference meter. A guided routine would make that less fiddly.
- **Enclosure revision:** the printed parts predate the screen and have nowhere to mount it.

### Assembly & Printing Tips:

- The construction of deciLight is designed to be straightforward and user-friendly.
- The ESP32 mount is adjustable, catering to various models of the ESP32. If the mounting holes differ, customizing your mounting plate might be necessary.
- To enhance light reflection and efficiency, applying aluminum tape to the reflector is recommended.
- The components, including the reflector, lens, and visor, are designed for a friction fit, simplifying the assembly process.
- Print the reflector in vase-mode.
- Print the lens in clear PETG with grid infill.
- Print the feet in TPU and attach them with double sided tape.

### Wiring:

Pin assignments and hookup diagrams for every module are in [docs/wiring.md](docs/wiring.md); [pins.txt](pins.txt) is the same information in short form.

### Schematic:

Note that this predates the optional OLED; [docs/wiring.md](docs/wiring.md) is the current reference and covers the screen.

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
| `web_control.{h,cpp}` | WiFi bring-up and the HTTP control interface |
| `web_page.h` | The control page, served from flash |
| `ota.{h,cpp}` | Over-the-air firmware updates |
| `display.{h,cpp}` | Optional SSD1306 status screen |
| `group_sync.{h,cpp}` | ESP-NOW link between units: level sharing, and relaying settings, mode and the self test |
| `tools/dev-server.py` | Serves the control page against a simulated device, for working on the UI without hardware |
| `sos-iir-filter.h` | Second-Order Sections filter kernel, with a hand-written Xtensa assembly inner loop. Upstream code from [esp32-i2s-slm](https://github.com/ikostoski/esp32-i2s-slm), unmodified |
| `math/*.m` | GNU Octave scripts that generate the equaliser coefficients for each supported microphone |

`sos-iir-filter.h` emits its filter kernel as file-scope assembly, so it *defines* symbols rather than declaring them. It must be included from exactly one translation unit - currently `sound_level.cpp`. Including it anywhere else will fail at link time with duplicate definitions.

#### How it works

Two FreeRTOS tasks, connected by a queue:

1. **Sampling task** (high priority). Reads 125ms blocks of audio from the I2S microphone at 48kHz, runs each block through the microphone equaliser and the A-weighting filter, and pushes the resulting sums of squares onto a queue. It does the minimum possible per block - no divisions, no logarithms.
2. **Main loop.** Pulls blocks off the queue, converts them to a decibel value averaged over 250ms, feeds that through a smoothing filter, and maps the result to a colour. In between it services the IR receiver and flushes any pending settings to flash.

The split matters because the FPU-heavy filtering can then be scheduled independently of the LED and remote work. The sample rate is fixed at 48kHz by the design of the IIR filters - changing it invalidates the coefficients.

Colour is chosen with hysteresis rather than a bare comparison, so a room sitting exactly on a threshold does not strobe between two colours. `DB_SMOOTHING` sets how quickly the light reacts, `DB_HYSTERESIS` sets how far past a threshold the level must travel before the colour changes.

When a unit belongs to a group, the level fed into that decision is the group's rather than its own, and everything downstream - smoothing, hysteresis, zone - works exactly as it does for a lone light. That is what keeps a stack of three and a single unit running the same code path.

#### Mobile control

Every unit brings up a web interface that mirrors what the IR remote can do, plus a live level readout. There is no app to install and no internet connection required.

On boot the firmware joins the WiFi network stored in its settings. If none is configured, or it cannot connect within 15 seconds, it starts its own access point instead:

- **Network name:** `deciLight`
- **Password:** `decilight`
- **Address:** [http://192.168.4.1/](http://192.168.4.1/)

That fallback is the point: a unit carried between classrooms works with no infrastructure at all. Join its access point from a phone and open the page. Once joined to a real network the unit also answers to [http://decilight.local/](http://decilight.local/) on clients that support mDNS, and prints its address to the serial console at boot either way.

The page shows the current level against a coloured scale, and offers sliders for both thresholds and brightness, buttons for automatic and off, and a palette of fixed colours. Changes apply immediately and are shared with the remote - either control surface can adjust the same settings, and neither overrides the other. Network credentials can be entered from the page, after which the unit restarts to join.

Behind the page is a small HTTP API, if you would rather script it:

| Endpoint | Purpose |
| --- | --- |
| `GET /api/state` | Level, quality, mode, zone, thresholds, brightness, network status and firmware version, as JSON |
| `GET /api/version` | Just the product name and firmware version. Separate from `/api/state` so checking what a unit is running does not require pulling a live measurement - which is the question worth asking right after an over-the-air update |
| `POST /api/set` | `dbMin`, `dbMax`, `brightness`, `displayBrightness` - any subset |
| `POST /api/mode` | `mode=auto`, `mode=off`, or `mode=manual&color=RRGGBB` |
| `POST /api/test` | Runs the LED self test, on the whole group |
| `POST /api/group` | `group`, `groupLevel`, `zones`, `combine`, `inactiveLevel` - any subset. Changing the name restarts the unit |
| `POST /api/wifi` | `ssid`, `pass` - saved to flash, then the unit restarts |
| `POST /api/update` | Multipart firmware upload. Requires HTTP basic auth, and is refused entirely unless `OTA_PASSWORD` is set |

Every value is clamped by the same code that guards the remote, so no request can produce an unusable device.

There is no authentication on the settings endpoints. Anything that can reach the unit can change its thresholds or colour, which is the right trade for a classroom light on a local network, but do not expose it to the internet. Firmware upload is the exception and is treated separately below.

#### Checking a new build

Two things in the Diagnostics section of the page exist for the hour after a unit is first assembled.

**Test the LEDs** walks the ring through red, green, blue and white, a little over a second each, and the page says which colour should be showing. Red and green come first and adjacent deliberately: WS2812 rings ship in both GRB and RGB orderings, and if those two look swapped the ring is wired the other way round. Without a known pattern that presents as the light showing red in a quiet room, which is indistinguishable from a threshold or hysteresis problem. White is last because it is the worst case for the power budget, so a marginal supply shows up there. The test only overrides what reaches the ring - the mode carries on underneath, so anything chosen while it runs takes effect the moment it ends.

**The last remote code** is shown below it: the code, its protocol, whether it is in the key map, and how long ago it arrived. That is enough to check a receiver is alive without a serial cable, and it turns mapping an unfamiliar remote into reading numbers off a phone. Add the code to `kKeyMap` in `remote_control.cpp` to bind it. Hold-down repeats are not recorded, since they would only overwrite the code you are trying to read.

#### Updating over the air

Once a unit is on the network it can be reflashed from the same page, so a light mounted on a wall does not have to come down for a USB cable. Build as usual, then upload the resulting `.bin` from the "Update firmware" section. The ring turns blue while the image is written and the unit restarts on its own.

**This is off by default and has to be turned on deliberately.** An update endpoint accepts arbitrary code, the access point password is published in this repository, and the two together would let anyone within WiFi range replace the firmware. So `OTA_PASSWORD` in `config.h` is empty out of the box and every upload is refused until you set it; the page hides the upload form entirely while that is the case. Pick a password you do not use elsewhere - it travels as base64 over plain HTTP, which is fine on a classroom LAN and not fine anywhere else.

Set `FEATURE_OTA` to 0 to leave the code out of the build altogether. It costs about 10KB.

Set `FEATURE_WIFI` to 0 in `config.h` to build without any of this. That saves about 502KB of flash and 19KB of RAM, and takes the group features with it.

#### Building

Requires the ESP32 core and three libraries. Verified against ESP32 core 2.0.5, FastLED 3.9.20 and IRremoteESP8266 2.9.0. Arch Linux users have a few distribution-specific hurdles - serial port groups and an easily missed dependency - covered in [docs/building-arch-linux.md](docs/building-arch-linux.md).

```sh
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@2.0.5

arduino-cli lib install FastLED@3.9.20
arduino-cli lib install IRremoteESP8266@2.9.0
arduino-cli lib install "Adafruit SSD1306@2.5.17"

arduino-cli compile --fqbn esp32:esp32:firebeetle32 .
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:firebeetle32 .
```

That builds to about 74% of the stock application partition - or 70% with the optional size trim below, and 31% with `FEATURE_WIFI` set to 0. No custom partition table either way.

Two boards are supported and CI builds both. For a generic ESP-WROOM-32 dev board - DevKit v1, DOIT, NodeMCU-32S and the like - swap the FQBN for `esp32:esp32:esp32:FlashFreq=80`. Same module, same firmware, same pins; only the board definition differs. See [docs/wiring.md](docs/wiring.md) for the two GPIOs whose behaviour is worth knowing about.

**Pin FastLED at 3.9.20.** The 3.10 series switched to a unity build that drags in the entire libstdc++ locale stack, which costs roughly 440KB of flash on a project whose only demand of the library is solid colours on seven LEDs. Nothing needs those 440KB, and with them the firmware no longer fits the stock partition layout. CI enforces a size budget so this cannot creep back in unnoticed.

Optionally, trim IRremoteESP8266 down to the protocols this project uses, which saves about 50KB:

```sh
arduino-cli compile --fqbn esp32:esp32:firebeetle32 \
  --build-property "compiler.cpp.extra_flags=-D_IR_ENABLE_DEFAULT_=false \
    -DDECODE_NEC=true -DSEND_NEC=true -DDECODE_HASH=true" .
```

The library compiles support for around a hundred protocols by default. This keeps NEC, which is what the bundled remote speaks, and the hash fallback so an unrecognised remote can still be identified from the serial log. It is genuinely optional - the firmware builds and fits either way - and `.vscode/arduino.json` and CI both apply it. The one thing you give up is protocol-specific decoding of non-NEC remotes: those report a stable hash rather than a named protocol and code, which is still perfectly usable for mapping keys, just less legible.

The sketch also opens directly in the Arduino IDE, and `.vscode/` carries a working configuration for the VS Code Arduino extension. Note that the `.ino` filename has to match the folder name, which is why it is `deciLight.ino`.

#### Tests

The hardware-independent modules - settings, the signal light's colour and dampening logic, and the remote key map - have host tests that need no board and no ESP32 toolchain:

```sh
make -C test check
```

They run in under a second and cover threshold clamping, hysteresis, smoothing, mode behaviour, deferred flash writes, and every one of the 24 remote keys individually. See [test/README.md](test/README.md) for what is deliberately *not* covered.

The web interface can be worked on without a board too. `tools/dev-server.py` serves the real page from `web_page.h` against a simulated unit, reading the thresholds, limits and colours out of `config.h` so the mock cannot drift from the firmware:

```sh
tools/dev-server.py             # then open http://127.0.0.1:8080/
tools/dev-server.py --level 72  # hold a level instead of sweeping
tools/dev-server.py --check     # assert its JSON still matches web_control.cpp
```

By default it sweeps the level across the whole range every 40 seconds, so every colour, the overload note and the noise-floor note all appear without any input. CI runs `--check` on every push.

#### Status screen

An SSD1306 128x64 OLED on the I2C pins shows the current level, the threshold window, the operating mode and the unit's address. At boot it shows the product name and firmware version for a couple of seconds first, which is the quickest way to tell what is actually running on a unit after an over-the-air update. Wiring is in [docs/wiring.md](docs/wiring.md).

It is genuinely optional. The panel is probed at both of its usual I2C addresses during boot, and if nothing answers every display call becomes a no-op, so the same firmware serves units built with and without a screen. Set `FEATURE_DISPLAY` to 0 to leave the code out entirely and save about 32KB.

Screen brightness is adjustable from the Advanced section of the page and persists across reboots. Zero is not the dimmest setting but powers the panel down, which is what you want for a room the light stays in overnight - and while it is down no frames are sent at all.

The scale is deliberately not linear. The panel's contrast register is roughly linear in drive current while perception is roughly logarithmic, so a linear slider spends most of its travel across a range that all looks much the same. `DISPLAY_BRIGHTNESS_GAMMA` curves it, putting half the travel at about a fifth of full drive.

Two details matter at the dim end. Contrast zero is not the dimmest setting but no output at all, so `DISPLAY_CONTRAST_MIN` floors it and every position above zero shows something. And once contrast is at that floor the only remaining handle is the pre-charge period, which is ramped across the bottom tenth of the slider up to the value the driver itself uses - ramped rather than stepped, because a step there reads as the brightness lurching partway along the travel.

Sending a full frame is roughly 1KB over I2C, about 22ms during which `loop()` is blocked. Two things keep that from mattering: the module compares what it is about to draw against what is already on the panel and skips the transfer when nothing visible has changed, and even a real change is never sent more often than `DISPLAY_MIN_INTERVAL_MS`. In a steady room the screen is usually not being written at all.

#### Groups

Units that share a radio channel and a group name behave as one light. There is no pairing step: give two units the same name in the Group section of the page and they find each other.

Each unit broadcasts what its microphone hears a few times a second, and each works out the group's level for itself - there is no leader and nothing to elect. Thresholds, LED brightness, mode, colour and the LED self test all propagate too, so one IR remote drives a whole room.

Three per-unit settings compose into every arrangement:

| Setting | What it does |
| --- | --- |
| Follow the group's level | Light from the group's reading rather than this unit's own microphone |
| Zones | Which of quiet, warn and loud this unit lights for. All three by default, which is how a lone light behaves |
| Combine | Whether the group's level is the loudest reading or the average of them |

A **traffic-light stack** is three units with follow-group on, one zone each, and combine set to average - they hear the same sound, so averaging cancels per-microphone variation. Exactly one lamp is lit at a time, like a real signal. A **mirrored room** is all three zones on every unit with combine set to loudest, so any noisy corner turns the whole room red. A unit with follow-group off ignores the others entirely. Two units make a stack too, with one of them covering a pair of zones so there is no dead band.

What an inactive lamp shows is configurable: dark like a real traffic signal, or a faint glow so the stack still reads as one and a dead unit is distinguishable from an unlit one.

Zone masks, group name, screen brightness and the inactive level are deliberately *not* shared. They describe a unit's place in the arrangement rather than the room, and copying them would collapse a stack into three identical lights.

**The channel is the thing that breaks this.** ESP-NOW only reaches peers on the same WiFi channel, and in station mode the channel belongs to whichever router the unit joined. Units on the same network are fine; units on different networks will never hear each other however they are configured. `WIFI_AP_CHANNEL` pins the fallback access point so un-networked units still land together. The Group section reports the current channel and the live peer count, because "configured but hearing nobody" is the failure worth diagnosing quickly.

Membership is a convention rather than a secret: the transport is broadcast and the group name is a filter, so anything in radio range running this firmware with the same name joins in.

Set `FEATURE_ESPNOW` to 0 to leave all of it out, which saves about 9KB.

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
| `FEATURE_WIFI` | Build with or without networking and the web interface |
| `FEATURE_DISPLAY` | Build with or without the OLED status screen |
| `FEATURE_ESPNOW` | Build with or without group synchronisation |
| `GROUP_BROADCAST_MS`, `GROUP_PEER_TIMEOUT_MS` | How often a unit speaks, and how long a silent peer still counts |
| `WIFI_AP_CHANNEL` | Channel the fallback access point uses, so un-networked units share one |
| `FIRMWARE_VERSION`, `PRODUCT_NAME` | Shown on the splash screen and logged at boot |
| `DISPLAY_SPLASH_MS` | How long the splash is held before measurements take the screen |
| `DISPLAY_ADDRESSES`, `DISPLAY_MIN_INTERVAL_MS` | Which I2C addresses to probe, and the floor on redraw rate |
| `DISPLAY_BRIGHTNESS_DEFAULT` | Panel contrast a factory-fresh unit starts at |
| `WIFI_AP_SSID`, `WIFI_AP_PASSWORD` | The fallback access point |
| `WIFI_HOSTNAME` | Also the mDNS name, so `decilight.local` follows it |

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
