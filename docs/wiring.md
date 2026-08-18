# Wiring

Every module in a deciLight, and how it attaches to the ESP32.

## Boards

Two boards are supported, and the wiring below is the same for both:

| Board                                                         | FQBN                             |
| ------------------------------------------------------------- | -------------------------------- |
| DFRobot FireBeetle ESP32                                      | `esp32:esp32:firebeetle32`       |
| Generic ESP-WROOM-32 dev board (DevKit v1, DOIT, NodeMCU-32S) | `esp32:esp32:esp32:FlashFreq=80` |

They carry the same ESP-WROOM-32 module, so the firmware is identical - the
board definitions differ only in pin aliases the project never uses and in
default flash settings. CI builds both on every push.

`FlashFreq=80` on the generic board matches what the FireBeetle definition
selects by default; leaving it off clocks the flash at 40MHz, which works but
is slower for no reason.

Every GPIO this project uses is broken out on the 30-pin DevKit v1 and on
everything larger. If you are adapting a smaller module, the pins are all
configurable in `config.h`.

**Pin numbers here are ESP32 GPIO numbers, not board silkscreen labels.** The
two are not the same thing on either board: a FireBeetle's pad marked `D2` is
GPIO25, while the LED ring wants GPIO2, which that board marks `D9`. Most
WROOM-32 dev boards label their pads with the GPIO number directly, which is
simpler, but check yours. Wire colours in brackets match the reference build
and are only a convention.

## Connections

| Signal               | GPIO | Goes to          | Notes                            |
| -------------------- | ---- | ---------------- | -------------------------------- |
| LED data             | 2    | Ring `DIN`       |                                  |
| IR signal            | 4    | Receiver `OUT`   |                                  |
| I2S SCK (bit clock)  | 14   | Mic `SCK`        | Must be output-capable           |
| I2S WS (word select) | 15   | Mic `WS`         | Must be output-capable           |
| I2S SD (data out)    | 32   | Mic `SD`         | May be an input-only pin (36-39) |
| Mic channel select   | -    | Mic `L/R` to GND | See the note below               |
| I2C SDA              | 21   | OLED `SDA`       | Board default                    |
| I2C SCL              | 22   | OLED `SCL`       | Board default                    |
| 3.3V                 | -    | Mic, IR, OLED    |                                  |
| 5V                   | -    | Ring `5V`        | See the power note               |
| Ground               | -    | Everything       | Must be common                   |

The three I2S pins are the ones set in `config.h`, and unlike some chips the
ESP32 can route them to almost any free pin, so they are a convention rather
than a constraint. `SCK` and `WS` have to be output-capable; `SD` can be one
of the input-only pins.

```
   ESP32                         Microphone (INMP441 / ICS-43434)
   ─────                         ────────────────────────────────
            3V3  ───────────────  VDD
            GND  ──────┬────────  GND
                       └────────  L/R     tie low = left channel
         GPIO14  ───────────────  SCK     bit clock       (green)
         GPIO15  ───────────────  WS      word select     (blue)
         GPIO32  ───────────────  SD      data out        (yellow)


   ESP32                         IR receiver (VS1838B / TSOP38238)
   ─────                         ─────────────────────────────────
            3V3  ───────────────  VCC
            GND  ───────────────  GND
          GPIO4  ───────────────  OUT                     (yellow)


   ESP32                         NeoPixel Jewel (7 LEDs)
   ─────                         ───────────────────────
          GPIO2  ───[330R]──────  DIN                     (yellow)
            GND  ──────┬────────  GND
                       │
      5V supply   GND  ┴
                  +5V  ─────────  5V


   ESP32                         OLED (SSD1306 128x64, I2C)
   ─────                         ──────────────────────────
            3V3  ───────────────  VCC
            GND  ───────────────  GND
   GPIO22 / SCL  ───────────────  SCL
   GPIO21 / SDA  ───────────────  SDA
```

## Two pins worth knowing about

**GPIO2 also drives the onboard LED.** Both board definitions set
`LED_BUILTIN` to 2, and the LED sits on that pin behind a resistor to ground.
That extra load is on the NeoPixel data line, which in practice is harmless -
the reference build has always worked this way - but it does mean the onboard
LED flickers whenever the ring is written, and on a long or marginal data lead
it is one more thing degrading the edge. If a build misbehaves on the first
pixel, moving `PIN_LED_DATA` in `config.h` to a plain GPIO such as 13, 25 or 27
is the cheapest thing to try.

GPIO2 is also a strapping pin, sampled at reset to decide boot mode. It needs
to be free to sit low while GPIO0 is low, which is what the USB bridge does to
enter download mode. A NeoPixel ring on the line does not prevent this, but if
a board suddenly refuses to accept an upload, disconnecting the LED data lead
is worth trying before anything more drastic.

**GPIO15 is a strapping pin too**, and holding it low at reset silences the
ROM bootloader's chatter on the serial console. The I2S peripheral drives it
only after boot, so this has no effect in practice - it is mentioned because a
quieter-than-expected boot log is otherwise puzzling.

Nothing else in the build touches a strapping pin. GPIO12, the one that can
brick a board by selecting the wrong flash voltage, is deliberately unused.

## Microphone channel

Tie `L/R` to ground. That puts the microphone on the left-hand slot, which is
what `channel_format` in `sound_level.cpp` is set up for.

Be aware that the arduino-esp32 1.0.2 to 1.0.3 update swapped the meanings of
`I2S_CHANNEL_FMT_ONLY_LEFT` and `ONLY_RIGHT`, and the driver has carried the
swap ever since - which is why the firmware asks for `ONLY_RIGHT` to read the
left slot. If a freshly wired unit reports a level pinned at the noise floor,
this pairing is the first thing to check: either move `L/R` to 3.3V or flip
`channel_format`, but not both.

## Display

The 0.96" SSD1306 modules sold as GME12864-11, -12 and -13 come in two
flavours, and the pin count tells them apart at a glance:

- **4 pins** (`GND`, `VCC`, `SCL`, `SDA`) - I2C. This is what the table above
  wires, and what the project assumes.
- **7 pins** (adds `RES`, `DC`, `CS`) - SPI. Faster, but it costs three more
  GPIOs for a display that only ever shows a couple of numbers.

Pin order varies between batches even within the same part number, so read the
silkscreen rather than trusting the order above.

Power it from 3.3V. Many of these modules carry a regulator and will accept
5V, but the SSD1306 itself is a 3.3V part and the ESP32's I2C lines are 3.3V,
so there is nothing to gain and a level mismatch to lose.

Almost all of these modules answer at I2C address **0x3C**. A few are strapped
to 0x3D, usually by a resistor or solder blob on the back next to a `0x78`/`0x7A`
marking - those are the 8-bit forms of the same two addresses. If the display
stays blank, an I2C scan sketch is the quickest way to settle it.

GPIO21 and GPIO22 are the ESP32 Arduino core's default `SDA` and `SCL`, so
`Wire.begin()` finds them with no arguments. Neither is a strapping pin and
neither clashes with anything else in this build.

## Power

The ring draws around 420mA with all seven pixels at full white, which is more
than should be pulled through a dev board's USB connector and traces. Feed it
from the 5V supply directly and tie the grounds together. The 3.3V rail
comfortably handles the microphone, the IR receiver and the display between
them - single-digit milliamps in total.

A 1000µF capacitor across the ring's 5V and ground steadies things if the LEDs
flicker on a sudden brightness change, and the 330R resistor in the data line
is cheap insurance against ringing on a long lead. Neither is essential on a
short bench setup.

One caveat that applies to any 3.3V microcontroller driving WS2812s: those LEDs
nominally want a logic high above about 3.5V when running on 5V. It works far
more often than not. If the first pixel misbehaves while the rest are fine,
that is the reason, and a level shifter fixes it.

## What the display shows

At boot, for a couple of seconds:

    ┌────────────────────────────────┐
    │                                │
    │        deciLight               │
    │          x.x.x                 │
    │                                │
    └────────────────────────────────┘

then, once measurements start:

    ┌────────────────────────────────┐
    │ AUTO                192.168.4.1│
    │                                │
    │  52 dBA                        │
    │                                │
    │        |          |            │
    │ ███████████████░░░░░░░░░░░░░░░ │
    │ 40 - 60                        │
    └────────────────────────────────┘

The operating mode and the unit's address across the top, the measured level
in whole decibels, a bar spanning the range the thresholds are allowed to take
with a tick above it for each threshold, and the window itself along the
bottom. When the microphone is out of range the bottom right reads `OVER` or
`QUIET`, which usually means a wiring problem rather than a genuinely extreme
room.

The panel is optional and probed at boot: if nothing answers at either address
the firmware carries on without it, so the same build serves units with and
without a screen. `FEATURE_DISPLAY` in `config.h` removes the code entirely
and saves about 29KB of flash.
