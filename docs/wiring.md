# Wiring

Every module in a deciLight, and how it attaches to the ESP32.

**Pin numbers here are ESP32 GPIO numbers, not board silkscreen labels.** The
two are not the same thing on the reference board: a FireBeetle's pad marked
`D2` is GPIO25, while the LED ring wants GPIO2, which that board marks `D9`.
Wire by GPIO number and check against your board's own pinout. Wire colours in
brackets match the reference build and are only a convention.

## Connections

| Signal | GPIO | Goes to | Notes |
| --- | --- | --- | --- |
| LED data | 2 | Ring `DIN` | |
| IR signal | 4 | Receiver `OUT` | |
| I2S SCK (bit clock) | 14 | Mic `SCK` | Must be output-capable |
| I2S WS (word select) | 15 | Mic `WS` | Must be output-capable |
| I2S SD (data out) | 32 | Mic `SD` | May be an input-only pin (36-39) |
| Mic channel select | - | Mic `L/R` to GND | See the note below |
| I2C SDA | 21 | OLED `SDA` | Board default |
| I2C SCL | 22 | OLED `SCL` | Board default |
| 3.3V | - | Mic, IR, OLED | |
| 5V | - | Ring `5V` | See the power note |
| Ground | - | Everything | Must be common |

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

## Status of the display

**Wired, not yet driven.** Nothing in the firmware talks to the OLED - there is
no driver, no pin constants in `config.h`, and no library dependency. This
section documents the hardware so the board can be built now and the code can
follow.

Two things worth knowing before that code gets written. Pushing a full 128x64
frame is about 1KB over I2C, which is roughly 22ms at 400kHz - long enough to
stall `loop()` and let the measurement queue back up, so the display will want
partial or throttled updates rather than a redraw per measurement. And an
SSD1306 library plus its graphics layer costs somewhere around 30-50KB of
flash; the current build sits at 65% of the application partition, so there is
room, but it is worth measuring against the CI size budget when the time comes.
