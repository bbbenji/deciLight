# Host tests

Logic tests for the firmware's hardware-independent modules, compiled and run
on a development machine. No board, no ESP32 toolchain, no network.

```sh
make -C test         # build and run
make -C test check   # also type-check every module against the stubs
make -C test clean
```

They run in well under a second, so there is no reason not to run them before
pushing. CI runs them on every push as a separate job from the firmware build.

## What is covered

| Suite | What it asserts |
| --- | --- |
| `test_settings.cpp` | Clamping and the threshold-window invariant, repair of a bad stored pair, and that a burst of edits produces one deferred flash write rather than dozens |
| `test_signal_light.cpp` | Colour mapping per band, that hysteresis holds steady while the level dithers across a threshold, that smoothing damps a single spike but not a sustained one, mode behaviour, and that the ring is only rewritten when the colour actually changes |
| `test_remote_control.cpp` | Every one of the 24 keys individually, hold-to-repeat and its rate limit, and that unmapped codes change nothing |
| `test_group_sync.cpp` | The wire format, group and version filtering, peer aging, loudest-versus-average, and that a received change is applied on the main thread without echoing back out. Packets are captured from the module's own transmissions and handed back, so encode and decode are checked against each other rather than against a copy of the struct |
| `test_display.cpp` | That a missing panel is handled, that invisible changes are not redrawn, and that redraws are rate limited - a frame costs 22ms of blocked `loop()`, so how *rarely* it draws is the property worth pinning |

Asserting each remote key separately is deliberate. The key map is a table of
raw hex codes, `lookup()` returns the first match, and a duplicated code would
leave the shadowed entry silently dead - which is exactly the kind of slip a
per-key assertion catches and a spot check does not.

## What is not covered, and why

**Anything hardware-dependent.** The stubs record what the firmware asked for;
they do not emulate a microphone, LEDs, an IR receiver or a radio. A test
passing here means the logic is right, not that the device works.

- `sound_level.cpp` is type-checked but not run. Its filter kernel is
  hand-written Xtensa assembly and cannot be linked on a host, and the DSP
  around it is upstream code from
  [esp32-i2s-slm](https://github.com/ikostoski/esp32-i2s-slm).
- `web_control.cpp` is absent entirely. Stubbing WiFi, WebServer and mDNS would
  be a great deal of surface for very little logic worth testing; the real
  `arduino-cli` build in CI compiles it on every push.
- Timing is simulated. `millis()` only advances when a test says so, which
  makes the deferred-write and repeat-interval tests deterministic, but means
  nothing here says anything about real-world latency.

## The stubs

`stubs/` holds cut-down versions of `Arduino.h`, `FastLED.h`, `Preferences.h`
and the IR headers. `fakes.cpp` implements them over in-memory state, and
`fakes.h` is the control surface tests use to move the clock, inspect the LED
ring, count flash writes and feed in key presses.

Where a stub can be either faithful or forgiving, it is faithful. `getString`
returns 0 and leaves the caller's buffer untouched when a key is missing,
exactly as the real Preferences does, because the friendlier behaviour would
hide the class of bug where a caller assumes otherwise - which it did, until
the stub was corrected.

The stubs will drift from the real libraries over time. That is tolerable
because they are not the thing that proves the firmware compiles - the ESP32
build in CI is. If a stub goes out of date, the symptom is a host build failure,
not a silently wrong test.

Colour constants in the FastLED stub carry the library's real values, so a test
asserting on `CRGB::Green` catches the same `0x008000`-versus-`0x00FF00`
distinction the hardware would.
