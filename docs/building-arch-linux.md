# Building and flashing deciLight on Arch Linux

Arch-specific notes for getting the firmware onto a board. Everything here was
checked against a working install (arduino-cli 1.4.1, ESP32 core 2.0.5,
Python 3.14.7, kernel 7.1.8-cachyos). For what the firmware actually does and
how it is laid out, see the Firmware section of the [README](../README.md).

## 1. Packages

```sh
sudo pacman -S --needed arduino-cli python-pyserial
```

Two packages, and the second one is the part people miss. The `arduino-cli`
package depends only on `glibc`, because it is a static Go binary - but on
Linux the ESP32 core does not upload with a bundled binary. Its `platform.txt`
sets `tools.esptool_py.cmd.linux=esptool.py`, so flashing runs a Python script
that imports `pyserial`. Compiling works without it; uploading fails.

Nothing else is needed. The compiler toolchain is downloaded by `arduino-cli`
in the next step, not from the repositories, and the `cp210x`, `ch341` and
`ftdi_sio` USB-serial modules are all in the stock Arch and CachyOS kernels.

The graphical `arduino-ide` package is optional and unrelated to any of the
commands below.

## 2. Toolchain

One-time setup. Create a config file if you do not have one - this is safe to
run either way, as it refuses to overwrite an existing config rather than
clobbering it:

```sh
arduino-cli config init
```

Then add Espressif's board index and install the core and libraries:

```sh
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@2.0.5

arduino-cli lib install FastLED
arduino-cli lib install IRremoteESP8266
```

The core is a large download - a few hundred megabytes into `~/.arduino15`.

Version 2.0.5 is pinned deliberately: it is what `.vscode/c_cpp_properties.json`
points at, and core 3.x moves to a different I2S driver API that this firmware
does not yet use.

## 3. Serial port access

**This is the one thing that differs from every Ubuntu tutorial you will
find.** On Arch, serial devices belong to the `uucp` group, not `dialout`:

```
/usr/lib/udev/rules.d/50-udev-default.rules:47
  KERNEL=="tty[A-Z]*[0-9]|...", GROUP="uucp"
```

Add yourself to it:

```sh
sudo usermod -aG uucp $USER
```

Then **log out and back in** - a new shell is not enough, group membership is
established at login. Check with `id -nG`, which should list `uucp`.

Without this, every upload fails with a permission error on `/dev/ttyUSB0`.
`sudo` is not the answer; it runs the upload as root with a different
`~/.arduino15`.

## 4. Find the board

Plug the board in and look for it:

```sh
arduino-cli board list
```

ESP32 boards use a USB-to-serial bridge and show up as `/dev/ttyUSB0` - CP2104
on the FireBeetle, CH340 on many clones. The board name column usually reads
"Unknown", which is fine; the port is what matters. If nothing appears:

```sh
sudo dmesg -w        # then unplug and replug, watch for cp210x or ch341
```

A cable that only carries power and no data is a common cause of silence.

## 5. Build

```sh
cd /path/to/deciLight
arduino-cli compile --fqbn esp32:esp32:firebeetle32 \
  --build-property upload.maximum_size=1966080 .
```

The `--build-property` is not optional with WiFi enabled. `partitions.csv` in
the sketch folder gives the application a 1.9MB slot, but the FireBeetle board
definition hardcodes `upload.maximum_size=1310720` and exposes no partition
menu, so the size check would reject a firmware that actually fits. See the
Building section of the README for the full explanation.

Building with `FEATURE_WIFI` set to `0` in `config.h` fits the stock layout and
needs no override.

## 6. Flash

```sh
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:firebeetle32 .
```

No `--build-property` here - `upload` flashes the existing build and does not
repeat the size check.

To do both in one go:

```sh
arduino-cli compile --fqbn esp32:esp32:firebeetle32 \
  --build-property upload.maximum_size=1966080 --upload -p /dev/ttyUSB0 .
```

The FireBeetle resets into the bootloader automatically. On boards that do not,
hold **BOOT**, tap **EN/RST**, release **BOOT**, and upload.

## 7. Watch it run

```sh
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Quit with `Ctrl-C`. You should see the measured level a few times a second,
the threshold window, and the address the web interface is on.

The port cannot be open in two places at once. Close the monitor before
uploading, or the upload will fail to claim the port.

## 8. Reach the web interface

If the unit has no network configured it starts its own access point. Join
`deciLight` with password `decilight` from a phone or laptop, then open
<http://192.168.4.1/>.

Once it has joined a real network it also answers to
<http://decilight.local/>. On this machine that resolves through
`systemd-resolved`, which is active with mDNS enabled - `nss-mdns` is installed
but is *not* wired into `/etc/nsswitch.conf`, and does not need to be. If the
name does not resolve, check that mDNS is enabled on the interface you are on:

```sh
resolvectl status | grep -A2 "Link.*wlan"
sudo resolvectl mdns wlan0 yes      # if it shows -mDNS
```

The address is printed to the serial console at boot either way, so mDNS is a
convenience rather than a requirement.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `Permission denied: '/dev/ttyUSB0'` | Not in the `uucp` group, or you have not logged out and back in since joining it. See section 3 |
| `ModuleNotFoundError: No module named 'serial'` | `python-pyserial` is not installed. Compiling works without it, uploading does not |
| `text section exceeds available space in board` | Missing `--build-property upload.maximum_size=1966080`. See section 5 |
| `Could not open /dev/ttyUSB0, the port doesn't exist` | Board not plugged in, a power-only USB cable, or the serial monitor still has the port open |
| No `/dev/ttyUSB*` on plug-in | Watch `sudo dmesg -w` while replugging. If the device enumerates and then vanishes a second later, something else has claimed it - see the two notes below |
| Upload starts then fails partway | Drop the speed: `--build-property upload.speed=115200` |
| `Failed to connect to ESP32: Timed out waiting for packet header` | Hold **BOOT**, tap **EN/RST**, release **BOOT**, retry |

### brltty

The standard advice on Arch forums is to uninstall `brltty`, because it used to
claim CH340 adapters as braille displays. **On a current Arch system this no
longer applies.** The shipped `90-brltty-usb-customized.rules` does not match
CH340 (`1a86:7523`), generic FTDI (`0403:6001`) or CP210x (`10c4:ea60`) - only
braille-specific product IDs. `brltty` being installed and in the `uucp` group
is normal and harmless. Only investigate it if a `ttyUSB` device really does
appear and then disappear within a second or two of plugging in.

### ModemManager

`modemmanager` probes new serial devices to see whether they are cellular
modems, which can hold the port open exactly when you want to flash. It is
installed but inactive here. If you enable it later and uploads start failing
intermittently, either stop it while flashing:

```sh
sudo systemctl stop ModemManager
```

or exempt the adapter permanently with a udev rule:

```sh
# /etc/udev/rules.d/99-esp32-no-modemmanager.rules
ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ENV{ID_MM_DEVICE_IGNORE}="1"
ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{ID_MM_DEVICE_IGNORE}="1"
```

Then `sudo udevadm control --reload-rules && sudo udevadm trigger`.

## VS Code

`.vscode/arduino.json` already carries the board, port and the partition size
override as a `buildPreferences` entry, so the Arduino extension builds and
uploads without any of the flags above. It needs the same `arduino-cli`,
`python-pyserial` and `uucp` setup underneath.

`.vscode/c_cpp_properties.json` hardcodes paths under
`~/.arduino15/packages/esp32/hardware/esp32/2.0.5/`. If IntelliSense shows
missing includes after a core update, that file is what needs adjusting - it
does not affect the build.
