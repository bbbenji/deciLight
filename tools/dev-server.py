#!/usr/bin/env python3
"""
Serve the deciLight control page against a simulated device.

Lets the web interface be worked on in a browser with no hardware: the page is
read straight out of web_page.h, and the endpoints behave like the firmware's,
including the clamping. Thresholds, limits and colours are parsed from
config.h rather than duplicated here, so the mock cannot quietly drift from
the device it is standing in for.

    tools/dev-server.py            # then open http://127.0.0.1:8080/
    tools/dev-server.py --port 9000
    tools/dev-server.py --level 72 # hold a fixed level instead of sweeping

Not a test and not a simulator of the DSP - it fakes the measurement so the UI
has something to draw. Real behaviour still needs the board.
"""

import argparse
import json
import math
import re
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs

ROOT = Path(__file__).resolve().parent.parent


def load_page() -> bytes:
    """Pull the HTML out of the C++ raw string literal in web_page.h."""
    src = (ROOT / "web_page.h").read_text()
    start = src.index('R"HTML(') + len('R"HTML(')
    return src[start : src.rindex(')HTML"')].encode()


def load_config() -> dict:
    """Read the constants the page depends on out of config.h."""
    src = (ROOT / "config.h").read_text()
    cfg = {}
    for name, value in re.findall(
        r"constexpr\s+\w+\s+(\w+)\s*=\s*(0x[0-9A-Fa-f]+|-?[\d.]+)f?\s*;", src
    ):
        cfg[name] = int(value, 16) if value.startswith("0x") else float(value)
    for name, value in re.findall(r'constexpr\s+char\s+(\w+)\[\]\s*=\s*"([^"]*)"\s*;', src):
        cfg[name] = value
    # Some string constants are defined via the preprocessor so they can be
    # pasted into JSON literals; pick those up too.
    for name, value in re.findall(r'#define\s+(\w+)\s+"([^"]*)"', src):
        cfg.setdefault(name.replace("_JSON", ""), value)
    units = re.search(r'#define\s+DB_UNITS\s+"([^"]+)"', src)
    cfg["DB_UNITS"] = units.group(1) if units else "dBA"
    return cfg


CFG = load_config()
PAGE = load_page()


def clamp(value, low, high):
    return max(low, min(high, value))


class Device:
    """Just enough state to make the page behave as it would against a unit."""

    def __init__(self, fixed_level=None):
        self.db_min = int(CFG["DB_MIN_DEFAULT"])
        self.db_max = int(CFG["DB_MAX_DEFAULT"])
        self.brightness = int(CFG["LED_BRIGHTNESS_DEFAULT"])
        self.mode = "auto"
        self.color = "%06X" % int(CFG["COLOR_QUIET"])
        self.zone = "unknown"
        self.smoothed = None
        self.fixed_level = fixed_level
        self.started = time.time()

    # --- measurement ---

    def raw_level(self):
        if self.fixed_level is not None:
            return self.fixed_level
        # Sweep the whole usable range every 40s so every zone, plus the
        # overload and noise-floor notes, gets exercised without any input.
        t = (time.time() - self.started) / 40.0
        mid, span = 62.0, 34.0
        return mid + span * math.sin(t * 2 * math.pi)

    def sample(self):
        db = self.raw_level()
        a = CFG["DB_SMOOTHING"]
        self.smoothed = db if self.smoothed is None else self.smoothed + a * (db - self.smoothed)

        # Same hysteresis rule as signal_light.cpp.
        h = CFG["DB_HYSTERESIS"]
        s, lo, hi = self.smoothed, self.db_min, self.db_max
        z = self.zone
        if z == "quiet":
            z = "loud" if s > hi + h else ("warn" if s > lo + h else "quiet")
        elif z == "warn":
            z = "loud" if s > hi + h else ("quiet" if s < lo - h else "warn")
        elif z == "loud":
            z = "quiet" if s < lo - h else ("warn" if s < hi - h else "loud")
        else:
            z = "loud" if s > hi else ("warn" if s > lo else "quiet")
        self.zone = z

        floor, ceiling = CFG["MIC_NOISE_DB"], CFG["MIC_OVERLOAD_DB"]
        quality = "ok"
        if self.smoothed >= ceiling:
            quality = "overload"
        elif self.smoothed <= floor:
            quality = "quiet"
        return clamp(self.smoothed, floor, ceiling), quality

    # --- endpoints ---

    def state(self):
        db, quality = self.sample()
        return {
            "name": CFG.get("PRODUCT_NAME", "deciLight"),
            "version": CFG.get("FIRMWARE_VERSION", "0.0.0"),
            "db": round(db, 1),
            "units": CFG["DB_UNITS"],
            "quality": quality,
            "mode": self.mode,
            "zone": self.zone,
            "color": self.color,
            "dbMin": self.db_min,
            "dbMax": self.db_max,
            "brightness": self.brightness,
            "net": "ap",
            "ssid": "deciLight (dev server)",
            "ip": "127.0.0.1",
            "ota": True,
        }

    def set(self, args):
        span = int(CFG["DB_MIN_SPAN"])
        if "dbMin" in args:
            self.db_min = int(clamp(int(args["dbMin"][0]), CFG["DB_LIMIT_LOW"], self.db_max - span))
        if "dbMax" in args:
            self.db_max = int(clamp(int(args["dbMax"][0]), self.db_min + span, CFG["DB_LIMIT_HIGH"]))
        if "brightness" in args:
            self.brightness = int(
                clamp(int(args["brightness"][0]), CFG["LED_BRIGHTNESS_MIN"], CFG["LED_BRIGHTNESS_MAX"])
            )

    def set_mode(self, args):
        mode = args.get("mode", [""])[0]
        if mode not in ("auto", "off", "manual"):
            return False
        self.mode = mode
        if mode == "manual":
            self.color = args.get("color", ["FFFFFF"])[0].upper()
        else:
            # Re-derive the zone from fresh samples, as the firmware does.
            self.zone = "unknown"
            self.smoothed = None
        return True


class Handler(BaseHTTPRequestHandler):
    device: Device

    def log_message(self, fmt, *args):
        print("  %s" % (fmt % args))

    def _send(self, code, body, ctype="application/json"):
        raw = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if self.path.startswith("/api/state"):
            self._send(200, json.dumps(self.device.state()))
        elif self.path.startswith("/api/version"):
            self._send(200, json.dumps({
                "name": CFG.get("PRODUCT_NAME", "deciLight"),
                "version": CFG.get("FIRMWARE_VERSION", "0.0.0"),
            }))
        else:
            # Anything else serves the page, matching the firmware's catch-all.
            self._send(200, PAGE, "text/html")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        args = parse_qs(self.rfile.read(length).decode(errors="replace")) if length else {}
        path = self.path.split("?")[0]

        if path == "/api/set":
            self.device.set(args)
            self._send(200, json.dumps(self.device.state()))
        elif path == "/api/mode":
            if self.device.set_mode(args):
                self._send(200, json.dumps(self.device.state()))
            else:
                self._send(400, "unknown mode", "text/plain")
        elif path == "/api/wifi":
            if "ssid" not in args:
                self._send(400, "ssid required", "text/plain")
            else:
                print("  (pretending to save credentials and restart)")
                self._send(200, json.dumps({"restarting": True}))
        elif path == "/api/update":
            # Accepts and discards the body so the upload UI can be exercised.
            print("  (pretending to accept %d bytes of firmware)" % length)
            self._send(200, json.dumps({"ok": True, "restarting": True}))
        else:
            self._send(404, "not found", "text/plain")


def check_contract() -> int:
    """Fail if the mock's JSON has drifted from the firmware's.

    The whole value of this server is that the page cannot tell it apart from
    a real unit, so the field names are worth pinning down. CI runs this.
    """
    src = (ROOT / "web_control.cpp").read_text()
    body = src[src.index("void sendState()") : src.index("void handleRoot()")]
    firmware = set(re.findall(r'\\"(\w+)\\":', body))
    mock = set(Device().state())

    missing = firmware - mock
    extra = mock - firmware
    for k in sorted(missing):
        print(f"  missing from the mock: {k}")
    for k in sorted(extra):
        print(f"  in the mock but not the firmware: {k}")
    if missing or extra:
        print(f"\n{len(missing) + len(extra)} field(s) out of step with web_control.cpp")
        return 1
    print(f"  {len(firmware)} fields match web_control.cpp")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="verify the mock's JSON matches the firmware's, then exit")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--level", type=float, help="hold this dB level instead of sweeping")
    opts = ap.parse_args()

    if opts.check:
        raise SystemExit(check_contract())

    Handler.device = Device(opts.level)
    print(f"deciLight dev server on http://{opts.host}:{opts.port}/")
    print(f"  page:       {len(PAGE)} bytes from web_page.h")
    print(f"  firmware:   {CFG.get('PRODUCT_NAME')} {CFG.get('FIRMWARE_VERSION')}")
    print(f"  thresholds: {int(CFG['DB_MIN_DEFAULT'])} / {int(CFG['DB_MAX_DEFAULT'])} {CFG['DB_UNITS']}")
    print("  level:      " + (f"fixed at {opts.level}" if opts.level else "sweeping 28-96 over 40s"))
    try:
        ThreadingHTTPServer((opts.host, opts.port), Handler).serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
