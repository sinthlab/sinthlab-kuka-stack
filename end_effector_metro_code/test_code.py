#!/usr/bin/env python3
"""Offline test suite for code.py — runs the real firmware on a desktop, no board required.

    python3 test_code.py            (from anywhere; exits non-zero on failure)

WHY THIS EXISTS
    The cue ring's firmware is hard to exercise on hardware: the board ends up bolted inside a
    closed end effector, its behaviour is timing-dependent, and the failure that matters most --
    a cue that does not fire, or one that latches ON -- is exactly the one you do not want to
    discover during an experiment.

HOW IT WORKS
    CircuitPython's modules (board, neopixel, digitalio, the ESP32 driver, the HTTP server,
    microcontroller.nvm) are stubbed in-process, and time.monotonic_ns is replaced with a virtual
    clock so a 2.5 s cue takes no real time and never flakes. code.py is then executed up to --
    but not including -- its `while True:` main loop, which leaves every function and every route
    handler callable directly.

    Nothing here is imported by the firmware, and this file is NEVER copied to the board.

WHAT IT DOES NOT COVER
    Anything physical: the optocoupler, the real ESP32, actual LED timing, current draw. It
    checks logic and state machines, not electrons. Bench-test with a jumper on D2 as well --
    see README.md.
"""
from __future__ import annotations

import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FIRMWARE = os.path.join(HERE, "code.py")


# ---------------------------------------------------------------------------
# CircuitPython stubs, built in-process so this stays a single file
# ---------------------------------------------------------------------------
def install_stubs():
    board = types.ModuleType("board")
    for pin in ("D5", "D2", "ESP_CS", "ESP_BUSY", "ESP_RESET", "SCK", "MOSI", "MISO"):
        setattr(board, pin, pin)

    busio = types.ModuleType("busio")
    busio.SPI = lambda *a, **k: "spi"

    digitalio = types.ModuleType("digitalio")

    class Direction:
        INPUT, OUTPUT = "in", "out"

    class Pull:
        UP, DOWN = "up", "down"

    class DigitalInOut:
        def __init__(self, pin):
            self.pin, self.direction, self.pull, self.value = pin, None, None, True

    digitalio.Direction, digitalio.Pull, digitalio.DigitalInOut = Direction, Pull, DigitalInOut

    neopixel = types.ModuleType("neopixel")
    neopixel.GRBW = "GRBW"

    class NeoPixel:
        """Enough of adafruit_pixelbuf to check what would be sent to the ring.

        Values are stored unscaled, as PixelBuf does — `brightness` is applied on the way out to
        the LEDs, which is exactly why the firmware no longer scales colours itself.
        """

        def __init__(self, pin, n, brightness=1.0, auto_write=True, pixel_order=None):
            self.n, self.brightness = n, brightness
            self._buf = [(0, 0, 0, 0)] * n
            self.shows = 0

        def fill(self, c):
            self._buf = [tuple(c)] * self.n

        def show(self):
            self.shows += 1

        def __len__(self):
            return self.n

        def __getitem__(self, i):
            return self._buf[i]

        def __setitem__(self, i, v):
            self._buf[i] = tuple(v)

    neopixel.NeoPixel = NeoPixel

    microcontroller = types.ModuleType("microcontroller")
    microcontroller.nvm = bytearray(256)

    esp_pkg = types.ModuleType("adafruit_esp32spi")
    esp_pkg.__path__ = []
    esp_mod = types.ModuleType("adafruit_esp32spi.adafruit_esp32spi")
    esp_mod.WL_IDLE_STATUS = 0

    class ESP_SPIcontrol:
        def __init__(self, *a):
            self.status = 0
            self.firmware_version = b"1.7.4"
            self.MAC_address = b"\x01\x02\x03\x04\x05\x06"
            self.ip_address = b"\xc0\xa8\x04\x01"       # 192.168.4.1
            self.joined = None                          # set only by connect_AP

        def create_AP(self, ssid, password, channel=1):
            if not ssid or not password:
                raise RuntimeError("bad credentials")

        def connect_AP(self, ssid, password, timeout_s=10):
            self.joined = ssid          # present so a regression to station mode is visible

        def pretty_ip(self, ip):
            return ".".join(str(b) for b in ip)

    esp_mod.ESP_SPIcontrol = ESP_SPIcontrol
    sock_mod = types.ModuleType("adafruit_esp32spi.adafruit_esp32spi_socketpool")

    class SocketPool:
        def __init__(self, esp):
            pass

    sock_mod.SocketPool = SocketPool

    http = types.ModuleType("adafruit_httpserver")

    class _QueryParams:
        def __init__(self, d):
            self._d = d

        def get(self, key, default=None):
            return self._d.get(key, default)

    class Request:
        def __init__(self, params=None):
            self.query_params = _QueryParams(params or {})

    class Response:
        def __init__(self, request, text):
            self.text = text

    class Server:
        def __init__(self, pool, debug=False):
            self.routes = {}

        def route(self, path, methods=None):
            def deco(fn):
                self.routes[path] = fn
                return fn
            return deco

        def start(self, ip, port=80):
            pass

        def poll(self):
            pass

    http.Request, http.Response, http.Server = Request, Response, Server

    for name, mod in (
        ("board", board), ("busio", busio), ("digitalio", digitalio), ("neopixel", neopixel),
        ("microcontroller", microcontroller), ("adafruit_esp32spi", esp_pkg),
        ("adafruit_esp32spi.adafruit_esp32spi", esp_mod),
        ("adafruit_esp32spi.adafruit_esp32spi_socketpool", sock_mod),
        ("adafruit_httpserver", http),
    ):
        sys.modules[name] = mod
    esp_pkg.adafruit_esp32spi = esp_mod
    esp_pkg.adafruit_esp32spi_socketpool = sock_mod
    return microcontroller, Request


NVM, Request = install_stubs()

# Virtual clock: cues advance instantly and deterministically, so a 2.5 s duration costs nothing
# and the suite cannot flake on a slow machine.
CLOCK = {"ns": 0}
time.monotonic_ns = lambda: CLOCK["ns"]
time.sleep = lambda s: CLOCK.__setitem__("ns", CLOCK["ns"] + int(s * 1e9))


def advance(seconds):
    CLOCK["ns"] += int(seconds * 1e9)


with open(FIRMWARE) as f:
    SOURCE = f.read()
BODY = SOURCE[:SOURCE.index("while True:")]     # everything but the main loop


def load(ssid="KUKA_NEOPIXEL", password="testpassword"):
    """Boot the firmware fresh, as a power cycle would."""
    for key in ("CIRCUITPY_WIFI_SSID", "CIRCUITPY_WIFI_PASSWORD", "WIFI_MODE"):
        os.environ.pop(key, None)
    if ssid:
        os.environ["CIRCUITPY_WIFI_SSID"] = ssid
    if password:
        os.environ["CIRCUITPY_WIFI_PASSWORD"] = password
    mod = types.ModuleType("firmware")
    exec(compile(BODY, FIRMWARE, "exec"), mod.__dict__)
    return mod


def call(fw, route, **params):
    """Invoke an HTTP route handler and return its body."""
    return fw.server.routes[route](Request({k: str(v) for k, v in params.items()})).text


def trigger_on(fw):
    """Assert the trigger line and let the debouncer see it."""
    fw.cue_in.value = False                     # active-low: asserted pulls the pin down
    fw.poll_trigger()
    advance(0.010)
    fw.poll_trigger()


def trigger_off(fw):
    fw.cue_in.value = True
    fw.poll_trigger()
    advance(0.010)
    fw.poll_trigger()


RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"   {detail}" if detail else ""))


def section(title):
    print(f"\n{title}")


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def test_access_point_only():
    section("Access point only — the board never joins another network")
    fw = load()
    check("hosts its own AP", fw.esp.joined is None)
    check("gateway is 192.168.4.1", fw.wifi_ip == "192.168.4.1", fw.wifi_ip)
    os.environ["WIFI_MODE"] = "client"
    fw2 = load()
    check("a stray WIFI_MODE is ignored", fw2.esp.joined is None)


def test_trigger_is_on_off_with_a_timer():
    section("The trigger is on/off with a timer — the whole behaviour")
    fw = load()
    call(fw, "/config", r=0, g=255, b=0, w=0, duration=1.0, pattern="solid")
    check("idle before the trigger", not fw.cue_active())
    trigger_on(fw)
    check("ON at the trigger", fw.cue_active() and fw._cue_color == (0, 255, 0, 0))
    trigger_off(fw)
    advance(0.5)
    fw.cue_service()
    check("stays on after the line drops — the board's timer owns it", fw.cue_active())
    advance(0.6)
    fw.cue_service()
    check("OFF when the timer expires", not fw.cue_active())
    check("ring left dark", all(fw.pixels[i] == (0, 0, 0, 0) for i in range(60)))
    check("fire count incremented", fw.fired == 1)

    for duration in (0.3, 1.0, 2.5):
        call(fw, "/config", duration=duration)
        call(fw, "/off")
        trigger_on(fw)
        trigger_off(fw)
        advance(duration - 0.05)
        fw.cue_service()
        still_on = fw.cue_active()
        advance(0.1)
        fw.cue_service()
        check(f"duration={duration}s: on before, off after", still_on and not fw.cue_active())


def test_colour_is_used_exactly_as_set():
    section("Colour is used exactly as set — no scaling surprises")
    fw = load()
    check("no per-cue intensity setting", "intensity" not in fw.cfg)
    call(fw, "/config", r=255, g=140, b=0, w=0, pattern="segment", segments=6, duration=5)
    trigger_on(fw)
    check("the configured colour is what runs", fw._cue_color == (255, 140, 0, 0), fw._cue_color)
    check("pattern and segments applied", fw._cue_pattern == "segment" and fw._cue_segments == 6)
    lit = sum(1 for i in range(60) if fw.pixels[i] != (0, 0, 0, 0))
    check("segment lights blocks, not the whole ring", 0 < lit < 60, f"{lit} of 60")
    body = call(fw, "/config")
    check("read-back is unrounded", "r=255" in body and "g=140" in body)


def test_brightness_is_uncapped_but_clamped():
    section("Brightness — the single scale, uncapped, the operator's call")
    fw = load()
    check("no MAX_BRIGHTNESS constant", not hasattr(fw, "MAX_BRIGHTNESS"))
    check("conservative 0.2 default", fw.cfg["brightness"] == 0.2)
    call(fw, "/config", brightness=1.0)
    check("reaches 1.0 — no firmware cap", fw.cfg["brightness"] == 1.0)
    check("applied to the ring", fw.pixels.brightness == 1.0)
    call(fw, "/config", brightness=2.5)
    check("clamped to the valid range at the top", fw.cfg["brightness"] == 1.0)
    call(fw, "/config", brightness=-1)
    check("and at the bottom", fw.cfg["brightness"] == 0.0)
    call(fw, "/config", brightness="banana")
    check("garbage keeps the previous value", fw.cfg["brightness"] == 0.0)


def test_settings_compose():
    section("One endpoint, and settings compose")
    fw = load()
    call(fw, "/config", r=10)
    check("one field alone",
          fw.cfg["r"] == 10 and fw.cfg["pattern"] == "flash" and fw.cfg["duration"] == 2.0)
    call(fw, "/config", pattern="chase", segments=8)
    check("more fields, earlier ones kept",
          fw.cfg["pattern"] == "chase" and fw.cfg["segments"] == 8 and fw.cfg["r"] == 10)
    call(fw, "/config", brightness=0.5, debounce_ms=12, mode="follow")
    check("board settings share the endpoint",
          fw.cfg["brightness"] == 0.5 and fw.cfg["debounce_ms"] == 12 and fw.cfg["mode"] == "follow")
    check("cue settings survived", fw.cfg["pattern"] == "chase" and fw.cfg["r"] == 10)
    body = call(fw, "/config")
    check("read-back shows both sections", "# cue" in body and "# board" in body)
    call(fw, "/config", pattern="disco")
    check("an unknown pattern is rejected", fw.cfg["pattern"] == "chase")


def test_persistence():
    section("Settings survive a power cycle")
    NVM.nvm[:] = bytearray(256)
    fw = load()
    call(fw, "/config", r=1, g=2, b=3, w=4, pattern="breathe", segments=9,
         duration=1.75, period=0.45, brightness=0.9, debounce_ms=15, mode="follow", save=1)
    fw2 = load()
    check("cue restored",
          (fw2.cfg["r"], fw2.cfg["g"], fw2.cfg["b"], fw2.cfg["w"]) == (1, 2, 3, 4)
          and fw2.cfg["pattern"] == "breathe" and fw2.cfg["segments"] == 9)
    check("timing restored", fw2.cfg["duration"] == 1.75 and fw2.cfg["period"] == 0.45)
    # brightness is one byte on the way to NVM, so expect ~0.4% quantisation, not equality
    check("brightness restored", abs(fw2.cfg["brightness"] - 0.9) < 0.005,
          f"{fw2.cfg['brightness']:.4f}")
    check("board settings restored", fw2.cfg["debounce_ms"] == 15 and fw2.cfg["mode"] == "follow")

    NVM.nvm[5] ^= 0xFF                          # corrupt one byte -> checksum fails
    fw3 = load()
    check("a corrupt record is rejected wholesale, not half-loaded",
          fw3.cfg["w"] == 255 and fw3.cfg["mode"] == "pulse")
    check("and the board still boots", fw3.server is not None)

    call(fw3, "/config", r=99, save=1)
    call(fw3, "/config", reset=1)
    check("reset clears NVM too", load().cfg["r"] == 0)


def test_flash_wear():
    section("Flash wear — a write only happens when something actually changed")
    NVM.nvm[:] = bytearray(256)
    fw = load()
    check("no writes at boot", fw.nvm_writes == 0)

    call(fw, "/config", r=42, save=1)
    check("first save writes once", fw.nvm_writes == 1)

    for _ in range(50):
        call(fw, "/config", save=1)
    check("50 identical saves cost ZERO further writes", fw.nvm_writes == 1,
          f"{fw.nvm_writes} writes")

    call(fw, "/config", r=43, save=1)
    check("a real change writes again", fw.nvm_writes == 2)

    call(fw, "/config", r=99)                   # changed, but not saved
    check("changing without save=1 does not write", fw.nvm_writes == 2)

    call(fw, "/config", reset=1)
    check("reset invalidates the record (one write)", fw.nvm_writes == 3)
    for _ in range(10):
        call(fw, "/config", reset=1)
    check("repeated resets cost nothing more", fw.nvm_writes == 3, f"{fw.nvm_writes} writes")

    check("/status reports the write count", "nvm_writes=" in call(fw, "/status"))
    check("real flash is reported as available", "nvm=available" in call(fw, "/status"))

    # The saved value must still be correct after all that skipping.
    call(fw, "/config", r=7, g=8, b=9, w=10, save=1)
    check("the record is still accurate after skipped saves",
          (load().cfg["r"], load().cfg["g"]) == (7, 8))


def test_survives_missing_nvm():
    section("A build with no NVM must still boot and cue")
    # The board this ships on HAS nvm (/status says so). This covers the failure mode, not the
    # probability: nvm_load() runs at module scope, before the trigger exists, so an unguarded
    # None here would stop code.py before poll_trigger() and the ring would never fire.
    real = sys.modules["microcontroller"]
    try:
        sys.modules["microcontroller"] = None       # `import microcontroller` -> ImportError
        fw = load()
        check("the board boots", fw.server is not None)
        check("falls back to a RAM buffer", not fw._nvm_is_flash)
        check("settings are still changeable", (call(fw, "/config", r=7) or True)
              and fw.cfg["r"] == 7)
        body = call(fw, "/config", save=1)
        check("save reports the truth, not a fake success", "RAM ONLY" in body, body.splitlines()[0])
        check("/status says it is not persistent", "RAM ONLY" in call(fw, "/status"))
        trigger_on(fw)
        check("and the wire still fires the cue", fw.cue_active())
        advance(2.1)
        fw.cue_service()
        check("which still times out", not fw.cue_active())
    finally:
        sys.modules["microcontroller"] = real


def test_wire_survives_wifi_failure():
    section("Wi-Fi down — the trigger must still work")
    NVM.nvm[:] = bytearray(256)
    fw = load(ssid=None, password=None)
    check("no HTTP server", fw.server is None)
    trigger_on(fw)
    check("the cue still runs", fw.cue_active() and fw._cue_color == (0, 0, 0, 255))
    advance(2.1)
    fw.cue_service()
    check("and still times out", not fw.cue_active())


def test_trigger_details():
    section("Trigger polarity, debounce, retrigger, follow mode")
    fw = load()
    call(fw, "/config", debounce_ms=50)
    fw.cue_in.value = False
    fw.poll_trigger()
    advance(0.020)
    fw.poll_trigger()
    check("20 ms < 50 ms debounce: no fire", not fw.cue_active())
    advance(0.040)
    fw.poll_trigger()
    check("60 ms: fires", fw.cue_active())

    fw = load()
    fw.cue_in.value = True                      # idle for active-low
    fw.poll_trigger()
    call(fw, "/config", active_low=0)           # flipping polarity must not look like an edge
    fw.poll_trigger()
    advance(0.020)
    fw.poll_trigger()
    check("flipping polarity does not fire a spurious cue", not fw.cue_active())
    check("pull direction followed the setting", fw.cue_in.pull == "down", fw.cue_in.pull)
    # The pin was already HIGH when the polarity flipped, so under active-high it is already
    # "asserted" and there is no new edge -- that is the resync working. Go LOW, then HIGH.
    fw.cue_in.value = False
    fw.poll_trigger()
    advance(0.020)
    fw.poll_trigger()
    check("active-high: LOW is now idle", not fw.cue_active())
    fw.cue_in.value = True
    fw.poll_trigger()
    advance(0.020)
    fw.poll_trigger()
    check("active-high: fires on HIGH", fw.cue_active())

    fw = load()
    trigger_on(fw)
    deadline = fw._cue_until_ns
    trigger_off(fw)
    trigger_on(fw)
    check("retrigger=0: a second edge does not restart the cue", fw._cue_until_ns == deadline)

    fw = load()
    call(fw, "/config", mode="follow", duration=0.5)
    trigger_on(fw)
    advance(3.0)
    fw.cue_service()
    check("follow mode ignores duration while the line is held", fw.cue_active())
    trigger_off(fw)
    check("and stops on release", not fw.cue_active())

    fw = load()
    call(fw, "/config", enabled=0)
    trigger_on(fw)
    check("enabled=0 disarms the wire", not fw.cue_active())
    call(fw, "/cue")
    check("but /cue still works", fw.cue_active())


def test_patterns():
    section("Patterns render distinctly")
    fw = load()
    for pattern in fw.PATTERNS:
        call(fw, "/off")
        call(fw, "/config", r=0, g=0, b=0, w=200, pattern=pattern, segments=6,
             duration=5, period=1.0)
        call(fw, "/cue")            # fire directly: this test is about rendering, and driving
                                    # the wire here would need an edge per iteration
        # Every pattern must be lit at phase 0: a cue has to be visible the instant it fires.
        check(f"{pattern}: lit at phase 0 (crisp onset)",
              any(fw.pixels[i] != (0, 0, 0, 0) for i in range(60)))
        frames = []
        for _ in range(12):
            advance(0.08)
            fw.cue_service()
            frames.append(tuple(fw.pixels[i] for i in range(0, 60, 6)))
        distinct = len(set(frames))
        # Sample the lit count at a known ON phase; after an arbitrary number of frames the
        # two-state patterns may be in their off half.
        fw.cue_stop()
        call(fw, "/cue")
        lit = sum(1 for i in range(60) if fw.pixels[i] != (0, 0, 0, 0))
        if pattern == "solid":
            check("solid: never changes", distinct == 1, f"{distinct} distinct frames")
        elif pattern == "flash":
            check("flash: two states", distinct == 2, f"{distinct} distinct frames")
        elif pattern == "breathe":
            check("breathe: many levels", distinct > 3, f"{distinct} distinct frames")
        elif pattern == "chase":
            check("chase: a moving arc", 0 < lit <= fw.CHASE_LEN and distinct > 3,
                  f"{lit} lit, {distinct} frames")
        elif pattern == "segment":
            blocks = 0
            for i in range(60):
                on = fw.pixels[i] != (0, 0, 0, 0)
                prev = fw.pixels[i - 1] != (0, 0, 0, 0)
                if on and not prev:
                    blocks += 1
            check("segment: 6 evenly spaced blocks", blocks == 6 and 0 < lit < 60,
                  f"{blocks} blocks, {lit} of 60 lit")
        fw.cue_stop()


def test_cue_is_non_destructive():
    section("A cue restores whatever the ring was showing")
    fw = load()
    call(fw, "/set_color", r=10, g=20, b=30, w=40)
    call(fw, "/config", duration=0.5, pattern="flash", r=0, g=0, b=0, w=255)
    trigger_on(fw)
    check("the cue overrides the ring", fw.pixels[0] == (0, 0, 0, 255))
    advance(0.6)
    fw.cue_service()
    check("the previous colour comes back", fw.pixels[0] == (10, 20, 30, 40), fw.pixels[0])
    trigger_on(fw)
    call(fw, "/set_color", r=1, g=2, b=3, w=4)
    check("a manual command cancels the cue and wins",
          not fw.cue_active() and fw.pixels[0] == (1, 2, 3, 4))


def test_file_server_is_read_only():
    section("File access is read-only and refuses the credential")
    fw = load()
    fw._FS_ROOT = HERE + "/"                    # on the board, "/" IS the CIRCUITPY drive
    listing = call(fw, "/fs")
    check("lists code.py", "code.py" in listing)
    check("serves code.py", "POWER" in call(fw, "/fs/get", path="code.py"))
    check("refuses settings.toml", "credential" in call(fw, "/fs/get", path="settings.toml"))
    check("refuses traversal", "not allowed" in call(fw, "/fs/get", path="../../etc/passwd"))
    check("refuses absolute paths", "not allowed" in call(fw, "/fs/get", path="/etc/passwd"))
    check("refuses binaries", "text files" in call(fw, "/fs/get", path="lib/neopixel.mpy"))
    check("no write route exists",
          not any(r.startswith(("/fs/put", "/fs/upload", "/fs/write")) for r in fw.server.routes))


def test_surface_is_the_documented_one():
    section("The API surface is the documented one")
    fw = load()
    check("routes", sorted(fw.server.routes) ==
          ["/config", "/cue", "/fs", "/fs/get", "/off", "/segments", "/set_color", "/status"],
          sorted(fw.server.routes))
    body = call(fw, "/status")
    for field in ("trigger_pin_raw", "trigger_asserted", "trigger_debounced", "wire_fired",
                  "cue_active", "uptime_s", "nvm"):
        check(f"/status reports {field}", field + "=" in body)
    # things removed in earlier redesigns; their return would be a regression
    for gone in ("armed", "arm_seq", "profiles", "N_PROFILES", "_decode_select",
                 "MAX_BRIGHTNESS", "wifi_mode"):
        check(f"stays removed: {gone}", not hasattr(fw, gone))


def main():
    print("Cue ring firmware — offline tests")
    print(f"  {FIRMWARE}")
    for test in (
        test_access_point_only,
        test_trigger_is_on_off_with_a_timer,
        test_colour_is_used_exactly_as_set,
        test_brightness_is_uncapped_but_clamped,
        test_settings_compose,
        test_persistence,
        test_flash_wear,
        test_survives_missing_nvm,
        test_wire_survives_wifi_failure,
        test_trigger_details,
        test_patterns,
        test_cue_is_non_destructive,
        test_file_server_is_read_only,
        test_surface_is_the_documented_one,
    ):
        NVM.nvm[:] = bytearray(256)             # each test starts from a blank board
        test()

    passed = sum(RESULTS)
    print("\n" + "=" * 62)
    print(f"{passed}/{len(RESULTS)} passed")
    return 0 if passed == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
