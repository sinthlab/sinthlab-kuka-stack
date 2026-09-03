# End-effector cue ring — Metro M4 AirLift (CircuitPython)
#
# Drives the 60-LED RGBW NeoPixel ring on the apple-pluck end effector.
#
# TWO INDEPENDENT CONTROL PATHS:
#
#   1. HARDWIRED CUE (primary, used by the experiment)
#      Sunrise app asserts a 24 V media-flange digital output -> optocoupler (24 V -> 3.3 V,
#      galvanically isolated) -> CUE_PIN on this board -> the ring runs a cue.
#      No network involved. Brought up FIRST and designed to keep working even if the WiFi
#      co-processor never starts.
#
#   2. WIFI + HTTP (secondary: configuration, manual control, read-back)
#      The board hosts its OWN access point; a laptop joins it and drives the ring by URL.
#
# The wire carries TIMING only (one bit). WHAT the cue looks like is configuration.
#
# DESIGN RULE -- CONFIGURATION, NOT CODE EDITS
#   The board is bolted inside a closed end effector, so reaching the USB port is expensive.
#   Everything an experiment might reasonably want to change is therefore a RUNTIME SETTING
#   reachable over HTTP (/config) and persistable to NVM (/config?save=1) -- colour, duration,
#   rate, pattern, brightness, trigger polarity, debounce, mode. Editing this file should be
#   reserved for genuinely new behaviour, not for tuning.
#
#   Only two classes of thing stay hard-coded:
#     * physical facts   -- pin assignments, LED count (changing them means re-wiring anyway)
#     * safety limits    -- MAX_BRIGHTNESS, which caps the ring's current draw
#
# See README.md.

import os
import math
import time
import board
import busio
import struct
import neopixel
from digitalio import DigitalInOut, Direction, Pull
from adafruit_esp32spi import adafruit_esp32spi
import adafruit_esp32spi.adafruit_esp32spi_socketpool as socketpool
from adafruit_httpserver import Server, Request, Response

# ---------------------------------------------------------------------------
# Physical facts -- these describe the wiring, not a preference
# ---------------------------------------------------------------------------
ORDER = neopixel.GRBW
PHYSICAL_LEDS = 60
LED_PIN = board.D5              # -> Pixel Shifter (3.3 V -> 5 V) -> ring DIN
CUE_PIN = board.D2              # <- optocoupler output (isolated from the 24 V side)

# ---------------------------------------------------------------------------
# Safety limit -- deliberately NOT settable over HTTP
# ---------------------------------------------------------------------------
# The ring pulls ~3.5 A at 5 V full white. Brightness scales that roughly linearly, so this
# ceiling is what stops a stray request asking the converter and the 5 V wiring for more than
# they are sized to deliver. Raise it only after re-checking both.
MAX_BRIGHTNESS = 0.35           # ~1.2 A worst case

FRAME_MS = 33                   # ~30 fps for the continuous patterns (breathe / chase)
CHASE_LEN = max(1, PHYSICAL_LEDS // 8)
PATTERNS = ("flash", "solid", "breathe", "chase")
MODES = ("pulse", "follow")
OFF = (0, 0, 0, 0)

# ---------------------------------------------------------------------------
# Runtime configuration
# ---------------------------------------------------------------------------
# DEFAULTS is what the board falls back to (fresh board, or /config?reset=1). The live copy
# is `cfg`, changed over HTTP and optionally persisted to NVM.
DEFAULTS = {
    # what the cue looks like
    "r": 0, "g": 0, "b": 0, "w": 255,   # white via the dedicated W channel
    "duration": 2.0,                    # seconds (ignored in "follow" mode)
    "period": 0.5,                      # seconds per full cycle -> 2 Hz flash
    "pattern": "flash",                 # flash | solid | breathe | chase
    "brightness": 0.2,
    # how the hardwired trigger behaves
    "mode": "pulse",                    # pulse: board owns the length
                                        # follow: ring runs while the line is held
    "active_low": True,                 # opto sinks the pin when the 24 V line is asserted;
                                        # the pull-up then makes "idle" = HIGH, so an
                                        # unplugged connector reads as "no cue"
    "debounce_ms": 5,
    "retrigger": False,                 # True = a new edge restarts an in-flight cue
    "enabled": True,                    # False = ignore the wire entirely
}
cfg = dict(DEFAULTS)

# ---------------------------------------------------------------------------
# Pixels
# ---------------------------------------------------------------------------
# auto_write=False -> writes are buffered until an explicit pixels.show().
# pixel_order is the WIRE order; colours are always passed as (R, G, B, W).
pixels = neopixel.NeoPixel(
    LED_PIN, PHYSICAL_LEDS, brightness=cfg["brightness"],
    auto_write=False, pixel_order=ORDER,
)
pixels.fill(OFF)
pixels.show()

# Boot indicator: two short dim flashes, before anything that can fail or hang, so you can
# confirm from outside a closed effector that the board came up and the ring is wired.
for _ in range(2):
    pixels.fill((0, 0, 0, 40))
    pixels.show()
    time.sleep(0.12)
    pixels.fill(OFF)
    pixels.show()
    time.sleep(0.12)

# ---------------------------------------------------------------------------
# Persistence -- microcontroller.nvm
# ---------------------------------------------------------------------------
# NVM is a small battery-free non-volatile byte area. It is used INSTEAD of a file because
# writing to the filesystem would require storage.remount() in boot.py, which makes CIRCUITPY
# read-only to the host computer -- see README. NVM has neither that cost nor that risk.
#
# Not every build exposes it, so every access is guarded: if NVM is unavailable the board
# still works, settings just revert to DEFAULTS on reset.
_NVM_MAGIC = 0xC5
_NVM_VER = 1
_NVM_FMT = "<BBBBBBHHBBBB"      # magic, ver, r, g, b, w, dur_cs, per_cs, bright, flags,
_NVM_LEN = struct.calcsize(_NVM_FMT) + 1        # debounce, pattern  (+1 checksum byte)

try:
    import microcontroller
    _nvm = microcontroller.nvm
    if _nvm is not None and len(_nvm) < _NVM_LEN:
        _nvm = None
except Exception:
    _nvm = None


def _nvm_pack():
    flags = ((1 if cfg["enabled"] else 0)
             | (2 if cfg["active_low"] else 0)
             | (4 if cfg["retrigger"] else 0)
             | (8 if cfg["mode"] == "follow" else 0))
    blob = struct.pack(
        _NVM_FMT, _NVM_MAGIC, _NVM_VER,
        cfg["r"], cfg["g"], cfg["b"], cfg["w"],
        min(int(cfg["duration"] * 100), 65535),
        min(int(cfg["period"] * 100), 65535),
        int(cfg["brightness"] * 255),
        flags, cfg["debounce_ms"], PATTERNS.index(cfg["pattern"]),
    )
    return blob + bytes([sum(blob) & 0xFF])


def nvm_save():
    if _nvm is None:
        return False
    _nvm[0:_NVM_LEN] = _nvm_pack()
    return True


def nvm_load():
    """Overlay saved settings onto cfg. Returns True only if a valid record was found."""
    if _nvm is None:
        return False
    blob = bytes(_nvm[0:_NVM_LEN])
    if blob[0] != _NVM_MAGIC or blob[1] != _NVM_VER:
        return False
    if (sum(blob[:-1]) & 0xFF) != blob[-1]:
        return False
    (_, _, r, g, b, w, dur, per, bright,
     flags, deb, pat) = struct.unpack(_NVM_FMT, blob[:-1])
    if pat >= len(PATTERNS):
        return False
    cfg.update({
        "r": r, "g": g, "b": b, "w": w,
        "duration": dur / 100.0, "period": max(per / 100.0, 0.02),
        "brightness": min(bright / 255.0, MAX_BRIGHTNESS),
        "pattern": PATTERNS[pat], "debounce_ms": max(deb, 1),
        "enabled": bool(flags & 1), "active_low": bool(flags & 2),
        "retrigger": bool(flags & 4), "mode": "follow" if flags & 8 else "pulse",
    })
    return True


def nvm_clear():
    if _nvm is None:
        return False
    _nvm[0:1] = b"\x00"         # wiping the magic byte is enough to invalidate the record
    return True


_nvm_loaded = nvm_load()
pixels.brightness = cfg["brightness"]

# ---------------------------------------------------------------------------
# Cue engine -- non-blocking
# ---------------------------------------------------------------------------
# Driven from the main loop by cue_service(); no handler ever sleeps. That is what lets a cue
# animate while the HTTP server stays responsive, and why the hardwired trigger's latency is
# the debounce time regardless of network traffic.
_cue_until_ns = 0               # 0 = idle; -1 = run until stopped ("follow" mode)
_cue_start_ns = 0
_cue_period_ns = 1
_cue_color = OFF
_cue_pattern = "flash"
_cue_frame_ns = 0
_cue_step = -1
_cue_restore = None             # ring contents to put back when the cue ends


def cue_active():
    return _cue_until_ns != 0


def _snapshot():
    return [pixels[i] for i in range(PHYSICAL_LEDS)]


def _restore(snap):
    for i in range(PHYSICAL_LEDS):
        pixels[i] = snap[i]
    pixels.show()


def _scaled(c, k):
    return (int(c[0] * k), int(c[1] * k), int(c[2] * k), int(c[3] * k))


def _render(phase):
    """Draw one frame. phase is 0.0-1.0 through the current cycle."""
    if _cue_pattern == "solid":
        pixels.fill(_cue_color)
    elif _cue_pattern == "breathe":
        pixels.fill(_scaled(_cue_color, 0.5 - 0.5 * math.cos(2 * math.pi * phase)))
    elif _cue_pattern == "chase":
        pixels.fill(OFF)
        head = int(phase * PHYSICAL_LEDS)
        for i in range(CHASE_LEN):
            pixels[(head + i) % PHYSICAL_LEDS] = _cue_color
    else:                                       # "flash"
        pixels.fill(_cue_color if phase < 0.5 else OFF)
    pixels.show()


def cue_start(color, duration, period, pattern):
    """Begin a cue. duration <= 0 means 'until cue_stop()' (follow mode)."""
    global _cue_until_ns, _cue_start_ns, _cue_period_ns, _cue_color
    global _cue_pattern, _cue_frame_ns, _cue_step, _cue_restore
    now = time.monotonic_ns()
    if not cue_active():
        _cue_restore = _snapshot()          # snapshot the pre-cue ring, never a cue frame
    _cue_color = color
    _cue_pattern = pattern if pattern in PATTERNS else "flash"
    _cue_period_ns = max(int(period * 1_000_000_000), 20_000_000)    # floor 20 ms
    _cue_until_ns = -1 if duration <= 0 else now + int(duration * 1_000_000_000)
    _cue_start_ns = now
    _cue_frame_ns = now
    _cue_step = 0
    _render(0.0)


def cue_stop(restore=True):
    global _cue_until_ns, _cue_restore, _cue_step
    if not cue_active():
        return
    _cue_until_ns = 0
    _cue_step = -1
    if restore and _cue_restore is not None:
        _restore(_cue_restore)
    _cue_restore = None


def cue_from_config():
    cue_start((cfg["r"], cfg["g"], cfg["b"], cfg["w"]),
              0 if cfg["mode"] == "follow" else cfg["duration"],
              cfg["period"], cfg["pattern"])


def cue_service():
    """Advance the animation. Call every pass of the main loop."""
    global _cue_frame_ns, _cue_step
    if not cue_active():
        return
    now = time.monotonic_ns()
    if _cue_until_ns > 0 and now >= _cue_until_ns:
        cue_stop()
        return
    if _cue_pattern == "solid":
        return                                  # nothing to animate
    phase = ((now - _cue_start_ns) % _cue_period_ns) / _cue_period_ns
    if _cue_pattern == "flash":
        step = 0 if phase < 0.5 else 1          # only two states -- redraw on the boundary
        if step == _cue_step:
            return
        _cue_step = step
    else:                                       # breathe / chase -- fixed frame rate
        if now - _cue_frame_ns < FRAME_MS * 1_000_000:
            return
        _cue_frame_ns = now
    _render(phase)


# ---------------------------------------------------------------------------
# Hardwired trigger -- debounced edge detection
# ---------------------------------------------------------------------------
cue_in = DigitalInOut(CUE_PIN)
cue_in.direction = Direction.INPUT

_trig_state = False             # debounced logical level ("asserted")
_trig_raw = False
_trig_since_ns = 0


def trigger_asserted():
    return (not cue_in.value) if cfg["active_low"] else cue_in.value


def apply_polarity():
    """(Re)configure the input pull and resync the debouncer.

    Called at boot and whenever active_low changes over HTTP -- resyncing matters, otherwise
    flipping the polarity would look like an edge and fire a spurious cue.
    """
    global _trig_state, _trig_raw, _trig_since_ns
    cue_in.pull = Pull.UP if cfg["active_low"] else Pull.DOWN
    _trig_state = _trig_raw = trigger_asserted()
    _trig_since_ns = time.monotonic_ns()


def poll_trigger():
    global _trig_state, _trig_raw, _trig_since_ns
    raw = trigger_asserted()
    now = time.monotonic_ns()

    if raw != _trig_raw:                        # level changed -- restart the debounce window
        _trig_raw = raw
        _trig_since_ns = now
        return
    if raw == _trig_state:
        return
    if now - _trig_since_ns < cfg["debounce_ms"] * 1_000_000:
        return

    _trig_state = raw                           # debounced edge
    if not cfg["enabled"]:
        return
    if raw:
        if cfg["mode"] == "follow" or cfg["retrigger"] or not cue_active():
            cue_from_config()
    elif cfg["mode"] == "follow":
        cue_stop()


apply_polarity()
print("Cue input on {} (active {}), mode={}, nvm={}".format(
    CUE_PIN, "low" if cfg["active_low"] else "high", cfg["mode"],
    "restored" if _nvm_loaded else ("available" if _nvm else "UNAVAILABLE")))

# ---------------------------------------------------------------------------
# HTTP route handlers
# ---------------------------------------------------------------------------
# Defined unconditionally, registered further down only if the WiFi stack came up. Nothing
# here sleeps -- every handler returns immediately and lets the main loop do the animation.

def _int(request, name, default, lo=0, hi=255):
    """Query param as int, clamped. Bad input falls back to the default, never raises."""
    try:
        return min(hi, max(lo, int(request.query_params.get(name, default))))
    except (TypeError, ValueError):
        return default


def _float(request, name, default, lo, hi):
    try:
        return min(hi, max(lo, float(request.query_params.get(name, default))))
    except (TypeError, ValueError):
        return default


def _bool(request, name, default):
    v = request.query_params.get(name)
    if v is None:
        return default
    return v.strip().lower() in ("1", "true", "yes", "on")


def _choice(request, name, default, allowed):
    v = request.query_params.get(name)
    return v if v in allowed else default


def _given(request, *names):
    """True if any of these query params was actually supplied."""
    return any(request.query_params.get(n) is not None for n in names)


def parse_color(request, default=(0, 0, 0, 0)):
    """Colour components in R, G, B, W order (GRBW is only the wire order)."""
    return (
        _int(request, "r", default[0]),
        _int(request, "g", default[1]),
        _int(request, "b", default[2]),
        _int(request, "w", default[3]),
    )


def _config_text(note=""):
    return (
        "{}r={}\ng={}\nb={}\nw={}\n"
        "duration={}\nperiod={}\npattern={}\nbrightness={}\n"
        "mode={}\nactive_low={}\ndebounce_ms={}\nretrigger={}\nenabled={}\n"
        "nvm={}\n"
    ).format(
        note + "\n" if note else "",
        cfg["r"], cfg["g"], cfg["b"], cfg["w"],
        cfg["duration"], cfg["period"], cfg["pattern"], cfg["brightness"],
        cfg["mode"], cfg["active_low"], cfg["debounce_ms"], cfg["retrigger"],
        cfg["enabled"], "available" if _nvm else "unavailable",
    )


# --- /config : read or change every runtime setting -------------------------
# Read:    http://192.168.4.1/config
# Write:   http://192.168.4.1/config?r=0&g=255&b=0&w=0&duration=1.5&pattern=breathe
# Persist: http://192.168.4.1/config?save=1          (survives reset, NVM permitting)
# Restore: http://192.168.4.1/config?reset=1         (back to DEFAULTS, clears NVM)
def configure(request: Request):
    note = ""
    if _bool(request, "reset", False):
        cfg.clear()
        cfg.update(DEFAULTS)
        nvm_clear()
        pixels.brightness = cfg["brightness"]
        apply_polarity()
        return Response(request, _config_text("reset to defaults, NVM cleared"))

    if _given(request, "r", "g", "b", "w", "duration", "period", "pattern", "brightness",
              "mode", "active_low", "debounce_ms", "retrigger", "enabled"):
        was_active_low = cfg["active_low"]
        cfg["r"], cfg["g"], cfg["b"], cfg["w"] = parse_color(
            request, (cfg["r"], cfg["g"], cfg["b"], cfg["w"]))
        cfg["duration"] = _float(request, "duration", cfg["duration"], 0.05, 300.0)
        cfg["period"] = _float(request, "period", cfg["period"], 0.02, 10.0)
        cfg["pattern"] = _choice(request, "pattern", cfg["pattern"], PATTERNS)
        cfg["mode"] = _choice(request, "mode", cfg["mode"], MODES)
        cfg["brightness"] = _float(request, "brightness", cfg["brightness"], 0.0, MAX_BRIGHTNESS)
        cfg["debounce_ms"] = _int(request, "debounce_ms", cfg["debounce_ms"], 1, 200)
        cfg["active_low"] = _bool(request, "active_low", cfg["active_low"])
        cfg["retrigger"] = _bool(request, "retrigger", cfg["retrigger"])
        cfg["enabled"] = _bool(request, "enabled", cfg["enabled"])

        pixels.brightness = cfg["brightness"]
        pixels.show()
        if cfg["active_low"] != was_active_low:
            apply_polarity()

    if _bool(request, "save", False):
        note = "saved to NVM" if nvm_save() else "NOT SAVED -- no NVM on this build"
    return Response(request, _config_text(note))


# --- /cue : fire a cue now, the software equivalent of the trigger wire ------
# Usage: http://192.168.4.1/cue
#        http://192.168.4.1/cue?r=255&duration=3&period=0.25&pattern=chase
def fire_cue(request: Request):
    color = parse_color(request, (cfg["r"], cfg["g"], cfg["b"], cfg["w"]))
    duration = _float(request, "duration", cfg["duration"], 0.0, 300.0)
    period = _float(request, "period", cfg["period"], 0.02, 10.0)
    pattern = _choice(request, "pattern", cfg["pattern"], PATTERNS)
    cue_start(color, duration, period, pattern)
    return Response(request, "Cue {} {} for {}s at {}s/cycle".format(
        pattern, color, duration, period))


# --- /set_color : solid colour on the whole ring ----------------------------
# Usage: http://192.168.4.1/set_color?r=255&g=0&b=0&w=50
def set_color(request: Request):
    cue_stop(restore=False)             # a manual command wins over an in-flight cue
    color = parse_color(request)
    pixels.fill(color)
    pixels.show()
    return Response(request, "Set to {}".format(color))


# --- /segments : split the ring into arcs -----------------------------------
# Usage: http://192.168.4.1/segments?factor=4&colors=255,0,0,0.0,255,0,0.0,0,255,0.0,0,0,255
#        segments separated by '.', components within a segment by ','
def set_multi_segments(request: Request):
    cue_stop(restore=False)
    factor = _int(request, "factor", 1, 1, PHYSICAL_LEDS)
    colors_str = request.query_params.get("colors", "")

    pixels.fill(OFF)

    if colors_str:
        seg_size = PHYSICAL_LEDS // factor      # truncates: a factor that does not divide 60
                                                # leaves the remaining pixels dark
        for i, config in enumerate(colors_str.split(".")):
            if i >= factor:
                break
            try:
                c = [int(p) for p in config.split(",")]
                if len(c) == 3:                 # auto-pad RGB -> RGBW
                    c.append(0)
                for p in range(i * seg_size, (i + 1) * seg_size):
                    pixels[p] = (c[0], c[1], c[2], c[3])
            except Exception as e:
                print("Segment error:", e)
                continue

    pixels.show()
    return Response(request, "Segments updated")


# --- /blink : flash without touching the configuration (non-blocking) -------
# Usage: http://192.168.4.1/blink?r=255&delay=0.5&count=5
def blink(request: Request):
    color = parse_color(request)
    delay = _float(request, "delay", 0.5, 0.02, 5.0)        # half-cycle, i.e. on-time
    count = _int(request, "count", 3, 1, 100)
    cue_start(color, count * 2 * delay, 2 * delay, "flash")
    return Response(request, "Blinking {}x -- returns immediately, runs in background".format(count))


# --- /off : clear the ring and cancel any running cue -----------------------
def set_off(request: Request):
    cue_stop(restore=False)
    pixels.fill(OFF)
    pixels.show()
    return Response(request, "NeoPixels are now OFF")


# --- /status : what is the board doing? First stop when a cue "does not work"
def status(request: Request):
    return Response(request, _config_text() + (
        "leds={}\nmax_brightness={}\npatterns={}\n"
        "cue_active={}\ncue_pattern={}\n"
        "trigger_pin_raw={}\ntrigger_asserted={}\ntrigger_debounced={}\n"
        "nvm_restored_at_boot={}\nuptime_s={:.1f}\n"
    ).format(
        PHYSICAL_LEDS, MAX_BRIGHTNESS, ",".join(PATTERNS),
        cue_active(), _cue_pattern,
        cue_in.value, trigger_asserted(), _trig_state,
        _nvm_loaded, time.monotonic_ns() / 1e9,
    ))


# ---------------------------------------------------------------------------
# Read-only file access
# ---------------------------------------------------------------------------
# Deliberately READ-ONLY. Serving files needs nothing special; WRITING them would require
# storage.remount() in boot.py, which makes CIRCUITPY read-only to the host computer and puts
# a boot-time failure between you and a working board. See README.
#
# settings.toml is refused: it holds the AP password.
_FS_ROOT = "/"                  # the CIRCUITPY drive root. Paths in the API are relative to
                                # it, so every filesystem call below is explicitly anchored
                                # rather than depending on the interpreter's cwd.
_FS_DENY = ("settings.toml",)
_FS_ALLOW_EXT = (".py", ".txt", ".json", ".toml")


def _fs_reject(path):
    """Return a refusal reason, or None if the path may be read."""
    if not path:
        return "no path given"
    if ".." in path or path.startswith("/"):
        return "path not allowed"
    if path.rsplit("/", 1)[-1] in _FS_DENY:
        return "refused: that file holds the WiFi credential"
    if not path.endswith(_FS_ALLOW_EXT):
        return "only text files ({}) can be read back".format(", ".join(_FS_ALLOW_EXT))
    return None


def _walk(rel, out, depth=0):
    try:
        entries = sorted(os.listdir(_FS_ROOT + rel))
    except OSError:
        return
    for e in entries:
        if e.startswith("."):
            continue
        full = (rel + "/" + e) if rel else e
        try:
            st = os.stat(_FS_ROOT + full)
        except OSError:
            continue
        if st[0] & 0x4000:                      # directory
            out.append("{}/".format(full))
            if depth < 2:
                _walk(full, out, depth + 1)
        else:
            out.append("{:<40s} {:>7d}".format(full, st[6]))


# --- /fs : list what is actually on the board -------------------------------
def fs_list(request: Request):
    out = []
    _walk("", out)
    return Response(request, "\n".join(out) + "\n")


# --- /fs/get?path=code.py : read a file back --------------------------------
def fs_get(request: Request):
    path = request.query_params.get("path", "")
    why = _fs_reject(path)
    if why:
        return Response(request, why + "\n")
    try:
        with open(_FS_ROOT + path, "r") as f:
            return Response(request, f.read())
    except OSError as e:
        return Response(request, "cannot read {}: {}\n".format(path, e))


ROUTES = (
    ("/config", configure),
    ("/cue", fire_cue),
    ("/status", status),
    ("/set_color", set_color),
    ("/segments", set_multi_segments),
    ("/blink", blink),
    ("/off", set_off),
    ("/fs", fs_list),
    ("/fs/get", fs_get),
)

# ---------------------------------------------------------------------------
# WiFi access point + HTTP server  (SECONDARY path -- failure here is not fatal)
# ---------------------------------------------------------------------------
# Wrapped so that a dead ESP32, a missing or unparseable settings.toml, or a WPA2-rejected
# password cannot stop the board servicing the hardwired cue. If this block fails the loop
# below still runs, just without the network path.
server = None
try:
    esp32_cs = DigitalInOut(board.ESP_CS)
    esp32_ready = DigitalInOut(board.ESP_BUSY)
    esp32_reset = DigitalInOut(board.ESP_RESET)
    spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
    esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

    if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
        print("ESP32 found and in idle mode")
    print("Firmware vers.", esp.firmware_version)
    print("MAC addr:", ":".join("%02X" % byte for byte in esp.MAC_address))

    ssid = os.getenv("CIRCUITPY_WIFI_SSID")
    password = os.getenv("CIRCUITPY_WIFI_PASSWORD")
    if not ssid or not password:
        raise RuntimeError(
            "CIRCUITPY_WIFI_SSID / CIRCUITPY_WIFI_PASSWORD missing from settings.toml "
            "(values must be QUOTED strings; the password must be 8-63 chars for WPA2)"
        )
    esp.create_AP(ssid, password, 1)

    pool = socketpool.SocketPool(esp)
    server = Server(pool, debug=True)
    for path, handler in ROUTES:
        server.route(path, methods=["GET"])(handler)

    server.start(str(esp.pretty_ip(esp.ip_address)), port=80)
    print("Server started at http://{}".format(esp.pretty_ip(esp.ip_address)))
except Exception as e:
    server = None
    print("WiFi/HTTP unavailable: {}".format(e))
    print("Continuing with the HARDWIRED cue path only.")

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
# poll_trigger() runs FIRST and cue_service() second, so the wired cue is serviced even while
# HTTP traffic is arriving. server.poll() is non-blocking and no route handler sleeps, so a
# pass through this loop is short and the trigger latency stays at roughly the debounce time.
while True:
    try:
        poll_trigger()
        cue_service()
        if server is not None:
            server.poll()
    except Exception as e:
        print("Error: {}".format(e))
        time.sleep(0.05)        # a persistent fault must not spin the console at full speed
