# End-effector cue ring — Metro M4 AirLift (CircuitPython)
#
# Drives the 60-LED RGBW NeoPixel ring on the apple-pluck end effector.
#
# ---------------------------------------------------------------------------------------------
# HOW IT WORKS
# ---------------------------------------------------------------------------------------------
# TRIGGER (the wire).  Sunrise asserts a 24 V media-flange output -> optocoupler -> CUE_PIN.
#                      The board runs its configured cue for its configured duration, then stops.
#                      On and off with a timer. That is the whole behaviour.
#                      No network is involved, so the trigger keeps working with Wi-Fi down.
#
# SETTINGS (the API).  The board hosts its OWN Wi-Fi access point. Join it from a laptop or phone
#                      and set what the cue looks like -- colour, brightness, pattern, segments,
#                      duration, rate -- with /config. `save=1` keeps it across reboots.
#
# The two are deliberately separate: the wire cannot carry a colour (it is one bit), and the
# experiment must never depend on a radio link. So the cabinet says WHEN, and the board -- already
# configured by hand -- decides WHAT.
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
# !!! POWER --- READ BEFORE RAISING `brightness` !!!
# ---------------------------------------------------------------------------
# `brightness` (0.0-1.0, set over /config) scales the ring's current draw very nearly linearly.
# It is NOT capped in firmware: you are responsible for keeping it inside what the hardware can
# actually deliver. At brightness = 1.0, 60 SK6812 RGBW LEDs draw roughly:
#
#     w=255 only            60 x ~20 mA  ~= 1.2 A     <- the white channel is the cheap one
#     r=g=b=255, w=0        60 x ~60 mA  ~= 3.6 A
#     r=g=b=w=255           60 x ~80 mA  ~= 4.8 A     <- absolute worst case
#
# Multiply by `brightness` for the actual draw. Before raising it, check ALL of:
#
#   * the Tobsun 24 V->5 V converter's current rating (and its derating when warm);
#   * the 5 V wiring gauge down the flange bore -- several amps through thin wire means
#     voltage drop, heat, and colour shift at the far end of the ring;
#   * that 5 V is injected at ALL FOUR quarter-ring joints. Feeding 60 LEDs through one arc's
#     traces browns out the far end and overheats the near end;
#   * the temperature inside the closed casing box. The effector is handled by an animal, so
#     surface temperature is a subject-safety limit, not just an electronics one;
#   * how bright a cue actually needs to be. A cue is a signal, not illumination -- the ring is
#     centimetres from the subject's face, so the lowest brightness that reads reliably is the
#     right one.
#
# NEVER feed the ring from the Metro's own 5 V pin: it comes off the board regulator and cannot
# source anything like these currents.
#
# The default of 0.2 is a deliberately conservative starting point (~0.24 A on the default
# white-channel cue), not a measured limit for your build.

FRAME_MS = 33                   # ~30 fps for the continuous patterns
CHASE_LEN = max(1, PHYSICAL_LEDS // 8)
PATTERNS = ("flash", "solid", "breathe", "chase", "segment")
MODES = ("pulse", "follow")
OFF = (0, 0, 0, 0)

# ---------------------------------------------------------------------------
# Settings -- the cue's appearance and the board's behaviour, in one place
# ---------------------------------------------------------------------------
# Every field is independent and settable over /config; send any subset and the rest is
# unchanged. This is the whole configurable surface of the board.
DEFAULTS = {
    # --- what the cue looks like ---
    "r": 0, "g": 0, "b": 0, "w": 255,   # colour; w is the dedicated white channel
    "pattern": "flash",                 # flash | solid | breathe | chase | segment
    "segments": 4,                      # lit blocks, for the "segment" pattern
    "duration": 2.0,                    # seconds the cue runs after a trigger
    "period": 0.5,                      # seconds per full cycle -> 2 Hz

    # --- how the board behaves ---
    "brightness": 0.2,                  # 0..1 -- scales the whole ring. READ THE POWER NOTE
                                        # ABOVE before raising this.
    "mode": "pulse",                    # pulse : the trigger starts a cue of `duration`
                                        # follow: the cue runs while the line is held
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
# NVM is used INSTEAD of a config file because writing to the filesystem would require
# storage.remount() in boot.py, which makes CIRCUITPY read-only to the host computer -- see
# README. NVM has neither that cost nor that risk. Not every build exposes it, so every access
# is guarded: without NVM the board still works, settings just revert to defaults on reset.
_NVM_MAGIC = 0xC9
_NVM_VER = 5
_NVM_FMT = "<BB" "BBBB" "BBBB" "HHB"
# magic, ver | r, g, b, w | brightness, flags, debounce, segments | dur_cs, per_cs, pattern
# The round trip quantises: brightness is one byte (~0.4% steps) and duration/period are
# centiseconds (10 ms steps). Far finer than anything that matters here, but a value read back
# after a save will not be bit-identical to what you sent.
_NVM_LEN = struct.calcsize(_NVM_FMT) + 1        # +1 checksum

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
        int(cfg["brightness"] * 255), flags, cfg["debounce_ms"], cfg["segments"],
        min(int(cfg["duration"] * 100), 65535), min(int(cfg["period"] * 100), 65535),
        PATTERNS.index(cfg["pattern"]),
    )
    return blob + bytes([sum(blob) & 0xFF])


def nvm_save():
    if _nvm is None:
        return False
    _nvm[0:_NVM_LEN] = _nvm_pack()
    return True


def nvm_load():
    """Overlay the saved settings onto cfg. True only if a valid record was found."""
    if _nvm is None:
        return False
    blob = bytes(_nvm[0:_NVM_LEN])
    if blob[0] != _NVM_MAGIC or blob[1] != _NVM_VER:
        return False
    if (sum(blob[:-1]) & 0xFF) != blob[-1]:
        return False
    (_, _, r, g, b, w, bright, flags, deb, seg,
     dur, per, pat) = struct.unpack(_NVM_FMT, blob[:-1])
    if pat >= len(PATTERNS):
        return False                    # reject the WHOLE record rather than load half of it
    cfg.update({
        "r": r, "g": g, "b": b, "w": w,
        "brightness": min(bright / 255.0, 1.0),
        "pattern": PATTERNS[pat], "segments": max(1, min(seg, PHYSICAL_LEDS // 2)),
        "duration": max(dur / 100.0, 0.05), "period": max(per / 100.0, 0.02),
        "debounce_ms": max(deb, 1),
        "enabled": bool(flags & 1), "active_low": bool(flags & 2),
        "retrigger": bool(flags & 4), "mode": "follow" if flags & 8 else "pulse",
    })
    return True


def nvm_clear():
    if _nvm is None:
        return False
    _nvm[0:1] = b"\x00"                 # wiping the magic byte invalidates the record
    return True


_nvm_loaded = nvm_load()
pixels.brightness = cfg["brightness"]

# ---------------------------------------------------------------------------
# Cue engine -- non-blocking
# ---------------------------------------------------------------------------
# Driven from the main loop by cue_service(); no handler ever sleeps. That is what lets a cue
# animate while the HTTP server stays responsive, and why the wire trigger's latency is the
# debounce time regardless of network traffic.
_cue_until_ns = 0                       # 0 = idle; -1 = run until stopped ("follow" mode)
_cue_start_ns = 0
_cue_period_ns = 1
_cue_color = OFF
_cue_pattern = "flash"
_cue_segments = 4
_cue_frame_ns = 0
_cue_step = -1
_cue_restore = None                     # ring contents to put back when the cue ends


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
        # Starts at FULL and dips, rather than starting dark and fading up. A cue has to be
        # visible the instant it fires; a rise-first breathe would delay perceived onset by half
        # a period (0.6 s at the default rate), which is exactly the thing the wire is precise
        # about. Every pattern is lit at phase 0 for the same reason.
        pixels.fill(_scaled(_cue_color, 0.5 + 0.5 * math.cos(2 * math.pi * phase)))
    elif _cue_pattern == "chase":
        pixels.fill(OFF)
        head = int(phase * PHYSICAL_LEDS)
        for i in range(CHASE_LEN):
            pixels[(head + i) % PHYSICAL_LEDS] = _cue_color
    elif _cue_pattern == "segment":
        # N lit blocks evenly spaced around the ring, flashing together. Visually distinct from
        # a full-ring flash at a glance, which is the point of having it at all.
        pixels.fill(OFF)
        if phase < 0.5:
            n = max(1, _cue_segments)
            block = max(1, PHYSICAL_LEDS // (2 * n))
            for k in range(n):
                start = (k * PHYSICAL_LEDS) // n
                for j in range(block):
                    pixels[(start + j) % PHYSICAL_LEDS] = _cue_color
    else:                                       # "flash"
        pixels.fill(_cue_color if phase < 0.5 else OFF)
    pixels.show()


def cue_start(spec, follow=False):
    """Run a cue spec. follow=True means 'until cue_stop()', ignoring the spec's duration."""
    global _cue_until_ns, _cue_start_ns, _cue_period_ns, _cue_color
    global _cue_pattern, _cue_segments, _cue_frame_ns, _cue_step, _cue_restore
    now = time.monotonic_ns()
    if not cue_active():
        _cue_restore = _snapshot()      # snapshot the pre-cue ring, never a cue frame
    # The colour is used exactly as set. Scaling happens once, in pixels.brightness, so the
    # values you configure are the values that come back from /status -- no rounding surprises.
    _cue_color = (spec["r"], spec["g"], spec["b"], spec["w"])
    _cue_segments = max(1, min(int(spec["segments"]), PHYSICAL_LEDS // 2))
    _cue_pattern = spec["pattern"] if spec["pattern"] in PATTERNS else "flash"
    _cue_period_ns = max(int(spec["period"] * 1_000_000_000), 20_000_000)    # floor 20 ms
    _cue_until_ns = -1 if follow else now + int(max(spec["duration"], 0.01) * 1_000_000_000)
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
    if _cue_pattern == "flash" or _cue_pattern == "segment":
        step = 0 if phase < 0.5 else 1          # two states -- redraw on the boundary only
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

_trig_state = False                     # debounced logical level ("asserted")
_trig_raw = False
_trig_since_ns = 0
fired = 0                               # wire-triggered cue count, echoed by /status


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
    global _trig_state, _trig_raw, _trig_since_ns, fired
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
        follow = cfg["mode"] == "follow"
        if follow or cfg["retrigger"] or not cue_active():
            fired += 1
            cue_start(cfg, follow=follow)       # on, then off after `duration`. That is all the
                                                # wire does -- it carries no colour, only timing.
    elif cfg["mode"] == "follow":
        cue_stop()


apply_polarity()
print("Cue input on {} (active {}), mode={}, nvm={}".format(
    CUE_PIN, "low" if cfg["active_low"] else "high", cfg["mode"],
    "restored" if _nvm_loaded else ("available" if _nvm else "UNAVAILABLE")))

# ---------------------------------------------------------------------------
# HTTP route handlers
# ---------------------------------------------------------------------------
# Defined unconditionally, registered further down only if the Wi-Fi stack came up. Nothing
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


CUE_FIELDS = ("r", "g", "b", "w", "pattern", "segments", "duration", "period")
BOARD_FIELDS = ("brightness", "mode", "active_low", "debounce_ms", "retrigger", "enabled")


def cfg_text(note=""):
    return ((note + "\n") if note else "") + (
        "# cue\nr={r}\ng={g}\nb={b}\nw={w}\n"
        "pattern={pattern}\nsegments={segments}\nduration={duration}\nperiod={period}\n"
        "# board\nbrightness={brightness}\nmode={mode}\nactive_low={active_low}\n"
        "debounce_ms={debounce_ms}\nretrigger={retrigger}\nenabled={enabled}\n"
    ).format(**cfg) + "nvm={}\n".format("available" if _nvm else "unavailable")


# --- /config : read or change EVERYTHING -------------------------------------
# The whole configurable surface of the board, in one place. Send any subset of the fields;
# the rest is unchanged. This is how you set what a cue looks like -- join the board's access
# point from a laptop or phone and call it.
#
#   http://192.168.4.1/config
#   http://192.168.4.1/config?r=0&g=255&b=0&w=0&pattern=segment&segments=6
#   http://192.168.4.1/config?duration=1.2&period=0.3
#   http://192.168.4.1/config?save=1        keep it across reboots
#   http://192.168.4.1/config?reset=1       back to the defaults in this file
def configure(request: Request):
    note = ""
    if _bool(request, "reset", False):
        cfg.clear()
        cfg.update(DEFAULTS)
        nvm_clear()
        pixels.brightness = cfg["brightness"]
        apply_polarity()
        return Response(request, cfg_text("reset to defaults, NVM cleared"))

    if _given(request, *CUE_FIELDS):
        cfg["r"] = _int(request, "r", cfg["r"])
        cfg["g"] = _int(request, "g", cfg["g"])
        cfg["b"] = _int(request, "b", cfg["b"])
        cfg["w"] = _int(request, "w", cfg["w"])
        cfg["pattern"] = _choice(request, "pattern", cfg["pattern"], PATTERNS)
        cfg["segments"] = _int(request, "segments", cfg["segments"], 1, PHYSICAL_LEDS // 2)
        cfg["duration"] = _float(request, "duration", cfg["duration"], 0.05, 300.0)
        cfg["period"] = _float(request, "period", cfg["period"], 0.02, 10.0)

    if _given(request, *BOARD_FIELDS):
        was_active_low = cfg["active_low"]
        # Not capped -- see the POWER note at the top of this file. Raising this is the
        # operator's decision, and it is the one setting that can damage hardware.
        cfg["brightness"] = _float(request, "brightness", cfg["brightness"], 0.0, 1.0)
        cfg["mode"] = _choice(request, "mode", cfg["mode"], MODES)
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
    return Response(request, cfg_text(note))


# --- /cue : run the configured cue now, without the wire ---------------------
# What the trigger does, on demand -- for checking a setting looks right before you walk away.
#   http://192.168.4.1/cue
def fire_cue(request: Request):
    cue_start(cfg)
    return Response(request, "firing the configured cue ({}s)\n".format(cfg["duration"]))


# --- /set_color : solid colour on the whole ring ----------------------------
def set_color(request: Request):
    cue_stop(restore=False)             # a manual command wins over an in-flight cue
    color = (_int(request, "r", 0), _int(request, "g", 0),
             _int(request, "b", 0), _int(request, "w", 0))
    pixels.fill(color)
    pixels.show()
    return Response(request, "Set to {}\n".format(color))


# --- /segments : static arcs (not a timed cue -- use pattern=segment for that)
#   http://<board>/segments?factor=4&colors=255,0,0,0.0,255,0,0.0,0,255,0.0,0,0,255
def set_multi_segments(request: Request):
    cue_stop(restore=False)
    factor = _int(request, "factor", 1, 1, PHYSICAL_LEDS)
    colors_str = request.query_params.get("colors", "")
    pixels.fill(OFF)
    if colors_str:
        seg_size = PHYSICAL_LEDS // factor      # truncates: a factor that does not divide 60
                                                # leaves the remaining pixels dark
        for i, part in enumerate(colors_str.split(".")):
            if i >= factor:
                break
            try:
                c = [int(v) for v in part.split(",")]
                if len(c) == 3:                 # auto-pad RGB -> RGBW
                    c.append(0)
                for p in range(i * seg_size, (i + 1) * seg_size):
                    pixels[p] = (c[0], c[1], c[2], c[3])
            except Exception as e:
                print("Segment error:", e)
                continue
    pixels.show()
    return Response(request, "Segments updated\n")


# --- /off : clear the ring and cancel any running cue -----------------------
def set_off(request: Request):
    cue_stop(restore=False)
    pixels.fill(OFF)
    pixels.show()
    return Response(request, "NeoPixels are now OFF\n")


# --- /status : what is the board doing? First stop when a cue "does not work"
def status(request: Request):
    return Response(request, cfg_text() + (
        "# state\nleds={}\npatterns={}\nip={}\n"
        "cue_active={}\nwire_fired={}\n"
        "trigger_pin_raw={}\ntrigger_asserted={}\ntrigger_debounced={}\n"
        "nvm_restored_at_boot={}\nuptime_s={:.1f}\n"
    ).format(
        PHYSICAL_LEDS, ",".join(PATTERNS), wifi_ip,
        cue_active(), fired,
        cue_in.value, trigger_asserted(), _trig_state,
        _nvm_loaded, time.monotonic_ns() / 1e9,
    ))


# ---------------------------------------------------------------------------
# Read-only file access
# ---------------------------------------------------------------------------
# Deliberately READ-ONLY. Serving files needs nothing special; WRITING them would require
# storage.remount() in boot.py, which makes CIRCUITPY read-only to the host computer and puts a
# boot-time failure between you and a working board. See README.
# settings.toml is refused: it holds the Wi-Fi credential.
_FS_ROOT = "/"                  # the CIRCUITPY drive root; every call below is explicitly
                                # anchored rather than depending on the interpreter's cwd
_FS_DENY = ("settings.toml",)
_FS_ALLOW_EXT = (".py", ".txt", ".json", ".toml")


def _fs_reject(path):
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


def fs_list(request: Request):
    out = []
    _walk("", out)
    return Response(request, "\n".join(out) + "\n")


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
    ("/off", set_off),
    ("/fs", fs_list),
    ("/fs/get", fs_get),
)

# ---------------------------------------------------------------------------
# Wi-Fi access point + HTTP server  (settings only -- failure here is NOT fatal)
# ---------------------------------------------------------------------------
# The board hosts its OWN network. Join it from a laptop or phone to change what the cue looks
# like; nothing else routes to it, and nothing needs to. The experiment's cue arrives on the
# WIRE, so the radio is never in the timing path and never in the experiment's path at all.
#
# Wrapped so that a dead ESP32, a missing settings.toml, or a rejected password cannot stop the
# board servicing the trigger. If this block fails the loop below still runs, using the settings
# restored from NVM (or the defaults in this file).
wifi_ip = "-"
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
            "(values must be QUOTED strings; a WPA2 password must be 8-63 chars)")
    esp.create_AP(ssid, password, 1)

    wifi_ip = str(esp.pretty_ip(esp.ip_address))        # the AP gateway: 192.168.4.1
    pool = socketpool.SocketPool(esp)
    server = Server(pool, debug=True)
    for path, handler in ROUTES:
        server.route(path, methods=["GET"])(handler)
    server.start(wifi_ip, port=80)
    print("Access point '{}' up, settings at http://{}".format(ssid, wifi_ip))
except Exception as e:
    server = None
    print("WiFi/HTTP unavailable: {}".format(e))
    print("Continuing with the WIRE ONLY (settings from NVM/defaults, not changeable until fixed).")

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
# poll_trigger() runs FIRST and cue_service() second, so the wire is serviced even while HTTP
# traffic is arriving. server.poll() is non-blocking and no route handler sleeps, so a pass
# through this loop is short and trigger latency stays at roughly the debounce time.
while True:
    try:
        poll_trigger()
        cue_service()
        if server is not None:
            server.poll()
    except Exception as e:
        print("Error: {}".format(e))
        time.sleep(0.05)        # a persistent fault must not spin the console at full speed
