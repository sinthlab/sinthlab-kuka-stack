#!/usr/bin/env python3
"""The end-effector cue board (Metro M4, end_effector_metro_code/code.py), for the Diagnostics page.

The browser cannot read the board's replies itself (another origin), so the dashboard relays each
command: POST /api/diag/board {addr, cmd, params} -> GET http://<addr>/<cmd>?<params>. Only the
board's own commands go through, and only to a private address -- the board serves 192.168.4.1 on
its own access point -- so the relay cannot be pointed anywhere else. This computer must be joined
to that access point (KUKA_NEOPIXEL).

Every command can also go over the serial link instead (path "serial"): the dashboard writes
`GET /<cmd>?<params>` to the StarTech adapter's port, and the board runs the very same route function
and answers its text on one line (end_effector_metro_code/README.md, "The serial link"). So the page's
controls and tests are identical on both paths. Only a USB serial port is accepted.

FakeBoard answers the same commands with the same text in --demo, so the page can be tried with no
hardware.
"""
from __future__ import annotations

import ipaddress
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Dict, Tuple

COMMANDS = ("status", "config", "cue", "set_color", "segments", "off", "led_test")
TIMEOUT_SEC = 2.0


def check(addr: str, cmd: str) -> None:
    """Refuse anything but the board's own commands, to a private address."""
    if cmd not in COMMANDS:
        raise ValueError(f"unknown board command {cmd!r}")
    try:
        if not ipaddress.ip_address(addr).is_private:
            raise ValueError
    except ValueError:
        raise ValueError(f"board address must be a private IPv4 address, got {addr!r}") from None


def call(addr: str, cmd: str, params: Dict[str, object]) -> Tuple[bool, str, float]:
    """(reached, reply text, round trip [ms])."""
    check(addr, cmd)
    query = urllib.parse.urlencode({k: v for k, v in params.items() if v is not None and v != ""})
    url = f"http://{addr}/{cmd}" + (f"?{query}" if query else "")
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(url, timeout=TIMEOUT_SEC) as r:
            text = r.read().decode(errors="replace")
        return True, text, (time.monotonic() - t0) * 1000.0
    except (urllib.error.URLError, OSError) as exc:
        return False, (f"could not reach the board at {addr} ({exc}). Is this computer joined to "
                       f"its Wi-Fi (KUKA_NEOPIXEL), and is the board powered?"), (time.monotonic() - t0) * 1000.0


SERIAL_BAUD = 115200
SERIAL_TIMEOUT_SEC = 2.0
_SERIAL_PORT = re.compile(r"^/dev/tty(USB|ACM)[0-9]+$")


def serial_check(port: str, cmd: str) -> None:
    """Refuse anything but the board's own commands, to a USB serial port."""
    if cmd not in COMMANDS:
        raise ValueError(f"unknown board command {cmd!r}")
    if not _SERIAL_PORT.match(port):
        raise ValueError(f"serial port must be /dev/ttyUSB<n> or /dev/ttyACM<n>, got {port!r}")


def _legacy_line(cmd: str, params: Dict[str, object]):
    """The same action in the experiment commands (CUE / SEG / OFF / PING), for board firmware without
    GET. None when there is no equivalent: settings and the LED test need GET, or the Wi-Fi path."""
    if cmd == "status":
        return "PING"
    if cmd == "cue":
        return "CUE"
    if cmd == "off":
        return "OFF"
    if cmd == "segments":
        return f"SEG {int(params.get('factor', 1))} {params.get('colors', '')}"
    if cmd == "set_color":
        rgbw = ",".join(str(int(params.get(k, 0) or 0)) for k in "rgbw")
        return f"SEG 1 {rgbw}"          # one arc of the whole ring = a solid colour
    return None


_LEGACY_NOTE = ("(board firmware without GET: over serial only Check board, Fire cue, Solid colour, "
                "Quarter 0-3 and Off work; settings and the LED test need the Wi-Fi path)")


def _exchange(port: str, line: str) -> Tuple[bool, str, float]:
    """Write one line, read one reply line: (opened, reply, round trip [ms])."""
    import serial
    try:
        with serial.Serial(port, SERIAL_BAUD, timeout=SERIAL_TIMEOUT_SEC) as s:
            s.reset_input_buffer()
            t0 = time.monotonic()
            s.write((line + "\n").encode())
            reply = s.readline().decode(errors="replace").rstrip("\r\n")
            return True, reply, (time.monotonic() - t0) * 1000.0
    except (OSError, serial.SerialException) as exc:
        return False, (f"cannot open {port} ({exc}). Is the StarTech adapter attached to WSL "
                       f"(usbipd attach --wsl --busid <id>) and this user in the dialout group?"), 0.0


def serial_call(port: str, cmd: str, params: Dict[str, object]) -> Tuple[bool, str, float]:
    """The same command as call(), over the serial link: `GET /<cmd>?<params>` runs the board's own
    route function and answers `OK <its text, lines joined by tabs>`. (reached, reply text, round trip [ms])."""
    serial_check(port, cmd)
    query = urllib.parse.urlencode({k: v for k, v in params.items() if v is not None and v != ""})
    line = f"GET /{cmd}" + (f"?{query}" if query else "")
    try:
        import serial  # noqa: F401
    except ImportError:
        return False, "python3-serial is not installed (sudo apt install python3-serial)", 0.0
    opened, reply, ms = _exchange(port, line)
    if not opened:
        return False, reply, 0.0
    if reply.startswith("ERR unknown command"):        # firmware from before GET: fall back
        legacy = _legacy_line(cmd, params)
        if legacy is None:
            return False, f"{cmd}: not available over serial {_LEGACY_NOTE}", ms
        opened, reply, ms = _exchange(port, legacy)
        if not opened:
            return False, reply, 0.0
        ok = reply == "PONG" or reply.startswith("ACK")
        text = (("serial=up\nfirmware=older: over serial only cues work; settings (brightness included) need the "
                 "Wi-Fi path, or the current code.py flashed over USB\n" + _LEGACY_NOTE + "\n")
                if cmd == "status" and ok else f"{legacy} -> {reply}\n")
        return ok, text if ok else (reply or "no reply"), ms
    if not reply:
        return False, (f"no reply on {port} within {SERIAL_TIMEOUT_SEC:.0f} s. Is the board powered and "
                       f"the link wired?"), ms
    if not reply.startswith("OK "):
        return False, reply, ms
    return True, reply[3:].replace("\t", "\n") + "\n", ms


class FakeBoard:
    """The firmware's replies, simulated, for --demo."""

    DEFAULTS = {"r": 0, "g": 0, "b": 0, "w": 255, "pattern": "flash", "segments": 4, "duration": 2.0,
                "period": 0.5, "brightness": 0.2, "mode": "pulse", "active_low": True, "debounce_ms": 5,
                "retrigger": False, "enabled": False}

    def __init__(self) -> None:
        self.cfg = dict(self.DEFAULTS)
        self.t0 = time.monotonic()
        self.cue_until = 0.0
        self.test = None              # (first, last, step, loop, started)

    def _cfg_text(self, note: str = "") -> str:
        c = {k: (int(v) if isinstance(v, bool) else v) for k, v in self.cfg.items()}
        return ((note + "\n") if note else "") + "".join(
            f"{k}={c[k]}\n" for k in ("r", "g", "b", "w", "pattern", "segments", "duration", "period",
                                     "brightness", "mode", "active_low", "debounce_ms", "retrigger",
                                     "enabled")) + "nvm=RAM ONLY (demo board)\n"

    def call(self, cmd: str, p: Dict[str, object]) -> Tuple[bool, str, float]:
        now = time.monotonic()
        if cmd == "status":
            px = -1
            if self.test:
                first, last, step, loop, t = self.test
                i = int((now - t) / (4 * step))
                px = first + (i % (last - first + 1) if loop else i)
                if px > last:
                    self.test, px = None, -1
            return True, self._cfg_text() + (
                f"leds=60\npatterns=flash,solid,breathe,chase,segment\nip=192.168.4.1\n"
                f"cue_active={int(now < self.cue_until)}\nwire_fired=0\nserial=up\n"
                f"serial_fired={getattr(self, 'serial_fired', 0)}\nnvm_writes=0\n"
                f"trigger_pin_raw=1\ntrigger_asserted=0\ntrigger_debounced=0\n"
                f"led_test_pixel={px}\nled_test_colour={'red' if px >= 0 else '-'}\n"
                f"nvm_restored_at_boot=0\nuptime_s={now - self.t0:.1f}\n"), 3.0
        if cmd == "config":
            if p.get("reset"):
                self.cfg = dict(self.DEFAULTS)
                return True, self._cfg_text("reset to defaults, NVM cleared"), 3.0
            for k, v in p.items():
                if k in self.cfg:
                    d = self.DEFAULTS[k]
                    self.cfg[k] = (str(v).lower() in ("1", "true", "yes", "on")) if isinstance(d, bool) \
                        else type(d)(v)
            return True, self._cfg_text("saved to RAM ONLY -- no NVM on this build" if p.get("save") else ""), 3.0
        if cmd == "cue":
            self.cue_until = now + float(self.cfg["duration"])
            return True, f"firing the configured cue ({self.cfg['duration']}s)\n", 3.0
        if cmd == "led_test":
            first, last = int(p.get("from", 0)), int(p.get("to", 59))
            self.test = (min(first, last), max(first, last), float(p.get("step", 0.25)),
                         str(p.get("loop", "0")) in ("1", "true"), now)
            return True, "LED test started\n", 3.0
        if cmd == "off":
            self.cue_until, self.test = 0.0, None
            return True, "NeoPixels are now OFF\n", 3.0
        if cmd == "set_color":
            return True, f"Set to ({p.get('r', 0)}, {p.get('g', 0)}, {p.get('b', 0)}, {p.get('w', 0)})\n", 3.0
        if cmd == "segments":
            return True, "Segments updated\n", 3.0
        raise ValueError(f"unknown board command {cmd!r}")
