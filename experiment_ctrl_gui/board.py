#!/usr/bin/env python3
"""The end-effector cue board (Metro M4, end_effector_metro_code/code.py), for the Diagnostics page.

The browser cannot read the board's replies itself (another origin), so the dashboard relays each
command: POST /api/diag/board {addr, cmd, params} -> GET http://<addr>/<cmd>?<params>. Only the
board's own commands go through, and only to a private address -- the board serves 192.168.4.1 on
its own access point -- so the relay cannot be pointed anywhere else. This computer must be joined
to that access point (KUKA_NEOPIXEL).

FakeBoard answers the same commands with the same text in --demo, so the page can be tried with no
hardware.
"""
from __future__ import annotations

import ipaddress
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
                f"cue_active={int(now < self.cue_until)}\nwire_fired=0\nnvm_writes=0\n"
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
