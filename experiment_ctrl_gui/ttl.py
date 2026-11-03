#!/usr/bin/env python3
"""The cabinet's TTL sync outputs (sunrise_controller_code/TtlPulser.java), for the Diagnostics page.

The cabinet app (LbrImpedanceControlServer, or PulseTestServer, which never moves the arm) serves
pulse commands on UDP 30002 -- the same ones the experiments send at trial start and end
(sinthlab_bringup/helpers/experiment_control.py). The browser cannot send UDP, so the dashboard
relays each command: POST /api/diag/ttl {addr, cmd} -> one datagram, and the cabinet's one-line
answer. Only the three TtlPulser commands go through, and only to a private address.

FakeCabinet answers the same commands with the same text in --demo, so the page can be tried with
no hardware.
"""
from __future__ import annotations

import ipaddress
import re
import socket
import time
from typing import Tuple

# Must equal TtlPulser.DEFAULT_PORT in sunrise_controller_code/TtlPulser.java.
PORT = 30002
CHANNELS = 4
WIDTH_MS = (10, 5000)       # as nsp_sync.pulse_width_ms (helpers/live_params.py)
TIMEOUT_SEC = 1.0
_COMMAND = re.compile(r"^(STATUS|ALLOFF|PULSE ([1-9]\d*) ([1-9]\d*))$")


def check(addr: str, cmd: str) -> None:
    """Refuse anything but TtlPulser's own commands, to a private address."""
    m = _COMMAND.match(cmd)
    if not m:
        raise ValueError(f"unknown command {cmd!r}: STATUS, ALLOFF or PULSE <output> <width ms>")
    if m.group(2):
        out, width = int(m.group(2)), int(m.group(3))
        if not 1 <= out <= CHANNELS:
            raise ValueError(f"output must be 1-{CHANNELS}, got {out}")
        if not WIDTH_MS[0] <= width <= WIDTH_MS[1]:
            raise ValueError(f"width must be {WIDTH_MS[0]}-{WIDTH_MS[1]} ms, got {width}")
    try:
        if not ipaddress.IPv4Address(addr).is_private:
            raise ValueError
    except ValueError:
        raise ValueError(f"cabinet address must be a private IPv4 address, got {addr!r}") from None


def call(addr: str, cmd: str) -> Tuple[bool, str, float]:
    """(answered, reply text, round trip [ms])."""
    check(addr, cmd)
    t0 = time.monotonic()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(TIMEOUT_SEC)
            s.sendto(cmd.encode("ascii"), (addr, PORT))
            reply = s.recv(256).decode("ascii", "replace").strip()
        return True, reply, (time.monotonic() - t0) * 1000.0
    except (socket.timeout, OSError) as exc:
        why = "no answer" if isinstance(exc, socket.timeout) else str(exc)
        return False, (f"{why} from {addr}:{PORT}. Is LbrImpedanceControlServer or PulseTestServer running "
                       f"on the cabinet, and is {addr} its address on this computer's network?"), \
            (time.monotonic() - t0) * 1000.0


class FakeCabinet:
    """TtlPulser's replies, simulated, for --demo."""

    def call(self, cmd: str) -> Tuple[bool, str, float]:
        if cmd == "STATUS":
            return True, f"OK available=true outputs={CHANNELS} group=demo cabinet", 2.0
        if cmd == "ALLOFF":
            return True, "OK ALLOFF", 2.0
        return True, "OK " + cmd, 2.0
