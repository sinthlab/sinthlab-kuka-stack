#!/usr/bin/env python3
"""Serial trigger for the visual cue -- the RS-422 link to the end-effector board.

VisualCue and RingStateCue use this when `visual_cue.trigger` is "serial":

    this computer ─USB─► StarTech ICUSB422IS ══ X76 ══ media flange ══ tool pins ══► MikroE ──► Metro

It offers the same calls as the Wi-Fi trigger (visual_cue_remote.py), so the experiments never know
which path is in use, and translates each board request into a serial command line
(end_effector_metro_code/README.md, "The serial link"):

    fire(label)                      CUE              -> ACK CUE <n>
    fire(label, (r, g, b, w))        CUE r g b w      -> ACK CUE <n>
    show("/segments?factor=f&colors=c")  SEG f c      -> ACK SEG
    show("/off")                     OFF              -> ACK OFF
    set_brightness(b)                GET /config?brightness=b  -> OK ...   (firmware with GET)
    check()                          PING             -> PONG

THE ACK MARKS THE LIGHT. The firmware writes ACK CUE only after pixels.show(), so the ack's arrival
here is within ~1 ms of the ring lighting. Each cue's round trip is reported through on_result,
exactly as the Wi-Fi trigger reports its HTTP round trip.

GUARANTEES (the same as the Wi-Fi trigger's)
    * fire() / show() never wait on the port: commands go out from one background thread.
    * Nothing here raises into the orchestrator. A failure logs a warning; the trial goes on.
    * One command at a time. A cue that waited longer than STALE_SEC is dropped, never lit late.
    * The port is opened on first use and reopened after an error, so unplugging and re-attaching
      the adapter (usbipd attach) recovers without restarting the experiment.
"""
from __future__ import annotations

import queue
import threading
import time
import urllib.parse
from pathlib import Path
from typing import Optional, Tuple

from rclpy.node import Node as rclpyNode

BAUD = 115200
REPLY_TIMEOUT_SEC = 0.5         # the board answers within a few ms; this only bounds a dead link
STALE_SEC = 0.5                 # a cue older than this when its turn comes is dropped, never lit late
REOPEN_WARN_SEC = 10.0          # repeat "cannot open the port" at most this often

_HINT = ("Is the StarTech adapter attached to WSL (usbipd attach --wsl --busid <id>), is this user in "
         "the dialout group, and is python3-serial installed?")


def command_for(path: str) -> Optional[str]:
    """The serial command for a Wi-Fi board request path, or None if it has no serial equivalent."""
    parsed = urllib.parse.urlparse(path)
    if parsed.path == "/off":
        return "OFF"
    if parsed.path == "/segments":
        q = urllib.parse.parse_qs(parsed.query)
        factor, colours = q.get("factor", ["1"])[0], q.get("colors", [""])[0]
        return f"SEG {factor} {colours}" if colours else "OFF"
    if parsed.path == "/cue":
        return "CUE"
    return None


class SerialCueTrigger:
    """Sends cue commands to the board over the serial link from one background thread. Share one
    instance per process."""

    def __init__(self, node: rclpyNode, port: str, on_result=None) -> None:
        # on_result(label, ok, rtt_ms) fires when the board answers ACK CUE: the ring is already lit.
        self._on_result = on_result
        self._log = node.get_logger()
        self._port_name = port
        self._port = None
        self._last_open_warn = 0.0
        self._jobs: queue.SimpleQueue = queue.SimpleQueue()
        threading.Thread(target=self._run, name="visual_cue_serial", daemon=True).start()
        self._log.info(f"Visual cues will fire over the serial link on {port}.")
        self._check_latency_timer()

    # ---------------------------------------------------------------- the trigger's calls
    def check(self) -> None:
        """PING the board and log whether it answered. Returns immediately."""
        self._jobs.put(("PING", None, time.monotonic(), False))

    def fire(self, label: str, colour: Optional[Tuple[int, int, int, int]] = None) -> None:
        """Run the board's cue -- in `colour` (r, g, b, w) if given. Returns immediately."""
        cmd = "CUE" if colour is None else "CUE {} {} {} {}".format(*colour)
        self._jobs.put((cmd, label, time.monotonic(), True))

    def set_brightness(self, value: float) -> None:
        """Set the ring's brightness (0-1) in the board's RAM. Needs firmware that answers GET over the
        serial link; older firmware replies ERR, and the warning says so. Returns immediately."""
        self._jobs.put((f"GET /config?brightness={value:g}", "brightness", time.monotonic(), False))

    def show(self, path: str, label: str) -> None:
        """Send a STATE change -- the same /segments or /off request the Wi-Fi trigger sends. Never
        dropped as stale: a late state is still the right state (ring_state_cue.py)."""
        cmd = command_for(path)
        if cmd is None:
            self._log.warn(f"Visual cue '{label}': {path} has no serial equivalent; not sent.")
            return
        self._jobs.put((cmd, label, time.monotonic(), label.startswith("go_")))

    # ---------------------------------------------------------------- the worker
    def _run(self) -> None:
        while True:
            cmd, label, queued_at, droppable = self._jobs.get()
            try:
                self._send(cmd, label, queued_at, droppable)
            except Exception as exc:    # this thread must outlive anything the port can throw
                self._close()
                self._log.warn(f"Visual cue serial link: unexpected error, carrying on: {exc!r}")

    def _open(self) -> bool:
        if self._port is not None:
            return True
        try:
            import serial               # python3-serial; imported here so a missing package only
                                        # disables this path instead of breaking the import
            self._port = serial.Serial(self._port_name, BAUD, timeout=REPLY_TIMEOUT_SEC)
            self._port.reset_input_buffer()
            return True
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_open_warn > REOPEN_WARN_SEC:
                self._last_open_warn = now
                self._log.warn(f"Visual cue serial port {self._port_name} cannot be opened ({exc}). {_HINT}")
            return False

    def _close(self) -> None:
        try:
            if self._port is not None:
                self._port.close()
        except Exception:
            pass
        self._port = None

    def _send(self, cmd: str, label: Optional[str], queued_at: float, droppable: bool) -> None:
        waited = time.monotonic() - queued_at
        if droppable and waited > (0.25 if label and label.startswith("go_") else STALE_SEC):
            self._log.warn(
                f"Visual cue '{label}' DROPPED: it waited {waited * 1000:.0f} ms behind a slow "
                "command and would have lit late.")
            return
        what = f"Visual cue '{label}'" if label else "Visual cue board"
        if not self._open():
            self._log.warn(f"{what} NOT sent: the serial port is not open.")
            return

        self._port.reset_input_buffer()             # a stale reply must not pass for this one's
        sent_at = time.monotonic()
        try:
            self._port.write((cmd + "\n").encode())
            reply = self._port.readline().decode(errors="replace").strip()
        except Exception as exc:                    # adapter unplugged or detached from WSL
            self._close()
            self._log.warn(f"{what} NOT sent: serial error ({exc}). {_HINT}")
            return
        ms = (time.monotonic() - sent_at) * 1000

        expected = "OK" if cmd.startswith("GET ") else {"PING": "PONG"}.get(cmd, "ACK " + cmd.split()[0])
        if not reply.startswith(expected):
            why = "no reply" if not reply else f"replied {reply!r}"
            hint = ("The board's firmware cannot change settings over the serial link: update it "
                    "(end_effector_metro_code/code.py), or set the brightness over Wi-Fi in Diagnostics."
                    if cmd.startswith("GET ") and reply.startswith("ERR") else
                    "Is the board powered and the link wired?")
            self._log.warn(f"{what} ({cmd}) NOT confirmed: {why} within "
                           f"{REPLY_TIMEOUT_SEC * 1000:.0f} ms. {hint}")
            return
        if label == "brightness":
            self._log.info(f"Ring brightness set to {cmd.rsplit('=', 1)[1]} on the board ({ms:.1f} ms): every colour "
                           f"now lights at about {round(255 * float(cmd.rsplit('=', 1)[1]))} of 255.")
            return

        if self._on_result is not None and label is not None and (cmd.startswith("CUE") or label.startswith("go_")):
            try:
                self._on_result(label, True, ms)
            except Exception:
                pass
        if label is None:
            self._log.info(f"Visual cue board answered on {self._port_name} ({ms:.1f} ms).")
        elif cmd.startswith("CUE"):
            self._log.info(f"Visual cue '{label}' fired over the serial link ({ms:.1f} ms round trip).")

    def _check_latency_timer(self) -> None:
        """Warn if the FTDI latency timer is not 1 ms: the default 16 ms delays every reply."""
        name = Path(self._port_name).name
        timer = Path(f"/sys/bus/usb-serial/devices/{name}/latency_timer")
        try:
            value = int(timer.read_text().strip())
        except (OSError, ValueError):
            return                      # not an FTDI device, or not attached yet: nothing to say
        if value != 1:
            self._log.warn(
                f"{name} latency_timer is {value} ms, not 1: each cue's ack arrives up to {value} ms "
                f"late. Fix: echo 1 | sudo tee {timer}")
