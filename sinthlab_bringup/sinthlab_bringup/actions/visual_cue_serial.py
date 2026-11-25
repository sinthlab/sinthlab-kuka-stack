#!/usr/bin/env python3
"""The RS-422 serial link to the end-effector board: visual cues, haptics and the force sensor.

    this computer ─USB─► StarTech ICUSB422IS ══ X76 ══ media flange ══ tool pins ══► MikroE ──► Metro

ONE LINK PER PORT, SHARED (shared()): the visual cue (when `visual_cue.trigger` is "serial"), the
haptic guidance and the force sensor all use the same object, so one thread writes the port and one
reads it (end_effector_metro_code/README.md, "The serial link"):

    fire(label)                      CUE              -> ACK CUE <n>
    fire(label, (r, g, b, w))        CUE r g b w      -> ACK CUE <n>
    show("/segments?factor=f&colors=c")  SEG f c      -> ACK SEG
    show("/off")                     OFF              -> ACK OFF
    set_brightness(b)                GET /config?brightness=b  -> OK ...   (firmware with GET)
    check()                          PING             -> PONG
    set_haptic(pct)                  HAP pct          -> ACK HAP <pct>   (refreshed; see below)
    set_stream(rate)                 FSR rate         -> ACK FSR <rate>, then S lines

THE READER. Every line the board sends goes through one reader thread: a sample line
(`S <t_ms> <raw> <hap>`) updates latest() and never reaches a command; anything else is a reply,
handed to the command waiting for it. So samples can stream continuously between commands.

THE ACK MARKS THE LIGHT. The firmware writes ACK CUE only after pixels.show(), so the ack's arrival
here is within ~1 ms of the ring lighting. Each cue's round trip is reported through on_result.

HAPTICS ARE A LEVEL, NOT A QUEUE. set_haptic() only records the strength wanted; the writer sends it
when it changes, and re-sends it every HAPTIC_REFRESH_SEC while it is above 0 -- the board drops the
motor to 0 if a HAP is not refreshed within 0.5 s, so a stalled or dead computer cannot leave it
buzzing. Only the latest strength is ever sent: a burst of changes never queues up behind a cue.

THE STREAM IS KEPT ALIVE. With a stream rate set, FSR <rate> is sent when the port opens and again
whenever no sample has arrived for STREAM_RESTART_SEC (the board was power-cycled, or the link was
re-attached).

GUARANTEES
    * No call waits on the port: commands go out from one background thread.
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
from typing import Dict, Optional, Tuple

from rclpy.node import Node as rclpyNode

BAUD = 115200
REPLY_TIMEOUT_SEC = 0.5         # the board answers within a few ms; this only bounds a dead link
STALE_SEC = 0.5                 # a cue older than this when its turn comes is dropped, never lit late
REOPEN_WARN_SEC = 10.0          # repeat "cannot open the port" at most this often
HAPTIC_REFRESH_SEC = 0.2        # re-send a held strength this often (the board's watchdog is 0.5 s)
STREAM_RESTART_SEC = 1.0        # no sample for this long while streaming: send FSR <rate> again
IDLE_TICK_SEC = 0.02            # the writer checks the haptic level and the stream this often

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


def parse_sample(line: str) -> Optional[Tuple[int, int, int]]:
    """(t_ms, raw, hap) from a board sample line `S <t_ms> <raw> <hap>`, or None if it is not one."""
    parts = line.split()
    if len(parts) != 4 or parts[0] != "S":
        return None
    try:
        return int(parts[1]), int(parts[2]), int(parts[3])
    except ValueError:
        return None


class SerialCueTrigger:
    """The serial link to the board. Use shared(): one instance per port, per process."""

    _shared: Dict[str, "SerialCueTrigger"] = {}
    _shared_lock = threading.Lock()

    @classmethod
    def shared(cls, node: rclpyNode, port: str, on_result=None) -> "SerialCueTrigger":
        """The link on `port`, created on first use. on_result, if given, replaces the cue-ack sink."""
        with cls._shared_lock:
            link = cls._shared.get(port)
            if link is None:
                link = cls._shared[port] = cls(node, port, on_result=on_result)
            elif on_result is not None:
                link._on_result = on_result
            return link

    def __init__(self, node: rclpyNode, port: str, on_result=None) -> None:
        # on_result(label, ok, rtt_ms) fires when the board answers ACK CUE: the ring is already lit.
        self._on_result = on_result
        self._log = node.get_logger()
        self._port_name = port
        self._port = None
        self._port_lock = threading.Lock()
        self._last_open_warn = 0.0
        self._jobs: queue.SimpleQueue = queue.SimpleQueue()
        self._replies: queue.SimpleQueue = queue.SimpleQueue()
        # Haptics: the strength wanted, and what was last sent and when.
        self._hap_want = 0
        self._hap_sent: Optional[int] = None
        self._hap_sent_at = 0.0
        # The force-sensor stream: the rate wanted, and the latest sample.
        self._stream_rate = 0
        self._stream_asked_at = 0.0
        self._sample: Optional[Tuple[int, int, int, float]] = None   # (t_ms, raw, hap, received monotonic)
        threading.Thread(target=self._run, name="ee_serial_writer", daemon=True).start()
        threading.Thread(target=self._read, name="ee_serial_reader", daemon=True).start()
        self._log.info(f"End-effector serial link on {port} (cues, haptics, force sensor).")
        self._check_latency_timer()

    # ---------------------------------------------------------------- the calls
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

    def set_haptic(self, pct: int) -> None:
        """The haptic strength wanted, 0-100 %. Returns immediately; the writer sends it (see the
        module doc, "HAPTICS ARE A LEVEL")."""
        self._hap_want = max(0, min(100, int(pct)))

    def set_stream(self, rate_hz: int) -> None:
        """Stream force samples at rate_hz (0 stops). Returns immediately; kept alive by the writer."""
        rate = max(0, int(rate_hz))
        if rate != self._stream_rate:
            self._stream_rate = rate
            self._stream_asked_at = 0.0            # ask at the next tick
            if rate == 0:
                self._jobs.put(("FSR 0", None, time.monotonic(), False))

    def latest(self) -> Optional[Tuple[int, int, int, float]]:
        """The latest force sample: (board t_ms, raw 0-65535, haptic % at that sample, received at
        time.monotonic()), or None before the first."""
        return self._sample

    # ---------------------------------------------------------------- the writer
    def _run(self) -> None:
        while True:
            try:
                job = self._jobs.get(timeout=IDLE_TICK_SEC)
            except queue.Empty:
                job = None
            try:
                if job is not None:
                    self._send(*job)
                self._service_haptic()
                self._service_stream()
            except Exception as exc:    # this thread must outlive anything the port can throw
                self._close()
                self._log.warn(f"End-effector serial link: unexpected error, carrying on: {exc!r}")

    def _service_haptic(self) -> None:
        want, now = self._hap_want, time.monotonic()
        due = want != self._hap_sent or (want > 0 and now - self._hap_sent_at >= HAPTIC_REFRESH_SEC)
        if not due:
            return
        if self._send(f"HAP {want}", None, now, False, quiet=True):
            self._hap_sent, self._hap_sent_at = want, now
        else:
            self._hap_sent = None                  # unknown: try again at the next tick

    def _service_stream(self) -> None:
        if self._stream_rate <= 0:
            return
        now = time.monotonic()
        last_rx = self._sample[3] if self._sample is not None else 0.0
        if now - last_rx < STREAM_RESTART_SEC or now - self._stream_asked_at < STREAM_RESTART_SEC:
            return
        self._stream_asked_at = now
        if self._send(f"FSR {self._stream_rate}", None, now, False, quiet=True):
            self._log.info(f"Force sensor: streaming at {self._stream_rate} samples/s.")

    def _open(self) -> bool:
        with self._port_lock:
            if self._port is not None:
                return True
            try:
                import serial           # python3-serial; imported here so a missing package only
                                        # disables this path instead of breaking the import
                self._port = serial.Serial(self._port_name, BAUD, timeout=REPLY_TIMEOUT_SEC)
                self._port.reset_input_buffer()
                self._stream_asked_at = 0.0         # (re)start the stream on a fresh port
                self._hap_sent = None
                return True
            except Exception as exc:
                now = time.monotonic()
                if now - self._last_open_warn > REOPEN_WARN_SEC:
                    self._last_open_warn = now
                    self._log.warn(f"End-effector serial port {self._port_name} cannot be opened ({exc}). {_HINT}")
                return False

    def _close(self) -> None:
        with self._port_lock:
            try:
                if self._port is not None:
                    self._port.close()
            except Exception:
                pass
            self._port = None

    def _send(self, cmd: str, label: Optional[str], queued_at: float, droppable: bool,
              quiet: bool = False) -> bool:
        """Write one command and wait for its reply. True if the board confirmed it."""
        waited = time.monotonic() - queued_at
        if droppable and waited > (0.25 if label and label.startswith("go_") else STALE_SEC):
            self._log.warn(
                f"Visual cue '{label}' DROPPED: it waited {waited * 1000:.0f} ms behind a slow "
                "command and would have lit late.")
            return False
        what = f"Visual cue '{label}'" if label else "End-effector board"
        if not self._open():
            if not quiet:
                self._log.warn(f"{what} NOT sent: the serial port is not open.")
            return False

        while True:                                   # a stale reply must not pass for this one's
            try:
                self._replies.get_nowait()
            except queue.Empty:
                break
        sent_at = time.monotonic()
        try:
            self._port.write((cmd + "\n").encode())
        except Exception as exc:                      # adapter unplugged or detached from WSL
            self._close()
            self._log.warn(f"{what} NOT sent: serial error ({exc}). {_HINT}", throttle_duration_sec=5.0)
            return False
        try:
            reply = self._replies.get(timeout=REPLY_TIMEOUT_SEC)
        except queue.Empty:
            reply = ""
        ms = (time.monotonic() - sent_at) * 1000

        expected = "OK" if cmd.startswith("GET ") else {"PING": "PONG"}.get(cmd, "ACK " + cmd.split()[0])
        if not reply.startswith(expected):
            why = "no reply" if not reply else f"replied {reply!r}"
            hint = ("The board's firmware cannot change settings over the serial link: update it "
                    "(end_effector_metro_code/code.py), or set the brightness over Wi-Fi in Diagnostics."
                    if cmd.startswith("GET ") and reply.startswith("ERR") else
                    "Is the board powered and the link wired?")
            self._log.warn(f"{what} ({cmd}) NOT confirmed: {why} within "
                           f"{REPLY_TIMEOUT_SEC * 1000:.0f} ms. {hint}",
                           throttle_duration_sec=5.0)
            return False
        if label == "brightness":
            self._log.info(f"Ring brightness set to {cmd.rsplit('=', 1)[1]} on the board ({ms:.1f} ms): every colour "
                           f"now lights at about {round(255 * float(cmd.rsplit('=', 1)[1]))} of 255.")
            return True

        if self._on_result is not None and label is not None and (cmd.startswith("CUE") or label.startswith("go_")):
            try:
                self._on_result(label, True, ms)
            except Exception:
                pass
        if cmd == "PING":
            self._log.info(f"End-effector board answered on {self._port_name} ({ms:.1f} ms).")
        elif cmd.startswith("CUE"):
            self._log.info(f"Visual cue '{label}' fired over the serial link ({ms:.1f} ms round trip).")
        return True

    # ---------------------------------------------------------------- the reader
    def _read(self) -> None:
        """Every line from the board: samples update latest(), everything else is a reply."""
        while True:
            port = self._port
            if port is None:
                time.sleep(0.05)
                continue
            try:
                line = port.readline().decode(errors="replace").strip()
            except Exception:
                time.sleep(0.05)                       # the writer reopens the port
                continue
            if not line:
                continue
            sample = parse_sample(line)
            if sample is not None:
                self._sample = (*sample, time.monotonic())
            else:
                self._replies.put(line)

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
                f"{name} latency_timer is {value} ms, not 1: each reply arrives up to {value} ms "
                f"late. Fix: echo 1 | sudo tee {timer}")
