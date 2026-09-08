#!/usr/bin/env python3
"""Visual cue — triggers the end effector's NeoPixel ring. Mirrors AudioCue.

    orchestrator ──TCP :30300──► Sunrise cue server ──► 24 V media-flange output
                              ──► optocoupler ──► Metro M4 ──► ring on, then off

This action sends **timing only**: one trigger, and the board runs its configured cue for its
configured duration. That is deliberate — the trigger line is one bit and cannot carry a colour,
and an experiment cue must not depend on a radio link.

WHAT THE CUE LOOKS LIKE IS NOT SET FROM HERE.
    Colour, brightness, pattern, segments, duration and rate live on the board and are set over
    its own Wi-Fi access point with a laptop or phone:

        curl "http://192.168.4.1/config?r=0&g=255&b=0&w=0&duration=1.2&save=1"

    See end_effector_metro_code/README.md. The board hosts that access point itself; the ROS box
    is on the KUKA network and cannot reach it, which is exactly why the cue's appearance is a
    commissioning step rather than a per-trial message.

NOTHING HERE BLOCKS THE EXPERIMENT. Short timeouts, guarded sockets, and `on_complete` fires even
when the cue server is unreachable. A missing cue is bad; a stalled orchestrator is worse.
"""
from __future__ import annotations

import socket
import threading
from typing import Callable, Dict, Optional, Tuple

from rclpy.node import Node as rclpyNode

DEFAULT_PORT = 30300
_CONNECT_TIMEOUT_S = 1.0
_COMMAND_TIMEOUT_S = 0.5


def optional_param(node: rclpyNode, name: str, default):
    """Parameter with a fallback — the visual_cue block may be absent or partial."""
    if node.has_parameter(name):
        value = node.get_parameter(name).value
        if value is not None:
            return value
    return default


class _CueLink:
    """Shared, lazily-opened connection to the cabinet's cue server.

    One connection per host is shared by every VisualCue in a node: the server serves one client
    at a time, and a fresh TCP handshake per cue would add a round trip to the path this design
    keeps short. Reconnection is attempted on demand — never in a retry loop that could stall a
    caller.
    """

    _instances: Dict[Tuple[str, int], "_CueLink"] = {}
    _registry_lock = threading.Lock()

    @classmethod
    def for_host(cls, host: str, port: int) -> "_CueLink":
        with cls._registry_lock:
            key = (host, port)
            if key not in cls._instances:
                cls._instances[key] = _CueLink(host, port)
            return cls._instances[key]

    def __init__(self, host: str, port: int) -> None:
        self._host, self._port = host, port
        self._sock: Optional[socket.socket] = None
        self._f = None
        self._io_lock = threading.Lock()
        self._warned = False

    def _connect(self) -> bool:
        try:
            s = socket.create_connection((self._host, self._port), timeout=_CONNECT_TIMEOUT_S)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # Nagle would add tens of ms
            s.settimeout(_COMMAND_TIMEOUT_S)
            self._sock = s
            self._f = s.makefile("rw", encoding="ascii", newline="\n")
            return True
        except OSError:
            self._sock, self._f = None, None
            return False

    def _drop(self) -> None:
        for obj in (self._f, self._sock):
            try:
                if obj is not None:
                    obj.close()
            except OSError:
                pass
        self._sock, self._f = None, None

    def command(self, text: str) -> Optional[str]:
        """Send one command and return the reply, or None on any failure. Never raises."""
        with self._io_lock:
            for _ in (0, 1):        # one silent reconnect: the cabinet app may have restarted
                if self._f is None and not self._connect():
                    return None
                try:
                    self._f.write(text + "\n")
                    self._f.flush()
                    reply = self._f.readline()
                    if reply:
                        return reply.strip()
                    self._drop()    # clean EOF — the server closed on us
                except OSError:
                    self._drop()
            return None

    def warn_once(self, node: rclpyNode, msg: str) -> None:
        if not self._warned:
            self._warned = True
            node.get_logger().warn(msg)


class VisualCue:
    """Fires the ring's configured cue when start() is called.

    Parameters, all under a single shared `visual_cue` block — there is nothing per cue site to
    configure, because every trigger runs the same board-side cue:

        enabled   bool  false (the default) makes every visual cue a no-op
        host      str   cabinet IP on the KUKA network — the FRI peer
        port      int   cue server port (default 30300)
        pulse_ms  int   0 (default) = a bare trigger; the board's `duration` owns the length.
                        Set this only if the board is in `follow` mode, where the pulse width
                        IS the cue length.

    `label` is for the log line, so you can tell which cue site fired.
    """

    @staticmethod
    def warmup(node: rclpyNode) -> None:
        """Open the connection once at startup so the first real cue does not pay for it.

        Mirrors AudioCue.warmup(). Only ever logs — never raises.
        """
        if not optional_param(node, "visual_cue.enabled", False):
            return
        host = optional_param(node, "visual_cue.host", None)
        if host is None:
            return
        port = int(optional_param(node, "visual_cue.port", DEFAULT_PORT))
        reply = _CueLink.for_host(str(host), port).command("PING")
        if reply is None:
            node.get_logger().warn(
                f"Cue server unreachable at {host}:{port} — visual cues will not fire. "
                "Is the Sunrise application running? (it logs 'Cue server listening on TCP')")
        else:
            node.get_logger().info(f"Visual cue ready at {host}:{port} ({reply})")

    def __init__(self, node: rclpyNode, *, label: str = "cue",
                 on_complete: Callable[[], None]) -> None:
        self._node = node
        self._label = label
        self._on_complete = on_complete

        self._enabled = bool(optional_param(node, "visual_cue.enabled", False))
        self._host = str(optional_param(node, "visual_cue.host", ""))
        self._port = int(optional_param(node, "visual_cue.port", DEFAULT_PORT))
        self._pulse_ms = int(optional_param(node, "visual_cue.pulse_ms", 0))

        self._link = _CueLink.for_host(self._host, self._port) if self._host else None
        self.last_reply: Optional[str] = None

    def start(self) -> None:
        if self._enabled and self._link is not None:
            cmd = "CUE" if self._pulse_ms <= 0 else f"CUE {self._pulse_ms}"
            reply = self._link.command(cmd)
            self.last_reply = reply
            if reply is None:
                self._link.warn_once(
                    self._node,
                    f"Cue server unreachable at {self._host}:{self._port} — continuing without "
                    "the visual cue. This warning is shown once.")
            elif reply.startswith("OK"):
                # Logged so the cue and the motion share the ROS timebase in the trial record,
                # and so the cabinet's sequence number is available to spot a dropped cue.
                self._node.get_logger().info(f"Visual cue ({self._label}): {reply}")
            else:
                self._node.get_logger().warn(f"Visual cue rejected: {reply}")
        self._shutdown()

    def _shutdown(self) -> None:
        if self._on_complete is not None:
            self._on_complete()
