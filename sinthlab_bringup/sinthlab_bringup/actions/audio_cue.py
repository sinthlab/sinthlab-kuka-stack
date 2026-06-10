#!/usr/bin/env python3
from __future__ import annotations

import subprocess
from typing import Callable, Optional

from rclpy.node import Node as rclpyNode
import rclpy

from sinthlab_bringup.helpers.common_threshold import get_required_param


class AudioCue:
    """Plays a single audio cue when start() is called."""

    @staticmethod
    def warmup(node: Optional[rclpyNode] = None) -> None:
        """Wake the Windows/WSL2 audio driver once at startup so the first real cue isn't delayed.

        One-shot side effect (a near-inaudible 37 Hz / 10 ms beep); safe no-op on non-WSL2 hosts.
        Orchestrators call this once in __init__ instead of issuing the subprocess inline.
        """
        try:
            subprocess.Popen(["powershell.exe", "-NoProfile", "-Command", "[console]::Beep(37, 10)"])
        except Exception:
            if node is not None:
                node.get_logger().debug("Audio warmup beep failed (non-WSL2 host?).")

    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Callable[[], None]) -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._frequency = int(get_required_param(node, self._param_prefix + "frequency_hz"))
        self._duration = int(get_required_param(node, self._param_prefix + "duration_ms"))

        self._played = False

    def start(self) -> None:
        if self._played:
            self._played = False # allow replay
        self._play_sound()
        self._shutdown()
    
    # This is a very specific implementation for WSL2
    # using powershell to play a beep sound.
    # For other OSes, this method should be modified accordingly.
    def _play_sound(self) -> None:
        try:
            # Popen is non-blocking
            subprocess.Popen(
                [
                    "powershell.exe",
                    "-NoProfile",
                    "-Command",
                    f"[console]::Beep({self._frequency},{self._duration})"
                ]
            )
        except Exception as exc:
            self._node.get_logger().warn(f"Console beep failed: {exc}")

    def _shutdown(self) -> None:
        if self._on_complete is not None:
            self._on_complete()
