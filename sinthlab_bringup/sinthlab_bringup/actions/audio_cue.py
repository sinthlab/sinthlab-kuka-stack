#!/usr/bin/env python3
from __future__ import annotations

import subprocess
from typing import Callable

from rclpy.node import Node as rclpyNode
import rclpy

from sinthlab_bringup.helpers.common_threshold import get_required_param


class AudioCue:
    """Plays a single audio cue when start() is called."""

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
            # This is a blocking call
            subprocess.run(
                [
                    "powershell.exe",
                    "-NoProfile",
                    "-Command",
                    f"[console]::Beep({self._frequency},{self._duration})"
                ],
                check=True,
            )
        except Exception as exc:
            self._node.get_logger().warn(f"Console beep failed: {exc}")

    def _shutdown(self) -> None:
        if self._on_complete is not None:
            self._on_complete()
