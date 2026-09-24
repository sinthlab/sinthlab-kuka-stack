#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import time
import threading
from typing import Callable, Optional

from rclpy.node import Node as rclpyNode
import rclpy

from sinthlab_bringup.helpers.common_threshold import get_optional_param, get_required_param


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

    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Callable[[], None],
                 on_finished: Callable[[float], None] = None) -> None:
        # on_finished(duration_s) fires when the beep process EXITS -- i.e. when the sound actually
        # stopped. `[console]::Beep` blocks for exactly duration_ms, so the sound's START is
        # (finish - duration_ms) and can be recovered from it.
        #
        # This matters because on_complete() fires the instant Popen returns, which on WSL2 is
        # ~49 ms in and roughly 290 ms BEFORE any sound comes out (the Windows process itself takes
        # ~340 ms to spawn). Nothing else in the system observes the audio at all, so without this
        # the trial record has no idea when the animal was actually cued.
        self._on_finished = on_finished
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self.reload()
        self._played = False

    def reload(self) -> None:
        """Re-read this cue's parameters. All three are live: an orchestrator calls this at a trial
        boundary after a change (see helpers/live_params.py).

        `audio_cue.enabled` is one switch for every beep. Off, start() stays silent but still calls
        on_complete, so the trial sequence is unchanged -- only on_finished (the measured end of
        the sound) does not fire, because there is no sound to measure."""
        node = self._node
        self._frequency = int(get_required_param(node, self._param_prefix + "frequency_hz"))
        self._duration = int(get_required_param(node, self._param_prefix + "duration_ms"))
        self._enabled = bool(get_optional_param(node, "audio_cue.enabled", True))

    def start(self) -> None:
        if self._played:
            self._played = False # allow replay
        if self._enabled:
            self._play_sound()
        self._shutdown()
    
    # This is a very specific implementation for WSL2
    # using powershell to play a beep sound.
    # For other OSes, this method should be modified accordingly.
    def _play_sound(self) -> None:
        try:
            # Popen is non-blocking
            proc = subprocess.Popen(
                [
                    "powershell.exe",
                    "-NoProfile",
                    "-Command",
                    f"[console]::Beep({self._frequency},{self._duration})"
                ]
            )
            if self._on_finished is not None:
                # Wait off-thread: the orchestrator must never block on a sound.
                t0 = time.monotonic()
                threading.Thread(target=self._await_exit, args=(proc, t0),
                                 name="audio_cue_wait", daemon=True).start()
        except Exception as exc:
            self._node.get_logger().warn(f"Console beep failed: {exc}")

    def _await_exit(self, proc, t0: float) -> None:
        try:
            proc.wait(timeout=30.0)
        except Exception:
            return
        try:
            self._on_finished(time.monotonic() - t0)
        except Exception as exc:
            self._node.get_logger().warn(f"audio on_finished failed: {exc}")

    def _shutdown(self) -> None:
        if self._on_complete is not None:
            self._on_complete()
