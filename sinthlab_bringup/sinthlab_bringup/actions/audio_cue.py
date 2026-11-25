#!/usr/bin/env python3
from __future__ import annotations

import base64
import queue
import select
import subprocess
import threading
import time
from typing import Callable, Optional

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.node_utils import get_required_param


# Keep one Windows audio process ready so starting a tone does not spawn PowerShell.
_BEEP_LOOP = """
$ready = $true
try { [Console]::Beep(37, 10) } catch { $ready = $false }
while ($null -ne ($line = [Console]::ReadLine())) {
    $values = $line.Split(' ')
    try {
        if (-not $ready) { throw 'Audio startup failed' }
        [Console]::WriteLine('START')
        [Console]::Beep([int]$values[0], [int]$values[1])
        [Console]::WriteLine('OK')
    } catch {
        [Console]::WriteLine('ERROR')
    }
}
"""
_BEEP_COMMAND = base64.b64encode(_BEEP_LOOP.encode("utf-16le")).decode("ascii")
_MAX_QUEUE_AGE_SEC = 0.15
_TIMING_CUES = {"audio_cue_play", "audio_cue_end"}


class _BeepPlayer:
    """Run Windows tones off the ROS executor. Drop a tone that would start late."""

    def __init__(self):
        self.jobs = queue.SimpleQueue()
        threading.Thread(target=self._run, name="audio_cues", daemon=True).start()

    def _run(self):
        proc = None
        while True:
            queued_at, frequency, duration, started, finished, log = self.jobs.get()
            try:
                if proc is None or proc.poll() is not None:
                    proc = subprocess.Popen(
                        ["powershell.exe", "-NoProfile", "-NonInteractive", "-EncodedCommand", _BEEP_COMMAND],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0,
                    )
                if frequency is None:
                    continue
                if time.monotonic() - queued_at > _MAX_QUEUE_AGE_SEC:
                    log.warn("Audio cue skipped because playback was delayed.")
                    continue
                proc.stdin.write(f"{frequency} {duration}\n".encode("ascii"))
                proc.stdin.flush()
                if (not select.select([proc.stdout], [], [], 1.0)[0]
                        or proc.stdout.readline().strip() != b"START"):
                    raise OSError("Windows audio did not start playback")
                if started is not None:
                    # The worker enters Windows Beep immediately after sending START.
                    # This timestamp is a software estimate of acoustic onset.
                    started()
                timeout = duration / 1000.0 + 3.0
                if not select.select([proc.stdout], [], [], timeout)[0]:
                    raise TimeoutError("Windows audio did not respond")
                if proc.stdout.readline().strip() != b"OK":
                    raise OSError("Windows audio did not confirm playback")
            except Exception as exc:
                if log is not None:
                    log.warn(f"Console beep failed: {exc}")
                if proc is not None:
                    if proc.poll() is None:
                        proc.kill()
                    proc.wait()
                    proc = None
                continue
            if finished is not None:
                try:
                    finished(time.monotonic() - queued_at)
                except Exception as exc:
                    log.warn(f"Audio cue completion callback failed: {exc}")


class AudioCue:
    """Queue a tone and continue the trial without waiting for playback."""

    _players: dict[str, _BeepPlayer] = {}

    @classmethod
    def warmup(cls, channel="audio_cue_play") -> None:
        """Start the shared Windows player while the experiment initializes."""
        if channel not in cls._players:
            player = cls._players[channel] = _BeepPlayer()
            player.jobs.put((time.monotonic(), None, None, None, None, None))

    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Callable[[], None],
                 on_finished: Optional[Callable[[float], None]] = None) -> None:
        self._on_finished = on_finished
        self._channel = param_prefix.rstrip(".")
        if self._channel in _TIMING_CUES:
            self.warmup(self._channel)
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self.reload()

    def reload(self) -> None:
        """Re-read this cue's parameters. All three are live: an orchestrator calls this at a trial
        boundary after a change (see helpers/live_params.py).

        `audio_cue.enabled` is one switch for every beep. Off, start() stays silent but still calls
        on_complete, so the trial sequence is unchanged -- only on_finished (the measured end of
        the sound) does not fire, because there is no sound to measure."""
        node = self._node
        self._frequency = int(get_required_param(node, self._param_prefix + "frequency_hz"))
        self._duration = int(get_required_param(node, self._param_prefix + "duration_ms"))
        self._enabled = bool(get_required_param(node, "audio_cue.enabled"))

    @classmethod
    def play_early_touch(cls, node: rclpyNode) -> None:
        cls._players["early_touch"].jobs.put((time.monotonic(), 220, 350, None, None, node.get_logger()))

    def enabled(self) -> bool:
        return self._enabled

    def start(self, *, on_started: Optional[Callable[[], None]] = None) -> None:
        if self._enabled:
            if self._channel in _TIMING_CUES:
                self._players[self._channel].jobs.put((time.monotonic(), self._frequency, self._duration,
                                                      on_started, self._on_finished, self._node.get_logger()))
            else:
                # Keep task-specific feedback independent of the go tone.
                try:
                    subprocess.Popen(["powershell.exe", "-NoProfile", "-Command",
                                      f"[console]::Beep({self._frequency},{self._duration})"])
                except OSError as exc:
                    self._node.get_logger().warn(f"Console beep failed: {exc}")
        if self._on_complete is not None:
            self._on_complete()
