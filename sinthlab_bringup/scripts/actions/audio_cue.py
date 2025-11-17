#!/usr/bin/env python3
from __future__ import annotations

import subprocess
from typing import Callable

from rclpy.node import Node
import rclpy

from helpers.common_threshold import get_required_param


class AudioCue:
	"""Plays a single audio cue when the gate provided by ``to_start`` opens."""

	def __init__(self, node: Node, *, to_start: Callable[[], None], on_complete: Callable[[], None]) -> None:
		self._node = node
		self._to_start = to_start
		self._on_complete = on_complete

		self._frequency = int(get_required_param(node, "frequency_hz"))
		self._duration = int(get_required_param(node, "duration_ms"))
	
		self._played = False
		self._timer = node.create_timer(0.05, self._step)

	def _step(self) -> None:
		if self._played:
			return
		if not self._to_start():
			return
		self._played = True
		self._play_sound()
		self._finish()
    
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

	def _finish(self) -> None:
		if self._timer is not None:
			self._timer.cancel()
			self._timer = None
		if self._on_complete is not None:
			self._on_complete()
		else:
			self._node.destroy_node()
			if rclpy.ok():
				rclpy.shutdown()
