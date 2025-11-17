#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from actions.audio_cue import AudioCue
from helpers.common_threshold import DoneGate, get_required_param


class AudioCuePlayNode(Node):
    """ROS 2 wrapper that plays up to two gated audio cues before shutting down."""

    def __init__(self) -> None:
        super().__init__(
            "audio_cue_play",
            automatically_declare_parameters_from_overrides=True,
        )
        
        self._finished = False
        
        self._start_gate_topic = str(get_required_param(self, "start_gate_topic"))
        self._start_gate = DoneGate(self, self._start_gate_topic)

        self._hold_gate_topic = str(get_required_param(self, "hold_gate_topic"))
        self._hold_gate = DoneGate(self, self._hold_gate_topic)
        self._start_action = AudioCue(
            self,
            to_start=self._start_gate_done,
            on_complete=self._on_start_action_complete,
        )
        
        self._hold_action: Optional[AudioCue] = None

    def _start_gate_done(self) -> bool:
        return self._start_gate.done

    def _hold_gate_done(self) -> bool:
        return bool(self._hold_gate and self._hold_gate.done)

    def _on_start_action_complete(self) -> None:
        self._start_action = None
        if self._hold_gate is not None:
            if self._hold_action is None:
                self._hold_action = AudioCue(
                    self,
                    to_start=self._hold_gate_done,
                    on_complete=self._on_hold_action_complete,
                )
        else:
            self._finalize()

    def _on_hold_action_complete(self) -> None:
        self._hold_action = None
        self._finalize()

    def _finalize(self) -> None:
        if self._finished:
            return
        self._finished = True
        self.get_logger().info("Audio cue playback sequence complete; shutting down.")
        try:
            self.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AudioCuePlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
