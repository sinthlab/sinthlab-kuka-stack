#!/usr/bin/env python3
"""Move-to-start perturbation that nudges the arm after baseline capture."""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node as rclpyNode
from std_msgs.msg import Bool

from actions.move_to_position import MoveToPositionAction
from helpers.common_threshold import DoneGate, create_transient_bool_publisher, get_required_param


class PerturbStartNode(rclpyNode):
    """Applies a secondary move-to-start offset once the gating condition opens."""

    def __init__(self) -> None:
        super().__init__(
            "perturb_start",
            automatically_declare_parameters_from_overrides=True,
        )

        self._move_done_topic = str(get_required_param(self, "move_done_topic"))
        self._move_done_pub = create_transient_bool_publisher(self, self._move_done_topic)
        self._done_published = False

        self._start_gate_topic = str(get_required_param(self, "start_gate_topic"))
        self._start_gate = DoneGate(self, self._start_gate_topic)

        self._start_delay_sec = float(get_required_param(self, "start_delay_sec"))
        self._subscriber_latch_delay_sec = float(get_required_param(self, "subscriber_latch_delay_sec"))

        self._delay_start_time: Optional[float] = None
        self._started_published = False
        self._move_started_pub = create_transient_bool_publisher(self, "perturb_move/started")

        self._action = MoveToPositionAction(
            self,
            to_start=self._to_start_action,
            on_complete=self._on_action_complete,
        )

    def _to_start_action(self) -> bool:
        if self._started_published:
            return True
        if not self._start_gate or not self._start_gate.done:
            return False
        if self._start_delay_sec <= 0.0:
            self._started_published = True
            self._move_started_pub.publish(Bool(data=True))
            return True
        if self._delay_start_time is None:
            self._delay_start_time = time.time()
            return False
        
        if (time.time() - self._delay_start_time) >= self._start_delay_sec:
            self._started_published = True
            self._move_started_pub.publish(Bool(data=True))
            return True
        return False

    def _on_action_complete(self) -> None:
        if self._done_published:
            return
        self._done_published = True
        try:
            self._move_done_pub.publish(Bool(data=True))
            self.get_logger().info(
                f"Perturbation move complete; published done=true on {self._move_done_topic}."
            )
            # Use ROS timer instead of time.sleep
            self._shutdown_timer = self.create_timer(self._subscriber_latch_delay_sec, self._shutdown)
        except Exception:
            self.get_logger().warn(
                f"Failed to publish perturbation completion on {self._move_done_topic}; proceeding to shutdown."
            )
            self._shutdown()

    def _shutdown(self) -> None:
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerturbStartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
