#!/usr/bin/env python3
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node as rclpyNode
from std_msgs.msg import Bool

from actions.force_torque_bias import ForceTorqueBias
from helpers.common_threshold import DoneGate, create_transient_bool_publisher, get_required_param


class ForceTorqueBiasCalibratorNode(rclpyNode):
    """ROS 2 wrapper around the ForceTorqueBias action."""

    def __init__(self) -> None:
        super().__init__(
            "force_torque_bias_calibrator",
            automatically_declare_parameters_from_overrides=True,
        )

        self._start_gate_topic = str(get_required_param(self, "start_gate_topic"))
        self._start_gate = DoneGate(self, self._start_gate_topic)

        self._done_topic = str(get_required_param(self, "done_topic"))
        self._done_pub = create_transient_bool_publisher(self, self._done_topic)

        self._action = ForceTorqueBias(
            self,
            to_start=self._start_gate_done,
            on_complete=self._on_action_complete,
        )

        self.get_logger().info(
            f"Waiting for start gate '{self._start_gate_topic}' before capturing force/torque bias."
        )

    # ------------------------------------------------------------------
    def _start_gate_done(self) -> bool:
        return self._start_gate.done

    def _on_action_complete(self, bias: np.ndarray) -> None:
        self._done_pub.publish(Bool(data=True))
        self.get_logger().info(f"Bias calibration complete; emitted '{self._done_topic}'.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForceTorqueBiasCalibratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
