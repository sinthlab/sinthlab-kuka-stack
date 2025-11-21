#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

from helpers.common_threshold import DoneGate, create_transient_bool_publisher, get_required_param


class ForceTorqueBiasCalibrator(Node):
    """Capture an average wrench after a start gate and publish the bias with gating."""

    def __init__(self) -> None:
        super().__init__(
            "force_torque_bias_calibrator",
            automatically_declare_parameters_from_overrides=True,
        )

        self._wrench_topic = str(get_required_param(self, "wrench_topic"))
        self._start_gate_topic = str(get_required_param(self, "start_gate_topic"))
        self._bias_topic = str(get_required_param(self, "bias_topic"))
        self._done_topic = str(get_required_param(self, "done_topic"))
        self._duration = Duration(seconds=float(get_required_param(self, "calibration_duration_sec")))
        self._timeout = Duration(seconds=float(get_required_param(self, "calibration_timeout_sec")))
        self._subscriber_latch_delay = float(get_required_param(self, "subscriber_latch_delay_sec"))
        self._debug_log_enabled = bool(get_required_param(self, "debug_log_enabled"))

        self._start_gate = DoneGate(self, self._start_gate_topic)

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._bias_pub = self.create_publisher(Wrench, self._bias_topic, qos)
        self._done_pub = create_transient_bool_publisher(self, self._done_topic)

        self._collecting = False
        self._completed = False
        self._collection_start: Optional[Time] = None
        self._samples_sum = np.zeros(6, dtype=float)
        self._sample_count = 0

        self._wrench_sub = self.create_subscription(WrenchStamped, self._wrench_topic, self._on_wrench, 10)
        self._timer = self.create_timer(0.05, self._on_timer)

        self.get_logger().info(
            f"Waiting for start gate on '{self._start_gate_topic}' to calibrate force-torque bias."
        )

    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        if self._completed:
            return

        if not self._collecting:
            if self._start_gate.done:
                self._collecting = True
                self._collection_start = self.get_clock().now()
                self._samples_sum[:] = 0.0
                self._sample_count = 0
                self.get_logger().info(
                    f"Start gate received; sampling '{self._wrench_topic}' for {self._duration.nanoseconds / 1e9:.2f}s."
                )
            return

        assert self._collection_start is not None
        now = self.get_clock().now()
        elapsed = now - self._collection_start

        if elapsed >= self._duration:
            self._finalise_calibration(reason="duration met")
            return

        if elapsed >= self._timeout and self._sample_count == 0:
            self.get_logger().warn(
                f"No wrench samples received within timeout ({self._timeout.nanoseconds / 1e9:.2f}s); publishing zero bias."
            )
            self._finalise_calibration(reason="timeout")

    # ------------------------------------------------------------------
    def _on_wrench(self, msg: WrenchStamped) -> None:
        if not self._collecting:
            return

        values = np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ],
            dtype=float,
        )
        self._samples_sum += values
        self._sample_count += 1

        if self._debug_log_enabled and (self._sample_count % 50 == 0):
            self.get_logger().info(
                f"Collected {self._sample_count} wrench samples while calibrating bias."
            )

    # ------------------------------------------------------------------
    def _finalise_calibration(self, *, reason: str) -> None:
        if self._completed:
            return

        bias = np.zeros(6, dtype=float)
        if self._sample_count > 0:
            bias = self._samples_sum / float(self._sample_count)
        else:
            self.get_logger().warn("Bias calibration completed with zero samples; publishing zero bias.")

        self._completed = True
        self._collecting = False
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        bias_msg = Wrench()
        bias_msg.force.x, bias_msg.force.y, bias_msg.force.z = bias[0:3]
        bias_msg.torque.x, bias_msg.torque.y, bias_msg.torque.z = bias[3:6]

        self._bias_pub.publish(bias_msg)
        self.get_logger().info(
            f"Published force-torque bias ({reason}): "
            f"Fx={bias[0]:.3f}, Fy={bias[1]:.3f}, Fz={bias[2]:.3f}, "
            f"Tx={bias[3]:.3f}, Ty={bias[4]:.3f}, Tz={bias[5]:.3f}."
        )

        self._done_pub.publish(Bool(data=True))
        self.get_logger().info(
            f"Published calibration completion gate on '{self._done_topic}'."
        )

        # allow latched publishers to propagate
        time.sleep(self._subscriber_latch_delay)
        self._shutdown()

    # ------------------------------------------------------------------
    def _shutdown(self) -> None:
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForceTorqueBiasCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
