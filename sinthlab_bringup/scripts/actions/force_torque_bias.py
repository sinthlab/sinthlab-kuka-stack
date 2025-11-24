#!/usr/bin/env python3
from __future__ import annotations

from typing import Callable, Optional

import time
import numpy as np

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Wrench
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node as rclpyNode
from rclpy.time import Time

from helpers.common_threshold import get_required_param

class ForceTorqueBias:
    """
    1) Collect wrench samples and averages them once the move-to-start gate opens
    2) publish the bias in the specified topic.
    Publishes a 'done' signal once the bias is published.
    This is done for force correction in subsequent admittance control
    (which is otherwise affected by the older State data causing arm drift).
    """

    def __init__(self, node: rclpyNode, *, to_start: Callable[[], None], on_complete: Callable[[], None]) -> None:
        self._node = node
        self._to_start = to_start
        self._on_complete = on_complete

        self._collecting = False
        self._completed = False
        self._collection_start: Optional[Time] = None
        self._sum = np.zeros(6, dtype=float)
        self._count = 0

        self._wrench_topic = str(get_required_param(node, "wrench_topic"))
        self._bias_topic = str(get_required_param(node, "bias_topic"))
        self._sample_duration = Duration(seconds=float(get_required_param(node, "calibration_duration_sec")))
        self._total_timeout = Duration(seconds=float(get_required_param(node, "calibration_timeout_sec")))
        self._subscriber_latch_delay_sec = float(get_required_param(node, "subscriber_latch_delay_sec"))
        self._debug_log_enabled = bool(get_required_param(node, "debug_log_enabled"))


        bias_qos = QoSProfile(depth=1)
        bias_qos.reliability = QoSReliabilityPolicy.RELIABLE
        bias_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._bias_pub = node.create_publisher(Wrench, self._bias_topic, bias_qos)

        self._wrench_sub = node.create_subscription(WrenchStamped, self._wrench_topic, self._on_wrench, 10)
        self._timer = node.create_timer(0.05, self._step)

        self._node.get_logger().info(
            f"Waiting for bias gate; monitoring '{self._wrench_topic}'."
        )

    def _step(self) -> None:
        if self._completed:
            return

        if not self._collecting:
            if not self._to_start():
                return
            self._collecting = True
            self._collection_start = self._node.get_clock().now()
            self._sum[:] = 0.0
            self._count = 0
            self._node.get_logger().info(
                f"Bias capture started; averaging wrench for {self._sample_duration.nanoseconds / 1e9:.2f}s."
            )
            return

        assert self._collection_start is not None
        now = self._node.get_clock().now()
        elapsed = now - self._collection_start

        if elapsed >= self._sample_duration:
            self._finalise("duration reached")
            return

        if elapsed >= self._total_timeout and self._count == 0:
            self._finalise("timeout with no samples")

    def _on_wrench(self, msg: WrenchStamped) -> None:
        if not self._collecting or self._completed:
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
        self._sum += values
        self._count += 1

        if self._debug_log_enabled and self._count % 50 == 0:
            self._node.get_logger().info(
                f"Collected {self._count} wrench samples while averaging bias."
            )

    def _finalise(self, reason: str) -> None:
        if self._completed:
            return

        bias = np.zeros(6, dtype=float)
        if self._count > 0:
            bias = self._sum / float(self._count)
        else:
            self._node.get_logger().warn("Bias calibration produced no samples; using zero bias.")

        self._completed = True
        self._collecting = False

        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        self._node.get_logger().info(
            f"Publishing Force-torque bias ({reason}): Fx={bias[0]:.3f}, Fy={bias[1]:.3f}, Fz={bias[2]:.3f}, Tx={bias[3]:.3f}, Ty={bias[4]:.3f}, Tz={bias[5]:.3f}"
        )
        
        
        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = bias[0:3]
        msg.torque.x, msg.torque.y, msg.torque.z = bias[3:6]
        self._bias_pub.publish(msg)

        self._on_complete(bias)

        time.sleep(self._subscriber_latch_delay_sec)
        self._shutdown()

    # ------------------------------------------------------------------
    def _shutdown(self) -> None:
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
