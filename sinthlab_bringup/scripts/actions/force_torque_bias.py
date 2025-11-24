#!/usr/bin/env python3
from __future__ import annotations

from typing import Callable, Optional

import time
import numpy as np
import optas

from geometry_msgs.msg import Wrench
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node as rclpyNode
from rclpy.time import Time

from lbr_fri_idl.msg import LBRState

from helpers.common_threshold import get_required_param


class ForceTorqueBias:
    """Sample external torques post move-to-start, map them to a wrench, and publish the averaged bias."""

    def __init__(
        self,
        node: rclpyNode,
        *,
        to_start: Callable[[], bool],
        on_complete: Callable[[np.ndarray], None],
    ) -> None:
        self._node = node
        self._to_start = to_start
        self._on_complete = on_complete

        self._collecting = False
        self._completed = False
        self._settle_complete = False
        self._collection_start: Optional[Time] = None
        self._sum = np.zeros(6, dtype=float)
        self._count = 0

        self._lbr_state_topic = str(get_required_param(node, "lbr_state_topic"))
        self._bias_topic = str(get_required_param(node, "bias_topic"))
        self._sample_duration = Duration(seconds=float(get_required_param(node, "calibration_duration_sec")))
        self._total_timeout = Duration(seconds=float(get_required_param(node, "calibration_timeout_sec")))
        # optional quiet window after move-to-start before collecting torque samples
        self._settle_duration = Duration(seconds=float(get_required_param(node, "settle_duration_sec")))
        self._subscriber_latch_delay_sec = float(get_required_param(node, "subscriber_latch_delay_sec"))
        self._debug_log_enabled = bool(get_required_param(node, "debug_log_enabled"))

        robot_description = str(get_required_param(node, "robot_description"))
        base_link = str(get_required_param(node, "base_link"))
        end_effector_link = str(get_required_param(node, "end_effector_link"))

        self._robot = optas.RobotModel(urdf_string=robot_description)
        self._jacobian_func = self._robot.get_link_geometric_jacobian_function(
            link=end_effector_link,
            base_link=base_link,
            numpy_output=True,
        )

        bias_qos = QoSProfile(depth=1)
        bias_qos.reliability = QoSReliabilityPolicy.RELIABLE
        bias_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._bias_pub = node.create_publisher(Wrench, self._bias_topic, bias_qos)

        self._lbr_state_sub = node.create_subscription(LBRState, self._lbr_state_topic, self._on_lbr_state, 10)
        self._timer = node.create_timer(0.05, self._step)

        self._node.get_logger().info(
            f"Waiting for bias gate; monitoring '{self._lbr_state_topic}'."
        )

    def _step(self) -> None:
        if self._completed:
            return

        if not self._collecting:
            if not self._to_start():
                return
            self._collecting = True
            self._collection_start = self._node.get_clock().now()
            self._settle_complete = self._settle_duration.nanoseconds == 0
            self._sum[:] = 0.0
            self._count = 0
            if self._settle_complete:
                self._node.get_logger().info(
                    f"Bias capture started; averaging wrench for {self._sample_duration.nanoseconds / 1e9:.2f}s."
                )
            else:
                self._node.get_logger().info(
                    f"Bias gate open; waiting {self._settle_duration.nanoseconds / 1e9:.2f}s before sampling."
                )
            return

        assert self._collection_start is not None
        now = self._node.get_clock().now()
        elapsed = now - self._collection_start

        if not self._settle_complete and elapsed >= self._settle_duration:
            self._settle_complete = True
            self._sum[:] = 0.0
            self._count = 0
            self._node.get_logger().info(
                f"Bias capture started; averaging wrench for {self._sample_duration.nanoseconds / 1e9:.2f}s."
            )

        if self._settle_complete:
            sample_elapsed = elapsed.nanoseconds - self._settle_duration.nanoseconds
            if sample_elapsed >= self._sample_duration.nanoseconds:
                self._finalise("duration reached")
                return

        if elapsed >= self._total_timeout and self._count == 0:
            self._finalise("timeout with no samples")

    def _on_lbr_state(self, msg: LBRState) -> None:
        if not self._collecting or self._completed or not self._settle_complete:
            return

        q = np.array(msg.measured_joint_position.tolist(), dtype=float)
        tau_ext = np.array(msg.external_torque.tolist(), dtype=float)

        jacobian = self._jacobian_func(q)
        jacobian_inv = np.linalg.pinv(jacobian, rcond=0.1)
        wrench = jacobian_inv.T @ tau_ext

        self._sum += wrench
        self._count += 1

        if self._debug_log_enabled and self._count % 50 == 0:
            self._node.get_logger().info(
                f"Collected {self._count} torque samples while averaging bias."
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
