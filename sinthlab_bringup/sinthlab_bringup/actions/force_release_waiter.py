#!/usr/bin/env python3
"""Waits until the operator releases the arm (external force ~ 0).

Subscribes to the estimated external wrench (`force_torque_broadcaster/wrench`). When the linear
force magnitude stays below `release_force_threshold_newton` continuously for `release_duration_sec`,
it calls `on_complete`. Used by the maze experiment: after a timeout (or goal), the arm returns to
start only once the operator has let go, so the recovery move doesn't fight the operator.
"""
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from geometry_msgs.msg import WrenchStamped

from sinthlab_bringup.helpers.common_threshold import DebugTicker, get_required_param


class ForceReleaseWaiter:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "",
                 on_complete: Callable[[], None]) -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._ready = False

        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._force_threshold = float(get_required_param(node, self._param_prefix + "release_force_threshold_newton"))
        self._release_duration = float(get_required_param(node, self._param_prefix + "release_duration_sec"))
        self._debug_log_enabled = bool(get_required_param(node, self._param_prefix + "debug_log_enabled"))
        self._dbg = DebugTicker(float(get_required_param(node, self._param_prefix + "debug_log_rate_hz")))

        self._latest_force: Optional[float] = None
        self._below_elapsed = 0.0
        self._done = False

        # The estimated external wrench (chained off estimated_wrench_interface) — relative topic
        # resolves under the node namespace, e.g. /lbr/force_torque_broadcaster/wrench.
        self._sub = node.create_subscription(
            WrenchStamped, "force_torque_broadcaster/wrench", self._on_wrench, 1
        )
        self._timer = node.create_timer(self._dt, self._step)

    def start(self) -> None:
        if self._ready:
            self._node.get_logger().warn("ForceReleaseWaiter is already running.")
            return
        self._below_elapsed = 0.0
        self._done = False
        self._ready = True
        self._node.get_logger().info(
            f"ForceReleaseWaiter: waiting for |force| < {self._force_threshold:.1f} N "
            f"sustained for {self._release_duration:.1f}s..."
        )

    def _on_wrench(self, msg: WrenchStamped) -> None:
        f = msg.wrench.force
        self._latest_force = float(np.sqrt(f.x * f.x + f.y * f.y + f.z * f.z))

    def _step(self) -> None:
        if not self._ready or self._done:
            return
        if self._latest_force is None:
            return  # no wrench yet

        if self._latest_force < self._force_threshold:
            self._below_elapsed += self._dt
        else:
            self._below_elapsed = 0.0

        if self._debug_log_enabled and self._dbg.tick(self._dt):
            self._node.get_logger().info(
                f"|force|={self._latest_force:.2f} N (thr {self._force_threshold:.1f}); "
                f"released {self._below_elapsed:.2f}/{self._release_duration:.1f}s"
            )

        if self._below_elapsed >= self._release_duration:
            self._done = True
            self._ready = False
            self._node.get_logger().info("Arm released — proceeding to recovery.")
            try:
                self._on_complete()
            except Exception as exc:
                self._node.get_logger().error(f"ForceReleaseWaiter on_complete error: {exc}")
