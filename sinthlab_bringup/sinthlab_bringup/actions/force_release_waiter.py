#!/usr/bin/env python3
"""Waits until the operator releases the arm (external force ~ 0).

Subscribes to the estimated external wrench (`force_torque_broadcaster/wrench`). When the linear
force magnitude stays below `release_force_threshold_newton` continuously for `release_duration_sec`,
it calls `on_complete`. Used by the maze experiment: after a timeout (or goal), the arm returns to
start only once the operator has let go, so the recovery move doesn't fight the operator.
"""
from __future__ import annotations

import time
from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from geometry_msgs.msg import WrenchStamped

from sinthlab_bringup.helpers.node_utils import DebugTicker, get_required_param


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

        self._low_since: Optional[float] = None
        self._last_sample: Optional[float] = None
        self._done = False

        # The estimated external wrench (chained off estimated_wrench_interface) — relative topic
        # resolves under the node namespace, e.g. /lbr/force_torque_broadcaster/wrench.
        self._sub = node.create_subscription(
            WrenchStamped, "force_torque_broadcaster/wrench", self._on_wrench, 1
        )

    def start(self) -> None:
        if self._ready:
            self._node.get_logger().warn("ForceReleaseWaiter is already running.")
            return
        self._low_since = self._last_sample = None
        self._done = False
        self._ready = True
        self._node.get_logger().info(
            f"ForceReleaseWaiter: waiting for |force| < {self._force_threshold:.1f} N "
            f"sustained for {self._release_duration:.1f}s..."
        )

    def stop(self) -> None:
        """Stop waiting, without calling on_complete (Stop)."""
        self._ready = False

    def _on_wrench(self, msg: WrenchStamped) -> None:
        if not self._ready or self._done:
            return
        now = time.monotonic()
        if self._last_sample is None or now - self._last_sample > max(0.05, 3 * self._dt):
            self._low_since = None
        elapsed = 0.0 if self._last_sample is None else now - self._last_sample
        self._last_sample = now
        f = msg.wrench.force
        force = float(np.linalg.norm([f.x, f.y, f.z]))
        if not force < self._force_threshold:
            self._low_since = None
            return
        if self._low_since is None:
            self._low_since = now
        duration = now - self._low_since
        if self._debug_log_enabled and self._dbg.tick(elapsed):
            self._node.get_logger().info(f"Release force {force:.2f} N for {duration:.2f}s")
        if duration >= self._release_duration:
            self._done = True
            self._ready = False
            self._node.get_logger().info("Arm released — proceeding to park.")
            try:
                self._on_complete()
            except Exception as exc:
                self._node.get_logger().error(f"ForceReleaseWaiter on_complete error: {exc}")
