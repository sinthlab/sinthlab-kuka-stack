#!/usr/bin/env python3
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class ApplePluckImpedanceControlNode(Node):
    """
    Minimal force threshold monitor for a robot already in controller-side Cartesian impedance.
    - Subscribes to ForceTorqueSensorBroadcaster (WrenchStamped), reads EE force.z.
    - Monitors Z-axis force with optional EMA smoothing; shuts down once threshold is exceeded.
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_impedance_control",
            automatically_declare_parameters_from_overrides=True,
        )

        # Parameter helper
        def _param(name: str, default):
            try:
                if self.has_parameter(name):
                    p = self.get_parameter(name)
                    return p.value if p is not None else default
            except Exception:
                pass
            return default

        # Timing
        self._update_rate = int(_param("update_rate", 200))
        self._dt = 1.0 / float(self._update_rate)

        # Wrench topic (ForceTorqueSensorBroadcaster)
        self._wrench_topic = str(_param("wrench_topic", "force_torque_broadcaster/wrench"))

        # Force threshold parameters
        self._pull_force_threshold = float(_param("pull_force_threshold", 15.0))  # [N], EE Z axis
        self._pull_force_filter_alpha = float(_param("pull_force_filter_alpha", 0.2))  # EMA 0..1
        self._use_abs_force = bool(_param("pull_force_use_absolute", True))

        # Debug logging for tuning
        self._debug_log_enabled = bool(_param("debug_log_enabled", True))
        self._debug_log_rate_hz = float(_param("debug_log_rate_hz", 2.0))  # logs per second
        self._debug_log_accum = 0.0

        # State
        self._pull_force_filtered = None  # type: Optional[float]
        self._init = False
        self._shutdown_latched = False
        self._last_fz_raw = None  # type: Optional[float]

        # ROS I/O
        self._ws_sub = self.create_subscription(WrenchStamped, self._wrench_topic, self._on_wrench, 10)
        self.get_logger().info(f"Subscribed to wrench topic: '{self._wrench_topic}'")
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f"Force monitor started: rate={self._update_rate}Hz, topic='{self._wrench_topic}', threshold_z={self._pull_force_threshold} N"
        )

    # ------------------------- Callbacks -------------------------
    def _on_wrench(self, msg: WrenchStamped) -> None:
        # Read Z force directly from message
        try:
            fz = float(msg.wrench.force.z)
            self._last_fz_raw = fz
            self._init = True
        except Exception:
            pass

    # ------------------------- Timer loop -------------------------
    def _step(self) -> None:
        if not self._init:
            return
        fz = self._last_fz_raw
        if fz is None:
            return
        # EMA filtering
        if self._pull_force_filtered is None:
            self._pull_force_filtered = fz
        else:
            a = float(np.clip(self._pull_force_filter_alpha, 0.0, 1.0))
            self._pull_force_filtered = a * fz + (1.0 - a) * self._pull_force_filtered

        comp = abs(self._pull_force_filtered) if self._use_abs_force else self._pull_force_filtered

        # Low-rate debug log for threshold tuning
        if self._debug_log_enabled:
            self._debug_log_accum += self._dt
            period = 1.0 / max(self._debug_log_rate_hz, 1e-3)
            if self._debug_log_accum >= period:
                self._debug_log_accum = 0.0
                mode = "abs" if self._use_abs_force else "direct"
                try:
                    self.get_logger().info(
                        f"Fz raw={fz:.2f} N, filt={self._pull_force_filtered:.2f} N, {mode}={comp:.2f}, thr={self._pull_force_threshold:.2f} N"
                    )
                except Exception:
                    # tolerate formatting errors if values are not finite yet
                    pass
        if not self._shutdown_latched and comp >= self._pull_force_threshold:
            self._shutdown_latched = True
            self.get_logger().info(
                f"Force threshold reached on EE Z: value={self._pull_force_filtered:.2f} N, threshold={self._pull_force_threshold:.2f} N. Stopping server."
            )
            # Clean shutdown
            try:
                self.destroy_node()
            finally:
                rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApplePluckImpedanceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # In case shutdown was not triggered from within the node
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()