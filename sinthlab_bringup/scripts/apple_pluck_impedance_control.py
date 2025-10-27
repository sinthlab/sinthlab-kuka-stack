#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class ApplePluckImpedanceControlNode(Node):
    """
    Minimal force threshold monitor for a robot already in controller-side Cartesian impedance.
    - Subscribes to ForceTorqueSensorBroadcaster (WrenchStamped), reads EE force.z.
    - Monitors absolute Z-axis force; shuts down once threshold is exceeded.
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

        # Optional gating on a done topic (published by move_to_start)
        self._start_on_done_topic = bool(_param("start_on_done_topic", True))
        self._done_topic = str(_param("done_topic", "move_to_start/done"))

        # Force threshold parameter (absolute EE Z force)
        self._pull_force_threshold = float(_param("pull_force_threshold", 80.0))  # [N], EE Z axis

        # Debug logging for tuning
        self._debug_log_enabled = bool(_param("debug_log_enabled", True))
        self._debug_log_rate_hz = float(_param("debug_log_rate_hz", 2.0))  # logs per second
        self._debug_log_accum = 0.0

        # State
        self._init = False
        self._stopping = False
        self._last_fz_raw = None  # type: Optional[float]
        self._done_received = not self._start_on_done_topic

        # ROS I/O
        self._ws_sub = self.create_subscription(WrenchStamped, self._wrench_topic, self._on_wrench, 10)
        self.get_logger().info(f"Subscribed to wrench topic: '{self._wrench_topic}'")
        if self._start_on_done_topic:
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = ReliabilityPolicy.RELIABLE
            self._done_sub = self.create_subscription(Bool, self._done_topic, self._on_done, qos)
            self.get_logger().info(
                f"Waiting for move-to-start done on topic: '{self._done_topic}' before starting force monitoring"
            )
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

    def _on_done(self, msg: Bool) -> None:
        try:
            if bool(msg.data):
                if not self._done_received:
                    self.get_logger().info("Received move-to-start done signal; starting force monitoring")
                self._done_received = True
        except Exception:
            pass

    # ------------------------- Timer loop -------------------------
    def _step(self) -> None:
        # Optionally wait for done signal
        if not self._done_received:
            return
        if not self._init:
            return
        fz = self._last_fz_raw
        if fz is None:
            return

        fz_abs = abs(fz)

        # Low-rate debug log for threshold tuning
        if self._debug_log_enabled:
            self._debug_log_accum += self._dt
            period = 1.0 / max(self._debug_log_rate_hz, 1e-3)
            if self._debug_log_accum >= period:
                self._debug_log_accum = 0.0
                try:
                    self.get_logger().info(
                        f"Fz raw={fz:.2f} N, abs={fz_abs:.2f} N, thr={self._pull_force_threshold:.2f} N"
                    )
                except Exception:
                    pass

        if not self._stopping and fz_abs >= self._pull_force_threshold:
            self._stopping = True
            self.get_logger().info(
                f"Force threshold reached on EE Z: value={fz_abs:.2f} N, threshold={self._pull_force_threshold:.2f} N. Stopping server."
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