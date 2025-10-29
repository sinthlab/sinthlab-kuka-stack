#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# TF for Cartesian pose lookup
import tf2_ros
from geometry_msgs.msg import TransformStamped

from sinthlab_bringup.helpers.common_threshold import DoneGate, DebugTicker, get_param


class ApplePluckImpedanceControlDisplacementNode(Node):
    """
    Cartesian displacement monitor for a robot already in controller-side Cartesian impedance.
    - Waits for move_to_start/done, captures baseline EE pose in base frame.
    - Monitors EE displacement (translation) and shuts down once threshold is exceeded.
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_displacement_control",
            automatically_declare_parameters_from_overrides=True,
        )

        # Parameters
        self._update_rate = int(get_param(self, "update_rate", 200))
        self._dt = 1.0 / float(self._update_rate)

        self._start_on_done_topic = bool(get_param(self, "start_on_done_topic", True))
        self._done_topic = str(get_param(self, "done_topic", "move_to_start/done"))

        # Cartesian frames
        self._base_frame = str(get_param(self, "base_frame", "lbr_link_0"))
        self._ee_frame = str(get_param(self, "ee_frame", "lbr_link_ee"))

        # Threshold
        self._disp_axis = str(get_param(self, "cartesian_axis", "z")).lower()  # x|y|z|norm
        self._disp_threshold_m = float(get_param(self, "cartesian_displacement_threshold_m", 0.01))

        # Debug
        dbg_rate = float(get_param(self, "debug_log_rate_hz", 2.0))
        self._dbg = DebugTicker(dbg_rate)

        # State
        self._baseline: Optional[TransformStamped] = None
        self._ready = not self._start_on_done_topic
        self._stopping = False

        # Done gating
        self._done_gate = DoneGate(self, self._done_topic) if self._start_on_done_topic else None

        # TF buffer/listener
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Timer
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f"Displacement monitor started: rate={self._update_rate}Hz, base='{self._base_frame}', ee='{self._ee_frame}', axis='{self._disp_axis}', thr={self._disp_threshold_m} m"
        )

    # ------------------------- Helpers -------------------------
    def _lookup(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return None

    @staticmethod
    def _axis_disp_m(ts_now: TransformStamped, ts_base: TransformStamped, axis: str) -> float:
        dx = ts_now.transform.translation.x - ts_base.transform.translation.x
        dy = ts_now.transform.translation.y - ts_base.transform.translation.y
        dz = ts_now.transform.translation.z - ts_base.transform.translation.z
        if axis == "x":
            return abs(dx)
        if axis == "y":
            return abs(dy)
        if axis == "z":
            return abs(dz)
        # default norm
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    # ------------------------- Timer loop -------------------------
    def _step(self) -> None:
        # Ready gating on done topic
        if not self._ready:
            if self._done_gate and self._done_gate.done:
                self._ready = True
            else:
                return

        # Ensure baseline
        if self._baseline is None:
            ts = self._lookup()
            if ts is not None:
                self._baseline = ts
                self.get_logger().info("Captured baseline EE pose for displacement thresholding")
            return

        # Evaluate displacement
        ts_now = self._lookup()
        if ts_now is None:
            if self._dbg.tick(self._dt):
                self.get_logger().warn("TF lookup failed; waiting for transform")
            return

        disp = self._axis_disp_m(ts_now, self._baseline, self._disp_axis)
        if self._dbg.tick(self._dt):
            self.get_logger().info(f"EE disp={disp:.4f} m (axis={self._disp_axis}, thr={self._disp_threshold_m:.4f} m)")

        if not self._stopping and disp >= self._disp_threshold_m:
            self._stopping = True
            self.get_logger().info(
                f"Displacement threshold reached: value={disp:.4f} m >= {self._disp_threshold_m:.4f} m. Stopping server."
            )
            try:
                self.destroy_node()
            finally:
                rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApplePluckImpedanceControlDisplacementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
