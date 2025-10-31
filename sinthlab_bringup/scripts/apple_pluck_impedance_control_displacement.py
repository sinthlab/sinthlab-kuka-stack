#!/usr/bin/env python3
from typing import Optional

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# TF for Cartesian pose lookup
import tf2_ros
from geometry_msgs.msg import TransformStamped, WrenchStamped
from std_msgs.msg import Bool

from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand
from helpers.common_threshold import DoneGate, DebugTicker, get_required_param
from helpers.param_logging import log_params_once

class ApplePluckImpedanceControlDisplacementNode(Node):
    """
    Cartesian displacement monitor for a robot already in controller-side Cartesian impedance.
    - Waits for move_to_start/done, captures baseline EE pose in base frame.
    - Once displacement threshold is exceeded, holds joint position and monitors wrench magnitude.
    - Publishes a force-release done event after forces stay below the configured threshold long enough.
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_impedance_control_displacement",
            automatically_declare_parameters_from_overrides=True,
        )

        # Parameters (must be provided via YAML or overrides)
        self._update_rate = int(get_required_param(self, "update_rate"))
        self._dt = 1.0 / float(self._update_rate)

        self._start_on_done_topic = bool(get_required_param(self, "start_on_done_topic"))
        self._done_topic = str(get_required_param(self, "done_topic"))

        # Cartesian frames
        self._base_frame = str(get_required_param(self, "base_frame"))
        self._ee_frame = str(get_required_param(self, "ee_frame"))

        # Thresholds and topics
        self._disp_axis = str(get_required_param(self, "cartesian_axis")).lower()  # x|y|z|norm
        self._disp_threshold_m = float(get_required_param(self, "cartesian_displacement_threshold_m"))
        self._state_topic = str(get_required_param(self, "state_topic"))
        self._command_topic = str(get_required_param(self, "command_topic"))
        self._wrench_topic = str(get_required_param(self, "wrench_topic"))
        self._force_release_threshold = float(get_required_param(self, "force_release_threshold_newton"))
        self._force_release_duration = float(get_required_param(self, "force_release_duration_sec"))
        self._release_done_topic = str(get_required_param(self, "force_release_done_topic"))

        # Debug
        self._debug_log_enabled = bool(get_required_param(self, "debug_log_enabled"))
        dbg_rate = float(get_required_param(self, "debug_log_rate_hz"))
        self._dbg = DebugTicker(dbg_rate)

        # One-time parameter dump, when debug enabled
        if self._debug_log_enabled:
            log_params_once(
                self,
                params={
                    "update_rate": self._update_rate,
                    "start_on_done_topic": self._start_on_done_topic,
                    "done_topic": self._done_topic,
                    "base_frame": self._base_frame,
                    "ee_frame": self._ee_frame,
                    "cartesian_axis": self._disp_axis,
                    "cartesian_displacement_threshold_m": self._disp_threshold_m,
                    "state_topic": self._state_topic,
                    "command_topic": self._command_topic,
                    "wrench_topic": self._wrench_topic,
                    "force_release_threshold_newton": self._force_release_threshold,
                    "force_release_duration_sec": self._force_release_duration,
                    "force_release_done_topic": self._release_done_topic,
                    "debug_log_enabled": self._debug_log_enabled,
                    "debug_log_rate_hz": dbg_rate,
                },
                context="apple_pluck_impedance_control_displacement",
            )

        # State
        self._baseline = None
        self._ready = not self._start_on_done_topic
        self._stopping = False
        self._holding = False
        self._hold_position = None
        self._joint_position = None
        self._force_magnitude = None
        self._release_elapsed = 0.0
        self._release_published = False
        self._release_reset_logged = False

        # Done gating: wait for done topic
        self._done_gate = DoneGate(self, self._done_topic) if self._start_on_done_topic else None

        # Publishers and subscribers to maintain hold and monitor forces
        self._state_sub = self.create_subscription(LBRState, self._state_topic, self._on_state, 1)
        self._wrench_sub = self.create_subscription(WrenchStamped, self._wrench_topic, self._on_wrench, 10)
        self._hold_pub = self.create_publisher(LBRJointPositionCommand, self._command_topic, 1)

        release_qos = rclpy.qos.QoSProfile(depth=1)
        release_qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        release_qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        self._release_pub = self.create_publisher(Bool, self._release_done_topic, release_qos)

        # TF listener fills the buffer with incoming transforms 
        # so EE pose lookups stay live
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Timer
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f"Displacement monitor started: rate={self._update_rate}Hz, base='{self._base_frame}', ee='{self._ee_frame}', "
            f"axis='{self._disp_axis}', disp_thr={self._disp_threshold_m} m, hold_cmd_topic='{self._command_topic}',"
            f" force_release_thr={self._force_release_threshold} N"
        )

    # ------------------------- Helpers -------------------------
    # Cache the latest measured joints so we can freeze 
    # the arm at that pose.
    def _on_state(self, msg: LBRState) -> None:
        try:
            self._joint_position = list(msg.measured_joint_position)
        except Exception:
            self._joint_position = None
    
    # Track current force magnitude at the end-effector for release timing.
    def _on_wrench(self, msg: WrenchStamped) -> None:
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        self._force_magnitude = math.sqrt(fx * fx + fy * fy + fz * fz)
    
    # Fetch the present EE transform relative to the base; 
    # None when TF is stale.
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
    
    # Compute displacement along the requested axis (or norm).
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
    
    # Publish the captured joint pose to keep the arm static after threshold.
    def _publish_hold(self) -> None:
        if self._hold_position is None:
            if self._joint_position is not None:
                self._hold_position = list(self._joint_position)
                self.get_logger().info("Captured joint pose for displacement hold")
            else:
                return
        cmd = LBRJointPositionCommand()
        cmd.joint_position = list(self._hold_position)
        self._hold_pub.publish(cmd)
    
    # Monitor EE force until it stays low long enough, then publish release.
    def _check_force_release(self) -> None:
        if self._force_magnitude is None:
            return
        if self._force_magnitude <= self._force_release_threshold:
            self._release_elapsed += self._dt
            self._release_reset_logged = False
            if self._release_elapsed >= self._force_release_duration and not self._release_published:
                self._release_published = True
                self._release_pub.publish(Bool(data=True))
                try:
                    time.sleep(0.05)
                except Exception:
                    pass
                self.get_logger().info(
                    f"Force below threshold ({self._force_magnitude:.2f} N <= {self._force_release_threshold:.2f} N) "
                    f"for {self._release_elapsed:.2f}s. Published '{self._release_done_topic}' and shutting down."
                )
                self._shutdown()
        else:
            if (
                self._release_elapsed > 0.0
                and self._debug_log_enabled
                and not self._release_reset_logged
            ):
                self.get_logger().info(
                    f"Force release timer reset: |F|={self._force_magnitude:.2f} N > {self._force_release_threshold:.2f} N"
                )
                self._release_reset_logged = True
            self._release_elapsed = 0.0

    def _shutdown(self) -> None:
        try:
            self._timer.cancel()
        except Exception:
            pass
        self.destroy_node()
        rclpy.shutdown()

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
        if self._debug_log_enabled and self._dbg.tick(self._dt):
            self.get_logger().info(f"EE disp={disp:.4f} m (axis={self._disp_axis}, thr={self._disp_threshold_m:.4f} m)")

        if not self._stopping and disp >= self._disp_threshold_m:
            self._stopping = True
            self._holding = True
            self.get_logger().info(
                f"Displacement threshold reached: value={disp:.4f} m >= {self._disp_threshold_m:.4f} m. Holding position and monitoring force."
            )

        if self._holding:
            self._publish_hold()
            self._check_force_release()


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
