#!/usr/bin/env python3
from __future__ import annotations

from typing import Callable, Optional

import math
import time
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import TransformStamped, WrenchStamped

from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand
from helpers.common_threshold import DebugTicker, get_required_param
from helpers.param_logging import log_params_once


class CartesianImpedanceDisplacementMonitor:
    """
    Cartesian Impedance displacement monitor for a robot already in controller-side Cartesian impedance.
    - captures baseline EE pose in base frame.
    - Once displacement threshold is exceeded, holds joint position and monitors wrench magnitude.
    - Shutdown after forces stay below the configured threshold long enough.
    """

    def __init__(self, node: Node, *, to_start: Callable[[], None], on_complete: Callable[[], None]) -> None:
        self._node = node
        self._to_start = to_start
        self._on_complete = on_complete
        
        # Tracks when the action is ready to be triggered
        self._ready = False

        # Parameters sourced from the hosting node (YAML/overrides)
        self._update_rate = int(get_required_param(node, "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._base_frame = str(get_required_param(node, "base_frame"))
        self._ee_frame = str(get_required_param(node, "ee_frame"))
        self._disp_axis = str(get_required_param(node, "cartesian_axis")).lower()
        self._disp_threshold_m = float(get_required_param(node, "cartesian_displacement_threshold_m"))
        self._state_topic = str(get_required_param(node, "state_topic"))
        self._command_topic = str(get_required_param(node, "command_topic"))
        self._wrench_topic = str(get_required_param(node, "wrench_topic"))
        self._force_release_threshold = float(get_required_param(node, "force_release_threshold_newton"))
        self._force_release_duration = float(get_required_param(node, "force_release_duration_sec"))
        self._release_shutdown_delay = max(0.0, float(get_required_param(node, "force_release_shutdown_delay_sec")))

        self._debug_log_enabled = bool(get_required_param(node, "debug_log_enabled"))
        dbg_rate = float(get_required_param(node, "debug_log_rate_hz"))
        self._dbg = DebugTicker(dbg_rate)

        if self._debug_log_enabled:
            log_params_once(
                node,
                params={
                    "update_rate": self._update_rate,
                    "base_frame": self._base_frame,
                    "ee_frame": self._ee_frame,
                    "cartesian_axis": self._disp_axis,
                    "cartesian_displacement_threshold_m": self._disp_threshold_m,
                    "state_topic": self._state_topic,
                    "command_topic": self._command_topic,
                    "wrench_topic": self._wrench_topic,
                    "force_release_threshold_newton": self._force_release_threshold,
                    "force_release_duration_sec": self._force_release_duration,
                    "force_release_shutdown_delay_sec": self._release_shutdown_delay,
                    "debug_log_enabled": self._debug_log_enabled,
                    "debug_log_rate_hz": dbg_rate,
                },
                context="cartesian_impedance_displacement_monitor",
            )

        # Runtime state
        self._baseline: Optional[TransformStamped] = None
        self._stopping = False
        self._holding = False
        self._hold_position: Optional[list[float]] = None
        self._joint_position: Optional[list[float]] = None
        self._force_magnitude: Optional[float] = None
        self._release_elapsed = 0.0
        self._release_published = False
        self._release_reset_logged = False
        self._shutdown_requested = False

        # ROS interfaces owned by the monitor
        self._state_sub = node.create_subscription(LBRState, self._state_topic, self._on_state, 1)
        self._wrench_sub = node.create_subscription(WrenchStamped, self._wrench_topic, self._on_wrench, 10)
        self._hold_pub = node.create_publisher(LBRJointPositionCommand, self._command_topic, 1)

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._timer = node.create_timer(self._dt, self._step)

        self._node.get_logger().info(
            f"Displacement monitor started: rate={self._update_rate}Hz, "
            f"base='{self._base_frame}', ee='{self._ee_frame}', axis='{self._disp_axis}', "
            f"disp_thr={self._disp_threshold_m:.4f} m, hold_cmd_topic='{self._command_topic}', "
            f"force_release_thr={self._force_release_threshold:.2f} N"
        )

    # ------------------------------------------------------------------
    def _on_state(self, msg: LBRState) -> None:
        try:
            self._joint_position = list(msg.measured_joint_position)
        except Exception:
            self._joint_position = None

    def _on_wrench(self, msg: WrenchStamped) -> None:
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        self._force_magnitude = math.sqrt(fx * fx + fy * fy + fz * fz)

    def _lookup(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_frame,
                Time(),
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
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _publish_hold(self) -> None:
        if self._hold_position is None:
            if self._joint_position is not None:
                self._hold_position = list(self._joint_position)
                self._node.get_logger().info("Captured joint pose for displacement hold")
            else:
                return
        if self._hold_pub is None:
            return
        cmd = LBRJointPositionCommand()
        cmd.joint_position = list(self._hold_position)
        self._hold_pub.publish(cmd)

    def _check_force_release(self) -> None:
        if self._force_magnitude is None:
            return
        if self._force_magnitude <= self._force_release_threshold:
            self._release_elapsed += self._dt
            self._release_reset_logged = False
            if self._release_elapsed >= self._force_release_duration and not self._release_published:
                self._release_published = True
                self._holding = False
                self._stop_hold_publish()
                self._node.get_logger().info(
                    "Force below threshold (|F|=%.2f N <= %.2f N) for %.2fs. requesting shutdown.",
                    self._force_magnitude,
                    self._force_release_threshold,
                    self._release_elapsed
                )
                if self._release_shutdown_delay > 0.0:
                    time.sleep(self._release_shutdown_delay)
                self._shutdown()
        else:
            if self._release_elapsed > 0.0 and self._debug_log_enabled and not self._release_reset_logged:
                self._node.get_logger().info(
                    "Force release timer reset: |F|=%.2f N > %.2f N",
                    self._force_magnitude,
                    self._force_release_threshold,
                )
                self._release_reset_logged = True
            self._release_elapsed = 0.0

    def _stop_hold_publish(self) -> None:
        if self._hold_pub is not None:
            try:
                self._node.destroy_publisher(self._hold_pub)
            except Exception:
                pass
            self._hold_pub = None

    def _shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._node.get_logger().info("Displacement monitor shutting down (release sequence complete).")
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:
                pass
            self._timer = None
        try:
            self._on_complete()
        except Exception as exc:
            self._node.get_logger().error("Exception during displacement monitor shutdown callback: %s", exc)
        # Give ROS2 QOS a moment to flush the latched release event before shutdown
        time.sleep(self._release_shutdown_delay)
        self._node.destroy_node()
        rclpy.shutdown() 

    def _step(self) -> None:
        # wait till ready callback is ready to start this action
        if not self._ready:
            if self._to_start():
                self._ready = True
            else:
                return
        
        if self._baseline is None:
            ts = self._lookup()
            if ts is not None:
                self._baseline = ts
                self._node.get_logger().info("Captured baseline EE pose for displacement thresholding")
            return

        ts_now = self._lookup()
        if ts_now is None:
            if self._dbg.tick(self._dt):
                self._node.get_logger().warn("TF lookup failed; waiting for transform")
            return

        disp = self._axis_disp_m(ts_now, self._baseline, self._disp_axis)
        if self._debug_log_enabled and not self._holding and self._dbg.tick(self._dt):
            self._node.get_logger().info(
                "EE disp=%.4f m (axis=%s, thr=%.4f m)", disp, self._disp_axis, self._disp_threshold_m
            )

        if not self._stopping and disp >= self._disp_threshold_m:
            self._stopping = True
            self._holding = True
            self._node.get_logger().info(
                f"Displacement threshold reached: value={disp:.4f} m >= {self._disp_threshold_m:.4f} m. Holding position and monitoring force.",
            )

        if self._holding:
            self._publish_hold()
            self._check_force_release()
