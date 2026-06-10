#!/usr/bin/env python3
from __future__ import annotations

from typing import Callable, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node as rclpyNode
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import TransformStamped

from sinthlab_bringup.helpers.common_threshold import DebugTicker, get_required_param
from sinthlab_bringup.helpers.param_logging import log_params_once


class CartesianImpedanceDisplacementMonitor:
    """
    Cartesian Impedance displacement monitor for a robot already in controller-side Cartesian impedance.
    - Captures baseline EE pose in base frame.
    - Once displacement threshold is exceeded, triggers 'snap' and initiates shutdown.
    - Relies on the Cartesian controller's natural spring physics to recoil the arm.
    """

    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Callable[[], None], on_snap: Optional[Callable[[], None]] = None, on_armed: Optional[Callable[[], None]] = None) -> None:
        self._node = node
        self._on_complete = on_complete
        self._on_snap = on_snap
        # Fired once when the baseline is locked (settle done) — i.e. "monitoring is live, pull now".
        self._on_armed = on_armed
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        
        # Tracks when the action is active
        self._ready = False

        # Parameters sourced from the hosting node (YAML/overrides)
        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._base_frame = str(get_required_param(node, self._param_prefix + "base_frame"))
        self._ee_frame = str(get_required_param(node, self._param_prefix + "ee_frame"))
        self._disp_axis = str(get_required_param(node, self._param_prefix + "cartesian_axis")).lower()
        self._disp_threshold_m = float(get_required_param(node, self._param_prefix + "cartesian_displacement_threshold_m"))
        self._release_shutdown_delay = max(0.0, float(get_required_param(node, self._param_prefix + "force_release_shutdown_delay_sec")))
        # Settle time to wait after start() before locking the baseline, so the "initial position"
        # is the SETTLED pose. For the perturbation experiment this guarantees the baseline is the
        # post-perturbation rest pose (the physical/tf arm lags the commanded anchor under
        # impedance). Optional; defaults to 0.0 (apple-pluck already settles via its quiet window).
        self._baseline_settle_sec = 0.0
        if node.has_parameter(self._param_prefix + "baseline_settle_sec"):
            self._baseline_settle_sec = max(0.0, float(node.get_parameter(self._param_prefix + "baseline_settle_sec").value))

        self._debug_log_enabled = bool(get_required_param(node, self._param_prefix + "debug_log_enabled"))
        dbg_rate = float(get_required_param(node, self._param_prefix + "debug_log_rate_hz"))
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
                    "release_shutdown_delay_sec": self._release_shutdown_delay,
                    "baseline_settle_sec": self._baseline_settle_sec,
                    "debug_log_enabled": self._debug_log_enabled,
                    "debug_log_rate_hz": dbg_rate,
                },
                context="cartesian_impedance_displacement_monitor",
            )

        # Runtime state
        self._baseline: Optional[TransformStamped] = None
        self._settle_elapsed = 0.0
        self._stopping = False
        self._shutdown_requested = False

        self._shutting_down = False
        self._shutdown_elapsed = 0.0

        # TF buffer for displacement calculation
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._timer = node.create_timer(self._dt, self._step)

        self._node.get_logger().info(
            f"Displacement monitor started: rate={self._update_rate}Hz, "
            f"base='{self._base_frame}', ee='{self._ee_frame}', axis='{self._disp_axis}', "
            f"disp_thr={self._disp_threshold_m:.4f} m"
        )

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
    
    def start(self) -> None:
        """Trigger the action to start monitoring displacement."""
        if self._ready:
            self._node.get_logger().warn("DisplacementMonitor is already running.")
            return

        self._baseline = None
        self._settle_elapsed = 0.0
        self._stopping = False
        self._shutdown_requested = False

        self._shutting_down = False
        self._shutdown_elapsed = 0.0

        self._ready = True
        self._node.get_logger().info("Displacement monitor activated for new trial.")

    def _step(self) -> None:
        if not self._ready:
            return

        if self._shutting_down:
            self._shutdown_elapsed += self._dt
            if self._shutdown_elapsed >= self._release_shutdown_delay:
                self._shutdown()
            return
        
        if self._baseline is None:
            # Wait for the arm to settle before locking the baseline, so the "initial position"
            # is the settled (post-perturbation) rest pose rather than a still-moving one.
            if self._settle_elapsed < self._baseline_settle_sec:
                self._settle_elapsed += self._dt
                return
            ts = self._lookup()
            if ts is not None:
                self._baseline = ts
                self._node.get_logger().info(
                    f"Captured baseline EE pose for displacement thresholding "
                    f"(after {self._settle_elapsed:.2f}s settle)"
                )
                if self._on_armed is not None:
                    self._on_armed()  # baseline locked -> signal "pull now"
            return

        ts_now = self._lookup()
        if ts_now is None:
            if self._dbg.tick(self._dt):
                self._node.get_logger().warn("TF lookup failed; waiting for transform")
            return

        disp = self._axis_disp_m(ts_now, self._baseline, self._disp_axis)
        if self._debug_log_enabled and not self._stopping and self._dbg.tick(self._dt):
            self._node.get_logger().info(
                f"EE disp={disp:.4f} m (axis={self._disp_axis}, thr={self._disp_threshold_m:.4f} m)"
            )

        if not self._stopping and disp >= self._disp_threshold_m:
            self._stopping = True
            
            self._node.get_logger().info(
                f"APPLE PLUCK! Threshold reached: value={disp:.4f} m >= {self._disp_threshold_m:.4f} m. "
                f"Snapping tension and entering recovery."
            )
            
            # Fire audio cue or external callback
            if self._on_snap is not None:
                self._on_snap()

            if self._release_shutdown_delay > 0.0:
                self._shutting_down = True
                self._shutdown_elapsed = 0.0
            else:
                self._shutdown()

    def _shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._ready = False
        self._shutting_down = False
        self._node.get_logger().info("Displacement monitor sequence complete. Yielding control.")
        
        try:
            self._on_complete()
        except Exception as exc:
            self._node.get_logger().error(f"Exception during displacement monitor complete callback: {exc}")
