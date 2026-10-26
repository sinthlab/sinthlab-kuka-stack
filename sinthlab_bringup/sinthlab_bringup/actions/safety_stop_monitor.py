#!/usr/bin/env python3
"""Runaway safety-stop for the compliant experiments.

Watches the EE (tf ``base_frame`` -> ``ee_frame``) and fires ``on_trip`` ONCE if the arm moves too far
from where the trial started, moves too fast, or takes the end effector below the floor (helpers/floor.py:
more than ``floor.trip_margin_m`` under ``floor.height_m``). It is a backstop for faults the rest of the stack does
NOT halt on — most importantly a gravity-driven fall when a *free* axis (e.g. Z in the vertical maze)
is left holding the tool weight with too little stiffness or wrong load data. (The FRI CommandGuard only
neutralises the outgoing command on a velocity fault; it does not stop the trial.)

It only observes; the orchestrator's ``on_trip`` decides the response (stop the fixture, drive back to a
safe pose). ``stop()`` disarms it for a normal trial end.
"""
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

from rclpy.duration import Duration
from rclpy.node import Node as rclpyNode
from rclpy.time import Time
import tf2_ros

from scipy.spatial.transform import Rotation

from sinthlab_bringup.helpers.floor import Floor
from sinthlab_bringup.helpers.node_utils import DebugTicker, get_required_param


class SafetyStopMonitor:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "",
                 on_trip: Callable[[str], None]) -> None:
        self._node = node
        self._on_trip = on_trip
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._base_frame = str(get_required_param(node, self._param_prefix + "base_frame"))
        self._ee_frame = str(get_required_param(node, self._param_prefix + "ee_frame"))
        # Trip if the EE gets this far from the trial-start pose (m). Must exceed the largest legitimate
        # excursion (e.g. the maze footprint) with margin, but be small enough to catch a runaway early.
        # 0 turns the distance check off (free move: it has no boundary; only the speed check runs).
        self._max_disp = float(get_required_param(node, self._param_prefix + "max_displacement_m"))
        # Trip if the EE speed exceeds this (m/s). Normal hand guiding stays well below this; a fall or
        # lurch blows past it. Catches a runaway before it travels max_displacement. 0 = off (apple
        # pluck: a monkey's pull may be fast, and its only check is the floor).
        self._max_speed = float(get_required_param(node, self._param_prefix + "max_speed_mps"))
        # Speed is a single-sample finite difference of a TF lookup, so it is NOISY: at 100 Hz a 1.3 cm
        # jitter (or one stale/dropped transform) reads as 1.3 m/s. Require the threshold to be exceeded
        # on this many CONSECUTIVE samples before tripping -- a real runaway sustains, a spike does not.
        self._speed_trip_samples = int(get_required_param(node, self._param_prefix + "speed_trip_samples"))
        self._speed_over = 0

        self._floor = Floor.from_node(node)
        self._dbg = DebugTicker(float(get_required_param(node, self._param_prefix + "debug_log_rate_hz")))
        self._debug_log_enabled = bool(get_required_param(node, self._param_prefix + "debug_log_enabled"))

        self._ready = False
        self._baseline: Optional[np.ndarray] = None
        self._prev: Optional[np.ndarray] = None
        self._speed_over = 0

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)
        self._timer = node.create_timer(self._dt, self._step)

        node.get_logger().info(
            f"SafetyStopMonitor: max displacement "
            f"{f'{self._max_disp:.2f} m' if self._max_disp > 0 else 'off'}, max speed "
            f"{f'{self._max_speed:.2f} m/s' if self._max_speed > 0 else 'off'}, floor {self._floor.describe()} "
            f"(trips {self._floor.trip_margin * 100:.0f} cm below)"
        )

    def start(self) -> None:
        self._baseline = None
        self._prev = None
        self._speed_over = 0
        self._ready = True

    def stop(self) -> None:
        self._ready = False

    def _ee_pose(self) -> Optional[np.ndarray]:
        try:
            ts = self._tf_buffer.lookup_transform(
                self._base_frame, self._ee_frame, Time(), timeout=Duration(seconds=0.05)
            )
            t, r = ts.transform.translation, ts.transform.rotation
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
            T[:3, 3] = [t.x, t.y, t.z]
            return T
        except Exception:
            return None

    def _step(self) -> None:
        if not self._ready:
            return
        T = self._ee_pose()
        if T is None:
            return
        p = T[:3, 3]
        if self._baseline is None:
            self._baseline = p.copy()
            self._prev = p.copy()
            self._node.get_logger().info(f"Safety monitor armed at EE {np.round(self._baseline, 3)}.")
            return

        disp = float(np.linalg.norm(p - self._baseline))
        speed = float(np.linalg.norm(p - self._prev)) / self._dt if self._prev is not None else 0.0
        self._prev = p.copy()

        if self._debug_log_enabled and self._dbg.tick(self._dt):
            self._node.get_logger().info(f"safety: disp={disp:.3f} m, speed={speed:.3f} m/s")

        # Debounce the speed check (see _speed_trip_samples); displacement is a position, not a
        # derivative, so it needs no debouncing.
        self._speed_over = self._speed_over + 1 if 0.0 < self._max_speed <= speed else 0
        clearance = self._floor.clearance(T)

        reason = None
        if clearance < -self._floor.trip_margin:
            reason = (f"the {self._floor.reference.replace('_', ' ')} is {-clearance * 100:.1f} cm below the "
                      f"floor ({self._floor.height:.3f} m)")
        elif self._max_disp > 0.0 and disp >= self._max_disp:   # 0 = no distance check (free move)
            reason = f"displacement {disp:.3f} m >= {self._max_disp:.2f} m"
        elif self._speed_over >= self._speed_trip_samples:
            reason = (f"speed {speed:.3f} m/s >= {self._max_speed:.2f} m/s for "
                      f"{self._speed_over} consecutive samples")
        if reason is not None:
            self._ready = False  # one-shot; the orchestrator drives recovery
            self._node.get_logger().error(f"SAFETY STOP: {reason}. Halting fixture and recovering.")
            try:
                self._on_trip(reason)
            except Exception as exc:
                self._node.get_logger().error(f"SafetyStopMonitor on_trip error: {exc}")
