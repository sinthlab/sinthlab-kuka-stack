#!/usr/bin/env python3
"""Maze checkpoint + goal monitor.

Watches the EE position (tf2 ``base_frame`` -> ``ee_frame``) and, in ANY order, fires a one‑time
reward callback (`on_reward(i)`) each time the EE first enters an unvisited checkpoint sphere. When
the EE enters the goal sphere it fires `on_complete`. Like the displacement monitor it only observes
— it commands no motion; the operator drives the compliant arm through the maze. `stop()` lets an
external timeout halt it.
"""
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

from rclpy.duration import Duration
from rclpy.node import Node as rclpyNode
from rclpy.time import Time
import tf2_ros

from sinthlab_bringup.helpers.common_threshold import DebugTicker, get_required_param


class CheckpointMonitor:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "",
                 on_complete: Callable[[], None], on_reward: Optional[Callable[[int], None]] = None) -> None:
        self._node = node
        self._on_complete = on_complete
        self._on_reward = on_reward
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._ready = False

        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._base_frame = str(get_required_param(node, self._param_prefix + "base_frame"))
        self._ee_frame = str(get_required_param(node, self._param_prefix + "ee_frame"))

        cx = [float(v) for v in get_required_param(node, self._param_prefix + "checkpoint_x")]
        cy = [float(v) for v in get_required_param(node, self._param_prefix + "checkpoint_y")]
        cz = [float(v) for v in get_required_param(node, self._param_prefix + "checkpoint_z")]
        cr = [float(v) for v in get_required_param(node, self._param_prefix + "checkpoint_radius")]
        if not (len(cx) == len(cy) == len(cz) == len(cr)):
            raise ValueError("checkpoint_x/y/z/radius must be equal-length arrays")
        self._checkpoints = [(np.array([cx[i], cy[i], cz[i]]), cr[i]) for i in range(len(cx))]

        self._goal = np.array([
            float(get_required_param(node, self._param_prefix + "goal_x")),
            float(get_required_param(node, self._param_prefix + "goal_y")),
            float(get_required_param(node, self._param_prefix + "goal_z")),
        ])
        self._goal_radius = float(get_required_param(node, self._param_prefix + "goal_radius"))

        self._debug_log_enabled = bool(get_required_param(node, self._param_prefix + "debug_log_enabled"))
        self._dbg = DebugTicker(float(get_required_param(node, self._param_prefix + "debug_log_rate_hz")))

        self._visited: set[int] = set()
        self._completed = False

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)
        self._timer = node.create_timer(self._dt, self._step)

        node.get_logger().info(
            f"CheckpointMonitor: {len(self._checkpoints)} checkpoints (any order), "
            f"goal r={self._goal_radius:.3f} m"
        )

    def start(self) -> None:
        if self._ready:
            self._node.get_logger().warn("CheckpointMonitor is already running.")
            return
        self._visited = set()
        self._completed = False
        self._ready = True
        self._node.get_logger().info("CheckpointMonitor activated for new trial.")

    def stop(self) -> None:
        """Halt monitoring (e.g. on a whole-experiment timeout)."""
        self._ready = False

    def _ee_pos(self) -> Optional[np.ndarray]:
        try:
            ts = self._tf_buffer.lookup_transform(
                self._base_frame, self._ee_frame, Time(), timeout=Duration(seconds=0.05)
            )
            t = ts.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def _step(self) -> None:
        if not self._ready or self._completed:
            return
        p = self._ee_pos()
        if p is None:
            if self._dbg.tick(self._dt):
                self._node.get_logger().warn("TF lookup failed; waiting for transform")
            return

        # Checkpoints — any order, reward once each on first entry.
        for i, (c, r) in enumerate(self._checkpoints):
            if i in self._visited:
                continue
            if float(np.linalg.norm(p - c)) <= r:
                self._visited.add(i)
                self._node.get_logger().info(
                    f"Checkpoint {i} reached ({len(self._visited)}/{len(self._checkpoints)})."
                )
                if self._on_reward is not None:
                    self._on_reward(i)

        # Goal — ends the trial.
        if float(np.linalg.norm(p - self._goal)) <= self._goal_radius:
            self._completed = True
            self._ready = False
            self._node.get_logger().info("GOAL reached — maze complete.")
            try:
                self._on_complete()
            except Exception as exc:
                self._node.get_logger().error(f"CheckpointMonitor on_complete error: {exc}")
            return

        if self._debug_log_enabled and self._dbg.tick(self._dt):
            self._node.get_logger().info(
                f"EE {np.round(p, 3)} | visited {len(self._visited)}/{len(self._checkpoints)} | "
                f"goal dist {float(np.linalg.norm(p - self._goal)):.3f} m"
            )
