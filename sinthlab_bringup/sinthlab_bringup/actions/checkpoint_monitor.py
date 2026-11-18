#!/usr/bin/env python3
"""Maze checkpoint + goal monitor.

Watches the EE position (tf2 ``base_frame`` -> ``ee_frame``) and, in ANY order, fires a one‑time
reward callback (`on_reward(i)`) each time the EE first enters an unvisited checkpoint sphere. When
the EE enters the goal sphere it fires `on_complete`. Like the displacement monitor it only observes
— it commands no motion; the operator drives the compliant arm through the maze. `stop()` lets an
external timeout halt it.

A maze shape can carry its own checkpoints and goal: pass `waypoints_prefix` (the active fixture
profile's block, e.g. "virtual_fixtures.fish.") and, if that block has `checkpoint_y`, the checkpoints,
goal and `goal_after_all_checkpoints` are read from it instead of from this monitor's own block.
`goal_after_all_checkpoints: true` makes the goal count only once every checkpoint has been visited --
for a closed loop whose goal is the start, that is one full lap.

In relative mode, pass `origin_provider` (the maze fixture's `anchor_position`) so checkpoints and
the goal are placed on exactly the origin the rails use. Without it the monitor anchors on its own
first tick -- which, when the fixture waits for the arm to settle, is ~4 cm above the rails.
"""
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

from rclpy.duration import Duration
from rclpy.node import Node as rclpyNode
from rclpy.time import Time
import tf2_ros

from sinthlab_bringup.helpers.node_utils import DebugTicker, get_required_param


class CheckpointMonitor:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "",
                 on_complete: Callable[[], None], on_reward: Optional[Callable[[int], None]] = None,
                 origin_provider: Optional[Callable[[], Optional[np.ndarray]]] = None,
                 waypoints_prefix: Optional[str] = None) -> None:
        self._node = node
        self._on_complete = on_complete
        self._on_reward = on_reward
        # Returns the base-frame origin the maze rails are anchored on, or None until they are.
        self._origin_provider = origin_provider
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._ready = False

        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._base_frame = str(get_required_param(node, self._param_prefix + "base_frame"))
        self._ee_frame = str(get_required_param(node, self._param_prefix + "ee_frame"))

        # Where the checkpoints and goal come from: the active maze shape's own block if it has them.
        wp = self._param_prefix
        if waypoints_prefix and node.has_parameter(waypoints_prefix + "checkpoint_y"):
            wp = waypoints_prefix
        self._waypoints_from = wp.rstrip(".")
        cx = [float(v) for v in get_required_param(node, wp + "checkpoint_x")]
        cy = [float(v) for v in get_required_param(node, wp + "checkpoint_y")]
        cz = [float(v) for v in get_required_param(node, wp + "checkpoint_z")]
        cr = [float(v) for v in get_required_param(node, wp + "checkpoint_radius")]
        if not (len(cx) == len(cy) == len(cz) == len(cr)):
            raise ValueError("checkpoint_x/y/z/radius must be equal-length arrays")
        self._checkpoints = [(np.array([cx[i], cy[i], cz[i]]), cr[i]) for i in range(len(cx))]

        self._goal = np.array([
            float(get_required_param(node, wp + "goal_x")),
            float(get_required_param(node, wp + "goal_y")),
            float(get_required_param(node, wp + "goal_z")),
        ])
        self._goal_radius = float(get_required_param(node, wp + "goal_radius"))
        # The goal counts only after every checkpoint (one full lap of a loop whose goal is the start).
        self._goal_after_all = bool(get_required_param(node, wp + "goal_after_all_checkpoints"))

        # RELATIVE: checkpoint_*/goal_* are OFFSETS from the start EE, captured on the first
        # tick, so they track move_to_start exactly like the relative maze corridors do. Keep this in
        # sync with the fixture's corridor_frame -- if the corridors move with the start but the
        # checkpoints don't, the rewards/goal land in the wrong place. ABSOLUTE: raw base-frame coords.
        self._relative = bool(get_required_param(node, self._param_prefix + "relative_to_start"))
        self._origin: Optional[np.ndarray] = None  # start EE, captured on first tick when relative

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
            f"{' after all checkpoints' if self._goal_after_all else ''}, from {self._waypoints_from}"
        )

    def start(self) -> None:
        if self._ready:
            self._node.get_logger().warn("CheckpointMonitor is already running.")
            return
        self._visited = set()
        self._completed = False
        self._origin = None  # re-capture the start EE for this trial (relative mode)
        self._ready = True
        self._node.get_logger().info("CheckpointMonitor activated for new trial.")

    def stop(self) -> None:
        """Halt monitoring (e.g. on a whole-experiment timeout)."""
        self._ready = False

    def view_geometry(self) -> dict:
        """Checkpoints and goal as [x, y, z, radius], in this monitor's frame (offsets from the anchor
        when relative), for the dashboard's live maze view."""
        return {
            "checkpoints": [[*map(float, c), float(r)] for c, r in self._checkpoints],
            "goal": [*map(float, self._goal), float(self._goal_radius)],
            "goal_after_all": self._goal_after_all,
        }

    def view_state(self) -> dict:
        """Which checkpoints this trial has visited, and whether reaching the goal would end it now."""
        return {
            "visited": sorted(self._visited),
            "goal_armed": not self._goal_after_all or len(self._visited) == len(self._checkpoints),
            "monitoring": self._ready,
            "completed": self._completed,
        }

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

        # Anchor the checkpoint/goal offsets (relative mode). With an origin_provider, use the SAME
        # origin as the rails -- the fixture anchors only after its settle, so wait until it has; nothing
        # can be rewarded before then because the arm is still at the start. Without one, fall back to
        # the EE on the first tick.
        if self._relative and self._origin is None:
            if self._origin_provider is not None:
                origin = self._origin_provider()
                if origin is None:
                    return
                self._origin = np.asarray(origin, dtype=float).copy()
                source = "the maze fixture's anchor"
            else:
                self._origin = p.copy()
                source = "the start EE"
            self._node.get_logger().info(
                f"Checkpoints anchored at {source} {np.round(self._origin, 3)} (relative); "
                f"EE is {np.round(p - self._origin, 3)} from it."
            )
        off = self._origin if (self._relative and self._origin is not None) else np.zeros(3)

        # Checkpoints — any order, reward once each on first entry.
        for i, (c, r) in enumerate(self._checkpoints):
            if i in self._visited:
                continue
            if float(np.linalg.norm(p - (c + off))) <= r:
                self._visited.add(i)
                self._node.get_logger().info(
                    f"Checkpoint {i} reached ({len(self._visited)}/{len(self._checkpoints)})."
                )
                if self._on_reward is not None:
                    self._on_reward(i)

        # Goal — ends the trial (only once every checkpoint is visited, if goal_after_all_checkpoints).
        armed = not self._goal_after_all or len(self._visited) == len(self._checkpoints)
        if armed and float(np.linalg.norm(p - (self._goal + off))) <= self._goal_radius:
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
                f"goal dist {float(np.linalg.norm(p - (self._goal + off))):.3f} m"
            )
