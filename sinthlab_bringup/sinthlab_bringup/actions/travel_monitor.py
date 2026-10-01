#!/usr/bin/env python3
"""Out-and-back travel monitor for the pre-training experiments.

Watches how far the end effector has moved from the fixture's anchor (the settled start) and drives
the trial outcome:

    armed -> |travel| >= threshold_m (in an allowed direction) -> on_threshold(signed_m)
          -> [require_return] back within return_tolerance_m of the start -> on_complete()

Without require_return, on_complete() fires together with on_threshold(). The position comes from a
`position_provider` -- the fixture action that already owns the arm state -- so there is no second
TF lookup or a second idea of where "start" is:

    provider() -> np.ndarray (3,) base-frame offset of the EE from the anchor, or None (not anchored)

`axis` picks what counts as travel: "x" | "y" | "z" (signed, along that base axis) or "norm" (the
straight-line distance, always positive). `direction` limits which side counts: "both", "positive"
or "negative" (ignored for "norm").

It only observes. The orchestrator owns the cues, the reward and the trial ending.
"""
from __future__ import annotations

from typing import Callable, Optional

import numpy as np
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.common_threshold import DebugTicker, get_optional_param, get_required_param

_AXES = {"x": 0, "y": 1, "z": 2}


class TravelMonitor:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "travel_task",
                 position_provider: Callable[[], Optional[np.ndarray]],
                 on_threshold: Callable[[float], None],
                 on_complete: Callable[[], None]) -> None:
        self._node = node
        self._p = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        self._provider = position_provider
        self._on_threshold = on_threshold
        self._on_complete = on_complete

        self._dt = 1.0 / float(get_required_param(node, self._p + "update_rate"))
        axis = str(get_required_param(node, self._p + "axis")).lower()
        if axis not in ("x", "y", "z", "norm"):
            raise ValueError(f"{self._p}axis must be x, y, z or norm, got {axis!r}")
        self._axis = axis
        self._dbg = DebugTicker(float(get_optional_param(node, self._p + "debug_log_rate_hz", 2.0)))
        self.reload()

        self._active = False
        self._reached = False
        self._timer = node.create_timer(self._dt, self._step)

    def reload(self) -> None:
        """Re-read the live task parameters (threshold, direction, return). Called between trials."""
        n, p = self._node, self._p
        self._threshold = float(get_required_param(n, p + "threshold_m"))
        self._direction = str(get_optional_param(n, p + "direction", "both")).lower()
        self._require_return = bool(get_optional_param(n, p + "require_return", True))
        self._return_tol = float(get_optional_param(n, p + "return_tolerance_m", 0.02))
        self._debug = bool(get_optional_param(n, p + "debug_log_enabled", False))
        self._node.get_logger().info(
            f"Travel task: {self._axis} >= {self._threshold:.3f} m ({self._direction})"
            + (f", then back within {self._return_tol:.3f} m" if self._require_return else ""))

    def threshold_m(self) -> float:
        return self._threshold

    def start(self) -> None:
        self._reached = False
        self._active = True

    def stop(self) -> None:
        self._active = False

    def _travel(self, off: np.ndarray) -> float:
        if self._axis == "norm":
            return float(np.linalg.norm(off))
        return float(off[_AXES[self._axis]])

    def _counts(self, d: float) -> bool:
        if abs(d) < self._threshold:
            return False
        if self._axis == "norm" or self._direction == "both":
            return True
        return d > 0 if self._direction == "positive" else d < 0

    def _step(self) -> None:
        if not self._active:
            return
        off = self._provider()
        if off is None:
            return          # the fixture has not anchored yet
        d = self._travel(off)
        if self._debug and self._dbg.tick(self._dt):
            self._node.get_logger().info(
                f"travel: {self._axis}={d:+.3f} m ({'returning' if self._reached else 'outbound'})")

        if not self._reached:
            if not self._counts(d):
                return
            self._reached = True
            self._fire(self._on_threshold, d)
            if not self._require_return:
                self._active = False
                self._fire(self._on_complete)
            return

        # Back at the start: measured as distance from the anchor, whatever the axis.
        if float(np.linalg.norm(off)) <= self._return_tol:
            self._active = False
            self._fire(self._on_complete)

    def _fire(self, fn, *args) -> None:
        try:
            fn(*args)
        except Exception as exc:
            self._node.get_logger().error(f"TravelMonitor callback error: {exc}")
