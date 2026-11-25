#!/usr/bin/env python3
"""Haptic guidance: the apple buzzes harder the closer the hand brings it to the goal.

    distance to the goal d [m]          strength
    d >= ramp_start_m                   0 (silent)
    ramp_full_m < d < ramp_start_m      min_pct -> max_pct, linearly as d falls
    d <= ramp_full_m                    max_pct

The experiment supplies the distance (distance_fn, None = silent):
    maze             to the nearest reward point not yet passed this trial -- an unvisited
                     checkpoint, or the goal (CheckpointMonitor.distance_to_next_reward). A
                     checkpoint already passed no longer pulls.
    Move Vertical /  to the goal end of the rail (CentreOutMonitor.distance_to_goal): with
    Horizontal       `either`, the nearer end; with `cued`, the cued end only. Silent around the
                     centre: ramp_start_m is shorter than half the rail.

It runs only between start() (the go) and stop() (the trial's end, Stop), at RATE_HZ. The strength
goes to the board as HAP over the serial link (helpers/end_effector.link); the board drops the motor
to 0 by itself if it stops hearing from here.

Parameters (all live, read at the next trial), under `haptic.`:
    enabled       bool    off = no buzz
    min_pct       int     strength as the goal comes within ramp_start_m [%] -- a coin ERM needs some
                          drive before it spins at all
    max_pct       int     strength at ramp_full_m and closer [%]
    ramp_start_m  float   the buzz starts this far from the goal [m]
    ramp_full_m   float   full strength this close to the goal [m]
"""
from __future__ import annotations

from typing import Callable, Optional

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers import end_effector
from sinthlab_bringup.helpers.node_utils import get_required_param

RATE_HZ = 20.0


def strength(d: Optional[float], min_pct: float, max_pct: float, ramp_start: float, ramp_full: float) -> int:
    """The haptic strength [%] for distance d to the goal (None = silent)."""
    if d is None or d >= ramp_start:
        return 0
    span = ramp_start - ramp_full
    k = 1.0 if span <= 0 else min(1.0, max(0.0, (ramp_start - d) / span))
    return int(round(min_pct + (max_pct - min_pct) * k))


class HapticGuidance:
    def __init__(self, node: rclpyNode, distance_fn: Callable[[], Optional[float]], *,
                 param_prefix: str = "haptic") -> None:
        self._node = node
        self._distance = distance_fn
        self._p = param_prefix + "."
        self._active = False
        self._link = None
        self._last = 0
        self.reload()
        node.create_timer(1.0 / RATE_HZ, self._tick)

    def reload(self) -> None:
        n, p = self._node, self._p
        self.enabled = bool(get_required_param(n, p + "enabled"))
        self._min = float(get_required_param(n, p + "min_pct"))
        self._max = float(get_required_param(n, p + "max_pct"))
        self._start = float(get_required_param(n, p + "ramp_start_m"))
        self._full = float(get_required_param(n, p + "ramp_full_m"))
        if self.enabled and self._link is None:
            self._link = end_effector.link(n)
            n.get_logger().info(
                f"Haptic guidance: {self._min:.0f}% at {self._start * 1000:.0f} mm from the goal, rising "
                f"to {self._max:.0f}% at {self._full * 1000:.0f} mm.")

    def start(self) -> None:
        """At the go: guide until stop()."""
        self._active = self.enabled

    def stop(self) -> None:
        """At the trial's end or Stop: silent at once."""
        self._active = False
        self._send(0)

    def level(self) -> int:
        """The strength being asked for now [%]."""
        return self._last

    def _tick(self) -> None:
        if not self._active:
            return
        try:
            d = self._distance()
        except Exception:
            d = None
        self._send(strength(d, self._min, self._max, self._start, self._full))

    def _send(self, pct: int) -> None:
        self._last = pct
        if self._link is not None:
            self._link.set_haptic(pct)
