#!/usr/bin/env python3
"""Centre-out reach on a rail -- the rewarded version of Move Vertical / Move Horizontal.

ONE REACH PER TRIAL. The centre is the start; the two goals are the rail's ends (free_move.rail_min_m
and rail_max_m from the start). At the go cue the ring lights the goal, and the trial ends at the first
end the arm reaches:

    go cue          the goal is lit -- event `target` (an end's name, or "either")
    a goal end      `reach` -> the reward pulse, then the release wait and park, as any trial
    the other end   `reach_wrong` (cued only) -> no reward; the release wait and park
    neither within response_timeout_sec -> `response_timeout`, as in every experiment

    centre_out.target  either -- both ends are goals; both end quarters light
                       cued   -- one end drawn at random per trial; only its quarter lights and only it
                                 is rewarded

THE ENDS ARE NAMED by where they are, seen from the front of the arm (README, "The ring"): the
rail_min_m / rail_max_m end is down / up on a z rail (Move Vertical), left / right on a y rail (Move
Horizontal). Every reach event carries the end's name. The position is
the arm's own, along the rail (AdmittanceMoveAction.rail_position), not the equilibrium, so a reach
counts only when the arm is really there. It only watches and reports: the motion is the admittance's.

Parameters (all live, read at the next trial):
    centre_out.enabled        bool   off = Move Vertical / Horizontal as plain free movement on the rail
    centre_out.target         str    either | cued
    centre_out.end_tol_m      float  this close to an end counts as reaching it [m]
    centre_out.quarter_<end>  int    ring quarter (0-3) that lights for that end: quarter_down / quarter_up
                                     (Move Vertical), quarter_left / quarter_right (Move Horizontal)
"""
from __future__ import annotations

import random
from typing import Callable, List, Optional

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.node_utils import get_required_param

TARGETS = ("either", "cued")
# The (rail_min_m, rail_max_m) ends' names, by the rail's base axis (an x rail: "min" / "max").
END_NAMES = {"z": ("down", "up"), "y": ("left", "right")}
RATE_HZ = 50.0


class CentreOutMonitor:
    def __init__(self, node: rclpyNode, admittance, *, param_prefix: str = "centre_out") -> None:
        self._node = node
        self._adm = admittance
        self._p = param_prefix + "."
        self._on_end: Optional[Callable[[str, bool], None]] = None
        self._target: Optional[str] = None
        axis = admittance.rail_axis()
        self._ends = END_NAMES.get(axis, ("min", "max"))   # (rail_min_m end, rail_max_m end)
        self.reload()
        node.create_timer(1.0 / RATE_HZ, self._tick)

    def reload(self) -> None:
        n, p = self._node, self._p
        self.enabled = bool(get_required_param(n, p + "enabled")) and self._adm.rail_ends() is not None
        self._mode = str(get_required_param(n, p + "target")).strip().lower()
        if self._mode not in TARGETS:
            raise ValueError(f"{p}target must be one of {TARGETS}, got {self._mode!r}")
        self._end_tol = float(get_required_param(n, p + "end_tol_m"))
        self._quarter = {end: int(get_required_param(n, p + "quarter_" + end)) for end in self._ends}
        if self.enabled:
            lo, hi = self._adm.rail_ends()
            n.get_logger().info(
                f"Centre-out: one reach per trial, {self._mode}; an end counts within "
                f"{self._end_tol * 1000:.0f} mm of the {self._ends[0]} ({lo:+.3f} m) / {self._ends[1]} "
                f"({hi:+.3f} m) end.")

    def pick_target(self) -> str:
        """The goal for this trial: an end's name (cued, drawn at random) or "either"."""
        self._target = random.choice(self._ends) if self._mode == "cued" else "either"
        return self._target

    def quarters(self) -> List[int]:
        """The ring quarters to light for this trial's goal."""
        if self._target in self._ends:
            return [self._quarter[self._target]]
        return [self._quarter[end] for end in self._ends]

    def start(self, on_end: Callable[[str, bool], None]) -> None:
        """At the go cue: watch until the arm reaches an end, then call on_end(end, rewarded) once."""
        if self._target is None:
            self.pick_target()
        self._on_end = on_end

    def stop(self) -> None:
        self._on_end = None
        self._target = None

    def _tick(self) -> None:
        if self._on_end is None:
            return
        x = self._adm.rail_position()
        if x is None:
            return
        lo, hi = self._adm.rail_ends()
        end = (self._ends[0] if x <= lo + self._end_tol else
               self._ends[1] if x >= hi - self._end_tol else None)
        if end is None:
            return
        rewarded = self._target == "either" or end == self._target
        on_end, self._on_end = self._on_end, None
        self._node.get_logger().info(
            f"Centre-out: reached the {end} end" + (" -- reward." if rewarded else
                                                    f", the target was {self._target} -- no reward."))
        on_end(end, rewarded)
