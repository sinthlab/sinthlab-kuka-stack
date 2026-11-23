#!/usr/bin/env python3
"""N-choice with depleting rewards, on an asterisk of rails (the N-Choice task).

The arm is held to an asterisk of straight rails from the start, in the plane facing the monkey
(free_move.rail_axis rays; free_move.ray_layout rails, 2-8, evenly spaced; free_move.ray_length_m;
helpers/rail_shapes.py). ONE CHOICE
PER TRIAL: at the go cue the whole ring lights (no cue: nothing says which direction is best), and the
trial ends when the arm reaches the end of a ray (within n_choice.end_tol_m). That direction pays with its
current probability; then it depletes and the others recover (helpers/bandit_schedule.py).

Events: `choice` (the direction, rewarded: the reward pulse and tone follow) or `choice_unrewarded` (the
direction, no reward), as the trial's outcome. Each trial's sidecar has `n_choice`: every direction's
probability before the choice. No choice within response_timeout_sec ends the trial too (no reward).

It only watches and reports: the motion is the admittance's.

Parameters (all live, from the next trial; the asterisk itself is per-run):
    n_choice.end_tol_m   float  this close to a ray's end counts as choosing it [m]
    n_choice.p_max       float  the highest probability a direction recovers to
    n_choice.p_min       float  the lowest it depletes to
    n_choice.deplete     float  0-1: how much of the way to p_min the chosen direction falls
    n_choice.recover     float  0-1: how much of the way to p_max every other direction rises
    n_choice.start       str    full (every direction at p_max) | random (between p_min and p_max);
                                read at the run's start
"""
from __future__ import annotations

from typing import Callable, Optional

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers import rail_shapes
from sinthlab_bringup.helpers.bandit_schedule import DepletingBandit
from sinthlab_bringup.helpers.node_utils import get_required_param

RATE_HZ = 50.0


class NChoiceMonitor:
    def __init__(self, node: rclpyNode, admittance, *, param_prefix: str = "n_choice") -> None:
        self._node = node
        self._adm = admittance
        self._p = param_prefix + "."
        degs, self._length = admittance.rays()
        if not degs:
            raise ValueError("the N-choice task needs free_move.rail_axis: rays (an asterisk of rails)")
        self.names = [rail_shapes.direction_name(a) for a in degs]
        self._on_choice: Optional[Callable[[str, bool], None]] = None
        n, p = node, self._p
        self._bandit = DepletingBandit(self.names, float(get_required_param(n, p + "p_max")),
                                       float(get_required_param(n, p + "p_min")),
                                       float(get_required_param(n, p + "deplete")),
                                       float(get_required_param(n, p + "recover")),
                                       str(get_required_param(n, p + "start")).strip().lower())
        self.reload()
        node.create_timer(1.0 / RATE_HZ, self._tick)

    def reload(self) -> None:
        n, p = self._node, self._p
        self._end_tol = float(get_required_param(n, p + "end_tol_m"))
        self._bandit.configure(float(get_required_param(n, p + "p_max")), float(get_required_param(n, p + "p_min")),
                               float(get_required_param(n, p + "deplete")), float(get_required_param(n, p + "recover")))
        b = self._bandit
        n.get_logger().info(
            f"N-choice: {len(self.names)} directions ({', '.join(self.names)}), {self._length * 100:.0f} cm rails; "
            f"probabilities {b.p_min:g}-{b.p_max:g}, the chosen one depletes {b.deplete:g}, the others recover "
            f"{b.recover:g} per trial.")

    def check_live(self, name: str, value, context: dict) -> Optional[str]:
        """ExperimentControl validator: p_min may not exceed p_max."""
        if name not in (self._p + "p_min", self._p + "p_max"):
            return None
        lo = float(context.get(self._p + "p_min", self._bandit.p_min))
        hi = float(context.get(self._p + "p_max", self._bandit.p_max))
        return None if 0 <= lo <= hi <= 1 else f"{self._p}p_min ({lo:g}) must be <= p_max ({hi:g}), both 0-1"

    def state(self) -> dict:
        """For the trial's sidecar (`n_choice`): every direction's probability before the choice."""
        return {"p": self._bandit.probabilities(), "best": self._bandit.best(), "choices": self._bandit.choices}

    def start(self, on_choice: Callable[[str, bool], None]) -> None:
        """At the go cue: watch until the arm reaches a ray's end, then call on_choice(direction, rewarded)."""
        self._on_choice = on_choice

    def stop(self) -> None:
        self._on_choice = None

    def _tick(self) -> None:
        if self._on_choice is None:
            return
        pos = self._adm.ray_position()
        if pos is None:
            return
        i, t = pos
        if t < self._length - self._end_tol:
            return
        name = self.names[i]
        p = self._bandit.probabilities()[name]
        rewarded = self._bandit.choose(name)
        on_choice, self._on_choice = self._on_choice, None
        self._node.get_logger().info(f"N-choice: {name} (p = {p:.2f}) -- " + ("reward." if rewarded else "no reward."))
        on_choice(name, rewarded)
