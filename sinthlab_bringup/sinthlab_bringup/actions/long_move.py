#!/usr/bin/env python3
"""Long-movement reward in Free Move: every movement longer than long_move.min_m is rewarded.

With long_move.reward `each_move` (the Free Move card's "Long moves" tab), a Free Move session pays for
each long enough movement instead of for its end. The session itself runs as before: for session_sec,
or until Stop. The movement logic is helpers/long_moves.py (no ROS, unit-tested); this samples the arm
where it is (AdmittanceMoveAction.offset_from_anchor) and reports:

    move_start   a movement has gone confirm_m from where the arm rested      (no argument)
    move_end     it has ended                                                 "length=..;straightness=..;
                                                                               duration=..;peak_speed=.."
    long_move    ...and earned the reward (the reward pulse and tone follow)  its length [m]

It only watches and reports: the motion is the admittance's.

Parameters (all live, read at the next session; long_move.reward is the card's paradigm):
    long_move.reward            str    session (the session's end pays, as Free Move) | each_move
    long_move.min_m             float  shortest movement that is rewarded [m]
    long_move.min_straightness  float  0-1: length / path travelled at least this (0 = not checked)
    long_move.min_gap_sec       float  at least this long between two rewards [s]
    long_move.start_speed_mps   float  a movement starts above this speed [m/s]
    long_move.stop_speed_mps    float  ...and ends once below this one for stop_sec [m/s]
    long_move.stop_sec          float  [s]
    long_move.confirm_m         float  a movement counts once it has gone this far [m]
"""
from __future__ import annotations

import time
from typing import Callable, Optional

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.long_moves import LongMoveRule, MovementSegmenter
from sinthlab_bringup.helpers.node_utils import get_required_param

MODES = ("session", "each_move")
RATE_HZ = 50.0


class LongMoveMonitor:
    def __init__(self, node: rclpyNode, admittance, *, param_prefix: str = "long_move") -> None:
        self._node = node
        self._adm = admittance
        self._p = param_prefix + "."
        self._seg = MovementSegmenter()
        self._rule = LongMoveRule()
        self._on_event: Optional[Callable[[str, object], None]] = None
        self.rewards = 0                   # this session's
        self.reload()
        node.create_timer(1.0 / RATE_HZ, self._tick)

    def reload(self) -> None:
        n, p = self._node, self._p
        mode = str(get_required_param(n, p + "reward")).strip().lower()
        if mode not in MODES:
            raise ValueError(f"{p}reward must be one of {MODES}, got {mode!r}")
        self.enabled = mode == "each_move"
        self._seg.configure(float(get_required_param(n, p + "start_speed_mps")),
                            float(get_required_param(n, p + "stop_speed_mps")),
                            float(get_required_param(n, p + "stop_sec")),
                            float(get_required_param(n, p + "confirm_m")))
        self._rule.configure(float(get_required_param(n, p + "min_m")),
                             float(get_required_param(n, p + "min_straightness")),
                             float(get_required_param(n, p + "min_gap_sec")))
        if self.enabled:
            r = self._rule
            n.get_logger().info(
                f"Long moves: a reward for every movement of at least {r.min_m * 100:.0f} cm"
                + (f", straightness >= {r.min_straightness:.2f}" if r.min_straightness > 0 else "")
                + f", at least {r.min_gap:g} s apart.")

    def check_live(self, name: str, value, context: dict) -> Optional[str]:
        """ExperimentControl validator: the stop speed may not exceed the start speed."""
        if name not in (self._p + "start_speed_mps", self._p + "stop_speed_mps"):
            return None
        start = float(context.get(self._p + "start_speed_mps", self._seg.start_speed))
        stop = float(context.get(self._p + "stop_speed_mps", self._seg.stop_speed))
        return None if 0 < stop <= start else f"{self._p}stop_speed_mps ({stop:g}) must be > 0 and <= start_speed_mps ({start:g})"

    def start(self, on_event: Callable[[str, object], None]) -> None:
        """At the go cue: watch the session's movements, reporting through on_event(token, arg)."""
        self._seg.reset()
        self._rule.reset()
        self.rewards = 0
        self._on_event = on_event

    def stop(self) -> None:
        self._on_event = None

    def _tick(self) -> None:
        if self._on_event is None:
            return
        p = self._adm.offset_from_anchor()
        if p is None:
            return
        kind, m = self._seg.update(time.monotonic(), p)
        if kind == "start":
            self._on_event("move_start", None)
        elif kind == "end":
            self._on_event("move_end", m.summary())
            ok, why = self._rule.check(m)
            if ok:
                self.rewards += 1
                self._node.get_logger().info(f"Long move {self.rewards}: {why} -- reward.")
                self._on_event("long_move", round(m.length_m, 3))
            else:
                self._node.get_logger().info(f"Movement {m.length_m * 100:.1f} cm: {why}, no reward.")
