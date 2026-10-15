#!/usr/bin/env python3
"""Move to the PARK pose (`move_to_park`): out of the monkey's reach, between trials.

Every experiment runs   park -> (hold_sec) -> start -> trial -> park -> (hold_sec) -> start -> ...
The run's first move goes to park from wherever the arm is -- the SmartPad app's ready pose, or where
a previous run left it -- at `first_move_speed_scale` (slow: for the maze family that move passes a
straight elbow, see below); every later move uses `speed_scale` as usual.

Start and park must bend the elbow (A4) the same way. Between poses whose A4 have opposite signs,
every joint-space move passes A4 = 0 -- a straight, singular elbow, where the cabinet's Cartesian
impedance cannot follow. A park pose on the other side from the start is refused at start-up.
"""
from __future__ import annotations

from typing import Callable, Optional

from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.helpers.live_params import check_same_elbow
from sinthlab_bringup.helpers.node_utils import get_required_param


class ParkMove(MoveToPositionJointSpace):
    def __init__(self, node: rclpyNode, *, on_complete: Callable[[], None]) -> None:
        super().__init__(node, param_prefix="move_to_park", on_complete=self._done)
        self._after = on_complete
        self._first_scale = float(get_required_param(node, "move_to_park.first_move_speed_scale"))
        self._on_first: Optional[Callable[[], None]] = None
        self._normal_scale = self._speed_scale
        # The wait at park before the next trial's move to the start (live: move_to_park.hold_sec).
        self._then: Optional[Callable[[], None]] = None
        self._hold = WaitAction(node, duration_sec=float(get_required_param(node, "move_to_park.hold_sec")),
                                on_complete=self._held_done, name="park_hold")
        problem = check_same_elbow(get_required_param(node, "move_to_start.target_joint_position"),
                                   get_required_param(node, "move_to_park.target_joint_position"))
        if problem:
            raise ValueError(f"move_to_park: {problem}")

    def reload(self) -> None:
        super().reload()
        self._hold.set_duration(float(get_required_param(self._node, "move_to_park.hold_sec")))

    def wait_then(self, fn: Callable[[], None]) -> None:
        """Wait hold_sec at park, then call fn (the next trial)."""
        self._then = fn
        self._hold.start()

    def cancel_hold(self) -> bool:
        """Drop a pending wait at park (Stop now); True if there was one."""
        if not self._hold.active():
            return False
        self._hold.stop()
        self._then = None
        return True

    def _held_done(self) -> None:
        fn, self._then = self._then, None
        if fn is not None:
            fn()

    def start_first(self, on_parked: Callable[[], None]) -> None:
        """The run's first move: to park from wherever the arm is, at first_move_speed_scale."""
        self._on_first = on_parked
        self._normal_scale = self._speed_scale
        self._speed_scale = self._first_scale
        self._node.get_logger().info(
            f"First move: to the park pose at speed scale {self._first_scale:g}, from wherever the arm is.")
        self.start()

    def set_speed_scale(self, scale: float) -> None:
        if self._on_first is None:
            super().set_speed_scale(scale)
        else:                        # the slow first move keeps its speed; the change applies after it
            self._normal_scale = float(scale)

    def _done(self) -> None:
        if self._on_first is not None:
            on_parked, self._on_first = self._on_first, None
            self._speed_scale = self._normal_scale
            on_parked()
        else:
            self._after()
