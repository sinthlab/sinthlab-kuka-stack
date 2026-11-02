#!/usr/bin/env python3
"""The quiet window, guarded: the apple must be left alone until the go cue.

Every experiment waits `quiet_window_sec` at the start before its go cue. The resting references are
taken then -- the pull baseline, the resting pose and force, where a fixture anchors -- so a hand on
the apple during that window corrupts the trial that follows. This is that wait, plus a check: if the
apple is MOVED or PUSHED during it, the trial is broken off (on_broken). The orchestrator ends it
without reward, parks, and adds a time-out (quiet_gate.timeout_sec) before the next trial -- shaping
"wait for the go cue". The trial records event `early_touch` with what broke the window.

Both checks are against the first BASELINE_SEC of the window, so each pose's resting sag and resting
force do not count:
  * movement of the end effector  > quiet_gate.move_m
  * change in the hand force      > quiet_gate.force_n   (from the joint torques, helpers/hand_force.py)
Calibrated on untouched quiet windows (recordings 23 and 26 Oct): the arm still settles by up to
16 mm on Uniform 100 (7 mm on 400) and the force reading wanders by up to 3.2 N, while a hand pulling
early moved it 74 mm with 31 N. Hence 3 cm and 5 N; a very light touch can pass unnoticed.

quiet_gate.enabled false = a plain timed wait, as before. All quiet_gate.* settings are live.
Same start() / stop() / active() / set_duration() shape as WaitAction, which it replaces.
"""
from __future__ import annotations

import time
from typing import Callable, Optional

import numpy as np
import optas
from lbr_fri_idl.msg import LBRState
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.hand_force import HandForce
from sinthlab_bringup.helpers.node_utils import get_launch_param, get_required_param

BASELINE_SEC = 0.2      # the window's first stretch, averaged as the resting reference
JACOBIAN_DAMPING = 0.1  # as Free Move: steady at every start pose


class QuietGate:
    def __init__(self, node: rclpyNode, *, on_complete: Callable[[], None],
                 on_broken: Callable[[str], None], name: str = "quiet_window") -> None:
        self._node = node
        self._on_complete, self._on_broken = on_complete, on_broken
        self._name = name
        robot_description = str(get_launch_param(node, "robot_description", ""))
        self._fk = optas.RobotModel(urdf_string=robot_description).get_global_link_transform_function(
            "lbr_link_ee", numpy_output=True)
        self._hand = HandForce(robot_description, "lbr_link_ee", JACOBIAN_DAMPING)
        self._duration = float(get_required_param(node, "quiet_window_sec"))
        self.reload()
        self._msg: Optional[LBRState] = None
        self._active = False
        node.create_subscription(LBRState, "lbr_state", self._on_state, 1)
        self._timer = node.create_timer(0.02, self._step)

    def reload(self) -> None:
        """Re-read quiet_gate.* (live: from the next trial)."""
        n = self._node
        self._enabled = bool(get_required_param(n, "quiet_gate.enabled"))
        self._force_n = float(get_required_param(n, "quiet_gate.force_n"))
        self._move_m = float(get_required_param(n, "quiet_gate.move_m"))

    def duration_sec(self) -> float:
        return self._duration

    def set_duration(self, duration_sec: float) -> None:
        """Change the window for its NEXT start() (live: quiet_window_sec)."""
        self._duration = float(duration_sec)

    def start(self) -> None:
        self._t0 = time.monotonic()
        self._base_p, self._base_f = [], []
        self._p0 = self._f0 = None
        self._active = True
        self._node.get_logger().info(
            f"Quiet window: {self._duration:.2f} s" + (
                f", broken off if the apple moves > {self._move_m * 100:.1f} cm or is pushed > {self._force_n:g} N"
                if self._enabled else " (not guarded)"))

    def active(self) -> bool:
        return self._active

    def stop(self) -> None:
        self._active = False

    def _on_state(self, msg: LBRState) -> None:
        self._msg = msg

    def _sample(self):
        msg = self._msg
        if msg is None:
            return None
        q = np.array(msg.measured_joint_position, dtype=float)
        if not np.all(np.isfinite(q)):
            return None
        return self._fk(q)[:3, 3], self._hand.force(np.array(msg.external_torque, dtype=float), q)

    def _step(self) -> None:
        if not self._active:
            return
        elapsed = time.monotonic() - self._t0
        if self._enabled:
            s = self._sample()
            if s is not None:
                p, f = s
                if elapsed < BASELINE_SEC or self._p0 is None and not self._base_p:
                    self._base_p.append(p)
                    self._base_f.append(f)
                else:
                    if self._p0 is None:
                        self._p0, self._f0 = np.mean(self._base_p, axis=0), np.mean(self._base_f, axis=0)
                    moved = float(np.linalg.norm(p - self._p0))
                    pushed = float(np.linalg.norm(f - self._f0))
                    if moved > self._move_m or pushed > self._force_n:
                        self._active = False
                        why = (f"the apple moved {moved * 100:.1f} cm" if moved > self._move_m
                               else f"the apple was pushed {pushed:.1f} N")
                        self._node.get_logger().warn(f"Quiet window broken at {elapsed:.2f} s: {why}.")
                        self._on_broken(why)
                        return
        if elapsed >= self._duration:
            self._active = False
            self._on_complete()
