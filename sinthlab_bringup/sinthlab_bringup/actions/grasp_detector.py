#!/usr/bin/env python3
"""Grasp detector for apple pluck's perturb_bump paradigm: has the hand taken hold of the apple?

There is no force sensor. The hand force comes from the joint torque sensors (helpers/hand_force.py),
so what is detected is FORCE on the apple, not touch: a hand resting on it without pushing or pulling
is not seen. The arm is still while this runs, which is when that estimate is cleanest.

    start()   at the start pose, apple untouched: collect the resting force (the TARE -- the
              tool-load model's error, a few N)
    arm()     at the go cue: the tare is fixed -- the mean over the last tare_sec only, so the arm's
              settling at the start of the quiet window is left out -- and detection begins
              |F - tare| >= threshold_n, continuously for hold_sec  ->  on_grasp(|F - tare|)
    stop()    disarm (timeout, safety, end of trial)

A tare above tare_max_n means the apple was being held while it was measured: the previous trial's
tare is kept instead, with a warning, so a hand already on the apple is not learnt as "rest".
"""
from __future__ import annotations

from collections import deque
from typing import Callable

import numpy as np
from lbr_fri_idl.msg import LBRState
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.hand_force import HandForce
from sinthlab_bringup.helpers.node_utils import DebugTicker, get_launch_param, get_required_param


class GraspDetector:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "grasp",
                 on_grasp: Callable[[float], None]) -> None:
        self._node = node
        self._p = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        self._on_grasp = on_grasp
        self._hand = HandForce(str(get_launch_param(node, "robot_description", "")),
                               str(get_required_param(node, self._p + "ee_frame")),
                               float(get_required_param(node, self._p + "jacobian_damping")))
        self._dbg = DebugTicker(float(get_required_param(node, self._p + "debug_log_rate_hz")))
        self.reload()

        self._taring = False
        self._armed = False
        self._samples: deque = deque()          # (cabinet time, force), the last tare_sec of them
        self._tare = np.zeros(3)
        self._above_sec = 0.0
        self._last_t = None
        node.create_subscription(LBRState, "lbr_state", self._on_state, 1)

    def reload(self) -> None:
        """Re-read the live detection settings. Called between trials."""
        n, p = self._node, self._p
        self._threshold = float(get_required_param(n, p + "threshold_n"))
        self._hold_sec = float(get_required_param(n, p + "hold_sec"))
        self._tare_max = float(get_required_param(n, p + "tare_max_n"))
        self._tare_sec = float(get_required_param(n, p + "tare_sec"))
        self._debug = bool(get_required_param(n, p + "debug_log_enabled"))

    def start(self) -> None:
        """Begin measuring the resting force (apple untouched)."""
        self._samples = deque()
        self._taring, self._armed = True, False

    def arm(self) -> None:
        """Fix the tare and start detecting."""
        if self._samples:
            tare = np.mean([f for _, f in self._samples], axis=0)
            if np.linalg.norm(tare) <= self._tare_max:
                self._tare = tare
            else:
                self._node.get_logger().warn(
                    f"Grasp: resting force {np.round(tare, 1)} N exceeds tare_max_n ({self._tare_max:g} N) "
                    f"-- was the apple being held? Keeping the previous tare {np.round(self._tare, 1)} N.")
        self._taring, self._armed = False, True
        self._above_sec, self._last_t = 0.0, None
        self._node.get_logger().info(
            f"Grasp detector armed: tare {np.round(self._tare, 2)} N, threshold {self._threshold:g} N "
            f"for {self._hold_sec:g} s")

    def stop(self) -> None:
        self._taring = self._armed = False

    def _on_state(self, msg: LBRState) -> None:
        if not (self._taring or self._armed):
            return
        q = np.array(msg.measured_joint_position, dtype=float)
        if not np.all(np.isfinite(q)):
            return
        f = self._hand.force(msg.external_torque, q)
        t = msg.time_stamp_sec + msg.time_stamp_nano_sec * 1e-9    # cabinet clock: exact sample spacing
        if self._taring:
            self._samples.append((t, f))
            while self._samples and t - self._samples[0][0] > self._tare_sec:
                self._samples.popleft()
            return
        dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), 0.05)
        self._last_t = t
        mag = float(np.linalg.norm(f - self._tare))
        self._above_sec = self._above_sec + dt if mag >= self._threshold else 0.0
        if self._debug and self._dbg.tick(dt):
            self._node.get_logger().info(f"grasp: |F| = {mag:.2f} N (threshold {self._threshold:g} N)")
        if self._above_sec >= self._hold_sec:
            self._armed = False
            self._node.get_logger().info(f"Grasp detected: |F| = {mag:.2f} N")
            try:
                self._on_grasp(mag)
            except Exception as exc:
                self._node.get_logger().error(f"GraspDetector callback error: {exc}")
