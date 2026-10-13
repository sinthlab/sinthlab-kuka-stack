#!/usr/bin/env python3
"""Resistance of the pull -- felt stiffness below the SmartPad profile, chosen per trial.

The cabinet's stiffness is fixed when the Sunrise app starts (SmartPad profile); FRI streams only the
spring's EQUILIBRIUM, never its stiffness. So ROS softens the spring the one way it can: during the
pull the equilibrium follows a fraction f of the arm's movement, and the spring is stretched by only
(1 - f) of it:

    q_cmd = q_hold + f * (q_measured - q_rest)        felt stiffness = base * (1 - f)
    f     = 1 - stiffness_n_per_m / base_stiffness_n_per_m

  q_rest  the arm where it rests when the monitor arms (its settled pose, sag included), so the
          resting sag is unchanged and only the PULL is softened
  q_hold  the equilibrium being commanded at that moment (the start, or the perturbed pose)

It works in joint space on the joint-position controller apple pluck and perturb already use -- no IK.
For pulls of a few cm, f * (q - q_rest) moves the end effector by ~f times the pull, so the felt
stiffness is ~base * (1 - f) in every direction. Orientation and the elbow soften by the same fraction.

SAFETY. The felt stiffness never goes below MIN_FELT_STIFFNESS (50 N/m), so that much spring always
remains whatever the base: following the arm completely would leave nothing holding it up (the
collapse freeze_at_pose.py describes). With
f = 0 (stiffness == base) nothing is published and the arm behaves exactly as with the bare profile.

Streams from start() (the monitor arms) to stop() (the threshold, where FreezeAtPoseAction takes over
from the equilibrium this left behind, so the hand-over is continuous).
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.helpers.common_threshold import get_optional_param
from sinthlab_bringup.helpers.live_params import MIN_FELT_STIFFNESS


class ResistanceSoftener:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "resistance", update_rate: int = 100) -> None:
        self._node = node
        self._p = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        self._dt = 1.0 / float(update_rate)
        robot_name = node.get_namespace().strip("/")
        ns = f"/{robot_name}" if robot_name else ""
        self._pub = node.create_publisher(LBRJointPositionCommand, f"{ns}/command/lbr_joint_position_command", 1)
        node.create_subscription(LBRState, "lbr_state", self._on_state, 1)
        self._measured: Optional[np.ndarray] = None
        self._commanded: Optional[np.ndarray] = None
        self._active = False
        self._q_rest: Optional[np.ndarray] = None
        self._q_hold: Optional[np.ndarray] = None
        self.reload()
        node.create_timer(self._dt, self._step)

    def reload(self) -> None:
        """Re-read the felt stiffness. Called at a trial boundary (live parameter)."""
        n, p = self._node, self._p
        self._base = float(get_optional_param(n, p + "base_stiffness_n_per_m", 0.0))
        k = float(get_optional_param(n, p + "stiffness_n_per_m", self._base))
        if self._base <= 0.0:
            self._follow = 0.0
        else:
            k = min(max(k, MIN_FELT_STIFFNESS), self._base)     # 50 N/m .. the base
            self._follow = max(1.0 - k / self._base, 0.0)
        n.get_logger().info(
            f"Resistance: {self.felt_stiffness():g} N/m felt (base {self._base:g} N/m from the SmartPad, "
            f"equilibrium follows {100 * self._follow:.0f} % of the pull)")

    def felt_stiffness(self) -> float:
        return self._base * (1.0 - self._follow)

    def follow_fraction(self) -> float:
        return self._follow

    def describe(self) -> dict:
        """For the trial sidecar: what this trial actually ran with."""
        return {"base_stiffness_n_per_m": self._base, "felt_stiffness_n_per_m": self.felt_stiffness(),
                "follow_fraction": self._follow}

    def start(self) -> None:
        """The monitor has armed: from here the pull is softened."""
        if self._follow <= 0.0 or self._measured is None:
            return
        self._q_rest = self._measured.copy()
        self._q_hold = self._commanded.copy() if self._commanded is not None else self._measured.copy()
        self._active = True

    def stop(self) -> None:
        self._active = False

    def _on_state(self, msg: LBRState) -> None:
        qm = np.array(msg.measured_joint_position, dtype=float)
        qc = np.array(msg.commanded_joint_position, dtype=float)
        if np.all(np.isfinite(qm)):
            self._measured = qm
        if np.all(np.isfinite(qc)) and np.any(qc != 0.0):
            self._commanded = qc

    def command_for(self, q_measured: np.ndarray) -> np.ndarray:
        return self._q_hold + self._follow * (q_measured - self._q_rest)

    def _step(self) -> None:
        if not self._active or self._measured is None:
            return
        cmd = LBRJointPositionCommand()
        cmd.joint_position = [float(v) for v in self.command_for(self._measured)]
        self._pub.publish(cmd)
