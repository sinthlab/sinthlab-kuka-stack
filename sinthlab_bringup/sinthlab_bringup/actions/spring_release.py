#!/usr/bin/env python3
"""Spring release — make the arm "give in" (apple breaks off the branch).

During the pull the cabinet runs Cartesian impedance with the equilibrium held at the start/perturbed
pose, so the operator feels a rising spring force. At threshold we want that force to *vanish* — the
arm should stop pulling back and yield. This action zeroes the spring by moving the commanded
equilibrium onto the arm itself: it ramps the command from the held pose to the **live measured**
joint position over `ramp_sec` (a smooth "tear", not a step that could trip a velocity guard), then
holds the measured pose (equilibrium = current pose → ~0 spring force → the arm free-floats).

Joint mode only (apple-pluck / perturb, LBRJointPositionCommandController). Streams from start() to
stop().
"""
from __future__ import annotations

from typing import Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand


class SpringReleaseAction:
    def __init__(self, node: rclpyNode, *, ramp_sec: float = 0.3, update_rate: int = 100) -> None:
        self._node = node
        self._ramp_sec = float(ramp_sec)
        self._dt = 1.0 / float(update_rate)

        robot_name = node.get_namespace().strip("/")
        ns = f"/{robot_name}" if robot_name else ""
        self._cmd_topic = f"{ns}/command/lbr_joint_position_command"
        self._pub = node.create_publisher(LBRJointPositionCommand, self._cmd_topic, 1)
        # lbr_state_broadcaster publishes on '<ns>/lbr_state'.
        self._state_sub = node.create_subscription(LBRState, "lbr_state", self._on_state, 1)

        self._measured: Optional[np.ndarray] = None
        self._commanded: Optional[np.ndarray] = None
        self._active = False
        self._t = 0.0
        self._q0: Optional[np.ndarray] = None  # equilibrium held at the moment of release

        self._timer = node.create_timer(self._dt, self._step)

    def start(self) -> None:
        self._active = True
        self._t = 0.0
        # Start the ramp from the equilibrium that was being held (commanded), falling back to measured.
        self._q0 = self._commanded if self._commanded is not None else self._measured
        self._node.get_logger().info("SpringRelease: releasing spring — arm gives / free-floats.")

    def stop(self) -> None:
        self._active = False
        self._node.get_logger().info("SpringRelease: stopped.")

    def _on_state(self, msg: LBRState) -> None:
        qm = np.array(msg.measured_joint_position.tolist(), dtype=float)
        qc = np.array(msg.commanded_joint_position.tolist(), dtype=float)
        if not np.isnan(qm).any():
            self._measured = qm
        if not np.isnan(qc).any():
            self._commanded = qc

    def _step(self) -> None:
        if not self._active or self._measured is None:
            return
        if self._q0 is None:
            self._q0 = self._measured
        self._t += self._dt
        alpha = 1.0 if self._ramp_sec <= 0.0 else min(1.0, self._t / self._ramp_sec)
        # Equilibrium glides from the held pose onto the live arm pose; at alpha=1 it tracks the arm
        # (zero spring force).
        cmd_q = self._q0 + alpha * (self._measured - self._q0)
        cmd = LBRJointPositionCommand()
        cmd.joint_position = [float(v) for v in cmd_q]
        self._pub.publish(cmd)
