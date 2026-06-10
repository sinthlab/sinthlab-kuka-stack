#!/usr/bin/env python3
"""Freeze the arm at its current pose when the pull threshold is reached.

At the threshold this captures the current (pulled) pose **once** and smoothly ramps the
commanded equilibrium onto it, then **holds that fixed pose**. Effect:
  * the spring stops pulling the arm back toward the start/perturbed pose (the felt pull drops to ~0),
  * but the arm stays supported by the cabinet's normal stiffness *around the frozen pose* — if it
    sags, the spring restores it. It does not go limp or fall.

Note: the ROS side cannot change the cabinet's stiffness (that is fixed by the SmartPad impedance
profile); the only safe lever here is *where* the equilibrium is, so we freeze it on the arm.
Joint mode only (apple-pluck / perturb). Streams from start() until stop().
"""
from __future__ import annotations

from typing import Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand


class FreezeAtPoseAction:
    def __init__(self, node: rclpyNode, *, ramp_sec: float = 0.3, update_rate: int = 100) -> None:
        self._node = node
        self._ramp_sec = float(ramp_sec)
        self._dt = 1.0 / float(update_rate)

        robot_name = node.get_namespace().strip("/")
        ns = f"/{robot_name}" if robot_name else ""
        self._cmd_topic = f"{ns}/command/lbr_joint_position_command"
        self._pub = node.create_publisher(LBRJointPositionCommand, self._cmd_topic, 1)
        self._state_sub = node.create_subscription(LBRState, "lbr_state", self._on_state, 1)

        self._measured: Optional[np.ndarray] = None
        self._commanded: Optional[np.ndarray] = None
        self._active = False
        self._t = 0.0
        self._q_frozen: Optional[np.ndarray] = None  # pose captured ONCE at threshold, held thereafter
        self._q0: Optional[np.ndarray] = None        # equilibrium held at threshold (ramp start)

    def start(self) -> None:
        self._active = True
        self._t = 0.0
        self._q_frozen = None
        self._q0 = None
        self._node.get_logger().info(
            "FreezeAtPose: holding the arm at its current pose (pull-back released, position frozen)."
        )

    def stop(self) -> None:
        self._active = False
        self._node.get_logger().info("FreezeAtPose: stopped.")

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
        if self._q_frozen is None:
            # Capture the threshold pose ONCE (not live), and ramp the equilibrium onto it from the
            # pose that was being held. After the ramp the command is constant -> frozen, with full
            # restoring stiffness around it (no runaway).
            self._q_frozen = self._measured.copy()
            self._q0 = self._commanded.copy() if self._commanded is not None else self._measured.copy()
        self._t += self._dt
        alpha = 1.0 if self._ramp_sec <= 0.0 else min(1.0, self._t / self._ramp_sec)
        cmd_q = self._q0 + alpha * (self._q_frozen - self._q0)
        cmd = LBRJointPositionCommand()
        cmd.joint_position = [float(v) for v in cmd_q]
        self._pub.publish(cmd)
