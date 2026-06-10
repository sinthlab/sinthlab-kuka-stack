#!/usr/bin/env python3
"""Joint-space PTP move.

From the arm's current configuration, drive to an absolute target joint configuration
(``target_joint_position``), streaming ``LBRJointPositionCommand`` to the
``LBRJointPositionCommandController``. The joint positions go straight to the FRI position
command (exact, no IK); the cabinet (LbrImpedanceControlServer) provides the compliance.
Completion is judged in joint space. Used by apple-pluck / perturb move-to-start and recover.
"""
from __future__ import annotations

import numpy as np
from rclpy.node import Node as rclpyNode

from lbr_fri_idl.msg import LBRJointPositionCommand
from sinthlab_bringup.actions._move_action_base import MoveActionBase


class MoveToPositionJointSpace(MoveActionBase):
    def _configure_target(self, node: rclpyNode) -> None:
        self._joint_pos_target = self._read_joint_target_param(node)
        # Optionally wait for the *physical* arm (measured) to arrive, not just the command anchor.
        self._wait_for_physical_arrival = False
        if node.has_parameter(self._param_prefix + "wait_for_physical_arrival"):
            self._wait_for_physical_arrival = bool(
                node.get_parameter(self._param_prefix + "wait_for_physical_arrival").value
            )

    def _configure_command(self, node: rclpyNode, ns: str) -> None:
        self._cmd_topic = f"{ns}/command/lbr_joint_position_command"
        self._pub = node.create_publisher(LBRJointPositionCommand, self._cmd_topic, 1)

    def _publish_setpoint(self, joint_positions: np.ndarray) -> None:
        cmd = LBRJointPositionCommand()
        cmd.joint_position = [float(q) for q in joint_positions]
        self._pub.publish(cmd)

    def _completion_reached(self) -> bool:
        # The controller commands exactly these joints, so judge in joint space.
        ref = self._q_meas_completion if self._wait_for_physical_arrival else self._q_cmd_completion
        max_err = float(np.max(np.abs(self._joint_pos_target - ref)))
        if max_err <= self._joint_pos_tol:
            self._node.get_logger().info(
                f"{type(self).__name__} reached target joints "
                f"(max err {max_err:.4f} rad <= {self._joint_pos_tol:.4f} rad); holding."
            )
            return True
        if self._debug_log_enabled and self._dbg.tick(self._dt):
            self._node.get_logger().info(
                f"{type(self).__name__}: joint max err {max_err:.4f} rad (tol {self._joint_pos_tol:.4f} rad)"
            )
        return False

    def _describe_target(self) -> str:
        return f"joint target(rad)={np.round(self._joint_pos_target, 4).tolist()}"
