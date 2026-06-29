#!/usr/bin/env python3
"""Cartesian-space PTP move for the ``kuka_clik_controller``.

From the arm's current configuration, drive to an absolute target joint configuration
(``target_joint_position``): Ruckig still plans a joint spline, but each sample is published as
the FK ``PoseStamped`` on the CLIK controller's ``target_frame``, which re-solves IK with its own
nullspace bias. Because CLIK won't reproduce the exact joints, completion is judged on the EE
**position** (``cartesian_move_tolerance``, default 0.03 m).

Completion uses the **commanded** EE pose, NOT the measured one: under the cabinet's Cartesian
impedance the physical arm sags below the commanded equilibrium, so the measured EE would never
reach the target and the move would hang forever (no quiet window, no start cue). The commanded
equilibrium (FRI commanded joints -> FK) does reach the target, so we key completion off that.
Used by the restricted-plane scenario.
"""
from __future__ import annotations

import numpy as np
from rclpy.node import Node as rclpyNode

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from sinthlab_bringup.actions._move_action_base import MoveActionBase


class MoveToPositionCartesianSpace(MoveActionBase):
    def _configure_target(self, node: rclpyNode) -> None:
        self._joint_pos_target = self._read_joint_target_param(node)
        # EE position tolerance [m]; optional, defaults to 0.03 m.
        self._cartesian_tol = 0.03
        if node.has_parameter(self._param_prefix + "cartesian_move_tolerance"):
            self._cartesian_tol = float(
                node.get_parameter(self._param_prefix + "cartesian_move_tolerance").value
            )

    def _configure_command(self, node: rclpyNode, ns: str) -> None:
        self._cmd_topic = f"{ns}/kuka_clik_controller/target_frame"
        self._pub = node.create_publisher(PoseStamped, self._cmd_topic, 1)

    def _publish_setpoint(self, joint_positions: np.ndarray) -> None:
        pose_mat = np.asarray(self._fk_func(joint_positions), dtype=float)
        cmd = PoseStamped()
        cmd.header.frame_id = "lbr_link_0"
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.pose.position.x = float(pose_mat[0, 3])
        cmd.pose.position.y = float(pose_mat[1, 3])
        cmd.pose.position.z = float(pose_mat[2, 3])
        quat = R.from_matrix(pose_mat[0:3, 0:3]).as_quat()
        cmd.pose.orientation.x = float(quat[0])
        cmd.pose.orientation.y = float(quat[1])
        cmd.pose.orientation.z = float(quat[2])
        cmd.pose.orientation.w = float(quat[3])
        self._pub.publish(cmd)

    def _completion_reached(self) -> bool:
        # Judge on the COMMANDED equilibrium (FRI commanded joints), not the measured arm: under
        # Cartesian impedance the measured EE sags below the equilibrium and would never reach the
        # target, hanging the move. The commanded EE does reach it once Ruckig finishes streaming.
        target_ee = np.asarray(self._fk_func(self._joint_pos_target), dtype=float)
        cmd_ee = np.asarray(self._fk_func(self._q_cmd_completion), dtype=float)
        pos_err = float(np.linalg.norm(cmd_ee[0:3, 3] - target_ee[0:3, 3]))
        if pos_err <= self._cartesian_tol:
            meas_ee = np.asarray(self._fk_func(self._q_meas_completion), dtype=float)
            meas_err = float(np.linalg.norm(meas_ee[0:3, 3] - target_ee[0:3, 3]))
            self._node.get_logger().info(
                f"{type(self).__name__} reached target EE pose "
                f"(cmd err {pos_err:.4f} m <= {self._cartesian_tol:.4f} m; measured {meas_err:.4f} m incl. droop); holding."
            )
            return True
        if self._debug_log_enabled and self._dbg.tick(self._dt):
            self._node.get_logger().info(
                f"{type(self).__name__}: commanded EE error {pos_err:.4f} m (tol {self._cartesian_tol:.4f} m)"
            )
        return False

    def _describe_target(self) -> str:
        ee = np.asarray(self._fk_func(self._joint_pos_target), dtype=float)[0:3, 3]
        return f"EE target(m)={np.round(ee, 4).tolist()} (via kuka_clik)"
