#!/usr/bin/env python3
"""Initial perturbation move.

From the current start pose, displace the end-effector by a POLAR ``(r, theta)`` offset and drive
there in joint space. This is a joint-space move (it subclasses ``MoveToPositionJointSpace`` -> same
``LBRJointPositionCommand`` streaming and joint-space completion); only the *target resolution*
differs: the target joints are recomputed each run from the live start EE pose via a damped-least-
squares position IK seeded at the start config (so the posture stays clean, no nullspace flip). The
cabinet (LbrImpedanceControlServer) provides the compliance.

Polar plane convention:
  * ``horizontal`` -> X-Y (theta from +X, CCW toward +Y)
  * ``sagittal``   -> X-Z (theta from +X, CCW toward +Z)
  * ``frontal``    -> Y-Z (theta from +Y, CCW toward +Z)
"""
from __future__ import annotations

import numpy as np
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.helpers.common_threshold import get_required_param


class PerturbInitialPosition(MoveToPositionJointSpace):
    def _configure_target(self, node: rclpyNode) -> None:
        # Polar perturbation: the joint target is resolved at run time (see _resolve_target),
        # not read from a static target_joint_position.
        self._polar_r_m = float(get_required_param(node, self._param_prefix + "polar_r_m"))
        self._polar_theta_deg = float(get_required_param(node, self._param_prefix + "polar_theta_deg"))
        self._polar_plane = "horizontal"
        if node.has_parameter(self._param_prefix + "polar_plane"):
            self._polar_plane = str(node.get_parameter(self._param_prefix + "polar_plane").value).strip().lower()
        # Joint-space completion (inherited) needs this flag; perturbation defaults to the anchor.
        self._wait_for_physical_arrival = False
        if node.has_parameter(self._param_prefix + "wait_for_physical_arrival"):
            self._wait_for_physical_arrival = bool(
                node.get_parameter(self._param_prefix + "wait_for_physical_arrival").value
            )
        # Placeholder until resolved from the live start pose.
        self._joint_pos_target = np.zeros(7)

    def _resolve_target(self) -> None:
        """Offset the live start EE position by (r, theta) in the configured plane, then IK to joints."""
        start_pos = np.asarray(self._fk_func(self._q_init), dtype=float)[0:3, 3]
        r = self._polar_r_m
        th = np.radians(self._polar_theta_deg)
        if self._polar_plane == "sagittal":      # X-Z
            offset = np.array([r * np.cos(th), 0.0, r * np.sin(th)])
        elif self._polar_plane == "frontal":     # Y-Z
            offset = np.array([0.0, r * np.cos(th), r * np.sin(th)])
        else:                                     # "horizontal" (default): X-Y
            offset = np.array([r * np.cos(th), r * np.sin(th), 0.0])
        target_pos = start_pos + offset

        q_target = self._ik_position_dls(target_pos, self._q_init)
        self._joint_pos_target = q_target

        achieved = np.asarray(self._fk_func(q_target), dtype=float)[0:3, 3]
        residual_mm = float(np.linalg.norm(achieved - target_pos)) * 1000.0
        self._node.get_logger().info(
            f"Polar perturbation: r={r:.3f} m, theta={self._polar_theta_deg:.1f} deg, plane={self._polar_plane} "
            f"-> EE {np.round(start_pos, 4)} -> {np.round(target_pos, 4)} (IK residual {residual_mm:.1f} mm); "
            f"target joints (deg)={np.round(np.degrees(q_target), 2)}"
        )
        if residual_mm > 5.0:
            self._node.get_logger().warn(
                f"Polar IK residual {residual_mm:.1f} mm > 5 mm — target may be near a joint limit or "
                f"singular; reduce polar_r_m or change polar_theta_deg/polar_plane."
            )

    def _ik_position_dls(self, target_pos: np.ndarray, q_seed: np.ndarray,
                         iters: int = 200, damping: float = 0.05, tol: float = 1e-5,
                         max_step: float = 0.2) -> np.ndarray:
        """Position-only damped-least-squares IK: dq = J^T (J J^T + lambda^2 I)^-1 e, seeded at q_seed."""
        q = np.array(q_seed, dtype=float).copy()
        eye3 = np.eye(3)
        for _ in range(iters):
            pos = np.asarray(self._fk_func(q), dtype=float)[0:3, 3]
            err = np.asarray(target_pos, dtype=float) - pos
            if np.linalg.norm(err) < tol:
                break
            jac = self._position_jacobian(q)
            dq = jac.T @ np.linalg.solve(jac @ jac.T + (damping ** 2) * eye3, err)
            step = float(np.linalg.norm(dq))
            if step > max_step:
                dq *= (max_step / step)
            q = q + dq
        return q

    def _position_jacobian(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        """Numeric 3x7 EE-position Jacobian via forward differences on FK."""
        base = np.asarray(self._fk_func(q), dtype=float)[0:3, 3]
        jac = np.zeros((3, 7))
        for i in range(7):
            dq = np.array(q, dtype=float).copy()
            dq[i] += eps
            jac[:, i] = (np.asarray(self._fk_func(dq), dtype=float)[0:3, 3] - base) / eps
        return jac

    def _describe_target(self) -> str:
        return f"POLAR r={self._polar_r_m:.3f} m, theta={self._polar_theta_deg:.1f} deg, plane={self._polar_plane}"
