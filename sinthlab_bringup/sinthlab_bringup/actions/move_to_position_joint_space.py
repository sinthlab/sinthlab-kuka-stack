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
        self._read_arrival_params(node)

    def _read_arrival_params(self, node: rclpyNode) -> None:
        """Optionally wait for the PHYSICAL arm to arrive, not just the commanded anchor.

          wait_for_physical_arrival     bool  also require the measured arm to have arrived
          cartesian_move_tolerance      [m]   judge that by END-EFFECTOR position; without it, every
                                              measured joint must be within joint_move_tolerance
          physical_arrival_timeout_sec  [s]   stop waiting after this long, with a warning, instead of
                                              waiting forever
        """
        def opt(name, cast, default):
            key = self._param_prefix + name
            if not node.has_parameter(key):
                return default
            value = node.get_parameter(key).value
            return default if value is None else cast(value)
        self._wait_for_physical_arrival = opt("wait_for_physical_arrival", bool, False)
        self._cartesian_tol = opt("cartesian_move_tolerance", float, None)
        self._arrival_timeout = opt("physical_arrival_timeout_sec", float, None)
        self._arrival_wait = 0.0
        self._next_stall_log = 2.0

    def start(self) -> None:
        self._arrival_wait = 0.0
        self._next_stall_log = 2.0
        super().start()

    def _ee_pos(self, q: np.ndarray) -> np.ndarray:
        return np.asarray(self._fk_func(q), dtype=float)[0:3, 3]

    def _configure_command(self, node: rclpyNode, ns: str) -> None:
        self._cmd_topic = f"{ns}/command/lbr_joint_position_command"
        self._pub = node.create_publisher(LBRJointPositionCommand, self._cmd_topic, 1)

    def _publish_setpoint(self, joint_positions: np.ndarray) -> None:
        cmd = LBRJointPositionCommand()
        cmd.joint_position = [float(q) for q in joint_positions]
        self._pub.publish(cmd)

    def _completion_reached(self) -> bool:
        # The COMMANDED trajectory is always judged in joint space (the controller commands exactly these
        # joints), so a move can't "finish" before Ruckig has ramped the equilibrium there. When
        # wait_for_physical_arrival is set (e.g. recover), the measured arm must also have arrived.
        # (Checking measured alone let recover complete early when the arm sprang back near the target
        # before the ramp finished -- leaving the equilibrium parked at the perturbed pose.)
        #
        # With cartesian_move_tolerance, physical arrival is judged by END-EFFECTOR position. Joint angles
        # are the wrong test for a 7-DOF arm under Cartesian impedance: the elbow can swing (null space)
        # with the end effector exactly at the target, and the cabinet holds that swing only weakly. At
        # the apple-pluck start a 1.6 cm elbow swing moves A1 by 0.15 rad without moving the apple at all,
        # so after a pull a joint could rest outside tolerance and recover waited forever.
        cmd_err = float(np.max(np.abs(self._joint_pos_target - self._q_cmd_completion)))
        meas_errs = np.abs(self._joint_pos_target - self._q_meas_completion)
        meas_err = float(np.max(meas_errs))
        cmd_ok = cmd_err <= self._joint_pos_tol
        ee_err = None
        if not self._wait_for_physical_arrival:
            reached = cmd_ok
        else:
            if self._cartesian_tol is not None:
                ee_err = float(np.linalg.norm(
                    self._ee_pos(self._q_meas_completion) - self._ee_pos(self._joint_pos_target)))
                phys_ok = ee_err <= self._cartesian_tol
            else:
                phys_ok = meas_err <= self._joint_pos_tol
            reached = cmd_ok and phys_ok
            if cmd_ok and not phys_ok:
                self._arrival_wait += self._dt
                where = self._arrival_text(ee_err, meas_errs)
                if self._arrival_timeout is not None and self._arrival_wait >= self._arrival_timeout:
                    self._node.get_logger().warn(
                        f"{type(self).__name__}: the arm did not physically arrive within "
                        f"{self._arrival_timeout:.1f} s ({where}); continuing anyway."
                    )
                    return True
                if self._arrival_wait >= self._next_stall_log:
                    self._next_stall_log += 2.0
                    self._node.get_logger().warn(
                        f"{type(self).__name__}: commanded pose reached; waiting for the arm to physically "
                        f"arrive ({where})."
                    )
        if reached:
            ee = f", EE err {ee_err * 1000:.0f} mm" if ee_err is not None else ""
            self._node.get_logger().info(
                f"{type(self).__name__} reached target joints "
                f"(cmd err {cmd_err:.4f}, meas err {meas_err:.4f} rad{ee}); holding."
            )
            return True
        if self._debug_log_enabled and self._dbg.tick(self._dt):
            self._node.get_logger().info(
                f"{type(self).__name__}: cmd err {cmd_err:.4f}, meas err {meas_err:.4f} "
                f"(tol {self._joint_pos_tol:.4f} rad)" + (f", EE err {ee_err * 1000:.0f} mm" if ee_err is not None else "")
            )
        return False

    def _arrival_text(self, ee_err, meas_errs) -> str:
        worst = int(np.argmax(meas_errs))
        joint = f"largest joint error A{worst + 1} {np.degrees(meas_errs[worst]):.1f} deg"
        if ee_err is None:
            return f"{joint}, tolerance {np.degrees(self._joint_pos_tol):.1f} deg"
        return f"end effector {ee_err * 1000:.0f} mm from target, tolerance {self._cartesian_tol * 1000:.0f} mm; {joint}"

    def _describe_target(self) -> str:
        return f"joint target(rad)={np.round(self._joint_pos_target, 4).tolist()}"
