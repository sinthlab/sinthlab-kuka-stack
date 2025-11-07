#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand

from lbr_demos_advanced_py.admittance_controller import AdmittanceController
from lbr_demos_advanced_py.lbr_base_position_command_node import (
    LBRBasePositionCommandNode,
)


class ImpedancePositionControlNode(LBRBasePositionCommandNode):
    """Software impedance wrapper that operates the robot in FRI position mode."""

    def __init__(self) -> None:
        super().__init__(node_name="impedance_position_control")

        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("end_effector_link", "lbr_link_ee")
        self.declare_parameter("force_threshold", [2.0, 2.0, 2.0, 0.5, 0.5, 0.5])
        self.declare_parameter("dq_gains", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("dx_gains", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.declare_parameter("exp_smooth", 0.95)

        self._init = False
        self._lbr_state = LBRState()
        self._exp_smooth = float(self.get_parameter("exp_smooth").value)
        if not 0.0 <= self._exp_smooth <= 1.0:
            raise ValueError("Exponential smoothing factor must be in [0, 1].")

        base_link = str(self.get_parameter("base_link").value)
        end_effector_link = str(self.get_parameter("end_effector_link").value)
        force_threshold = np.array(self.get_parameter("force_threshold").value, dtype=float)
        dq_gains = np.array(self.get_parameter("dq_gains").value, dtype=float)
        dx_gains = np.array(self.get_parameter("dx_gains").value, dtype=float)

        self._impedance = AdmittanceController(
            robot_description=self._robot_description,
            base_link=base_link,
            end_effector_link=end_effector_link,
            f_ext_th=force_threshold,
            dq_gains=dq_gains,
            dx_gains=dx_gains,
        )

        self._log_parameters()

    def _log_parameters(self) -> None:
        self.get_logger().info("*** Impedance position-control parameters")
        self.get_logger().info("*   base_link: %s" % self.get_parameter("base_link").value)
        self.get_logger().info(
            "*   end_effector_link: %s" % self.get_parameter("end_effector_link").value
        )
        self.get_logger().info(
            "*   force_threshold: %s" % self.get_parameter("force_threshold").value
        )
        self.get_logger().info("*   dq_gains: %s" % self.get_parameter("dq_gains").value)
        self.get_logger().info("*   dx_gains: %s" % self.get_parameter("dx_gains").value)
        self.get_logger().info("*   exp_smooth: %.3f" % self._exp_smooth)

    def _smooth_state(self, lbr_state: LBRState) -> None:
        if not self._init:
            self._lbr_state = lbr_state
            self._init = True
            return

        mjp = np.array(self._lbr_state.measured_joint_position.tolist(), dtype=float)
        new_mjp = np.array(lbr_state.measured_joint_position.tolist(), dtype=float)
        self._lbr_state.measured_joint_position = (
            (1.0 - self._exp_smooth) * mjp + self._exp_smooth * new_mjp
        ).tolist()

        torque = np.array(self._lbr_state.external_torque.tolist(), dtype=float)
        new_torque = np.array(lbr_state.external_torque.tolist(), dtype=float)
        self._lbr_state.external_torque = (
            (1.0 - self._exp_smooth) * torque + self._exp_smooth * new_torque
        ).tolist()

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        self._smooth_state(lbr_state)
        command: LBRJointPositionCommand = self._impedance(self._lbr_state, self._dt)
        self._lbr_joint_position_command_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Node = ImpedancePositionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
