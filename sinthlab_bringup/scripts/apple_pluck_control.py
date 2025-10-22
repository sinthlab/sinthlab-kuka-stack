#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
import optas

from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState


class ApplePluckControlNode(Node):
    """
    Apple pluck controller: simulates resistance at the end-effector by commanding
    joint positions that oppose the estimated external wrench (negative admittance),
    and gradually releases that resistance over time after a trigger.

    Design notes:
    - No physical FT sensor: wrench is estimated from external joint torques via J^+.
    - Works similar to admittance_controller demo: subscribes to 'state' and publishes
      'command/joint_position' at the controller_manager update_rate.
    - Resistance scale gamma starts near 0.8 and ramps to 0 after a held force trigger.
    """

    def __init__(self):
        super().__init__("apple_pluck_control")

        # Retrieve required system parameters from parameter services (like base demo)
        self._robot_description = self._retrieve_parameter(
            "robot_state_publisher/get_parameters", "robot_description"
        ).string_value
        update_rate_val = self._retrieve_parameter(
            "controller_manager/get_parameters", "update_rate"
        ).integer_value
        if update_rate_val is None or update_rate_val == 0:
            self.get_logger().warn("update_rate parameter missing; defaulting to 100 Hz")
            update_rate_val = 100
        self._update_rate: int = int(update_rate_val)
        self._dt: float = 1.0 / float(self._update_rate)

        # Node parameters (mirror admittance demo with extras for pluck)
        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("end_effector_link", "lbr_link_ee")
        self.declare_parameter("f_ext_th", [2.0, 2.0, 2.0, 0.5, 0.5, 0.5])
        self.declare_parameter("dq_gains", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("dx_gains", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.declare_parameter("exp_smooth", 0.95)
        self.declare_parameter("dq_smooth", 0.95)
        self.declare_parameter("pinv_rcond", 0.1)

        # Apple pluck-specific parameters
        self.declare_parameter("gamma_initial", 0.8)  # initial resistance fraction
        self.declare_parameter("release_trigger_force", 10.0)  # [N] norm threshold
        self.declare_parameter("release_hold_time", 1.0)  # [s] sustain above threshold
        self.declare_parameter("release_duration", 1.5)  # [s] ramp-down time

        # Read and validate parameters
        self._base_link = self.get_parameter("base_link").get_parameter_value().string_value
        self._ee_link = (
            self.get_parameter("end_effector_link").get_parameter_value().string_value
        )
        self._f_ext_th = np.array(self.get_parameter("f_ext_th").value, dtype=float)
        self._dq_gains = np.diag(np.array(self.get_parameter("dq_gains").value, dtype=float))
        self._dx_gains = np.diag(np.array(self.get_parameter("dx_gains").value, dtype=float))
        self._exp_smooth = float(self.get_parameter("exp_smooth").value)
        self._dq_smooth = float(self.get_parameter("dq_smooth").value)
        self._pinv_rcond = float(self.get_parameter("pinv_rcond").value)

        # Pluck parameters
        self._gamma_initial = float(self.get_parameter("gamma_initial").value)
        self._release_trigger_force = float(self.get_parameter("release_trigger_force").value)
        self._release_hold_time = float(self.get_parameter("release_hold_time").value)
        self._release_duration = float(self.get_parameter("release_duration").value)

        if not (0.0 <= self._exp_smooth <= 1.0):
            raise ValueError("exp_smooth must be in [0, 1]")
        if not (0.0 <= self._dq_smooth <= 1.0):
            raise ValueError("dq_smooth must be in [0, 1]")
        # Build kinematic model and Jacobian function
        self._robot = optas.RobotModel(urdf_string=self._robot_description)
        self._jacobian_func = (
            self._robot.get_link_geometric_jacobian_function(
                link=self._ee_link, base_link=self._base_link, numpy_output=True
            )
            if self._robot
            else None
        )

        # Internal state
        self._init = False
        self._lbr_state = LBRState()
        self._dof = 7
        self._q = np.zeros(self._dof)
        self._dq = np.zeros(self._dof)
        self._jacobian = np.zeros((6, self._dof))
        self._jacobian_inv = np.zeros((self._dof, 6))
        self._tau_ext = np.zeros(self._dof)
        self._f_ext = np.zeros(6)

        # Resistance scheduling
        self._gamma = self._gamma_initial
        self._releasing = False
        self._release_elapsed = 0.0
        self._above_thresh_time = 0.0

        # ROS I/O
        self._state_sub = self.create_subscription(LBRState, "state", self._on_state, 1)
        self._cmd_pub = self.create_publisher(LBRJointPositionCommand, "command/joint_position", 1)

        # Log configuration
        self._log_parameters()

    def _log_parameters(self) -> None:
        self.get_logger().info("*** Apple Pluck Control Parameters:")
        self.get_logger().info(f"*   base_link: {self._base_link}")
        self.get_logger().info(f"*   end_effector_link: {self._ee_link}")
        self.get_logger().info(f"*   f_ext_th: {self._f_ext_th.tolist()}")
        self.get_logger().info(f"*   dq_gains: {np.diag(self._dq_gains).tolist()}")
        self.get_logger().info(f"*   dx_gains: {np.diag(self._dx_gains).tolist()}")
        self.get_logger().info(f"*   exp_smooth: {self._exp_smooth}")
        self.get_logger().info(f"*   dq_smooth: {self._dq_smooth}")
        self.get_logger().info(f"*   pinv_rcond: {self._pinv_rcond}")
        self.get_logger().info(
            f"*   gamma_initial: {self._gamma_initial}, release_trigger_force: {self._release_trigger_force}, "
            f"hold_time: {self._release_hold_time}, release_duration: {self._release_duration}"
        )

    def _retrieve_parameter(self, service: str, parameter_name: str) -> ParameterValue:
        client = self.create_client(GetParameters, service)
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for parameter service")
                raise RuntimeError("Parameter retrieval interrupted")
            self.get_logger().info(f"Waiting for '{service}' service...")
        req = GetParameters.Request(names=[parameter_name])
        future = client.call_async(request=req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError(f"Failed to retrieve parameter '{parameter_name}' from '{service}'")
        self.get_logger().info(f"Received '{parameter_name}' from '{service}'.")
        return future.result().values[0]

    def _on_state(self, msg: LBRState) -> None:
        # Exponential smoothing of inputs (first message initializes)
        if not self._init:
            self._lbr_state = msg
            self._init = True
            return

        s = self._exp_smooth
        self._lbr_state.measured_joint_position = (
            (1 - s) * np.array(self._lbr_state.measured_joint_position.tolist())
            + s * np.array(msg.measured_joint_position.tolist())
        ).data
        self._lbr_state.external_torque = (
            (1 - s) * np.array(self._lbr_state.external_torque.tolist())
            + s * np.array(msg.external_torque.tolist())
        ).data

        # Compute command
        cmd = self._compute_command(self._lbr_state)
        if cmd is not None:
            self._cmd_pub.publish(cmd)

    def _compute_command(self, state: LBRState) -> Optional[LBRJointPositionCommand]:
        if self._jacobian_func is None:
            return None

        self._q = np.array(state.measured_joint_position.tolist())
        self._tau_ext = np.array(state.external_torque.tolist())

        # Jacobian and pseudo-inverse
        self._jacobian = self._jacobian_func(self._q)
        self._jacobian_inv = np.linalg.pinv(self._jacobian, rcond=self._pinv_rcond)

        # Estimate EE wrench from joint torques
        self._f_ext = self._jacobian_inv.T @ self._tau_ext

        # Deadband and gains in Cartesian space (baseline compliance motion)
        dx_compliance = np.where(
            np.abs(self._f_ext) > self._f_ext_th,
            np.diag(self._dx_gains) * np.sign(self._f_ext) * (np.abs(self._f_ext) - self._f_ext_th),
            0.0,
        )

        # Resistance: move against the applied force (negative admittance)
        self._update_release_schedule(self._f_ext, self._dt)
        dx = -self._gamma * dx_compliance

        # Map to joint velocities and smooth
        dq_target = self._dq_gains @ (self._jacobian_inv @ dx)
        a = self._dq_smooth
        self._dq = a * self._dq + (1.0 - a) * dq_target

        # Integrate to joint positions
        q_cmd = self._q + self._dt * self._dq
        out = LBRJointPositionCommand()
        out.joint_position = q_cmd.data
        return out

    def _update_release_schedule(self, f_ext: np.ndarray, dt: float) -> None:
        # Compute thresholded norm to avoid noise triggering
        f_eff = np.where(np.abs(f_ext) > self._f_ext_th, np.abs(f_ext) - self._f_ext_th, 0.0)
        f_norm = float(np.linalg.norm(f_eff[:3]))  # focus on forces (ignore torques) for trigger

        if not self._releasing:
            if f_norm >= self._release_trigger_force:
                self._above_thresh_time += dt
                if self._above_thresh_time >= self._release_hold_time:
                    self._releasing = True
                    self._release_elapsed = 0.0
                    self.get_logger().info(
                        f"Release triggered: f_norm={f_norm:.2f} N, ramping down over {self._release_duration:.2f}s"
                    )
            else:
                self._above_thresh_time = 0.0
            self._gamma = self._gamma_initial
        else:
            # Ramp gamma to zero
            self._release_elapsed += dt
            tau = max(self._release_duration, 1e-6)
            phase = min(self._release_elapsed / tau, 1.0)
            self._gamma = (1.0 - phase) * self._gamma_initial


def main(args=None):
    rclpy.init(args=args)
    node = ApplePluckControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
