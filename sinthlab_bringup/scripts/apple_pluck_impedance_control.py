#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

try:
    import optas
except ImportError:
    optas = None

from lbr_fri_idl.msg import LBRWrenchCommand, LBRState


class ApplePluckImpedanceControlNode(Node):
    """
    Apple pluck impedance controller:
    - Mirrors the Sunrise example's idea: hold pose with Cartesian stiffness (virtual impedance),
      providing resistance, then ramp stiffness down to "release" the apple.
    - Publishes LBRWrenchCommand on command/wrench (requires LBRWrenchCommandController active and
      robot in CARTESIAN_IMPEDANCE_CONTROL mode on the FRI side).
    - No physical FT sensor required; this controller computes virtual wrench from pose error and velocity.
    """

    def __init__(self) -> None:
        super().__init__("apple_pluck_impedance_control")

        # Retrieve required system params
        self._robot_description = self._retrieve_parameter(
            "robot_state_publisher/get_parameters", "robot_description"
        ).string_value
        update_rate_val = self._retrieve_parameter(
            "controller_manager/get_parameters", "update_rate"
        ).integer_value
        if update_rate_val is None or update_rate_val == 0:
            self.get_logger().warn("update_rate missing; defaulting to 100 Hz")
            update_rate_val = 100
        self._update_rate: int = int(update_rate_val)
        self._dt: float = 1.0 / float(self._update_rate)

        # Parameters
        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("end_effector_link", "lbr_link_ee")
        self.declare_parameter("exp_smooth", 0.95)
        self.declare_parameter("pinv_rcond", 0.1)  # kept for parity; not needed for FK

        # Virtual impedance gains (N/m for XYZ, Nm/rad for RPY) – default mirrors the Java example
        self.declare_parameter("stiffness", [1500.0, 700.0, 2500.0, 0.0, 0.0, 0.0])
        self.declare_parameter("damping", [80.0, 60.0, 100.0, 0.0, 0.0, 0.0])
        self.declare_parameter("max_wrench", [100.0, 100.0, 150.0, 10.0, 10.0, 10.0])

        # Release scheduling
        self.declare_parameter("gamma_initial", 1.0)  # scales stiffness/damping at start
        self.declare_parameter("release_displacement", 0.03)  # [m] on Z or norm trigger
        self.declare_parameter("release_axis", "z")  # "z" or "norm"
        self.declare_parameter("release_hold_time", 1.0)  # [s]
        self.declare_parameter("release_duration", 1.5)  # [s]

        # Read params
        self._base_link = self.get_parameter("base_link").get_parameter_value().string_value
        self._ee_link = (
            self.get_parameter("end_effector_link").get_parameter_value().string_value
        )
        self._exp_smooth = float(self.get_parameter("exp_smooth").value)
        self._pinv_rcond = float(self.get_parameter("pinv_rcond").value)
        self._K = np.array(self.get_parameter("stiffness").value, dtype=float)
        self._B = np.array(self.get_parameter("damping").value, dtype=float)
        self._max_wrench = np.array(self.get_parameter("max_wrench").value, dtype=float)
        self._gamma_initial = float(self.get_parameter("gamma_initial").value)
        self._release_disp = float(self.get_parameter("release_displacement").value)
        self._release_axis = str(self.get_parameter("release_axis").value)
        self._release_hold_time = float(self.get_parameter("release_hold_time").value)
        self._release_duration = float(self.get_parameter("release_duration").value)

        if not (0.0 <= self._exp_smooth <= 1.0):
            raise ValueError("exp_smooth must be in [0,1]")
        if optas is None:
            self.get_logger().error(
                "optas is not available. Install 'optas' in your Python environment to run this node."
            )

        # Kinematics (FK for pose)
        self._robot = optas.RobotModel(urdf_string=self._robot_description) if optas else None
        self._fk_pos_func = (
            self._robot.get_global_link_position_function(
                link=self._ee_link, base_link=self._base_link, numpy_output=True
            )
            if self._robot
            else None
        )

        # State
        self._init = False
        self._q = np.zeros(7)
        self._x0 = np.zeros(3)
        self._x_prev = np.zeros(3)
        self._v = np.zeros(3)
        self._above_thresh_time = 0.0
        self._releasing = False
        self._release_elapsed = 0.0
        self._gamma = self._gamma_initial

        # ROS I/O
        self._sub = self.create_subscription(LBRState, "state", self._on_state, 1)
        self._pub = self.create_publisher(LBRWrenchCommand, "command/wrench", 1)

        self._log_parameters()

    def _log_parameters(self) -> None:
        self.get_logger().info("*** Apple Pluck Impedance Parameters:")
        self.get_logger().info(f"* base_link: {self._base_link}")
        self.get_logger().info(f"* end_effector_link: {self._ee_link}")
        self.get_logger().info(f"* stiffness: {self._K.tolist()}")
        self.get_logger().info(f"* damping: {self._B.tolist()}")
        self.get_logger().info(f"* max_wrench: {self._max_wrench.tolist()}")
        self.get_logger().info(
            f"* gamma_initial: {self._gamma_initial}, release_displacement: {self._release_disp} ({self._release_axis}), "
            f"hold: {self._release_hold_time}s, duration: {self._release_duration}s"
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
            raise RuntimeError(f"Failed to retrieve '{parameter_name}' from '{service}'")
        self.get_logger().info(f"Received '{parameter_name}' from '{service}'.")
        return future.result().values[0]

    def _on_state(self, msg: LBRState) -> None:
        q = np.array(msg.measured_joint_position.tolist())
        if not self._init:
            self._q = q
            x = self._fk_pos(q)
            self._x0 = x.copy()
            self._x_prev = x.copy()
            self._init = True
            return

        # Smooth q for FK if desired (reuse exp_smooth to filter position estimate)
        s = self._exp_smooth
        self._q = (1 - s) * self._q + s * q
        x = self._fk_pos(self._q)

        # Finite-difference velocity (smoothed implicitly by q smoothing)
        v = (x - self._x_prev) / max(self._dt, 1e-6)
        self._x_prev = x
        self._v = v

        cmd = self._compute_command(x, v, self._q)
        if cmd is not None:
            self._pub.publish(cmd)

    def _fk_pos(self, q: np.ndarray) -> np.ndarray:
        if self._fk_pos_func is None:
            return np.zeros(3)
        return np.array(self._fk_pos_func(q)).reshape(-1)

    def _compute_command(self, x: np.ndarray, v: np.ndarray, q: np.ndarray) -> Optional[LBRWrenchCommand]:
        # Update release schedule based on displacement
        self._update_release(x)

        # Virtual impedance wrench (XYZ only by default)
        K = self._gamma * self._K
        B = self._gamma * self._B
        e = x - self._x0
        F = -K[:3] * e - B[:3] * v
        tau = np.zeros(3)

        wrench = np.concatenate([F, tau])
        # Clip wrench per-axis
        wrench = np.clip(wrench, -self._max_wrench, self._max_wrench)

        out = LBRWrenchCommand()
        out.joint_position = q.data  # hold current q as reference
        out.wrench = wrench.data
        return out

    def _update_release(self, x: np.ndarray) -> None:
        disp = x - self._x0
        if self._release_axis.lower() == "z":
            metric = abs(float(disp[2]))
        else:
            metric = float(np.linalg.norm(disp))

        if not self._releasing:
            if metric >= self._release_disp:
                self._above_thresh_time += self._dt
                if self._above_thresh_time >= self._release_hold_time:
                    self._releasing = True
                    self._release_elapsed = 0.0
                    self.get_logger().info(
                        f"Release triggered: disp={metric:.3f} m, ramping gains down over {self._release_duration:.2f}s"
                    )
            else:
                self._above_thresh_time = 0.0
            self._gamma = self._gamma_initial
        else:
            self._release_elapsed += self._dt
            T = max(self._release_duration, 1e-6)
            phase = min(self._release_elapsed / T, 1.0)
            self._gamma = (1.0 - phase) * self._gamma_initial


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApplePluckImpedanceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
