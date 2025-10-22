#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
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
        self.get_logger().info("Initializing Apple Pluck Impedance Control Node")
        # Declare local parameters (no rcl_interfaces services)
        # Consumers should pass these via launch or a params file.
        self.declare_parameter("robot_description", "")
        self.declare_parameter("update_rate", 100)
        self._robot_description: str = str(self.get_parameter("robot_description").value)
        self._update_rate: int = int(self.get_parameter("update_rate").value or 100)
        if not self._update_rate:
            self.get_logger().warn("update_rate missing; defaulting to 100 Hz")
            self._update_rate = 100
        self._dt: float = 1.0 / float(self._update_rate)

        # Parameters
        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("end_effector_link", "lbr_link_ee")
        self.declare_parameter("exp_smooth", 0.95)
        self.declare_parameter("pinv_rcond", 0.1)  # kept for parity; not needed for FK

        # Move-to-start behavior (like Sunrise example)
        self.declare_parameter("use_initial_joint_position", True)
        # Default: [0, 10deg, 0, -80deg, 0, 90deg, 0]
        self.declare_parameter(
            "initial_joint_position",
            [0.0, math.radians(10.0), 0.0, math.radians(-80.0), 0.0, math.radians(90.0), 0.0],
        )
        self.declare_parameter("joint_move_max_speed", 0.5)  # rad/s per joint
        self.declare_parameter("joint_move_tolerance", 0.01)  # rad norm threshold

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

        self._use_initial = bool(self.get_parameter("use_initial_joint_position").value)
        self._q_target = np.array(self.get_parameter("initial_joint_position").value, dtype=float)
        self._q_speed = float(self.get_parameter("joint_move_max_speed").value)
        self._q_tol = float(self.get_parameter("joint_move_tolerance").value)

        if not (0.0 <= self._exp_smooth <= 1.0):
            raise ValueError("exp_smooth must be in [0,1]")
        if optas is None:
            self.get_logger().error(
                "optas is not available. Install 'optas' in your Python environment to run this node."
            )

        # Kinematics (FK for pose)
        if optas and self._robot_description:
            try:
                self._robot = optas.RobotModel(urdf_string=self._robot_description)
            except Exception as e:
                self.get_logger().error(f"Failed to construct RobotModel from robot_description: {e}")
                self._robot = None
        else:
            if not self._robot_description:
                self.get_logger().error(
                    "robot_description parameter is empty. Provide URDF via launch parameters or a YAML file."
                )
            self._robot = None
        self._fk_pos_func = (
            self._robot.get_global_link_position_function(
                link=self._ee_link, base_link=self._base_link, numpy_output=True
            )
            if self._robot
            else None
        )

        # State
        self._init = False
        self._phase = "move_to_start" if self._use_initial else "impedance"
        self._q = np.zeros(7)
        self._x0 = np.zeros(3)
        self._x_prev = np.zeros(3)
        self._v = np.zeros(3)
        self._above_thresh_time = 0.0
        self._releasing = False
        self._release_elapsed = 0.0
        self._gamma = self._gamma_initial
        self._started_move_to_start = False

        # Periodic progress logging for move-to-start (1 Hz)
        self._progress_timer = self.create_timer(1.0, self._log_move_progress)

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
        self.get_logger().info(
            f"* move_to_start: {self._use_initial}, q_target(rad): {self._q_target.tolist()}, "
            f"max_speed: {self._q_speed} rad/s, tol: {self._q_tol} rad"
        )

    # rcl_interfaces parameter service retrieval removed; using local get_parameter instead.

    def _on_state(self, msg: LBRState) -> None:
        q = np.array(msg.measured_joint_position.tolist())
        if not self._init:
            self._q = q
            if self._phase == "impedance":
                x = self._fk_pos(q)
                self._x0 = x.copy()
                self._x_prev = x.copy()
            elif self._phase == "move_to_start" and not self._started_move_to_start:
                self.get_logger().info(
                    "Starting move_to_start phase: commanding joint_position towards initial target"
                )
                self._started_move_to_start = True
            self._init = True
            return

        # Smooth q for FK if desired (reuse exp_smooth to filter position estimate)
        s = self._exp_smooth
        self._q = (1 - s) * self._q + s * q
        # Phase handling: move to start or impedance hold
        if self._phase == "move_to_start":
            cmd = self._compute_move_to_start(self._q)
            # Check convergence
            if cmd is not None:
                # Publish and return; convergence check happens inside as well
                self._pub.publish(cmd)
            return

        # Impedance mode
        x = self._fk_pos(self._q)
        # Finite-difference velocity (smoothed implicitly by q smoothing)
        v = (x - self._x_prev) / max(self._dt, 1e-6)
        self._x_prev = x
        self._v = v

        cmd = self._compute_command(x, v, self._q)
        if cmd is not None:
            self._pub.publish(cmd)

    def _compute_move_to_start(self, q: np.ndarray) -> Optional[LBRWrenchCommand]:
        # Joint-space incremental motion towards q_target with speed cap
        err = self._q_target - q
        err_norm = float(np.linalg.norm(err))
        if err_norm <= self._q_tol:
            # Arrived: set impedance reference from this pose and switch phase
            x = self._fk_pos(q)
            self._x0 = x.copy()
            self._x_prev = x.copy()
            self._phase = "impedance"
            self.get_logger().info(
                f"Reached initial joint position (err_norm={err_norm:.4f} rad). Switching to impedance hold."
            )
            return None

        # Compute per-joint step limited by max speed
        max_step = self._q_speed * self._dt
        step = np.clip(err, -max_step, max_step)
        q_cmd = q + step

        out = LBRWrenchCommand()
        out.joint_position = q_cmd.data
        out.wrench = np.zeros(6).data  # no wrench overlay during move-to-start
        return out

    def _log_move_progress(self) -> None:
        # Periodically log progress while in move_to_start
        if not self._init or self._phase != "move_to_start":
            return
        err = self._q_target - self._q
        err_norm = float(np.linalg.norm(err))
        self.get_logger().info(
            f"move_to_start progress: err_norm={err_norm:.4f} rad (tol={self._q_tol:.4f}), "
            f"current_q[deg]={np.degrees(self._q).round(1).tolist()}"
        )

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
