#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
import optas

from lbr_fri_idl.msg import LBRWrenchCommand, LBRState, LBRJointPositionCommand


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
        # Auto-declare parameters that are provided via YAML/CLI; only keep minimal in-code defaults
        super().__init__(
            "apple_pluck_impedance_control",
            automatically_declare_parameters_from_overrides=True,
        )
        self.get_logger().info("Initializing Apple Pluck Impedance Control Node")

        # Helper to fetch parameters with safe defaults when not provided via YAML
        def _param(name: str, default):
            try:
                if self.has_parameter(name):
                    p = self.get_parameter(name)
                    return p.value if (p is not None and p.value is not None) else default
            except Exception:
                # Fallback to default if parameter infrastructure isn't ready yet
                pass
            return default

        # Read params (YAML overrides these defaults)
        self._robot_description = str(_param("robot_description", ""))
        self._update_rate = int(_param("update_rate", 100))
        self._dt: float = 1.0 / float(self._update_rate)
        self._base_link = str(_param("base_link", "lbr_link_0"))
        self._ee_link = str(_param("end_effector_link", "lbr_link_ee"))
        self._exp_smooth = float(_param("exp_smooth", 0.95))
        self._K = np.array(_param("stiffness", [1500.0, 700.0, 2500.0, 0.0, 0.0, 0.0]), dtype=float)
        self._B = np.array(_param("damping", [80.0, 60.0, 100.0, 0.0, 0.0, 0.0]), dtype=float)
        self._max_wrench = np.array(
            _param("max_wrench", [100.0, 100.0, 150.0, 10.0, 10.0, 10.0]), dtype=float
        )
        self._gamma_initial = float(_param("gamma_initial", 1.0))
        self._release_disp = float(_param("release_displacement", 0.03))
        self._release_axis = str(_param("release_axis", "z"))
        self._release_hold_time = float(_param("release_hold_time", 1.0))
        self._release_duration = float(_param("release_duration", 1.5))

        self._use_initial = bool(_param("use_initial_joint_position", True))
        self._q_target = np.array(
            _param(
                "initial_joint_position",
                [
                    0.0,
                    math.radians(10.0),
                    0.0,
                    math.radians(-80.0),
                    0.0,
                    math.radians(90.0),
                    0.0,
                ],
            ),
            dtype=float,
        )
        self._q_speed = float(_param("joint_move_max_speed", 0.5))
        self._q_tol = float(_param("joint_move_tolerance", 0.01))
        # Move-to-start Cartesian gains (XYZ), used to generate force towards target
        self._mts_k = np.array(_param("move_to_start_k_xyz", [50.0, 50.0, 50.0]), dtype=float)
        # Move-to-start mode: 'joint_position' (PTP-like) or 'cartesian_wrench'
        self._mts_mode = str(_param("move_to_start_mode", "joint_position")).lower()
        if self._mts_mode not in ("joint_position", "cartesian_wrench"):
            self.get_logger().warn(
                f"Unknown move_to_start_mode '{self._mts_mode}', defaulting to 'joint_position'"
            )
            self._mts_mode = "joint_position"

        if not (0.0 <= self._exp_smooth <= 1.0):
            raise ValueError("exp_smooth must be in [0,1]")

        # Kinematics (FK for pose)
        if not self._robot_description:
            self.get_logger().error(
                "robot_description parameter is empty. Provide URDF via launch parameters or a YAML file."
            )
        try:
            self._robot = optas.RobotModel(urdf_string=self._robot_description)
        except Exception as e:
            self.get_logger().error(f"Failed to construct RobotModel from robot_description: {e}")
            self._robot = None
        self._fk_pos_func = (
            self._robot.get_global_link_position_function(
                link=self._ee_link, numpy_output=True
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
        self._pub_joint = self.create_publisher(
            LBRJointPositionCommand, "command/joint_position", 1
        )

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
            f"* move_to_start: {self._use_initial} (mode={self._mts_mode}), "
            f"q_target(rad): {self._q_target.tolist()}, max_speed: {self._q_speed} rad/s, "
            f"tol: {self._q_tol} rad, k_xyz: {self._mts_k.tolist()}"
        )

    def _on_state(self, msg: LBRState) -> None:
        q = np.array(msg.measured_joint_position.tolist())
        if not self._init:
            self._q = q
            if self._phase == "impedance":
                x = self._fk_pos(q)
                self._x0 = x.copy()
                self._x_prev = x.copy()
            elif self._phase == "move_to_start" and not self._started_move_to_start:
                if self._mts_mode == "joint_position":
                    self.get_logger().info(
                        "Starting move_to_start phase: commanding joint positions towards the initial target (PTP-like)"
                    )
                else:
                    self.get_logger().info(
                        "Starting move_to_start phase: applying Cartesian wrench towards the initial target"
                    )
                self._started_move_to_start = True
            self._init = True
            return

        # Smooth q for FK if desired (reuse exp_smooth to filter position estimate)
        s = self._exp_smooth
        self._q = (1 - s) * self._q + s * q
        # Phase handling: move to start or impedance hold
        if self._phase == "move_to_start":
            # Compute and publish according to mode; function handles publishing
            self._compute_move_to_start(self._q)
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

    def _compute_move_to_start(self, q: np.ndarray) -> None:
        # Move to initial configuration either with joint position commands (PTP-like)
        # or by applying a Cartesian wrench (works under CARTESIAN_IMPEDANCE_CONTROL).
        err_q = self._q_target - q
        err_q_norm = float(np.linalg.norm(err_q))
        if err_q_norm <= self._q_tol:
            # Arrived: set impedance reference from this pose and switch phase
            x_here = self._fk_pos(q)
            self._x0 = x_here.copy()
            self._x_prev = x_here.copy()
            self._phase = "impedance"
            self.get_logger().info(
                f"Reached initial joint position (err_norm={err_q_norm:.4f} rad). Switching to impedance hold."
            )
            return
        if self._mts_mode == "joint_position":
            # Joint-space move using direct setpoint to target (PTP-like)
            q_cmd = self._q_target
            cmd = LBRJointPositionCommand()
            cmd.joint_position = q_cmd.data
            self._pub_joint.publish(cmd)
        else:
            # Cartesian wrench towards the target end-effector pose
            x = self._fk_pos(q)
            x_tgt = self._fk_pos(self._q_target)
            e = x_tgt - x
            F = self._mts_k * e[:3]
            F = np.clip(F, -self._max_wrench[:3], self._max_wrench[:3])
            wrench = np.zeros(6)
            wrench[:3] = F
            out = LBRWrenchCommand()
            out.joint_position = q.data
            out.wrench = wrench.data
            self._pub.publish(out)

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
