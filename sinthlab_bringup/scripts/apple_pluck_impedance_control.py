#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
import pinocchio as pin
from lbr_fri_idl.msg import LBRState, LBRWrenchCommand


class ApplePluckImpedanceControlNode(Node):
    """
    Minimal Cartesian impedance controller publishing LBRWrenchCommand.
    - Builds a Pinocchio model from robot_description
    - Computes EE pose and Jacobian, applies stiffness/damping to track a target pose
    - Publishes wrench overlay along with a joint-position hold (measured q)
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_impedance_control",
            automatically_declare_parameters_from_overrides=True,
        )

        self._pin = pin

        # Parameters
        def _param(name: str, default):
            try:
                if self.has_parameter(name):
                    p = self.get_parameter(name)
                    return p.value if p is not None else default
            except Exception:
                pass
            return default

        self._update_rate = int(_param("update_rate", 200))
        self._dt = 1.0 / float(self._update_rate)

        # Links and frames
        self._base_link = str(_param("base_link", "lbr_link_0"))
        self._ee_link = str(_param("ee_link", "lbr_link_ee"))

        # Impedance gains (diagonal)
        self._K_lin = np.array(_param("impedance_stiffness_linear", [800.0, 800.0, 800.0]), dtype=float)
        self._K_ang = np.array(_param("impedance_stiffness_angular", [30.0, 30.0, 30.0]), dtype=float)
        self._D_lin = np.array(_param("impedance_damping_linear", [40.0, 40.0, 40.0]), dtype=float)
        self._D_ang = np.array(_param("impedance_damping_angular", [5.0, 5.0, 5.0]), dtype=float)

        # Wrench saturation
        self._wrench_max_lin = np.array(_param("wrench_max_linear", [60.0, 60.0, 60.0]), dtype=float)
        self._wrench_max_ang = np.array(_param("wrench_max_angular", [8.0, 8.0, 8.0]), dtype=float)

        # Target pose parameters; by default we capture current pose as target on first run
        self._use_current_as_target = bool(_param("use_current_pose_as_target", True))
        self._target_pos = np.array(_param("impedance_target_position", [0.5, 0.0, 0.4]), dtype=float)
        self._target_rpy = np.array(_param("impedance_target_rpy", [0.0, math.pi, 0.0]), dtype=float)

        # State
        self._q = np.zeros(7)
        self._qd = np.zeros(7)
        self._q_prev: Optional[np.ndarray] = None
        self._init = False
        self._model_built = False
        self._frame_id: Optional[int] = None

        # Pinocchio model/data
        self._model = None
        self._data = None

        # ROS I/O
        self._sub = self.create_subscription(LBRState, "state", self._on_state, 1)
        self._pub_wrench = self.create_publisher(LBRWrenchCommand, "command/wrench", 1)
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f"Impedance node started: rate={self._update_rate}Hz, ee_link={self._ee_link}, base_link={self._base_link}"
        )

    # ------------------------- ROS Callbacks -------------------------
    def _on_state(self, msg: LBRState) -> None:
        try:
            q = np.array(msg.measured_joint_position, dtype=float)
        except Exception:
            return
        if q.shape[0] == 7:
            if self._q_prev is not None:
                self._qd = (q - self._q_prev) / self._dt
            self._q_prev = q.copy()
            self._q = q
            if not self._init:
                self._init = True

    # ------------------------- Pinocchio Setup -------------------------
    def _ensure_model(self) -> None:
        if self._model_built or self._pin is None:
            return
        robot_description = None
        try:
            if self.has_parameter("robot_description"):
                robot_description = str(self.get_parameter("robot_description").value)
        except Exception:
            robot_description = None
        if not robot_description:
            self.get_logger().warn("robot_description parameter not available yet; waiting...")
            return
        try:
            pin = self._pin  # type: ignore
            assert pin is not None
            self._model = pin.buildModelFromXML(robot_description)
            self._data = self._model.createData()
            # Resolve frame id
            self._frame_id = self._model.getFrameId(self._ee_link)
            if int(self._frame_id) == len(self._model.frames):
                # Not found
                self.get_logger().error(f"EE link '{self._ee_link}' not found in model frames.")
                return
            self._model_built = True
            self.get_logger().info("Pinocchio model built from robot_description.")
        except Exception as e:
            self.get_logger().error(f"Failed to build Pinocchio model: {e}")

    # ------------------------- Control Core -------------------------
    def _impedance_step(self) -> Optional[np.ndarray]:
        if not (self._model_built and self._init and self._pin is not None and self._frame_id is not None):
            return None
        pin = self._pin  # type: ignore
        assert self._model is not None and self._data is not None

        q = self._q
        qd = self._qd

        # Kinematics
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        oMf = self._data.oMf[self._frame_id]

        # Desired pose
        if self._use_current_as_target and not hasattr(self, "_target_cached"):
            self._target_pos = oMf.translation.copy()
            # Keep current orientation
            self._target_rpy = np.array([0.0, 0.0, 0.0], dtype=float)
            setattr(self, "_target_cached", True)

        R_d = pin.rpy.rpyToMatrix(float(self._target_rpy[0]), float(self._target_rpy[1]), float(self._target_rpy[2]))
        p_d = self._target_pos
        R = oMf.rotation
        p = oMf.translation

        # Errors in world frame
        e_p = p_d - p
        R_err = R_d @ R.T
        try:
            e_o = pin.log3(R_err)
        except Exception:
            # Fallback small-angle approx if log3 fails
            e_o = 0.5 * np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ])

        # EE spatial velocity (world aligned)
        J6 = pin.computeFrameJacobian(self._model, self._data, q, self._frame_id, pin.ReferenceFrame.WORLD)
        v6 = J6 @ qd  # [vx, vy, vz, wx, wy, wz] in world

        e_v_lin = -v6[:3]
        e_v_ang = -v6[3:]

        # Wrench = K*e + D*e_dot
        F_lin = self._K_lin * e_p + self._D_lin * e_v_lin
        F_ang = self._K_ang * e_o + self._D_ang * e_v_ang

        # Saturation
        F_lin = np.clip(F_lin, -self._wrench_max_lin, self._wrench_max_lin)
        F_ang = np.clip(F_ang, -self._wrench_max_ang, self._wrench_max_ang)

        wrench = np.concatenate([F_lin, F_ang])
        return wrench

    def _publish_wrench(self, wrench: np.ndarray) -> None:
        msg = LBRWrenchCommand()
        # Hold current posture (send measured q as setpoint)
        msg.joint_position = self._q.tolist()
        msg.wrench = wrench.tolist()
        self._pub_wrench.publish(msg)

    # ------------------------- Timer -------------------------
    def _step(self) -> None:
        # Ensure we have model and state
        if not self._model_built:
            self._ensure_model()
            return
        if not self._init:
            return
        wrench = self._impedance_step()
        if wrench is not None:
            self._publish_wrench(wrench)


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
