#!/usr/bin/env python3
"""Move-to-pose for TORQUE mode.

Under the FRI torque overlay there is no position-command controller, so the exact-posture
move-to-start of the impedance experiments is done here: FK the desired start joint configuration
to an EE pose and stream it (ramped, to avoid a jump) to a joint-space impedance controller's
`target_frame`, which IK-solves it and drives there with a joint-space spring. Completes once the
measured EE pose is within tolerance of the goal.

Replaces MoveToPositionJointSpace for the restricted-plane and maze experiments. The fixture anchors
its manifold on the ACTUAL start pose, so what matters here is reaching the goal EE pose+orientation,
not a bit-exact 7-DOF posture (the redundant DOF is biased by the controller's nullspace overlay).
"""
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState
from geometry_msgs.msg import PoseStamped

import optas
from scipy.spatial.transform import Rotation as R, Slerp

from sinthlab_bringup.helpers.common_threshold import get_required_param


class MoveToFrameAction:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "",
                 on_complete: Callable[[], None], target_controller: str = "joint_impedance_controller") -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._active = False
        self._done = False

        state_topic = str(get_required_param(node, self._param_prefix + "state_topic"))
        self.base_link = str(get_required_param(node, self._param_prefix + "base_link"))
        self.ee_link = str(get_required_param(node, self._param_prefix + "end_effector_link"))

        robot_name = node.get_namespace().strip("/")
        cmd_topic = (f"/{robot_name}/{target_controller}/target_frame"
                     if robot_name else f"/{target_controller}/target_frame")

        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        # ramp duration for the target pose [s] and the arrival tolerance [m]
        self._move_duration = 4.0
        if node.has_parameter(self._param_prefix + "move_duration_sec"):
            self._move_duration = float(node.get_parameter(self._param_prefix + "move_duration_sec").value)
        self._tol = float(get_required_param(node, self._param_prefix + "cartesian_move_tolerance"))

        # start joint configuration (degrees) -> the goal EE pose via FK
        target_deg = list(get_required_param(node, self._param_prefix + "target_joint_position"))
        self._q_goal = np.deg2rad(np.array(target_deg, dtype=float))

        robot_description = ""
        if node.has_parameter("robot_description"):
            robot_description = str(node.get_parameter("robot_description").value)
        self.robot = optas.RobotModel(urdf_string=robot_description)
        self._fk = self.robot.get_link_transform_function(
            link=self.ee_link, base_link=self.base_link, numpy_output=True
        )
        self._goal_T = self._fk(self._q_goal)

        self._last_measured = np.zeros(self.robot.ndof)
        self._have_state = False
        self._state_sub = node.create_subscription(LBRState, state_topic, self._state_cb, 1)
        self._pub = node.create_publisher(PoseStamped, cmd_topic, 1)
        self._timer = node.create_timer(self._dt, self._step)

        self._node.get_logger().info(
            f"MoveToFrame ready: -> {cmd_topic}, goal EE=({self._goal_T[0,3]:.3f},"
            f"{self._goal_T[1,3]:.3f},{self._goal_T[2,3]:.3f}), ramp={self._move_duration:.1f}s"
        )

    def start(self) -> None:
        if self._active:
            return
        self._active = True
        self._done = False
        self._t = 0.0
        self._start_T = None  # captured on first tick from the measured pose
        self._node.get_logger().info("MoveToFrame started.")

    def stop(self) -> None:
        self._active = False

    def _state_cb(self, msg: LBRState) -> None:
        self._last_measured = np.array(msg.measured_joint_position)
        self._have_state = True

    def _step(self) -> None:
        if not self._active or self._done or not self._have_state:
            return

        if self._start_T is None:
            self._start_T = self._fk(self._last_measured)
            self._slerp = Slerp([0.0, 1.0], R.from_matrix(
                np.stack([self._start_T[0:3, 0:3], self._goal_T[0:3, 0:3]])))

        self._t += self._dt
        a = min(1.0, self._t / max(1e-3, self._move_duration))
        pos = (1.0 - a) * self._start_T[0:3, 3] + a * self._goal_T[0:3, 3]
        rot = self._slerp([a])[0].as_matrix()
        self._publish(pos, rot)

        # arrival: ramp finished AND measured EE within tolerance of the goal
        if a >= 1.0:
            meas = self._fk(self._last_measured)
            err = float(np.linalg.norm(meas[0:3, 3] - self._goal_T[0:3, 3]))
            if err <= self._tol:
                self._done = True
                self._active = False
                self._node.get_logger().info(f"MoveToFrame arrived (EE err={err*1000:.1f} mm).")
                try:
                    self._on_complete()
                except Exception as exc:
                    self._node.get_logger().error(f"MoveToFrame on_complete error: {exc}")

    def _publish(self, pos: np.ndarray, rot: np.ndarray) -> None:
        cmd = PoseStamped()
        cmd.header.frame_id = self.base_link
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.pose.position.x = float(pos[0])
        cmd.pose.position.y = float(pos[1])
        cmd.pose.position.z = float(pos[2])
        q = R.from_matrix(rot).as_quat()
        cmd.pose.orientation.x = float(q[0])
        cmd.pose.orientation.y = float(q[1])
        cmd.pose.orientation.z = float(q[2])
        cmd.pose.orientation.w = float(q[3])
        self._pub.publish(cmd)
