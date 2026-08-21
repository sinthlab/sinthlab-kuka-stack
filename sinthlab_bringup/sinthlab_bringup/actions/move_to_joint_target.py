#!/usr/bin/env python3
"""Move-to-start / recover for TORQUE mode, commanded in JOINT space.

Under the FRI torque overlay there is no position-command controller, so the exact-posture moves of
the impedance experiments run here: this action ramps the arm from its current joint configuration
to `target_joint_position` and streams the interpolated configuration to the joint-impedance
controller's `target_joints` topic (std_msgs/Float64MultiArray, radians).

WHY JOINT SPACE: the controller's upstream path takes a Cartesian `target_frame` and runs KDL IK
(ChainIkSolverPos_NR_JL) to recover joints. Since we already KNOW the target joint configuration,
that is a joints -> Cartesian -> joints round trip -- and its IK fails on this 7-DOF redundant arm
(near-singular configurations), leaving the impedance target unchanged so the arm never moves. The
vendored controller therefore accepts a joint target directly (SINTHLAB PATCH #2, see
ros2_effort_controller/README.md); this action feeds it.

Completion is still judged at the END EFFECTOR (`cartesian_move_tolerance`): under impedance control
there is always some steady-state joint error, and what the experiments care about is that the tool
arrived, not that every joint hit its setpoint exactly.
"""
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState
from std_msgs.msg import Float64MultiArray

import optas

from sinthlab_bringup.helpers.common_threshold import get_required_param


class MoveToJointTargetAction:
    def __init__(self, node: rclpyNode, *, param_prefix: str = "",
                 on_complete: Callable[[], None],
                 target_controller: str = "joint_impedance_controller") -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._active = False
        self._done = False

        # state_topic / base_link / end_effector_link are TOP-LEVEL params (like the fixture reads
        # them), NOT under the move_to_start prefix.
        state_topic = str(get_required_param(node, "state_topic"))
        self.base_link = str(get_required_param(node, "base_link"))
        self.ee_link = str(get_required_param(node, "end_effector_link"))

        robot_name = node.get_namespace().strip("/")
        cmd_topic = (f"/{robot_name}/{target_controller}/target_joints"
                     if robot_name else f"/{target_controller}/target_joints")

        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._move_duration = 4.0
        if node.has_parameter(self._param_prefix + "move_duration_sec"):
            self._move_duration = float(node.get_parameter(self._param_prefix + "move_duration_sec").value)
        self._tol = float(get_required_param(node, self._param_prefix + "cartesian_move_tolerance"))

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
        self._pub = node.create_publisher(Float64MultiArray, cmd_topic, 1)
        self._timer = node.create_timer(self._dt, self._step)

        self._node.get_logger().info(
            f"MoveToJointTarget ready: -> {cmd_topic}, goal EE=({self._goal_T[0,3]:.3f},"
            f"{self._goal_T[1,3]:.3f},{self._goal_T[2,3]:.3f}), ramp={self._move_duration:.1f}s"
        )

    def start(self) -> None:
        if self._active:
            return
        self._active = True
        self._done = False
        self._t = 0.0
        self._q_start = None  # captured on the first tick from the measured joints
        self._node.get_logger().info("MoveToJointTarget started.")

    def stop(self) -> None:
        self._active = False

    def _state_cb(self, msg: LBRState) -> None:
        self._last_measured = np.array(msg.measured_joint_position)
        self._have_state = True

    def _step(self) -> None:
        if not self._active or self._done or not self._have_state:
            return

        if self._q_start is None:
            self._q_start = self._last_measured.copy()

        self._t += self._dt
        a = min(1.0, self._t / max(1e-3, self._move_duration))
        q_cmd = (1.0 - a) * self._q_start + a * self._q_goal
        self._publish(q_cmd)

        if a >= 1.0:
            meas = self._fk(self._last_measured)
            err = float(np.linalg.norm(meas[0:3, 3] - self._goal_T[0:3, 3]))
            if err <= self._tol:
                self._done = True
                self._active = False
                self._node.get_logger().info(f"MoveToJointTarget arrived (EE err={err*1000:.1f} mm).")
                try:
                    self._on_complete()
                except Exception as exc:
                    self._node.get_logger().error(f"MoveToJointTarget on_complete error: {exc}")

    def _publish(self, q: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q]
        self._pub.publish(msg)
