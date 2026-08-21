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

COMMANDING STRATEGY -- a "leash", not a time ramp. The commanded configuration is always the measured
one nudged at most `joint_leash_rad` toward the goal:

    q_cmd = q_measured + clamp(q_goal - q_measured, +/- leash)

so the target can never run far ahead of where the arm actually is. This matters for two reasons:

  * SAFETY. The controller's stiffness torque is K*(q_cmd - q), so it is bounded by K*leash by
    construction. effort_controller_base hard-aborts the process (std::terminate) if a newly desired
    joint torque differs from the currently applied one by more than 10 Nm, and applied torque only
    creeps at `delta_tau_max` (1 Nm) per cycle -- so an unbounded target is a process kill.
  * ROBUSTNESS. A time-based ramp keeps advancing even when the arm is not moving (e.g. the
    controller has not activated yet), so by the time it engages the target is far away and the
    torque spikes. The leash simply waits: no motion, no advance.

The arm then settles into a steady glide at roughly K*leash/D rad/s (D = 2*sqrt(K) here), i.e. a few
tens of degrees per second -- slow and self-limiting.
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
        # Kept only as a "this is taking too long" warning threshold (the move is leash-driven, not timed).
        self._move_duration = 4.0
        if node.has_parameter(self._param_prefix + "move_duration_sec"):
            self._move_duration = float(node.get_parameter(self._param_prefix + "move_duration_sec").value)
        # Max radians the commanded configuration may lead the measured one, per joint. Bounds the
        # stiffness torque to K*leash: keep K_max*leash well under the controller's 10 Nm abort.
        self._leash = 0.03
        if node.has_parameter(self._param_prefix + "joint_leash_rad"):
            self._leash = float(node.get_parameter(self._param_prefix + "joint_leash_rad").value)
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
            f"{self._goal_T[1,3]:.3f},{self._goal_T[2,3]:.3f}), leash={self._leash:.3f} rad"
        )

    def start(self) -> None:
        if self._active:
            return
        self._active = True
        self._done = False
        self._t = 0.0
        self._warned = False
        self._node.get_logger().info("MoveToJointTarget started.")

    def stop(self) -> None:
        self._active = False

    def _state_cb(self, msg: LBRState) -> None:
        self._last_measured = np.array(msg.measured_joint_position)
        self._have_state = True

    def _step(self) -> None:
        if not self._active or self._done or not self._have_state:
            return

        self._t += self._dt

        # Leash: never command more than `leash` radians ahead of where the arm actually is.
        q_meas = self._last_measured
        delta = np.clip(self._q_goal - q_meas, -self._leash, self._leash)
        self._publish(q_meas + delta)

        meas = self._fk(q_meas)
        err = float(np.linalg.norm(meas[0:3, 3] - self._goal_T[0:3, 3]))
        if err <= self._tol:
            self._done = True
            self._active = False
            self._node.get_logger().info(f"MoveToJointTarget arrived (EE err={err*1000:.1f} mm).")
            try:
                self._on_complete()
            except Exception as exc:
                self._node.get_logger().error(f"MoveToJointTarget on_complete error: {exc}")
        elif not self._warned and self._t > max(3.0 * self._move_duration, 15.0):
            self._warned = True
            self._node.get_logger().warn(
                f"MoveToJointTarget still {err*1000:.0f} mm from the goal after {self._t:.0f}s "
                f"(max joint error {float(np.max(np.abs(self._q_goal - q_meas))):.3f} rad). "
                f"Arm blocked, stiffness too low, or leash too small?"
            )

    def _publish(self, q: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q]
        self._pub.publish(msg)
