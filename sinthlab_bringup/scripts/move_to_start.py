#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from ruckig import Ruckig, InputParameter, OutputParameter

from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand


class MoveToStartNode(Node):
    """
    Standalone PTP-like move-to-start node using Ruckig.
    - Subscribes to robot state (LBRState)
    - Plans and publishes joint setpoints to reach an initial target configuration
    - Exits the process once the target is reached
    """

    def __init__(self) -> None:
        super().__init__(
            "move_to_start",
            automatically_declare_parameters_from_overrides=True,
        )

        # Helper to get parameter or default
        def _param(name: str, default):
            try:
                if self.has_parameter(name):
                    p = self.get_parameter(name)
                    return p.value if p is not None else default
            except Exception:
                pass
            return default

        # Parameters (minimal set)
        self._update_rate = int(_param("update_rate", 100))
        self._dt = 1.0 / float(self._update_rate)
        self._use_initial = bool(_param("use_initial_joint_position", True))
        self._joint_pos_target = np.array(
            _param(
                "initial_joint_position",
                [0.0, math.radians(10.0), 0.0, math.radians(-80.0), 0.0, math.radians(90.0), 0.0],
            ),
            dtype=float,
        )
        self._joint_pos_tol = float(_param("joint_move_tolerance", 0.01))
        # Limits (can be scalar or 7-long list)
        self._v_max_param = _param("move_to_start_v_max", 1.0)
        self._a_max_param = _param("move_to_start_a_max", 2.0)
        self._j_max_param = _param("move_to_start_j_max", 10.0)

        # State
        self._init = False
        self._moving = self._use_initial
        self._joint_pos = np.zeros(7)
        self._shutdown_requested = False

        # Trajectory generator (Ruckig) members (lazy init)
        self._trajectory_generation: Optional[Ruckig] = None
        self._trajectory_gen_in: Optional[InputParameter] = None
        self._trajectory_gen_out: Optional[OutputParameter] = None

        # ROS I/O
        self._sub = self.create_subscription(LBRState, "state", self._on_state, 1)
        self._pub_joint = self.create_publisher(LBRJointPositionCommand, "command/joint_position", 1)

        # Control timer
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f"move-to-start enabled={self._moving}, trajectory_gen_target(rad)={self._joint_pos_target.tolist()}, tol={self._joint_pos_tol}"
        )

    def _on_state(self, msg: LBRState) -> None:
        # Cache measured joint position
        try:
            self._joint_pos = np.array(msg.measured_joint_position.tolist(), dtype=float)
        except Exception:
            return
        if not self._init:
            self._init = True

    def _request_shutdown(self) -> None:
        if not self._shutdown_requested:
            self._shutdown_requested = True
            self.get_logger().info("Move-to-start reached target; shutting down this node.")
            # Delay slightly to allow last publish to flush
            self.create_timer(0.05, lambda: rclpy.shutdown())

    def _compute_move_to_start(self, trajectory_generation: np.ndarray) -> None:
        # Prepare Ruckig once
        if self._trajectory_generation is None:
            self._prepare_ruckig(trajectory_generation)
            if self._trajectory_generation is None:
                # Could not prepare; stop trying
                self._moving = False
                self._request_shutdown()
                return
        # Check completion
        err = self._joint_pos_target - trajectory_generation
        if float(np.linalg.norm(err)) <= self._joint_pos_tol:
            self._moving = False
            self.get_logger().info("Move-to-start complete; holding position.")
            self._request_shutdown()
            return
        # Step Trajectory generation and publish
        _ = self._trajectory_generation.update(self._trajectory_gen_in, self._trajectory_gen_out)
        trajectory_gen_cmd = np.array(self._trajectory_gen_out.new_position, dtype=float)
        cmd = LBRJointPositionCommand()
        cmd.joint_position = trajectory_gen_cmd.tolist()
        self._pub_joint.publish(cmd)
        # Feed state forward for smooth next step
        self._trajectory_gen_in.current_position = self._trajectory_gen_out.new_position
        self._trajectory_gen_in.current_velocity = self._trajectory_gen_out.new_velocity
        self._trajectory_gen_in.current_acceleration = self._trajectory_gen_out.new_acceleration

    def _prepare_ruckig(self, trajectory_gen_now: np.ndarray) -> None:
        dofs = 7
        self._trajectory_generation = Ruckig(dofs, self._dt)
        self._trajectory_gen_in = InputParameter(dofs)
        self._trajectory_gen_out = OutputParameter(dofs)
        # Current state
        self._trajectory_gen_in.current_position = np.array(trajectory_gen_now, dtype=float).tolist()
        self._trajectory_gen_in.current_velocity = [0.0] * dofs
        self._trajectory_gen_in.current_acceleration = [0.0] * dofs
        # Target state
        self._trajectory_gen_in.target_position = self._joint_pos_target.tolist()
        self._trajectory_gen_in.target_velocity = [0.0] * dofs
        self._trajectory_gen_in.target_acceleration = [0.0] * dofs

        # Limits: support scalar or 7-array forms
        def _to_arr(x, default_val):
            if isinstance(x, (list, tuple)) and len(x) == dofs:
                return list(map(float, x))
            try:
                return [float(x)] * dofs
            except Exception:
                return [default_val] * dofs

        self._trajectory_gen_in.max_velocity = _to_arr(self._v_max_param, 1.0)
        self._trajectory_gen_in.max_acceleration = _to_arr(self._a_max_param, 2.0)
        self._trajectory_gen_in.max_jerk = _to_arr(self._j_max_param, 10.0)

        self.get_logger().info(
            f"Trajectory generation v_max={np.round(self._trajectory_gen_in.max_velocity,2)}, a_max={np.round(self._trajectory_gen_in.max_acceleration,2)}, j_max={np.round(self._trajectory_gen_in.max_jerk,2)}"
        )

    def _step(self) -> None:
        # Timer-driven control loop
        if not self._init:
            return
        if self._moving:
            self._compute_move_to_start(self._joint_pos)
        elif not self._shutdown_requested:
            # If not moving anymore and shutdown not yet requested, do it now
            self._request_shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveToStartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
