#!/usr/bin/env python3
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from ruckig import Ruckig, InputParameter, OutputParameter
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import time

from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand
from helpers.common_threshold import get_required_param, DebugTicker
from helpers.param_logging import log_params_once


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

        # Parameters (all required, provided via YAML or overrides)
        self._update_rate = int(get_required_param(self, "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._use_initial = bool(get_required_param(self, "use_initial_joint_position"))
        self._joint_pos_target_rad = np.array(get_required_param(self, "initial_joint_position"), dtype=float)
        self._joint_pos_target = np.radians(self._joint_pos_target_rad) 
        self._joint_pos_tol = float(get_required_param(self, "joint_move_tolerance"))
        # Limits (can be scalar or 7-long list)
        self._v_max_param_deg = np.array(get_required_param(self, "move_to_start_v_max"), dtype=float)
        self._v_max_param = np.radians(self._v_max_param_deg)
        self._a_max_param = get_required_param(self, "move_to_start_a_max")
        self._j_max_param = get_required_param(self, "move_to_start_j_max")

        # Debug logging once if enabled
        self._debug_log_enabled = bool(get_required_param(self, "debug_log_enabled"))
        self._debug_log_rate_hz = float(get_required_param(self, "debug_log_rate_hz"))  # logs per second
        self._dbg = DebugTicker(self._debug_log_rate_hz)

        if self._debug_log_enabled:
            log_params_once(
                self,
                params={
                    "update_rate": self._update_rate,
                    "use_initial_joint_position": self._use_initial,
                    "initial_joint_position": self._joint_pos_target.tolist(),
                    "joint_move_tolerance": self._joint_pos_tol,
                    "move_to_start_v_max": self._v_max_param.tolist(),
                    "move_to_start_a_max": self._a_max_param,
                    "move_to_start_j_max": self._j_max_param,
                },
                context="move_to_start",
            )

        # State
        # Simple readiness gating: wait until our command topic has a subscriber
        self._cmd_topic = "command/joint_position"
        self._subscribers_ready = False

        self._init = False
        self._moving = False
        self._joint_pos = np.zeros(7)
        self._shutdown_requested = False

        # Trajectory generator (Ruckig) members (lazy init)
        self._trajectory_generation: Optional[Ruckig] = None
        self._trajectory_gen_in: Optional[InputParameter] = None
        self._trajectory_gen_out: Optional[OutputParameter] = None

        # ROS I/O
        self._sub = self.create_subscription(LBRState, "state", self._on_state, 1)
        self._pub_joint = self.create_publisher(LBRJointPositionCommand, self._cmd_topic, 1)
        # Done signal (latched/transient-local so late subscribers still receive it)
        done_qos = QoSProfile(depth=1)
        done_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        done_qos.reliability = ReliabilityPolicy.RELIABLE
        self._pub_done = self.create_publisher(Bool, "move_to_start/done", done_qos)

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
            # Publish done signal before shutting down, so downstream nodes can proceed
            try:
                self._pub_done.publish(Bool(data=True))
                self.get_logger().info("Move-to-start reached target; published done=true. Shutting down this node.")
            except Exception:
                self.get_logger().warn("Failed to publish move_to_start/done; proceeding to shutdown.")
            # Small delay to allow message to flush over DDS
            try:
                time.sleep(0.05)
            except Exception:
                pass
            # Shutdown
            rclpy.shutdown()

    def _compute_move_to_start(self, trajectory_generation: np.ndarray) -> None:
        # Prepare Ruckig once
        if self._trajectory_generation is None:
            self._prepare_ruckig(trajectory_generation)
            if self._trajectory_generation is None:
                # Could not prepare; stop trying
                self._moving = False
                self._request_shutdown()
                return
        # Check completion (per-joint max error)
        err = self._joint_pos_target - trajectory_generation
        max_err = float(np.max(np.abs(err)))
        if max_err <= self._joint_pos_tol:
            self._moving = False
            self.get_logger().info(
                f"Move-to-start complete; holding position. max_err={max_err:.4f} rad (tol={self._joint_pos_tol:.4f} rad)"
            )
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
        # Wait until our command topic has at least one subscriber (controller ready)
        if not self._subscribers_ready:
            try:
                if self.count_subscribers(self._cmd_topic) >= 1:
                    self._subscribers_ready = True
                    self._moving = self._use_initial
                    self.get_logger().info(
                        f"Subscriber detected on '{self._cmd_topic}'. Starting move-to-start."
                    )
                else:
                    if self._debug_log_enabled and self._dbg.tick(self._dt):
                        self.get_logger().info(
                            f"Waiting for a subscriber on '{self._cmd_topic}'..."
                        )
                    return
            except Exception:
                return

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