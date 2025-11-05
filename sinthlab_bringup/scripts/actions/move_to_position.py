#!/usr/bin/env python3
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from ruckig import InputParameter, OutputParameter, Ruckig

from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState
from helpers.common_threshold import DebugTicker, get_required_param
from helpers.param_logging import log_params_once


class MoveToPositionAction:
    """
    Reusable motion action that drives the robot to a target joint configuration.
    Standalone PTP-like action using Ruckig.
    - Subscribes to robot state (LBRState)
    - Plans and publishes joint setpoints to reach a target configuration
    - Exits the process once the target is reached
    """

    def __init__(self, node: Node, *, on_complete: Callable[[], None]) -> None:
        self._node = node
        self._on_complete = on_complete

        # Parameters
        self._update_rate = int(get_required_param(node, "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._joint_pos_target_deg = np.array(
            get_required_param(node, "target_joint_position"), dtype=float
        )
        self._joint_pos_target = np.radians(self._joint_pos_target_deg)
        self._joint_pos_tol = float(get_required_param(node, "joint_move_tolerance"))

        self._v_max_param = np.radians(np.array(get_required_param(node, "move_to_pos_v_max"), dtype=float))
        self._a_max_param = get_required_param(node, "move_to_pos_a_max")
        self._j_max_param = get_required_param(node, "move_to_pos_j_max")

        self._debug_log_enabled = bool(get_required_param(node, "debug_log_enabled"))
        debug_rate_hz = float(get_required_param(node, "debug_log_rate_hz"))
        self._dbg = DebugTicker(debug_rate_hz)

        if self._debug_log_enabled:
            log_params_once(
                node,
                params={
                    "update_rate": self._update_rate,
                    "initial_joint_position": self._joint_pos_target.tolist(),
                    "joint_move_tolerance": self._joint_pos_tol,
                    "move_to_pos_v_max": self._v_max_param.tolist(),
                    "move_to_pos_a_max": self._a_max_param,
                    "move_to_pos_j_max": self._j_max_param,
                },
                context="move_to_pos",
            )

        # State
        self._cmd_topic = "command/joint_position"
        self._subscribers_ready = False 
        self._init = False
        self._moving = False
        self._joint_pos = np.zeros(7)
        self._shutdown_requested = False

        # Trajectory generator (Ruckig)
        self._trajectory_generation: Optional[Ruckig] = None
        self._trajectory_gen_in: Optional[InputParameter] = None
        self._trajectory_gen_out: Optional[OutputParameter] = None

        # ROS interfaces owned by the action
        self._state_sub = node.create_subscription(LBRState, "state", self._on_state, 1)
        self._pub_joint = node.create_publisher(LBRJointPositionCommand, self._cmd_topic, 1)
       
        # Small delay to allow message to flush over DDS
        self._timer = node.create_timer(self._dt, self._step)

        node.get_logger().info(
            f"Move-to-pos action initialised: target(rad)={self._joint_pos_target.tolist()} tol={self._joint_pos_tol}",
        )

    # ------------------------------------------------------------------
    def _on_state(self, msg: LBRState) -> None:
        try:
            self._joint_pos = np.array(msg.measured_joint_position.tolist(), dtype=float)
        except Exception:
            return
        # This flag makes sure we’ve actually seen the robot’s 
        # measured joint positions before we start generating 
        # a trajectory or checking tolerances, 
        # so the action never runs on stale or uninitialized state data.
        if not self._init:
            self._init = True

    def _step(self) -> None:
        if self._shutdown_requested:
            return

        # Wait until our command topic has a subscriber
        # That avoids publishing motion commands into the void
        # e.g., before the joint-position command controller comes up 
        # or after a reconnect
        if not self._subscribers_ready:
            try:
                if self._node.count_subscribers(self._cmd_topic) >= 1:
                    self._subscribers_ready = True
                    self._moving = True
                    self._node.get_logger().info(
                        "Subscriber detected on '%s'. Starting move-to-pos.",
                        self._cmd_topic,
                    )
                else:
                    if self._debug_log_enabled and self._dbg.tick(self._dt):
                        self._node.get_logger().info(
                            "Waiting for a subscriber on '%s'...", self._cmd_topic
                        )
                    return
            except Exception:
                return

        if not self._init:
            return

        if self._moving:
            self._compute_move_to_pos(self._joint_pos)

    # ------------------------------------------------------------------
    def _compute_move_to_pos(self, trajectory_generation: np.ndarray) -> None:
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
                f"Move-to-position complete; holding position. rad (tol={self._joint_pos_tol:.4f} rad)"
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
        
    # ------------------------------------------------------------------
    def _request_shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:
                pass
            self._timer = None
        try:
            self._on_complete()
        except Exception as exc:
            self._node.get_logger().error(
                "Exception raised while move-to-pos shutdown: %s", exc
            )
        self.destroy_node()
        rclpy.shutdown()