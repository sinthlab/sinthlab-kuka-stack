#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Callable, Optional
import numpy as np

import rclpy
from rclpy.node import Node as rclpyNode
from ruckig import InputParameter, OutputParameter, Ruckig

from lbr_fri_idl.msg import LBRState
from geometry_msgs.msg import PoseStamped
import optas
from scipy.spatial.transform import Rotation as R
from sinthlab_bringup.helpers.common_threshold import DebugTicker, get_required_param
from sinthlab_bringup.helpers.param_logging import log_params_once


class MoveToPositionAction:
    """
    Reusable motion action that drives the robot to a target joint configuration.
    Standalone PTP-like action using Ruckig.
    - Subscribes to robot state (LBRState)
    - Plans and publishes joint setpoints to reach a target configuration
    - Calls on_complete() and goes idle when the target is reached
    """

    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Callable[[], None]) -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix
        
        # Tracks when the action is active
        self._ready = False

        # Parameters
        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._joint_pos_target_deg = np.array(
            get_required_param(node, self._param_prefix + "target_joint_position"), dtype=float
        )
        self._joint_pos_target = np.radians(self._joint_pos_target_deg)
        self._joint_pos_tol = float(get_required_param(node, self._param_prefix + "joint_move_tolerance"))
        
        if node.has_parameter(self._param_prefix + "wait_for_physical_arrival"):
            self._wait_for_physical_arrival = bool(node.get_parameter(self._param_prefix + "wait_for_physical_arrival").value)
        else:
            self._wait_for_physical_arrival = False

        self._v_max_param = np.radians(np.array(get_required_param(node, self._param_prefix + "move_to_pos_v_max"), dtype=float)).tolist()
        self._a_max_param = get_required_param(node, self._param_prefix + "move_to_pos_a_max")
        self._j_max_param = get_required_param(node, self._param_prefix + "move_to_pos_j_max")

        self._debug_log_enabled = bool(get_required_param(node, self._param_prefix + "debug_log_enabled"))
        debug_rate_hz = float(get_required_param(node, self._param_prefix + "debug_log_rate_hz"))
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
        robot_name = node.get_namespace().strip("/")
        self._cmd_topic = f"/{robot_name}/kuka_clik_controller/target_frame" if robot_name else "/kuka_clik_controller/target_frame"
        self._subscribers_ready = False 
        self._init = False
        self._moving = False
        self._q_init = np.zeros(7)
        self._q_cmd_sync = np.zeros(7)
        self._q_cmd_completion = np.zeros(7)
        self._q_meas_completion = np.zeros(7)
        self._shutdown_requested = False

        # Trajectory generator (Ruckig)
        self._trajectory_generation: Optional[Ruckig] = None
        self._trajectory_gen_in: Optional[InputParameter] = None
        self._trajectory_gen_out: Optional[OutputParameter] = None

        # ROS interfaces owned by the action
        self._state_sub = node.create_subscription(LBRState, "state", self._on_state, 1)
        self._pub_joint = node.create_publisher(PoseStamped, self._cmd_topic, 1)

        robot_desc = str(node.get_parameter("robot_description").value) if node.has_parameter("robot_description") else ""
        self.robot = optas.RobotModel(urdf_string=robot_desc)
        self._fk_func = self.robot.get_link_transform_function(link="lbr_link_ee", base_link="lbr_link_0", numpy_output=True)
       
        # Small delay to allow message to flush over DDS
        self._timer = node.create_timer(self._dt, self._step)

        node.get_logger().info(
            f"Move-to-pos action initialised: target(rad)={self._joint_pos_target.tolist()} tol={self._joint_pos_tol}",
        )

    # ------------------------------------------------------------------
    def _on_state(self, msg: LBRState) -> None:
        try:
            q_ipo = np.array(msg.ipo_joint_position.tolist(), dtype=float)
            q_cmd = np.array(msg.commanded_joint_position.tolist(), dtype=float)
            q_meas = np.array(msg.measured_joint_position.tolist(), dtype=float)
            
            # 1. Base equilibrium to start Ruckig from (avoid droop jump)
            # Priority reversed: q_cmd (anchor) over q_ipo (physics) prevents velocity faults
            if not np.isnan(q_cmd).any():
                self._q_init = q_cmd
            elif not np.isnan(q_ipo).any():
                self._q_init = q_ipo
            else:
                if not self._init:
                    self._node.get_logger().warn("ipo & commanded joint pos are NaN. Falling back to measured for initialization, this MAY cause a jump in Impedance Mode if not held tightly!")
                self._q_init = q_meas

            # 2. Command sync reference for Ruckig to ensure packets aren't dropped
            if not np.isnan(q_cmd).any():
                self._q_cmd_sync = q_cmd
            else:
                self._q_cmd_sync = q_meas
                
            # 3. Trajectory completion must ALWAYS check the commanded anchor!
            # Under Cartesian Impedance, the physical arm (q_meas) sags physically due
            # to gravity. Wait for q_cmd instead so the action completes as soon as
            # the virtual spring anchor reaches the target, regardless of droop.
            if not np.isnan(q_cmd).any():
                self._q_cmd_completion = q_cmd
            else:
                self._q_cmd_completion = q_meas
                
            if not np.isnan(q_meas).any():
                self._q_meas_completion = q_meas
            else:
                self._q_meas_completion = q_cmd

        except Exception:
            return
        # This flag makes sure we’ve actually seen the robot’s 
        # measured joint positions before we start generating 
        # a trajectory or checking tolerances, 
        # so the action never runs on stale or uninitialized state data.
        if not self._init:
            self._init = True

    def start(self) -> None:
        """Trigger the action to start."""
        if self._ready:
            self._node.get_logger().warn("MoveToPositionAction is already running.")
            return
        
        self._init = False
        self._moving = False
        self._subscribers_ready = False
        self._trajectory_generation = None
        self._shutdown_requested = False
        self._ready = True
        self._node.get_logger().info("MoveToPositionAction started.")

    def _step(self) -> None:
        # early exit if not active
        if not self._ready:
            return
        
        # Early exit if shutdown requested
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
                        f"Subscriber detected on '{self._cmd_topic}'. Starting move-to-pos."
                    )
                else:
                    if self._debug_log_enabled and self._dbg.tick(self._dt):
                        self._node.get_logger().info(
                            f"Waiting for a subscriber on '{self._cmd_topic}'..."
                        )
                    return
            except Exception:
                return

        if not self._init:
            return

        if self._moving:
            self._compute_move_to_pos()

    # ------------------------------------------------------------------
    def _compute_move_to_pos(self) -> None:
        # Prepare Ruckig once
        if self._trajectory_generation is None:
            self._prepare_ruckig(self._q_init)
            if self._trajectory_generation is None:
                # Could not prepare; stop trying
                self._node.get_logger().info(f"Move-to-position couldnot generate trajectory; stopping.")
                self._moving = False
                self._request_shutdown()
                return
        
        # Check completion against the *commanded* anchor (per-joint max error)
        # OR the *measured* physical joint positions, depending on context!
        if self._wait_for_physical_arrival:
            # During "recovery", we want to wait for the actual arm to recoil
            err = self._joint_pos_target - self._q_meas_completion
        else:
            # During "init", we want to complete as soon as spline arrives
            # so we don't get stuck waiting for physical perfection fighting against sagging gravity droop
            err = self._joint_pos_target - self._q_cmd_completion
            
        max_err = float(np.max(np.abs(err)))
        if max_err <= self._joint_pos_tol:
            self._moving = False
            self._node.get_logger().info(
                f"Move-to-position command completion check passed; holding position. rad (tol={self._joint_pos_tol:.4f} rad)"
            )
            self._request_shutdown()
            return
        
        # Synchronize Ruckig with the controller's actual received commands
        # If the ROS2 controller is lagging, inactive, or dropping packets, 
        # pause the spline generation until the controller catches up.
        sync_err = np.abs(np.array(self._trajectory_gen_in.current_position, dtype=float) - self._q_cmd_sync)
        if np.max(sync_err) > 0.05:
            # Controller is lagging behind our spline by more than 0.05 radians (~3 degrees)
            # which usually means the node started prior to the hardware or controller manager activating.
            # Pause generation and just republish the currently safe hold spline.
            if self._debug_log_enabled and self._dbg.tick(self._dt):
                self._node.get_logger().info(
                    f"Trajectory synchronization pause: controller commands lagging by {np.max(sync_err):.4f} rad. Waiting to catch up..."
                )
            pose_mat = self._fk_func(np.array(self._trajectory_gen_in.current_position, dtype=float))
            cmd = PoseStamped()
            cmd.header.frame_id = "lbr_link_0"
            cmd.header.stamp = self._node.get_clock().now().to_msg()
            cmd.pose.position.x = float(pose_mat[0, 3])
            cmd.pose.position.y = float(pose_mat[1, 3])
            cmd.pose.position.z = float(pose_mat[2, 3])
            quat = R.from_matrix(pose_mat[0:3, 0:3]).as_quat()
            cmd.pose.orientation.x = float(quat[0])
            cmd.pose.orientation.y = float(quat[1])
            cmd.pose.orientation.z = float(quat[2])
            cmd.pose.orientation.w = float(quat[3])
            self._pub_joint.publish(cmd)
            return

        # Step Trajectory generation and publish
        _ = self._trajectory_generation.update(self._trajectory_gen_in, self._trajectory_gen_out)
        trajectory_gen_cmd = np.array(self._trajectory_gen_out.new_position, dtype=float)
        pose_mat = self._fk_func(trajectory_gen_cmd)
        cmd = PoseStamped()
        cmd.header.frame_id = "lbr_link_0"
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.pose.position.x = float(pose_mat[0, 3])
        cmd.pose.position.y = float(pose_mat[1, 3])
        cmd.pose.position.z = float(pose_mat[2, 3])
        quat = R.from_matrix(pose_mat[0:3, 0:3]).as_quat()
        cmd.pose.orientation.x = float(quat[0])
        cmd.pose.orientation.y = float(quat[1])
        cmd.pose.orientation.z = float(quat[2])
        cmd.pose.orientation.w = float(quat[3])
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

        self._node.get_logger().info(
            f"Trajectory generation v_max={np.round(self._trajectory_gen_in.max_velocity,2)}, a_max={np.round(self._trajectory_gen_in.max_acceleration,2)}, j_max={np.round(self._trajectory_gen_in.max_jerk,2)}"
        )
        
    # ------------------------------------------------------------------
    def _request_shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._ready = False
        try:
            # Notify orchestrator that we are done
            self._on_complete()
        except Exception as exc:
            self._node.get_logger().error(
                f"Exception raised while move-to-pos completion callback: {exc}"
            )