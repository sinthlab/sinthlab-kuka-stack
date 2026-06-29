#!/usr/bin/env python3
"""Shared machinery for the move / perturb actions.

``MoveActionBase`` owns everything the concrete actions have in common: the ``LBRState``
subscription, the Ruckig joint-trajectory stepping, the start/step/shutdown lifecycle, and the
forward-kinematics handle. Concrete subclasses provide ONLY the parts that actually differ:

  * ``_configure_target(node)``      - read the target params, set ``self._joint_pos_target``
  * ``_configure_command(node, ns)`` - set ``self._cmd_topic`` and create ``self._pub``
  * ``_publish_setpoint(q)``         - publish one trajectory sample
  * ``_completion_reached()``        - decide when the move is done
  * ``_resolve_target()`` (optional) - recompute the target at run time (perturbation)
  * ``_describe_target()`` (optional)- one-line description for the init log

Concrete classes (one focused file each):
  * ``MoveToPositionJointSpace``     - move_to_position_joint_space.py
  * ``PerturbInitialPosition``       - perturb_initial_position.py
"""
from __future__ import annotations

from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from ruckig import InputParameter, OutputParameter, Ruckig

from lbr_fri_idl.msg import LBRState
import optas
from sinthlab_bringup.helpers.common_threshold import DebugTicker, get_required_param
from sinthlab_bringup.helpers.param_logging import log_params_once


class MoveActionBase:
    """Ruckig PTP move with a pluggable target / command / completion. Not used directly.

    - Subscribes to robot state (``LBRState`` on ``<ns>/lbr_state``).
    - Plans and streams joint setpoints with Ruckig to reach ``self._joint_pos_target``.
    - Calls ``on_complete()`` and goes idle once the (subclass-defined) completion test passes.
    """

    def __init__(self, node: rclpyNode, *, param_prefix: str = "",
                 on_complete: Callable[[], None]) -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        # Tracks when the action is active
        self._ready = False

        # --- common parameters -------------------------------------------------
        self._update_rate = int(get_required_param(node, self._param_prefix + "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        self._joint_pos_tol = float(get_required_param(node, self._param_prefix + "joint_move_tolerance"))
        # move_to_pos_v_max is in deg/s (converted to rad/s here); a_max / j_max are rad/s^2, rad/s^3.
        self._v_max_param = np.radians(np.array(get_required_param(node, self._param_prefix + "move_to_pos_v_max"), dtype=float)).tolist()
        self._a_max_param = get_required_param(node, self._param_prefix + "move_to_pos_a_max")
        self._j_max_param = get_required_param(node, self._param_prefix + "move_to_pos_j_max")
        self._debug_log_enabled = bool(get_required_param(node, self._param_prefix + "debug_log_enabled"))
        self._dbg = DebugTicker(float(get_required_param(node, self._param_prefix + "debug_log_rate_hz")))

        # --- forward kinematics (shared by completion / perturbation IK) -------
        robot_desc = str(node.get_parameter("robot_description").value) if node.has_parameter("robot_description") else ""
        self.robot = optas.RobotModel(urdf_string=robot_desc)
        self._fk_func = self.robot.get_link_transform_function(link="lbr_link_ee", base_link="lbr_link_0", numpy_output=True)

        # Joint target (subclass replaces this; a perturbation resolves it at run time).
        self._joint_pos_target = np.zeros(7)
        self._configure_target(node)

        # --- runtime state -----------------------------------------------------
        self._subscribers_ready = False
        self._init = False
        self._moving = False
        self._target_resolved = False
        self._q_init = np.zeros(7)
        self._q_cmd_sync = np.zeros(7)
        self._q_cmd_completion = np.zeros(7)
        self._q_meas_completion = np.zeros(7)
        self._shutdown_requested = False

        # Trajectory generator (Ruckig)
        self._trajectory_generation: Optional[Ruckig] = None
        self._trajectory_gen_in: Optional[InputParameter] = None
        self._trajectory_gen_out: Optional[OutputParameter] = None

        # --- ROS interfaces ----------------------------------------------------
        robot_name = node.get_namespace().strip("/")
        ns = f"/{robot_name}" if robot_name else ""
        # The lbr_state_broadcaster publishes LBRState on '<ns>/lbr_state' (not 'state').
        self._state_sub = node.create_subscription(LBRState, "lbr_state", self._on_state, 1)
        self._configure_command(node, ns)  # subclass sets self._cmd_topic and self._pub

        if self._debug_log_enabled:
            log_params_once(
                node,
                params={
                    "update_rate": self._update_rate,
                    "joint_move_tolerance": self._joint_pos_tol,
                    "move_to_pos_v_max": self._v_max_param,
                    "move_to_pos_a_max": self._a_max_param,
                    "move_to_pos_j_max": self._j_max_param,
                },
                context=type(self).__name__,
            )

        self._timer = node.create_timer(self._dt, self._step)
        node.get_logger().info(
            f"{type(self).__name__} initialised: {self._describe_target()} tol={self._joint_pos_tol}"
        )

    # ===================== subclass hooks =====================================
    def _configure_target(self, node: rclpyNode) -> None:
        """Read target params and set ``self._joint_pos_target`` (radians). Required."""
        raise NotImplementedError

    def _configure_command(self, node: rclpyNode, ns: str) -> None:
        """Set ``self._cmd_topic`` and create ``self._pub``. Required."""
        raise NotImplementedError

    def _publish_setpoint(self, joint_positions: np.ndarray) -> None:
        """Publish one trajectory sample on ``self._pub``. Required."""
        raise NotImplementedError

    def _completion_reached(self) -> bool:
        """Return True once the move is done. Required."""
        raise NotImplementedError

    def _resolve_target(self) -> None:
        """Optionally recompute ``self._joint_pos_target`` from the live start pose. Default: no-op."""
        return

    def _describe_target(self) -> str:
        """One-line target description for the init log. Optional."""
        return "target unset"

    # ----- helper shared by the absolute-target subclasses --------------------
    def _read_joint_target_param(self, node: rclpyNode) -> np.ndarray:
        deg = np.array(get_required_param(node, self._param_prefix + "target_joint_position"), dtype=float)
        return np.radians(deg)

    # ===================== shared lifecycle ===================================
    def _on_state(self, msg: LBRState) -> None:
        try:
            q_ipo = np.array(msg.ipo_joint_position.tolist(), dtype=float)
            q_cmd = np.array(msg.commanded_joint_position.tolist(), dtype=float)
            q_meas = np.array(msg.measured_joint_position.tolist(), dtype=float)

            # 1. Base equilibrium to start Ruckig from (avoid droop jump).
            # Priority: q_cmd (anchor) over q_ipo (physics) prevents velocity faults.
            if not np.isnan(q_cmd).any():
                self._q_init = q_cmd
            elif not np.isnan(q_ipo).any():
                self._q_init = q_ipo
            else:
                if not self._init:
                    self._node.get_logger().warn(
                        "ipo & commanded joint pos are NaN. Falling back to measured for initialization, "
                        "this MAY cause a jump in Impedance Mode if not held tightly!"
                    )
                self._q_init = q_meas

            # 2. Command sync reference for Ruckig to ensure packets aren't dropped.
            if not np.isnan(q_cmd).any():
                self._q_cmd_sync = q_cmd
            else:
                self._q_cmd_sync = q_meas

            # 3. Completion checks the commanded anchor: under Cartesian impedance the physical arm
            # (q_meas) sags under gravity, so wait for q_cmd to reach the target regardless of droop.
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
        # Don't plan/check until we've actually seen the robot's joint state once.
        if not self._init:
            self._init = True

    def start(self) -> None:
        """Trigger the action to start."""
        if self._ready:
            self._node.get_logger().warn(f"{type(self).__name__} is already running.")
            return

        self._init = False
        self._moving = False
        self._subscribers_ready = False
        self._target_resolved = False
        self._trajectory_generation = None
        self._shutdown_requested = False
        self._ready = True
        self._node.get_logger().info(f"{type(self).__name__} started.")

    def _step(self) -> None:
        if not self._ready or self._shutdown_requested:
            return

        # Wait until our command topic has a subscriber, so we don't publish into the void
        # (e.g. before the controller comes up or after a reconnect).
        if not self._subscribers_ready:
            try:
                if self._node.count_subscribers(self._cmd_topic) >= 1:
                    self._subscribers_ready = True
                    self._moving = True
                    self._node.get_logger().info(
                        f"Subscriber detected on '{self._cmd_topic}'. Starting {type(self).__name__}."
                    )
                else:
                    if self._debug_log_enabled and self._dbg.tick(self._dt):
                        self._node.get_logger().info(f"Waiting for a subscriber on '{self._cmd_topic}'...")
                    return
            except Exception:
                return

        if not self._init:
            return

        if self._moving:
            self._compute_move_to_pos()

    def _compute_move_to_pos(self) -> None:
        # Prepare Ruckig once.
        if self._trajectory_generation is None:
            # Resolve a run-time target (e.g. a polar perturbation) from the live start pose.
            if not self._target_resolved:
                self._resolve_target()
                self._target_resolved = True
            self._prepare_ruckig(self._q_init)
            if self._trajectory_generation is None:
                self._node.get_logger().info(f"{type(self).__name__} could not generate trajectory; stopping.")
                self._moving = False
                self._request_shutdown()
                return

        # Completion (subclass-defined).
        if self._completion_reached():
            self._moving = False
            self._request_shutdown()
            return

        # Step the joint trajectory and publish the next setpoint (subclass-defined).
        _ = self._trajectory_generation.update(self._trajectory_gen_in, self._trajectory_gen_out)
        self._publish_setpoint(np.array(self._trajectory_gen_out.new_position, dtype=float))

        # Feed state forward for a smooth next step.
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
            f"Trajectory generation v_max={np.round(self._trajectory_gen_in.max_velocity,2)}, "
            f"a_max={np.round(self._trajectory_gen_in.max_acceleration,2)}, "
            f"j_max={np.round(self._trajectory_gen_in.max_jerk,2)}"
        )

    def _request_shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._ready = False
        try:
            self._on_complete()
        except Exception as exc:
            self._node.get_logger().error(f"Exception raised while {type(self).__name__} completion callback: {exc}")
