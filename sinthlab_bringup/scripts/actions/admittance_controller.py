#!/usr/bin/env python3
from typing import Callable
import re
import numpy as np
from numpy.typing import NDArray

import optas

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRJointPositionCommand, LBRState
from helpers.common_threshold import get_required_param

"""
NOTE: Most part of this code has been taken from the lbr_demos_advanced_py package 
present underlay in lbr_fri_ros2_stack.
All the credits for the original code goes to its original authors.
"""

class AdmittanceControlAction:
    def __init__(self, node: rclpyNode, 
                 *, 
                 to_start: Callable[[], None],
                 in_action: Callable[[], None], 
                 on_complete: Callable[[], None]
            ) -> None:
        self._node = node
        self._to_start = to_start
        self._in_action = in_action
        self._on_complete = on_complete

        self._init = False
        # Flag to track readiness to start the action
        self._ready = False
        # Flag to track if the start action has been started
        # to signal the on_complete only once
        self._start_done = False

        self._lbr_state = LBRState()

        self._update_rate = int(get_required_param(node, "update_rate"))
        self._dt = 1.0 / float(self._update_rate)
        
        # create subscriber to receive LBRState messages on state change
        self._state_topic = str(get_required_param(node, "lbr_state_topic"))
        self._lbr_state_sub = node.create_subscription(
            LBRState, self._state_topic, self._on_lbr_state, 1
        )
        
        # create publisher to publish joint position commands
        self._command_topic = str(get_required_param(node, "lbr_joint_position_command_topic"))
        self._lbr_joint_position_command_pub = node.create_publisher(
            LBRJointPositionCommand, self._command_topic, 1 
        )
        
        self._exp_smooth = float(get_required_param(node, "exp_smooth"))
        if not 0.0 <= self._exp_smooth <= 1.0:
            raise ValueError("Exponential smoothing factor must be in [0, 1].")
        
        stiffness_scale = float(get_required_param(node, "stiffness_scale"))
        if not 0.0 < stiffness_scale <= 1.0:
            raise ValueError("stiffness_scale must be in (0, 1].")

        scaled_f_ext_th = stiffness_scale * np.array(get_required_param(node, "f_ext_th"))
        scaled_dq_gains = stiffness_scale * np.array(get_required_param(node, "dq_gains"))
        scaled_dx_gains = stiffness_scale * np.array(get_required_param(node, "dx_gains"))

        self._controller = AdmittanceController(
            robot_description=self._strip_ros2_control(str(get_required_param(node, "robot_description"))),
            base_link=str(get_required_param(node, "base_link")),
            end_effector_link=str(get_required_param(node, "end_effector_link")),
            f_ext_th=scaled_f_ext_th,
            dq_gains=scaled_dq_gains,
            dx_gains=scaled_dx_gains,
        )
    
    def _strip_ros2_control(self, urdf: str) -> str:
        # Remove ros2_control blocks which optas cannot parse in 
        # the URDF of robot description
        return re.sub(r"<ros2_control[^>]*>.*?</ros2_control>", "", urdf, flags=re.DOTALL)

    def _on_lbr_state(self, lbr_state: LBRState) -> None:
        if not self._ready:
            if not self._to_start():
                return # stay idle until ready to start the admittance control
            self._ready = True # latch once the gate opens
        
        self._smooth_lbr_state(lbr_state)
        lbr_command = self._controller(self._lbr_state, self._dt)
        self._lbr_joint_position_command_pub.publish(lbr_command)

        if self._ready and not self._start_done:
            # using in_action callback to signal that admittance control has started
            self._in_action()
            self._start_done = True

        elif self._ready and self._start_done:
            # using on_complete callback to signal that admittance control has completed
            # This will be called repeatedly to check for completion condition
            self._on_complete()

    def _smooth_lbr_state(self, lbr_state: LBRState) -> None:
        if not self._init:
            self._lbr_state = lbr_state
            self._init = True
            return

        self._lbr_state.measured_joint_position = (
            (1 - self._exp_smooth)
            * np.array(self._lbr_state.measured_joint_position.tolist())
            + self._exp_smooth * np.array(lbr_state.measured_joint_position.tolist())
        ).data

        self._lbr_state.external_torque = (
            (1 - self._exp_smooth) * np.array(self._lbr_state.external_torque.tolist())
            + self._exp_smooth * np.array(lbr_state.external_torque.tolist())
        ).data

class AdmittanceController(object):
    """
    Cartesian admittance controller operating in joint position space.
    Code directly taken from lbr_demos_advanced_py/admittance_controller.py
    """
    def __init__(
        self,
        robot_description: str,
        base_link: str,
        end_effector_link: str,
        f_ext_th: NDArray,
        dq_gains: NDArray,
        dx_gains: NDArray
    ) -> None:
        self._lbr_joint_position_command = LBRJointPositionCommand()

        self._robot = optas.RobotModel(urdf_string=robot_description)

        self._jacobian_func = self._robot.get_link_geometric_jacobian_function(
            link=end_effector_link, base_link=base_link, numpy_output=True
        )

        self._dof = self._robot.ndof
        self._jacobian = np.zeros((6, self._dof))
        self._jacobian_inv = np.zeros((self._dof, 6))
        self._q = np.zeros(self._dof)
        self._dq = np.zeros(self._dof)
        self._tau_ext = np.zeros(6)
        self._dq_gains = np.diag(dq_gains)
        self._dx_gains = np.diag(dx_gains)
        self._f_ext = np.zeros(6)
        self._f_ext_th = f_ext_th
        self._alpha = 0.95

    def __call__(self, lbr_state: LBRState, dt: float) -> LBRJointPositionCommand:
        self._q = np.array(lbr_state.measured_joint_position.tolist())
        self._tau_ext = np.array(lbr_state.external_torque.tolist())

        self._jacobian = self._jacobian_func(self._q)
        self._jacobian_inv = np.linalg.pinv(self._jacobian, rcond=0.1)
        self._f_ext = self._jacobian_inv.T @ self._tau_ext

        dx = np.where(
            abs(self._f_ext) > self._f_ext_th,
            self._dx_gains @ np.sign(self._f_ext) * (abs(self._f_ext) - self._f_ext_th),
            0.0,
        )

        # additional smoothing required in python
        self._dq = (
            self._alpha * self._dq
            + (1 - self._alpha) * self._dq_gains @ self._jacobian_inv @ dx
        )

        self._lbr_joint_position_command.joint_position = (
            np.array(lbr_state.measured_joint_position.tolist()) + dt * self._dq
        ).data

        return self._lbr_joint_position_command