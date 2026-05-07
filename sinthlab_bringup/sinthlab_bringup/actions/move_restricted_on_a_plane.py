#!/usr/bin/env python3
from __future__ import annotations

import copy
from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

from sinthlab_bringup.helpers.common_threshold import get_required_param

class MoveRestrictedOnAPlaneAction:
    """
    Action that applies mathematical virtual fixtures (boundaries)
    by overriding the requested Cartesian pose via rapid IK through MoveItPy.
    """
    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Optional[Callable[[], None]] = None) -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._active = False

        state_topic = get_required_param(node, self._param_prefix + "state_topic").value
        cmd_topic = get_required_param(node, self._param_prefix + "command_topic").value
        self.ee_link = get_required_param(node, self._param_prefix + "end_effector_link").value
        self.base_link = get_required_param(node, self._param_prefix + "base_link").value

        self._state_sub = node.create_subscription(LBRState, state_topic, self._state_cb, 1)
        self._cmd_pub = node.create_publisher(LBRJointPositionCommand, cmd_topic, 1)

        node.get_logger().info("Initializing MoveIt Python API for rapid kinematics...")
        self.moveit_py = MoveItPy(node_name=node.get_name())
        self.robot_model = self.moveit_py.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        self.last_measured_joints = np.zeros(7)
        
    def start(self) -> None:
        if self._active:
            self._node.get_logger().warn("MoveRestrictedOnAPlaneAction is already active.")
            return
        self._active = True
        self._node.get_logger().info("Restricted Plane Action started, applying boundary IK.")

    def apply_surface_constraints(self, transform: np.ndarray) -> tuple[np.ndarray, bool]:
        """
        Takes in a 4x4 homogenous transformation matrix (X,Y,Z position and rotation).
        Applies mathematical clipping based on virtual fixtures.
        Returns the clipped transformation matrix, and a boolean indicating if it was restricted.
        """
        orig_transform = copy.deepcopy(transform)
        restricted = False
        
        # Extract current Cartesian XYZ
        x = transform[0, 3]
        y = transform[1, 3]
        z = transform[2, 3]

        # ---- CONSTRAINT 1: Flat floor/table (Z >= 0.3 meters) ----
        if z < 0.3:
            z = 0.3
            restricted = True

        # ---- CONSTRAINT 2: Bounding Box (X must be between 0.2 and 0.5) ----
        if x < 0.2:
            x = 0.2
            restricted = True
        elif x > 0.5:
            x = 0.5
            restricted = True

        # Re-pack the XYZ back into the transformation matrix
        transform[0, 3] = x
        transform[1, 3] = y
        transform[2, 3] = z

        return transform, restricted


    def _state_cb(self, msg: LBRState):
        """
        Runs continuously at the hardware frequency (~100-200Hz).
        Reads measured state -> Forward Kinematics -> Constraint Check -> Inverse Kinematics -> Command.
        """
        if not self._active:
            return

        self.last_measured_joints = np.array(msg.measured_joint_position)

        # 1. Forward Kinematics: Where is the arm mathematically right now?
        self.robot_state.set_joint_group_positions("arm", self.last_measured_joints)
        self.robot_state.update()
        current_pose = self.robot_state.get_global_link_transform(self.ee_link)

        # 2. Check and Apply our Mathematical Surface boundaries
        target_pose, is_restricted = self.apply_surface_constraints(current_pose)

        cmd = LBRJointPositionCommand()
        
        if is_restricted:
            # 3. Inverse Kinematics: The arm crossed the wall. 
            # Solve for the exact joint angles needed to hold the arm perfectly onto the edge of the wall.
            success = self.robot_state.set_from_ik("arm", target_pose, self.ee_link, timeout=0.005)
            if success:
                cmd.joint_position = self.robot_state.get_joint_group_positions("arm").tolist()
            else:
                # IK Failed to find a solution on the wall. Fallback to measured to prevent jumping.
                cmd.joint_position = msg.measured_joint_position.tolist()
        else:
            # The arm is in free-space. Shadow the hand perfectly so it feels weightless (Zero displacement).
            cmd.joint_position = msg.measured_joint_position.tolist()

        # 4. Command the spring equilibrium
        self._cmd_pub.publish(cmd)