#!/usr/bin/env python3
from __future__ import annotations

import copy
from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand

# Robotics Toolbox for intuitive Python kinematics
import roboticstoolbox as rtb
import tempfile
import os

from sinthlab_bringup.helpers.common_threshold import get_required_param

class MoveRestrictedOnAPlaneAction:
    """
    Action that applies mathematical virtual fixtures (boundaries)
    by overriding the requested Cartesian pose via rapid IK through Robotics Toolbox.
    """
    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Optional[Callable[[], None]] = None) -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._active = False

        state_topic = str(get_required_param(node, self._param_prefix + "state_topic"))
        cmd_topic = str(get_required_param(node, self._param_prefix + "command_topic"))
        self.ee_link = str(get_required_param(node, self._param_prefix + "end_effector_link"))
        self.base_link = str(get_required_param(node, self._param_prefix + "base_link"))

        self._state_sub = node.create_subscription(LBRState, state_topic, self._state_cb, 1)
        self._cmd_pub = node.create_publisher(LBRJointPositionCommand, cmd_topic, 1)

        node.get_logger().info("Initializing Robotics Toolbox for rapid kinematics...")
        
        robot_description = ""
        if node.has_parameter("robot_description"):
            robot_description = str(node.get_parameter("robot_description").value)
        
        # roboticstoolbox only accept a file path and not the urdf string
        # Write URDF to a temporary file for roboticstoolbox to parse
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as temp_urdf:
            temp_urdf.write(robot_description)
            temp_urdf_path = temp_urdf.name
        self.robot = rtb.ERobot.URDF(temp_urdf_path)
        os.remove(temp_urdf_path)
        
        self.last_measured_joints = np.zeros(self.robot.n)
        
        # Load virtual fixture profile
        self.active_profile = "bounding_box"
        if node.has_parameter(self._param_prefix + "virtual_fixture_profile"):
            self.active_profile = str(node.get_parameter(self._param_prefix + "virtual_fixture_profile").value)
            
        self.profile_config = {}
        prefix = self._param_prefix + f"virtual_fixtures.{self.active_profile}."
        if node.has_parameter(prefix + "type"):
            self.profile_config["type"] = str(node.get_parameter(prefix + "type").value)
            for key in ["z_min", "x_min", "x_max", "radius", "center_x", "center_y", "amplitude", "spatial_freq", "y_offset"]:
                if node.has_parameter(prefix + key):
                    self.profile_config[key] = float(node.get_parameter(prefix + key).value)
        else:
            node.get_logger().warn(f"Could not find virtual fixture profile {self.active_profile}")
        
    def start(self) -> None:
        if self._active:
            self._node.get_logger().warn("MoveRestrictedOnAPlaneAction is already active.")
            return
        self._active = True
        self._node.get_logger().info("Restricted Plane Action started, applying boundary IK.")

    def stop(self) -> None:
        self._active = False
        self._node.get_logger().info("Restricted Plane Action stopped.")

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

        # Apply dynamic profile constraints
        ptype = self.profile_config["type"]
        
        if ptype == "cartesian_box":
            if "z_min" in self.profile_config and z < self.profile_config["z_min"]:
                z = self.profile_config["z_min"]
                restricted = True
            if "x_min" in self.profile_config and x < self.profile_config["x_min"]:
                x = self.profile_config["x_min"]
                restricted = True
            if "x_max" in self.profile_config and x > self.profile_config["x_max"]:
                x = self.profile_config["x_max"]
                restricted = True
        elif ptype == "cylinder":
            cx = self.profile_config["center_x"]
            cy = self.profile_config["center_y"]
            r_max = self.profile_config["radius"]
            
            dx = x - cx
            dy = y - cy
            dist = np.sqrt(dx*dx + dy*dy)
            
            if dist > r_max:
                scale = r_max / dist
                x = cx + dx * scale
                y = cy + dy * scale
                restricted = True
        elif ptype == "sinusoid":
            amp = self.profile_config["amplitude"]
            freq = self.profile_config["spatial_freq"]
            y_off = self.profile_config["y_offset"]
            z_m = self.profile_config["z_min"]
            
            # 1. Enforce floor height so we don't hit the table while tracing the wave
            if z < z_m:
                z = z_m
                restricted = True
                
            # 2. Enforce the sinusoidal mathematical manifold on the XY plane
            ideal_y = amp * np.sin(freq * x) + y_off
            
            # Snap the Y axis perfectly to the wave form for any given X
            if not np.isclose(y, ideal_y, atol=1e-4):
                y = ideal_y
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
        # fkine returns an SE3 object, .A gives the 4x4 numpy matrix
        current_pose = self.robot.fkine(self.last_measured_joints).A

        # We must clone the pose so the constraint application doesn't overwrite current_pose in memory
        target_pose_input = copy.deepcopy(current_pose)
        
        # 2. Check and Apply our Mathematical Surface boundaries
        target_pose, is_restricted = self.apply_surface_constraints(target_pose_input)

        cmd = LBRJointPositionCommand()
        
        if is_restricted:
            # 3. Inverse Kinematics using Robotics Toolbox Levenberg-Marquardt
            # We seed the solver with our current joints to calculate the minimal distance push
            ik_solution = self.robot.ikine_LM(target_pose, q0=self.last_measured_joints)
            
            if ik_solution.success:
                cmd.joint_position = ik_solution.q.tolist()
            else:
                # Fallback to current joints if IK fails
                cmd.joint_position = msg.measured_joint_position.tolist()
        else:
            # The arm is in free-space. Shadow the hand perfectly so it feels weightless (Zero displacement).
            cmd.joint_position = msg.measured_joint_position.tolist()

        # 4. Command the spring equilibrium
        self._cmd_pub.publish(cmd)