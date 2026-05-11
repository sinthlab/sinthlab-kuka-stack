#!/usr/bin/env python3
from __future__ import annotations

import copy
from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand

# Optas for fast Kinematics and Jacobian extraction
import optas

from sinthlab_bringup.helpers.common_threshold import get_required_param

class MoveRestrictedOnAPlaneAction:
    """
    Action that applies mathematical virtual fixtures (boundaries)
    by overriding the requested Cartesian pose via rapid IK through Optas.
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

        node.get_logger().info("Initializing Optas for rapid kinematics...")
        
        robot_description = ""
        if node.has_parameter("robot_description"):
            robot_description = str(node.get_parameter("robot_description").value)

        self.robot_model = optas.RobotModel(urdf_string=robot_description)
        
        # Get callable Casadi functions mapping joint angles (numpy array) to 4x4 transform and Jacobian
        self.fk_func = self.robot_model.get_global_link_transform_function(
            self.ee_link, numpy_output=True
        )
        self.jacobian_func = self.robot_model.get_link_geometric_jacobian_function(
            link=self.ee_link, base_link=self.base_link, numpy_output=True
        )

        self.last_measured_joints = np.zeros(self.robot_model.ndof)
        
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
        current_pose_flat = self.fk_func(self.last_measured_joints)
        # CasADi matrices are column-major. We must use order='F' to reshape to 4x4 correctly!
        current_pose = current_pose_flat.reshape((4, 4), order='F')

        # We must clone the pose so the constraint application doesn't overwrite current_pose in memory
        target_pose_input = copy.deepcopy(current_pose)
        
        # 2. Check and Apply our Mathematical Surface boundaries
        target_pose, is_restricted = self.apply_surface_constraints(target_pose_input)

        cmd = LBRJointPositionCommand()
        
        if is_restricted:
            # 3. Inverse Kinematics: The arm crossed the wall. 
            # We use first-order Jacobian pseudo-inverse to track positional changes quickly.
            
            # Cartesian error vector (dx, dy, dz)
            error_x = target_pose[0, 3] - current_pose[0, 3]
            error_y = target_pose[1, 3] - current_pose[1, 3]
            error_z = target_pose[2, 3] - current_pose[2, 3]
            
            # Build 6D twist/error vector (dx, dy, dz, rx=0, ry=0, rz=0)
            dx = np.array([error_x, error_y, error_z, 0.0, 0.0, 0.0])
            
            # Get Jacobian (Optas returns 6x7 matrix for 7 DOF arm)
            J = self.jacobian_func(self.last_measured_joints)
            
            # Calculate required joint delta (dq) via pseudo-inverse
            # J_pinv @ dx converts the 6D cartesian push back into a 7D joint push
            J_pinv = np.linalg.pinv(J, rcond=1e-2)
            dq = J_pinv @ dx
            
            # Since this is a velocity twist applied over dt, and we are acting as a rigid spring,
            # we just push the commanded joint position exactly dq away from the measured to clamp it to the wall.
            cmd.joint_position = (self.last_measured_joints + dq).tolist()
        else:
            # The arm is in free-space. Shadow the hand perfectly so it feels weightless (Zero displacement).
            cmd.joint_position = msg.measured_joint_position.tolist()

        # 4. Command the spring equilibrium
        self._cmd_pub.publish(cmd)