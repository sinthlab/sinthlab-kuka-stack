#!/usr/bin/env python3
from __future__ import annotations

import copy
from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand

# optas for fast kinematics
import optas

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

        node.get_logger().info("Initializing optas for rapid kinematics...")
        
        robot_description = ""
        if node.has_parameter("robot_description"):
            robot_description = str(node.get_parameter("robot_description").value)
        
        self.robot = optas.RobotModel(urdf_string=robot_description)
        self._fk_func = self.robot.get_global_link_transform_function(
            link=self.ee_link, base_link=self.base_link, numpy_output=True
        )
        self._jacobian_func = self.robot.get_link_geometric_jacobian_function(
            link=self.ee_link, base_link=self.base_link, numpy_output=True
        )
        
        self.last_measured_joints = np.zeros(self.robot.ndof)
        
        # Load virtual fixture profile
        self.active_profile = "bounding_box"
        if node.has_parameter(self._param_prefix + "virtual_fixture_profile"):
            self.active_profile = str(node.get_parameter(self._param_prefix + "virtual_fixture_profile").value)
            
        # Load High-Level Tuning Parameters
        base_prefix = self._param_prefix + "virtual_fixtures."
        self.force_deadband = float(get_required_param(node, base_prefix + "force_deadband"))
        self.admittance_gain = float(get_required_param(node, base_prefix + "admittance_gain"))
            
        self.profile_config = {}
        prefix = base_prefix + f"{self.active_profile}."
        if node.has_parameter(prefix + "type"):
            self.profile_config["type"] = str(node.get_parameter(prefix + "type").value)
            for key in ["z_min", "x_min", "x_max", "radius", "center_x", "center_y", "amplitude", "spatial_freq", "y_offset"]:
                if node.has_parameter(prefix + key):
                    self.profile_config[key] = float(node.get_parameter(prefix + key).value)
            for key in ["pull_axis", "osc_axis", "restricted_axis"]:
                if node.has_parameter(prefix + key):
                    self.profile_config[key] = str(node.get_parameter(prefix + key).value)
        else:
            node.get_logger().warn(f"Could not find virtual fixture profile {self.active_profile}")
        
    def start(self) -> None:
        if self._active:
            self._node.get_logger().warn("MoveRestrictedOnAPlaneAction is already active.")
            return
        self._active = True
        self._needs_bias_capture = True
        self._initial_transform = None
        
        # CRITICAL FIX: Wipe the old commanded position from the previous trial!
        # This forces the script to re-orient itself to the exact joint positions
        # the arm was moved to by the recovery script, preventing velocity faults!
        if hasattr(self, 'last_commanded'):
            delattr(self, 'last_commanded')
            
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
        elif ptype == "plane":
            # Locks motion purely to the 2D plane based on the restricted axis.
            restricted = True
            restricted_axis = self.profile_config.get("restricted_axis", "z").lower()
            
            if self._initial_transform is not None:
                if restricted_axis == "z":
                    z = self._initial_transform[2, 3] # Lock Z, free X/Y
                elif restricted_axis == "y":
                    y = self._initial_transform[1, 3] # Lock Y, free X/Z
                elif restricted_axis == "x":
                    x = self._initial_transform[0, 3] # Lock X, free Y/Z
                
                # Lock orientation completely
                transform[0:3, 0:3] = self._initial_transform[0:3, 0:3]
                
        elif ptype == "sinusoid":
            amp = self.profile_config.get("amplitude")
            freq = self.profile_config.get("spatial_freq")
            pull_axis = self.profile_config.get("pull_axis") # e.g. 'z' = pull down
            osc_axis = self.profile_config.get("osc_axis")   # e.g. 'x' = wobbles left/right
            
            # For a 1D Rail, we are ALWAYS restricting the geometry to perfectly
            # snap onto the mathematical manifold (no thresholding)
            restricted = True
            
            if self._initial_transform is not None:
                start_x = self._initial_transform[0, 3]
                start_y = self._initial_transform[1, 3]
                start_z = self._initial_transform[2, 3]
                
                # Apply the sine wave dynamically based on configured axes
                if pull_axis == "z" and osc_axis == "x":
                    y = start_y
                    x = start_x + amp * np.sin(freq * (z - start_z))
                elif pull_axis == "z" and osc_axis == "y":
                    x = start_x
                    y = start_y + amp * np.sin(freq * (z - start_z))
                elif pull_axis == "x" and osc_axis == "y":
                    z = start_z
                    y = start_y + amp * np.sin(freq * (x - start_x))
                elif pull_axis == "x" and osc_axis == "z":
                    y = start_y
                    z = start_z + amp * np.sin(freq * (x - start_x))
                elif pull_axis == "y" and osc_axis == "x":
                    z = start_z
                    x = start_x + amp * np.sin(freq * (y - start_y))
                elif pull_axis == "y" and osc_axis == "z":
                    x = start_x
                    z = start_z + amp * np.sin(freq * (y - start_y))
                else:
                    # Legacy fallback
                    z = start_z
                    y = amp * np.sin(freq * x) + self.profile_config.get("y_offset", 0.0)

                # Lock orientation completely so the end effector doesn't twist
                transform[0:3, 0:3] = self._initial_transform[0:3, 0:3]

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

        # Initialize our smooth command tracker if it doesn't exist yet
        if not hasattr(self, 'last_commanded'):
            q_cmd = np.array(msg.commanded_joint_position)
            if np.isnan(q_cmd).any() or np.all(q_cmd == 0):
                self.last_commanded = self.last_measured_joints.copy()
            else:
                self.last_commanded = q_cmd

        # Capture gravity / imperfect modeling torque bias on the first tick
        if getattr(self, '_needs_bias_capture', True):
            self.torque_bias = np.array(msg.external_torque)
            self._needs_bias_capture = False
            self._initial_transform = self._fk_func(self.last_commanded)
            self._node.get_logger().info(f"Captured gravity bias & initial pose lock: {np.round(self.torque_bias, 2)}")

        # ---------------------------------------------------------------------
        # ADMITTANCE FIX: Do not track `measured_joints` directly to avoid gravity droop!
        # Instead, only shift the commanded target if the user physically pushes with force.
        # ---------------------------------------------------------------------
        # Subtract the static bias so it doesn't perpetually trigger the deadband
        tau_ext = np.array(msg.external_torque) - getattr(self, 'torque_bias', np.zeros(7))
        
        # ---------------------------------------------------------------------
        # PURE CARTESIAN ADMITTANCE
        # Instead of moving joints proportionally to joint torques (which causes
        # weird sweeping arcs and wobbles), convert the forces to Cartesian space!
        # ---------------------------------------------------------------------
        
        # 1. Get the spatial Jacobian (Base frame) to map physics to Cartesian
        J = self._jacobian_func(self.last_commanded)
        
        # 2. Convert joint torques into 6D Cartesian Wrench [Fx, Fy, Fz, Tx, Ty, Tz]
        # wrench = (J^T)^+ * tau (Moore-Penrose pseudo-inverse)
        wrench = np.linalg.pinv(J.T) @ tau_ext
        
        # 3. Apply a deadband on the calculated physical Push Forces (Newtons)
        f_deadband = self.force_deadband
        wrench_active = np.where(np.abs(wrench) > f_deadband, np.sign(wrench) * (np.abs(wrench) - f_deadband), 0.0)
        
        # Prevent runaway leaps if pushed violently
        wrench_active = np.clip(wrench_active, -80.0, 80.0) 
        
        # 4. Apply pure linear Cartesian Admittance based on the hand push
        cartesian_gain_linear = self.admittance_gain
        
        current_pose = self._fk_func(self.last_commanded)
        target_pose_input = current_pose.copy()
        
        # Apply the linear force translations
        target_pose_input[0, 3] += wrench_active[0] * cartesian_gain_linear
        target_pose_input[1, 3] += wrench_active[1] * cartesian_gain_linear
        target_pose_input[2, 3] += wrench_active[2] * cartesian_gain_linear

        # 5. Snap the intended linear push to our Mathematical Rails (Sine Wave)
        target_pose, is_restricted = self.apply_surface_constraints(target_pose_input)

        cmd = LBRJointPositionCommand()
        target_joints = self.last_commanded.copy()
        
        # 6. FAST JACOBIAN IK (runs in < 1ms, keeps 200Hz loop alive!)
        # Instead of calling ikine_LM (which is an iterative solver that crashes
        # the 200Hz loop and causes the robot to violently jerk/wobble),
        # we calculate the required spatial twist and push it through the Jacobian!
        if np.any(np.abs(wrench_active) > 0.01) or is_restricted:
            # How far do we need to move?
            dx = target_pose[0, 3] - current_pose[0, 3]
            dy = target_pose[1, 3] - current_pose[1, 3]
            dz = target_pose[2, 3] - current_pose[2, 3]
            
            # 6D Twist vector [dx, dy, dz, dRx, dRy, dRz]
            # (We only enforce translation rigidly here to prevent math singularities
            # in rotation math during pure admittance)
            twist = np.array([dx, dy, dz, 0.0, 0.0, 0.0])
            
            # Avoid massive leaps that cause structural singularities
            twist = np.clip(twist, -0.05, 0.05) 
            
            J_inv = np.linalg.pinv(J, rcond=0.01) # Stiff singularity handling
            delta_q = J_inv @ twist
            
            target_joints += delta_q

        # 7. Heavy EMA filter to guarantee buttery-smooth joint signals
        alpha = 0.5 # 0.5 matches the official optas admittance demo (1-0.95 = 0.05, etc)
        safe_q = (1.0 - alpha) * self.last_commanded + (alpha * target_joints)
        
        self.last_commanded = safe_q
        cmd.joint_position = safe_q.tolist()

        # 5. Command the spring equilibrium
        self._cmd_pub.publish(cmd)