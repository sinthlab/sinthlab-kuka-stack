#!/usr/bin/env python3
from __future__ import annotations

import copy
from typing import Callable, Optional
import numpy as np

from rclpy.node import Node as rclpyNode
from lbr_fri_idl.msg import LBRState
from geometry_msgs.msg import PoseStamped

# optas for fast kinematics
import optas

from sinthlab_bringup.helpers.common_threshold import get_required_param
from sinthlab_bringup.actions.trajectory_recorder import TrajectoryRecorder

class MoveRestrictedOnAPlaneAction:
    """
    Action that applies mathematical virtual fixtures (boundaries)
    by updating a target Pose based on physical pushes, mapping it to geometric 
    rails, and sending it to a real-time C++ CLIK controller.
    """
    def __init__(self, node: rclpyNode, *, param_prefix: str = "", on_complete: Optional[Callable[[], None]] = None) -> None:
        self._node = node
        self._on_complete = on_complete
        self._param_prefix = param_prefix + "." if param_prefix and not param_prefix.endswith(".") else param_prefix

        self._active = False

        state_topic = str(get_required_param(node, self._param_prefix + "state_topic"))
        
        # Override to point directly to the CLIK controller target frame topic
        robot_name = node.get_namespace().strip("/")
        cmd_topic = f"/{robot_name}/kuka_clik_controller/target_frame" if robot_name else "/kuka_clik_controller/target_frame"
        
        self.ee_link = str(get_required_param(node, self._param_prefix + "end_effector_link"))
        self.base_link = str(get_required_param(node, self._param_prefix + "base_link"))

        self._state_sub = node.create_subscription(LBRState, state_topic, self._state_cb, 1)
        self._cmd_pub = node.create_publisher(PoseStamped, cmd_topic, 1)

        node.get_logger().info("Initializing optas for rapid kinematics...")
        
        robot_description = ""
        if node.has_parameter("robot_description"):
            robot_description = str(node.get_parameter("robot_description").value)
        
        self.robot = optas.RobotModel(urdf_string=robot_description)
        self._fk_func = self.robot.get_link_transform_function(
            link=self.ee_link, base_link=self.base_link, numpy_output=True
        )
        
        self.last_measured_joints = np.zeros(self.robot.ndof)
        self.recorder = TrajectoryRecorder() # Setup modular recorder
        
        # Load virtual fixture profile
        self.active_profile = "bounding_box"
        if node.has_parameter(self._param_prefix + "virtual_fixture_profile"):
            self.active_profile = str(node.get_parameter(self._param_prefix + "virtual_fixture_profile").value)
            
        # Tuning parameters. The KUKA cabinet runs Cartesian impedance (LbrImpedanceControlServer);
        # this node streams the fixture-constrained EQUILIBRIUM (measured pose projected onto the
        # allowed manifold). The cabinet's 1 kHz spring provides the free-motion + soft-wall feel
        # and absorbs jerks.
        base_prefix = self._param_prefix + "virtual_fixtures."

        # Cap on how far the commanded equilibrium may move per state tick [m] — a jerk guard so a
        # sudden push/spike can't make the target leap (clik also clamps velocity downstream).
        self.max_target_step_m = 0.01
        if node.has_parameter(base_prefix + "max_target_step_m"):
            self.max_target_step_m = float(node.get_parameter(base_prefix + "max_target_step_m").value)
        self._published_xyz = None  # last commanded equilibrium translation (for the step clamp)
            
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
        self._needs_init_capture = True
        self._initial_transform = None
        self._published_xyz = None
        
        self.recorder.start() # Start modular recorder 
        
        # CRITICAL FIX: Wipe the old commanded position from the previous trial!
        # This forces the script to re-orient itself to the exact joint positions
        # the arm was moved to by the recovery script, preventing velocity faults!
        if hasattr(self, 'last_commanded'):
            delattr(self, 'last_commanded')
            
        self._node.get_logger().info("Restricted Plane Action started, applying boundary IK.")

    def stop(self) -> None:
        self._active = False
        
        # Save recorded trajectory automatically on stop using the modular recorder
        self.recorder.stop_and_save(self._node.get_logger())
            
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
        Runs at the hardware/state rate (~100-200 Hz).

        The KUKA cabinet runs Cartesian impedance (LbrImpedanceControlServer), so this node streams
        the fixture-constrained *equilibrium*: the measured EE pose projected onto the allowed
        manifold. Along allowed directions the equilibrium tracks the arm (free motion); off the
        manifold it stays on it, so the cabinet's 1 kHz spring softly pulls the arm back (a soft
        virtual wall) and absorbs sudden jerks. The result is step-clamped and published to CLIK.
        """
        if not self._active:
            return

        self.last_measured_joints = np.array(msg.measured_joint_position)

        # Initialize the commanded anchor (used to lock the manifold origin to the start pose)
        if not hasattr(self, 'last_commanded'):
            q_cmd = np.array(msg.commanded_joint_position)
            if np.isnan(q_cmd).any() or np.all(q_cmd == 0):
                self.last_commanded = self.last_measured_joints.copy()
            else:
                self.last_commanded = q_cmd

        # On the first tick, anchor the fixture manifold at the start pose
        if getattr(self, '_needs_init_capture', True):
            self._needs_init_capture = False
            self._initial_transform = self._fk_func(self.last_commanded)
            self._published_xyz = self._initial_transform[0:3, 3].copy()
            self._node.get_logger().info("Fixture anchored at start pose.")

        target_pose = self._compute_cabinet_target(msg)

        # Jerk guard: bound how far the commanded equilibrium may jump in one tick
        xyz = target_pose[0:3, 3].copy()
        if self._published_xyz is not None and self.max_target_step_m > 0.0:
            delta = xyz - self._published_xyz
            dist = float(np.linalg.norm(delta))
            if dist > self.max_target_step_m:
                xyz = self._published_xyz + delta * (self.max_target_step_m / dist)
        self._published_xyz = xyz
        target_pose[0:3, 3] = xyz

        self._target_pose = target_pose
        self._publish_pose(target_pose)

        # Record the real (measured) Cartesian trajectory at the state rate
        self.recorder.record_pose(self._fk_func(self.last_measured_joints))

    def _compute_cabinet_target(self, msg: LBRState) -> np.ndarray:
        """
        Project the *measured* EE pose onto the fixture manifold and command it as the cabinet
        impedance equilibrium. Orientation is held at the start orientation via the cabinet's
        rotational stiffness.
        """
        measured_pose = self._fk_func(self.last_measured_joints)
        constrained_pose, _is_restricted = self.apply_surface_constraints(measured_pose)
        constrained_pose[0:3, 0:3] = self._initial_transform[0:3, 0:3]
        return constrained_pose

    def _publish_pose(self, target_pose: np.ndarray) -> None:
        from scipy.spatial.transform import Rotation as R
        cmd = PoseStamped()
        cmd.header.frame_id = self.base_link
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.pose.position.x = float(target_pose[0, 3])
        cmd.pose.position.y = float(target_pose[1, 3])
        cmd.pose.position.z = float(target_pose[2, 3])
        quat = R.from_matrix(target_pose[0:3, 0:3]).as_quat()
        cmd.pose.orientation.x = float(quat[0])
        cmd.pose.orientation.y = float(quat[1])
        cmd.pose.orientation.z = float(quat[2])
        cmd.pose.orientation.w = float(quat[3])
        self._cmd_pub.publish(cmd)