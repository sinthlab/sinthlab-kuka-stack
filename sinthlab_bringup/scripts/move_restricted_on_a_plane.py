#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import copy
from lbr_fri_idl.msg import LBRState, LBRJointPositionCommand

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from helpers.common_threshold import get_required_param

class VirtualFixtureNode(Node):
    def __init__(self):
        super().__init__('virtual_fixture_node', automatically_declare_parameters_from_overrides=True)

        # ---------------------------------------------------------
        # 1. SETUP KUKA FRI PUBLISHERS/SUBSCRIBERS
        # ---------------------------------------------------------
        state_topic = get_required_param(self, "state_topic").value
        cmd_topic = get_required_param(self, "command_topic").value
        self.ee_link = get_required_param(self, "end_effector_link").value
        self.base_link = get_required_param(self, "base_link").value

        self._state_sub = self.create_subscription(LBRState, state_topic, self._state_cb, 1)
        self._cmd_pub = self.create_publisher(LBRJointPositionCommand, cmd_topic, 1)

        # ---------------------------------------------------------
        # 2. SETUP KINEMATICS (moveit_py)
        # ---------------------------------------------------------
        self.get_logger().info("Initializing MoveIt Python API for rapid kinematics...")
        self.moveit_py = MoveItPy(node_name=self.get_name())
        self.robot_model = self.moveit_py.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        self.last_measured_joints = np.zeros(7)
        self.get_logger().info("Virtual Fixture Node initialized and active.")

    # ---------------------------------------------------------
    # 3. MODULAR MATHEMATICAL CONSTRAINTS
    # ---------------------------------------------------------
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

        # ---- CONSTRAINT 3: Angled incline plane (Z >= 0.5*X + 0.1) ----
        # Uncomment to activate!
        # limit_z = 0.5 * x + 0.1
        # if z < limit_z:
        #     z = limit_z
        #     restricted = True

        # Re-pack the XYZ back into the transformation matrix
        transform[0, 3] = x
        transform[1, 3] = y
        transform[2, 3] = z

        return transform, restricted


    # ---------------------------------------------------------
    # 4. REAL-TIME CONTROL LOOP CALLBACK
    # ---------------------------------------------------------
    def _state_cb(self, msg: LBRState):
        """
        Runs continuously at the hardware frequency (~100-200Hz).
        Reads measured state -> Forward Kinematics -> Constraint Check -> Inverse Kinematics -> Command.
        """
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


def main(args=None):
    rclpy.init(args=args)
    node = VirtualFixtureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()