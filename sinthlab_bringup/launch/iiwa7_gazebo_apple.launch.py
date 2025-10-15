from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Args consistent with upstream
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    # Optional apple params
    apple_radius = LaunchConfiguration("apple_radius", default="0.04")
    apple_mass = LaunchConfiguration("apple_mass", default="0.15")
    adapter_length = LaunchConfiguration("adapter_length", default="0.1")

    # robot_description from our iiwa7 + apple xacro in Gazebo mode
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sinthlab_description"),
                            "urdf",
                            "iiwa7_apple",
                            "iiwa7_apple.xacro",
                        ]
                    ),
                    " robot_name:=",
                    LaunchConfiguration("robot_name", default="lbr"),
                    " mode:=gazebo",
                    " apple_radius:=",
                    apple_radius,
                    " apple_mass:=",
                    apple_mass,
                    " adapter_length:=",
                    adapter_length,
                ]
            ),
            value_type=str,
        )
    }

    # RSP
    ld.add_action(LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=True
    ))

    # Gazebo
    ld.add_action(GazeboMixin.include_gazebo())
    ld.add_action(GazeboMixin.node_clock_bridge())
    ld.add_action(GazeboMixin.node_create())

    # Controllers
    ld.add_action(LBRROS2ControlMixin.node_controller_spawner(controller="joint_state_broadcaster"))
    ld.add_action(LBRROS2ControlMixin.node_controller_spawner(controller=LaunchConfiguration("ctrl")))

    return ld
