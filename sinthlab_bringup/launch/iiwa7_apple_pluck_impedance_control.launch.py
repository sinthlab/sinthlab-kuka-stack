from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin


def generate_launch_description():
    # Parameters via YAML; default to installed config/apple_pluck_impedance.yaml
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("sinthlab_bringup"),
            "config",
            "apple_pluck_impedance.yaml",
        ]),
        description="Path to YAML with parameters for apple_pluck_impedance_control",
    )

    # Robot description from mixin (xacro evaluated at launch)
    robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="iiwa7",
        description="Robot variant passed to lbr_description xacro (e.g., iiwa7, iiwa14)",
    )
    robot_description = LBRDescriptionMixin.param_robot_description(
        model=LaunchConfiguration("robot_type")
    )

    node = Node(
        package="sinthlab_bringup",
        executable="apple_pluck_impedance_control.py",
        name="apple_pluck_impedance_control",
        namespace="lbr",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
            # All other parameters are sourced from the YAML (or node defaults)
        ],
    )

    return LaunchDescription(
        [
            params_file,
            robot_type,
            node,
        ]
    )
