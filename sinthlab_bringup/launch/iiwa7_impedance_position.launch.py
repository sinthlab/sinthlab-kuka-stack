from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("sinthlab_bringup"), "config", "apple_pluck_impedance.yaml"]
        ),
        description="Path to YAML file with impedance-position parameters.",
    )

    robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="iiwa7",
        description="Robot variant passed to the xacro (e.g., iiwa7, iiwa14).",
    )

    robot_description = LBRDescriptionMixin.param_robot_description(
        model=LaunchConfiguration("robot_type")
    )

    impedance_position_node = Node(
        package="sinthlab_bringup",
        executable="impedance_position_control.py",
        name="impedance_position_control",
        namespace="lbr",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    return LaunchDescription([
        params_file,
        robot_type,
        impedance_position_node,
    ])
