from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution


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

    # Xacro evaluation for robot_description (auto-inject) — require robot_type to be specified
    robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="iiwa7",
        description="Robot variant passed to lbr_description xacro (e.g., iiwa7, iiwa14)",
    )
    xacro_path = PathJoinSubstitution([
        FindPackageShare("lbr_description"),
        "urdf",
        "lbr.urdf.xacro",
    ])
    robot_description_content = Command([
        TextSubstitution(text="xacro "),
        xacro_path,
        TextSubstitution(text=" "),
        TextSubstitution(text="robot_type:="),
        LaunchConfiguration("robot_type"),
    ])

    node = Node(
        package="sinthlab_bringup",
        executable="apple_pluck_impedance_control.py",
        name="apple_pluck_impedance_control",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"robot_description": robot_description_content},
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
