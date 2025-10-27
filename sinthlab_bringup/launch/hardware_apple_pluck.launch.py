from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.description import LBRDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    # Optional pass-through arguments for the apple pluck launch
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("sinthlab_bringup"),
            "config",
            "apple_pluck_impedance.yaml",
        ]),
        description="Path to YAML with parameters for apple_pluck_impedance_control",
    )

    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="iiwa7",
        description="Robot variant passed to lbr_description xacro (e.g., iiwa7, iiwa14)",
    )

    # Robot description for the apple pluck nodes
    robot_description = LBRDescriptionMixin.param_robot_description(
        model=LaunchConfiguration("robot_type")
    )

    # Include the hardware bringup (ros2_control + controllers)
    include_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lbr_bringup"),
                "launch",
                "hardware.launch.py",
            ])
        )
        # If you need to override hardware.launch.py args, add launch_arguments here
    )

    # Define the two apple pluck nodes directly so we can hook a shutdown on the monitor exit
    move_to_start_node = Node(
        package="sinthlab_bringup",
        executable="move_to_start.py",
        name="move_to_start",
        namespace="lbr",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    monitor_node = Node(
        package="sinthlab_bringup",
        executable="apple_pluck_impedance_control.py",
        name="apple_pluck_impedance_control",
        namespace="lbr",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    # When the monitor exits (threshold hit), shut down everything (including hardware include)
    shutdown_on_monitor_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor_node,
            on_exit=[EmitEvent(event=Shutdown(reason="apple_pluck_monitor_exited"))],
        )
    )

    return LaunchDescription(
        [
            params_file_arg,
            robot_type_arg,
            include_hardware,
            move_to_start_node,
            monitor_node,
            shutdown_on_monitor_exit,
        ]
    )
