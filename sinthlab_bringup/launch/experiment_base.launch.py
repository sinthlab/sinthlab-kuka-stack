"""Shared base launch for all sinthlab experiments.

Brings up the KUKA hardware (FRI position mode + controllers via custom_hardware.launch.py) and
starts the experiment's orchestrator node. The three experiment launches are thin wrappers that
include this and pass three arguments:

  * params_file  - the experiment parameter YAML (loaded onto the orchestrator)
  * orchestrator - the orchestrator executable (e.g. apple_pluck_orchestrator.py)
  * ctrl         - the controller spawned via FRI (joint-position for apple/perturb, clik for restricted)

Not meant to be run directly (params_file and orchestrator are required).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lbr_bringup.description import LBRDescriptionMixin


def generate_launch_description():
    # --- experiment-specific (required; supplied by each experiment launch) ---
    params_file = DeclareLaunchArgument(
        "params_file",
        description="Experiment parameter YAML loaded onto the orchestrator.",
    )
    orchestrator = DeclareLaunchArgument(
        "orchestrator",
        description="Orchestrator executable to run (e.g. apple_pluck_orchestrator.py).",
    )

    # --- common hardware/robot args (defaults; override on the CLI if needed) ---
    robot_type = DeclareLaunchArgument("robot_type", default_value="iiwa7")
    robot_name = LBRDescriptionMixin.arg_robot_name()
    ctrl = DeclareLaunchArgument(
        "ctrl",
        default_value="lbr_joint_position_command_controller",
        description="Default controller spawned via FRI for hardware commands.",
    )

    robot_description = LBRDescriptionMixin.param_robot_description(
        model=LaunchConfiguration("robot_type"),
        robot_name=LaunchConfiguration("robot_name"),
        mode="hardware",
    )

    # Bring up the KUKA hardware (FRI position mode + controllers) — shared by all experiments.
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sinthlab_bringup"), "launch", "custom_hardware.launch.py"]
            )
        ),
        launch_arguments={
            "model": LaunchConfiguration("robot_type"),
            "robot_name": LaunchConfiguration("robot_name"),
            "ctrl": LaunchConfiguration("ctrl"),
        }.items(),
    )

    # The experiment's orchestrator (a state machine composed of actions). Its node name comes from
    # the orchestrator code; params are matched via the YAML's /** wildcard regardless of name.
    orchestrator_node = Node(
        package="sinthlab_bringup",
        executable=LaunchConfiguration("orchestrator"),
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    return LaunchDescription(
        [
            params_file,
            orchestrator,
            robot_type,
            robot_name,
            ctrl,
            hardware_launch,
            orchestrator_node,
        ]
    )
