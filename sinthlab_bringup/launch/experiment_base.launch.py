"""Shared base launch for all sinthlab experiments.

Brings up the KUKA hardware (FRI position mode + controllers via iiwa7_hardware.launch.py) and
starts the experiment's orchestrator node. The three experiment launches are thin wrappers that
include this and pass three arguments:

  * params_file  - the experiment parameter YAML (loaded onto the orchestrator)
  * orchestrator - the orchestrator executable (e.g. apple_pluck_orchestrator.py)
  * ctrl         - the controller spawned via FRI (joint-position for apple/perturb, clik for restricted)

Not meant to be run directly (params_file and orchestrator are required).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    # Seconds to wait before starting the orchestrator, so the hardware + controller are fully
    # ACTIVE before the first move streams setpoints. The CLIK (restricted-plane) controller resets
    # its target to the current pose on activation, so an orchestrator that starts streaming before
    # activation races that reset; this delay avoids it. Joint-position experiments can leave it 0.
    startup_delay = DeclareLaunchArgument(
        "startup_delay",
        default_value="0.0",
        description="Seconds to delay the orchestrator so controllers are active first.",
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
                [FindPackageShare("sinthlab_bringup"), "launch", "iiwa7_hardware.launch.py"]
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
    # Hold the orchestrator back by `startup_delay` s so the controller is active before it streams.
    delayed_orchestrator = TimerAction(
        period=LaunchConfiguration("startup_delay"),
        actions=[orchestrator_node],
    )

    return LaunchDescription(
        [
            params_file,
            orchestrator,
            robot_type,
            robot_name,
            ctrl,
            startup_delay,
            hardware_launch,
            delayed_orchestrator,
        ]
    )
