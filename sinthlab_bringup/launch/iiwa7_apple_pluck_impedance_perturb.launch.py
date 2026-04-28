from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("sinthlab_bringup"),
            "config",
            "apple_pluck_impedance_perturb.yaml",
        ]),
        description="Path to YAML with parameters for perturbation experiment",
    )

    robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="iiwa7",
        description="Robot variant passed to lbr_description xacro (e.g., iiwa7, iiwa14)",
    )

    robot_name = LBRDescriptionMixin.arg_robot_name()
    ctrl = LBRROS2ControlMixin.arg_ctrl()

    robot_description = LBRDescriptionMixin.param_robot_description(
        model=LaunchConfiguration("robot_type"),
        robot_name=LaunchConfiguration("robot_name"),
        mode="hardware",
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("lbr_bringup"), "launch", "hardware.launch.py"]
            )
        ),
        launch_arguments={
            "model": LaunchConfiguration("robot_type"),
            "robot_name": LaunchConfiguration("robot_name"),
            "ctrl": LaunchConfiguration("ctrl"),
        }.items(),
    )

    move_to_start_node = Node(
        package="sinthlab_bringup",
        executable="move_to_start.py",
        name="move_to_start",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    force_torque_bias_calibrator_node = Node(
        package="sinthlab_bringup",
        executable="force_torque_bias_calibrator.py",
        name="force_torque_bias_calibrator",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    audio_cue_play_node = Node(
        package="sinthlab_bringup",
        executable="audio_cue_play.py",
        name="audio_cue_play",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
        ],
    )

    perturb_start_node = Node(
        package="sinthlab_bringup",
        executable="perturb_start.py",
        name="perturb_start",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    impedance_displacement_node = Node(
        package="sinthlab_bringup",
        executable="apple_pluck_impedance_control_displacement.py",
        name="apple_pluck_impedance_control_displacement",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    move_to_start_recover_node = Node(
        package="sinthlab_bringup",
        executable="move_to_start.py",
        name="move_to_start_recover",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    shutdown_after_recover = RegisterEventHandler(
        OnProcessExit(
            target_action=move_to_start_recover_node,
            on_exit=[
                LogInfo(msg="move_to_start recovery complete; shutting down perturbation launch."),
                EmitEvent(event=Shutdown(reason="move_to_start recovery complete")),
            ],
        )
    )

    status_message = LogInfo(
        msg="Perturbation launch active. Robot will nudge start pose after baseline beep."
    )

    return LaunchDescription(
        [
            params_file,
            robot_type,
            robot_name,
            ctrl,
            hardware_launch,
            move_to_start_node,
            force_torque_bias_calibrator_node,
            audio_cue_play_node,
            perturb_start_node,
            impedance_displacement_node,
            move_to_start_recover_node,
            shutdown_after_recover,
            status_message,
        ]
    )
