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
    ctrl = DeclareLaunchArgument(
        "ctrl",
        default_value="kuka_clik_controller",
        description="Desired default controller for hardware commands via FRI.",
    )

    robot_description = LBRDescriptionMixin.param_robot_description(
        model=LaunchConfiguration("robot_type"),
        robot_name=LaunchConfiguration("robot_name"),
        mode="hardware",
    )

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

    orchestrator_node = Node(
        package="sinthlab_bringup",
        executable="perturb_orchestrator.py",
        name="perturb_orchestrator",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    status_message = LogInfo(
        msg="Perturbation logic started via multi-trial Orchestrator."
    )

    return LaunchDescription(
        [
            params_file,
            robot_type,
            robot_name,
            ctrl,
            hardware_launch,
            orchestrator_node,
            status_message,
        ]
    )
