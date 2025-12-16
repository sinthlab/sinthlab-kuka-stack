from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("sinthlab_bringup"),
            "config",
            "admittance_tuning.yaml",
        ]),
        description="Parameter YAML for the admittance controller",
    )

    gui_enabled = DeclareLaunchArgument(
        "gui_enabled",
        default_value="true",
        description="Launch the admittance gains GUI.",
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

    admittance_control_node = Node(
        package="sinthlab_bringup",
        executable="admittance_tuning.py",
        name="admittance_controller",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    admittance_gui_node = Node(
        package="sinthlab_bringup",
        executable="admittance_gains_gui.py",
        name="admittance_gains_gui",
        output="screen",
        arguments=[
            "--namespace",
            LaunchConfiguration("robot_name"),
        ],
        condition=IfCondition(LaunchConfiguration("gui_enabled")),
    )

    status_message = LogInfo(
        msg="Admittance controller launched. Use the GUI to tune dq/dx gains in real time."
    )

    return LaunchDescription(
        [
            params_file,
            gui_enabled,
            robot_type,
            robot_name,
            ctrl,
            hardware_launch,
            admittance_control_node,
            admittance_gui_node,
            status_message,
        ]
    )
