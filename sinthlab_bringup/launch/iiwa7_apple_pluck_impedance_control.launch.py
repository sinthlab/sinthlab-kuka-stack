from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


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

    stiffness_scale = DeclareLaunchArgument(
        "stiffness_scale",
        default_value="1.0",
        description="Scalar applied to admittance gains ( (0,1] stiffer to softer )",
    )
    robot_description = LBRDescriptionMixin.param_robot_description(
        model=LaunchConfiguration("robot_type"),
        mode="hardware"
    )
    
    # Added robot_state_publisher to the launch so all nodes read the same pre-expanded robot_description
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description,
        use_sim_time=False,
    )


    # Move the arm to the predefined start position.
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

    # Start admittance control (software impedance while the robot stays in FRI position mode).
    admittance_control_node = Node(
        package="sinthlab_bringup",
        executable="admittance_control_mode.py",
        name="admittance_controller",
        namespace="lbr",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
            {"stiffness_scale": LaunchConfiguration("stiffness_scale")},
        ],
    )

    # Monitor displacement and force-release while admittance control keeps running.
    impedance_displacement_node = Node(
        package="sinthlab_bringup",
        executable="apple_pluck_impedance_control_displacement.py",
        name="apple_pluck_impedance_control_displacement",
        namespace="lbr",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    # Recover to start once the displacement monitor signals force release.
    move_to_start_recover_node = Node(
        package="sinthlab_bringup",
        executable="move_to_start.py",
        name="move_to_start_recover",
        namespace="lbr",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
    )

    # Shutdown launch once recovery move_to_start completes
    shutdown_after_recover = RegisterEventHandler(
        OnProcessExit(
            target_action=move_to_start_recover_node,
            on_exit=[
                LogInfo(msg="move_to_start recovery complete; shutting down launch."),
                EmitEvent(event=Shutdown(reason="move_to_start recovery complete")),
            ],
        )
    )

    return LaunchDescription(
        [
            params_file,
            robot_type,
            robot_state_publisher,
            stiffness_scale,
            move_to_start_node,
            admittance_control_node,
            impedance_displacement_node,
            move_to_start_recover_node,
            shutdown_after_recover,
        ]
    )