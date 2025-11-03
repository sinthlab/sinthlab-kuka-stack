from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    # Stop threshold criteria selection.
    threshold_condition = DeclareLaunchArgument(
        "threshold_condition",
        default_value="force",
        description="Stop criterion: 'force' (EE Z force) or 'displacement' (Cartesian EE displacement)",
        choices=["force", "displacement"],
    )

    # First: run move_to_start node. It will exit once the target is reached.
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

    # Second: apple_pluck_impedance_control node (force mode). It will internally wait for the
    # move_to_start '/move_to_start/done' topic before starting to monitor force.
    impedance_force_node = Node(
        package="sinthlab_bringup",
        executable="apple_pluck_impedance_control_force.py",
        name="apple_pluck_impedance_control_force",
        namespace="lbr",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            robot_description,
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("threshold_condition"), "' == 'force'"
        ])),
    )

    # Alternative: displacement stop criterion node
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
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("threshold_condition"), "' == 'displacement'"
        ])),
    )
    
    start_displacement_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=move_to_start_node,
            on_exit=[
                GroupAction(
                    actions=[
                        LogInfo(msg="move_to_start complete; launching displacement monitor."),
                        impedance_displacement_node,
                    ],
                    condition=IfCondition(PythonExpression([
                        "'", LaunchConfiguration("threshold_condition"), "' == 'displacement'"
                    ])),
                )
            ],
        )
    )

    move_to_start_recover_node = Node(
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

    restart_move_to_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=impedance_displacement_node,
            on_exit=[
                LogInfo(msg="Displacement monitoring finished; relaunching move_to_start."),
                move_to_start_recover_node,
            ],
        )
    )

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
            move_to_start_node,
            threshold_condition,
            impedance_force_node,
            # impedance_displacement_node,
            start_displacement_handler,
            restart_move_to_start_handler,
            shutdown_after_recover,
        ]
    )
