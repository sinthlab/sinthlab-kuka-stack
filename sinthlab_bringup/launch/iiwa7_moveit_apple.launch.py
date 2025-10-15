from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.moveit import LBRMoveGroupMixin
from lbr_bringup.rviz import RVizMixin


def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    # Args
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    ld.add_action(RVizMixin.arg_rviz())
    # Optional apple params
    apple_radius = LaunchConfiguration("apple_radius", default="0.04")
    apple_mass = LaunchConfiguration("apple_mass", default="0.15")
    adapter_length = LaunchConfiguration("adapter_length", default="0.1")

    # Build robot_description from our iiwa7 + apple xacro
    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = True if mode == "gazebo" else False

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sinthlab_description"),
                            "urdf",
                            "iiwa7_apple",
                            "iiwa7_apple.xacro",
                        ]
                    ),
                    " robot_name:=",
                    LaunchConfiguration("robot_name", default="lbr"),
                    " mode:=",
                    LaunchConfiguration("mode", default="mock"),
                    " apple_radius:=",
                    apple_radius,
                    " apple_mass:=",
                    apple_mass,
                    " adapter_length:=",
                    adapter_length,
                ]
            ),
            value_type=str,
        )
    }

    # robot_state_publisher
    ld.add_action(
        LBRROS2ControlMixin.node_robot_state_publisher(
            robot_description=robot_description, use_sim_time=use_sim_time
        )
    )

    # Start Gazebo early
    if mode == "gazebo":
        ld.add_action(GazeboMixin.include_gazebo())
        ld.add_action(GazeboMixin.node_clock_bridge())
        ld.add_action(GazeboMixin.node_create())

    # If mock or gazebo, start control and controllers
    if mode in ["mock", "gazebo"]:
        ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
            use_sim_time=use_sim_time, robot_description=robot_description
        )
        ld.add_action(ros2_control_node)

        js_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
            controller="joint_state_broadcaster"
        )
        controller = LBRROS2ControlMixin.node_controller_spawner(
            controller=LaunchConfiguration("ctrl")
        )
        ld.add_action(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=ros2_control_node,
                    on_start=[js_broadcaster, controller],
                )
            )
        )

    # (Gazebo already started above if requested)

    # MoveIt move_group + RViz
    model = "iiwa7"
    moveit_configs_builder = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model, package_name=f"{model}_moveit_config"
    )
    move_group_params = LBRMoveGroupMixin.params_move_group()

    robot_name = LaunchConfiguration("robot_name", default="lbr")
    ld.add_action(
        LBRMoveGroupMixin.node_move_group(
            parameters=[
                moveit_configs_builder.to_dict(),
                move_group_params,
                # Ensure MoveIt uses the same apple-equipped robot_description
                robot_description,
                {"use_sim_time": use_sim_time},
            ],
            namespace=robot_name,
        )
    )

    ld.add_action(
        RVizMixin.node_rviz(
            rviz_cfg_pkg=f"{model}_moveit_config",
            rviz_cfg="config/moveit.rviz",
            parameters=LBRMoveGroupMixin.params_rviz(
                moveit_configs=moveit_configs_builder.to_moveit_configs()
            )
            + [robot_description, {"use_sim_time": use_sim_time}],
            remappings=[
                (
                    "display_planned_path",
                    PathJoinSubstitution([robot_name, "display_planned_path"]),
                ),
                ("joint_states", PathJoinSubstitution([robot_name, "joint_states"])),
                (
                    "monitored_planning_scene",
                    PathJoinSubstitution([robot_name, "monitored_planning_scene"]),
                ),
                ("planning_scene", PathJoinSubstitution([robot_name, "planning_scene"])),
                (
                    "planning_scene_world",
                    PathJoinSubstitution([robot_name, "planning_scene_world"]),
                ),
                (
                    "robot_description",
                    PathJoinSubstitution([robot_name, "robot_description"]),
                ),
                (
                    "robot_description_semantic",
                    PathJoinSubstitution([robot_name, "robot_description_semantic"]),
                ),
                (
                    "recognized_object_array",
                    PathJoinSubstitution([robot_name, "recognized_object_array"]),
                ),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    # expose args
    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    ld.add_action(RVizMixin.arg_rviz())
    # MoveIt-related args required by LBRMoveGroupMixin.params_move_group()
    from lbr_bringup.moveit import LBRMoveGroupMixin as _LM
    ld.add_action(_LM.arg_allow_trajectory_execution())
    ld.add_action(_LM.arg_capabilities())
    ld.add_action(_LM.arg_disable_capabilities())
    ld.add_action(_LM.arg_monitor_dynamics())
    ld.add_action(_LM.args_publish_monitored_planning_scene())
    # Declare apple parameters so they are available for CLI overrides and evaluation
    ld.add_action(DeclareLaunchArgument("apple_radius", default_value="0.04"))
    ld.add_action(DeclareLaunchArgument("apple_mass", default_value="0.15"))
    ld.add_action(DeclareLaunchArgument("adapter_length", default_value="0.1"))

    ld.add_action(OpaqueFunction(function=hidden_setup))
    return ld
