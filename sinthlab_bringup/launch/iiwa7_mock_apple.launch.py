from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Args (reuse upstream)
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg())
    # Optional apple params
    apple_radius = LaunchConfiguration("apple_radius", default="0.04")
    apple_mass = LaunchConfiguration("apple_mass", default="0.15")
    adapter_length = LaunchConfiguration("adapter_length", default="0.1")

    # Build robot_description from our iiwa7 + apple xacro
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
                    " mode:=mock",
                    " apple_radius:=",
                    apple_radius,
                    " apple_mass:=",
                    apple_mass,
                    " adapter_length:=",
                    adapter_length,
                    " system_config_path:=",
                    PathJoinSubstitution(
                        [
                            FindPackageShare(
                                LaunchConfiguration("sys_cfg_pkg", default="lbr_description")
                            ),
                            LaunchConfiguration(
                                "sys_cfg", default="ros2_control/lbr_system_config.yaml"
                            ),
                        ]
                    ),
                    " initial_joint_positions_path:=",
                    PathJoinSubstitution(
                        [
                            FindPackageShare(
                                LaunchConfiguration("sys_cfg_pkg", default="lbr_description")
                            ),
                            LaunchConfiguration(
                                "init_jnt_pos", default="ros2_control/initial_joint_positions.yaml"
                            ),
                        ]
                    ),
                ]
            ),
            value_type=str,
        )
    }

    # RSP
    ld.add_action(LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    ))

    # ros2_control node
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        use_sim_time=False, robot_description=robot_description
    )
    ld.add_action(ros2_control_node)

    # Controllers
    js_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("ctrl")
    )

    ld.add_action(RegisterEventHandler(
        OnProcessStart(target_action=ros2_control_node, on_start=[js_broadcaster, controller])
    ))

    return ld
