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

# Reuse mixins from upstream bringup for args and nodes
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Declare the same arguments as upstream so configs remain compatible
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg())

    # Build robot_description from our custom iiwa7 + apple xacro
    # sinthlab_lbr_description installs urdf/iiwa7_apple/iiwa7_apple.xacro
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("sinthlab_lbr_description"),
                        "urdf",
                        "iiwa7_apple",
                        "iiwa7_apple.xacro",
                    ]
                ),
                " robot_name:=",
                LaunchConfiguration("robot_name", default="lbr"),
                " mode:=mock",
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
        )
    }

    # robot state publisher
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    ld.add_action(robot_state_publisher)

    # ros2 control node
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        use_sim_time=False, robot_description=robot_description
    )
    ld.add_action(ros2_control_node)

    # controller spawners
    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("ctrl")
    )

    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster,
                controller,
            ],
        )
    )
    ld.add_action(controller_event_handler)

    return ld