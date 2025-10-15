from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Reuse upstream mixins
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Args consistent with upstream
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg())

    # robot_description from upstream iiwa7
    robot_description = LBRDescriptionMixin.param_robot_description(
        mode="mock",
        robot_name=LaunchConfiguration("robot_name", default="lbr"),
        system_config=LaunchConfiguration("sys_cfg", default="ros2_control/lbr_system_config.yaml"),
        sys_cfg_pkg=LaunchConfiguration("sys_cfg_pkg", default="lbr_description"),
        initial_joint_positions=LaunchConfiguration("init_jnt_pos", default="ros2_control/initial_joint_positions.yaml"),
    )

    # robot_state_publisher
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    ld.add_action(robot_state_publisher)

    # ros2_control
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        use_sim_time=False, robot_description=robot_description
    )
    ld.add_action(ros2_control_node)

    # controllers
    js_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("ctrl")
    )

    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[js_broadcaster, controller]
        )
    ))

    return ld
