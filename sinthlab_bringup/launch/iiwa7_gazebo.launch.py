from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Args as upstream
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())

    # Upstream robot_description for Gazebo
    robot_description = LBRDescriptionMixin.param_robot_description(
        mode="gazebo",
        robot_name=LaunchConfiguration("robot_name", default="lbr"),
    )

    # RSP
    ld.add_action(LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=True
    ))

    # Gazebo
    ld.add_action(GazeboMixin.include_gazebo())
    ld.add_action(GazeboMixin.node_clock_bridge())
    ld.add_action(GazeboMixin.node_create())

    # Controllers
    ld.add_action(LBRROS2ControlMixin.node_controller_spawner(controller="joint_state_broadcaster"))
    ld.add_action(LBRROS2ControlMixin.node_controller_spawner(controller=LaunchConfiguration("ctrl")))

    return ld
