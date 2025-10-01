from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())

    robot_name = LaunchConfiguration("robot_name", default="lbr")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("kuka_bringup"),
                "launch",
                "iiwa7_gazebo_apple.launch.py",
            ])
        ),
        launch_arguments={
            "robot_name": robot_name,
            "init_jnt_pos": LaunchConfiguration("init_jnt_pos", default="ros2_control/initial_joint_positions.yaml"),
            "ctrl": LaunchConfiguration("ctrl", default="lbr_joint_position_command_controller"),
        }.items(),
    )
    ld.add_action(base)

    # Bridge services for apply_link_wrench and entity removal
    wrench_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="apple_pluck_bridge",
        arguments=[
            "/world/empty/apply_link_wrench@ros_gz_interfaces/srv/ApplyLinkWrench",
            "/world/empty/remove@ros_gz_interfaces/srv/Entity",
        ],
        output="screen",
    )
    ld.add_action(wrench_bridge)

    pluck_node = Node(
        package="kuka_bringup",
        executable="apple_pluck_controller.py",
        name="apple_pluck_controller",
        namespace=robot_name,
        output="screen",
        parameters=[
            {"world_name": "empty", "robot_name": robot_name, "release_torque_threshold": 8.0},
        ],
    )
    ld.add_action(pluck_node)

    return ld
