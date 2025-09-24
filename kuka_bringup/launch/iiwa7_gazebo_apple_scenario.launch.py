from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Include the base Gazebo + robot launch
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_bringup"),
                    "launch",
                    "iiwa7_gazebo_apple.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name", default="lbr"),
            "init_jnt_pos": LaunchConfiguration("init_jnt_pos", default="ros2_control/initial_joint_positions.yaml"),
            "ctrl": LaunchConfiguration("ctrl", default="lbr_joint_position_command_controller"),
        }.items(),
    )
    ld.add_action(base_launch)

    # Bridge Gazebo services for wrench application and entity removal
    apply_wrench_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/apply_link_wrench@ros_gz_interfaces/srv/ApplyLinkWrench",
            "/world/empty/remove@ros_gz_interfaces/srv/Entity",
        ],
        output="screen",
    )
    ld.add_action(apply_wrench_bridge)

    # Start the scenario driver node
    apple_puller = Node(
        package="kuka_bringup",
        executable="apple_puller.py",
        name="apple_puller",
        output="screen",
        parameters=[
            {"world_name": "empty", "robot_name": LaunchConfiguration("robot_name", default="lbr")}
        ],
        namespace=LaunchConfiguration("robot_name"),
    )
    ld.add_action(apple_puller)

    return ld
