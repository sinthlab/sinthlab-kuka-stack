from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
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
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Args
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())
    # Override initial joints to down
    ld.add_action(DeclareLaunchArgument('init_jnt_pos', default_value='ros2_control/initial_joint_positions_down.yaml'))
    # Apple parameters
    apple_radius = LaunchConfiguration('apple_radius', default='0.04')
    apple_mass = LaunchConfiguration('apple_mass', default='0.15')
    adapter_length = LaunchConfiguration('adapter_length', default='0.1')

    # Start base Gazebo setup similar to gazebo apple
    ld.add_action(LBRDescriptionMixin.arg_mode())

    # Use the existing gazebo apple flow via mixins
    ld.add_action(GazeboMixin.include_gazebo())
    ld.add_action(GazeboMixin.node_clock_bridge())

    # Build apple-equipped robot_description via xacro
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('sinthlab_description'),
                    'urdf',
                    'iiwa7_apple',
                    'iiwa7_apple.xacro',
                ]),
                ' robot_name:=', LaunchConfiguration('robot_name', default='lbr'),
                ' mode:=gazebo',
                ' apple_radius:=', apple_radius,
                ' apple_mass:=', apple_mass,
                ' adapter_length:=', adapter_length,
                ' initial_joint_positions_path:=',
                PathJoinSubstitution([
                    FindPackageShare('sinthlab_description'),
                    LaunchConfiguration('init_jnt_pos'),
                ]),
            ]),
            value_type=str,
        )
    }

    # RSP
    ld.add_action(LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=True
    ))

    # Create entity in Gazebo
    ld.add_action(GazeboMixin.node_create())

    # ros2_control and controllers
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        use_sim_time=True,
        robot_description=robot_description,
    )
    ld.add_action(ros2_control_node)
    js_broadcaster = LBRROS2ControlMixin.node_controller_spawner(controller='joint_state_broadcaster')
    controller = LBRROS2ControlMixin.node_controller_spawner(controller=LaunchConfiguration('ctrl'))
    ld.add_action(RegisterEventHandler(OnProcessStart(target_action=ros2_control_node, on_start=[js_broadcaster, controller])))

    # Bridge a wrench topic for apple_link so we can apply forces
    # ROS -> Gazebo direction uses ']' before gz type
    # Remap to Gazebo's canonical link wrench topic: /world/empty/model/<name>/link/<link>/wrench
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lbr/apple_link/wrench@geometry_msgs/msg/Wrench]gz.msgs.Wrench',
            '--ros-args',
            '-r', '/lbr/apple_link/wrench:=/world/empty/model/lbr/link/apple_link/wrench',
        ],
        output='screen',
    ))

    # Pull controller node (installed in libexec via CMake)
    ld.add_action(Node(
        package='sinthlab_bringup',
        executable='apple_pull_controller.py',
        name='apple_pull_controller',
        output='screen',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name', default='lbr'),
            'pull_force': 8.0,
            'effort_threshold': 20.0,
            'publish_rate': 50.0,
        }]
    ))

    return ld
