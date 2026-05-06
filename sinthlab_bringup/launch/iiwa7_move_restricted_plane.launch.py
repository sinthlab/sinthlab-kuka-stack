from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lbr_bringup.description import LBRDescriptionMixin
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # 1. Base Setup Parameters
    robot_type = DeclareLaunchArgument("robot_type", default_value="iiwa7")
    robot_name = LBRDescriptionMixin.arg_robot_name()
    ctrl = DeclareLaunchArgument("ctrl", default_value="lbr_joint_position_command_controller")

    # 2. Load standard Robot Description for Hardware
    robot_description = LBRDescriptionMixin.param_robot_description(
        model=LaunchConfiguration("robot_type"),
        robot_name=LaunchConfiguration("robot_name"),
        mode="hardware"
    )
    
    # 3. Initialize MoveIt Configs to supply semantic/kinematics params for `moveit_py`
    # We dynamically build the moveit properties for your specific KUKA model
    moveit_config = MoveItConfigsBuilder(
        robot_name="iiwa7", package_name="iiwa7_moveit_config"
    ).to_moveit_configs()

    # 4. Bring up KUKA Hardware in Impedance/Position Mode
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("lbr_bringup"), "launch", "hardware.launch.py"])
        ),
        launch_arguments={
            "model": LaunchConfiguration("robot_type"),
            "robot_name": LaunchConfiguration("robot_name"),
            "ctrl": LaunchConfiguration("ctrl"),
        }.items(),
    )

    # 5. Launch the Virtual Fixture Node with MoveIt Py parameters
    virtual_fixture_node = Node(
        package="sinthlab_bringup",
        executable="move_restricted_on_a_plane.py",
        name="virtual_fixture_node",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            PathJoinSubstitution([
                FindPackageShare("sinthlab_bringup"),
                "config",
                "virtual_fixtures_params.yaml"
            ])
        ],
    )

    return LaunchDescription(
        [
            robot_type,
            robot_name,
            ctrl,
            hardware_launch,
            virtual_fixture_node,
        ]
    )