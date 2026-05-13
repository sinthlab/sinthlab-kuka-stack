from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lbr_bringup.description import LBRDescriptionMixin

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

    # 3. Bring up KUKA Hardware in Impedance/Position Mode
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

    # Force torque bias calibrator to zero out external torques
    force_torque_bias_calibrator_node = Node(
        package="sinthlab_bringup",
        executable="force_torque_bias_calibrator.py",
        name="force_torque_bias_calibrator",
        namespace=LaunchConfiguration("robot_name"),
    )

    # 4. Launch the Virtual Fixture Node
    orchestrator_node = Node(
        package="sinthlab_bringup",
        executable="restricted_plane_orchestrator.py",
        name="restricted_plane_orchestrator",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        # We rely exclusively on python-based PyKDL inside the node using standard parameter passing.
        parameters=[
            robot_description,
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
            force_torque_bias_calibrator_node,
            orchestrator_node,
        ]
    )
