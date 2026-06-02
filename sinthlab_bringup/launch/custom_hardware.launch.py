from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_prefix

def generate_launch_description() -> LaunchDescription:
    lbr_ros2_control_prefix = get_package_prefix("lbr_ros2_control")
    # lib/ contains .so plugin files; lib/lbr_ros2_control/ contains executables (different dirs)
    lbr_ros2_control_lib_path = os.path.join(lbr_ros2_control_prefix, "lib")

    # Shared substitution so both robot_state_publisher and ros2_control_node receive
    # robot_description directly — eliminates the race where the spawner triggers
    # estimated_wrench_interface.on_init() before ros2_control_node has received the
    # description from the /robot_description topic, causing get_robot_description()
    # to return an empty string and the WrenchEstimator URDF parse to throw.
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathSubstitution(FindPackageShare("lbr_description"))
            / "urdf"
            / LaunchConfiguration("model")
            / LaunchConfiguration("model"),
            ".xacro",
            " robot_name:=",
            LaunchConfiguration("robot_name"),
            " mode:=hardware",
            " system_config_path:=",
            PathSubstitution(
                FindPackageShare(LaunchConfiguration("sys_cfg_pkg"))
            )
            / LaunchConfiguration("sys_cfg"),
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=[EnvironmentVariable("LD_LIBRARY_PATH", default_value=""), ":", lbr_ros2_control_lib_path]
            ),
            DeclareLaunchArgument(
                name="model",
                default_value="iiwa7",
                description="The LBR model in use.",
                choices=["iiwa7", "iiwa14", "med7", "med14"],
            ),
            DeclareLaunchArgument(
                name="robot_name",
                default_value="lbr",
                description="The robot's name. Links in the tf tree will be prefixed as <robot_name>_link. Same applies to joints.",
            ),
            DeclareLaunchArgument(
                name="namespace",
                default_value="lbr",
                description="Nodes in this launch file will be spawned with this namespace.",
            ),
            DeclareLaunchArgument(
                name="sys_cfg_pkg",
                default_value="lbr_description",
                description="Package containing the lbr_system_config.yaml file for FRI configurations.",
            ),
            DeclareLaunchArgument(
                name="sys_cfg",
                default_value="ros2_control/lbr_system_config.yaml",
                description="The relative path from sys_cfg_pkg to the lbr_system_config.yaml file.",
            ),
            DeclareLaunchArgument(
                name="ctrl_cfg_pkg",
                default_value="sinthlab_bringup",
                description="Controller configuration package. The package containing the ctrl_cfg.",
            ),
            DeclareLaunchArgument(
                name="ctrl_cfg",
                default_value="config/custom_hardware_controllers.yaml",
                description="Relative path from ctrl_cfg_pkg to the controllers.",
            ),
            DeclareLaunchArgument(
                name="ctrl",
                default_value="kuka_clik_controller",
                description="Desired default controller. Must be defined in the ctrl_cfg.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description_content},
                    {"use_sim_time": False},
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description_content},
                    {"use_sim_time": False},
                    PathSubstitution(
                        FindPackageShare(LaunchConfiguration("ctrl_cfg_pkg"))
                    )
                    / LaunchConfiguration("ctrl_cfg"),
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager",
                    "controller_manager",
                    "joint_state_broadcaster",
                    "estimated_wrench_interface",
                    "lbr_state_broadcaster",
                    "force_torque_broadcaster",
                    LaunchConfiguration("ctrl"),
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
        ]
    )