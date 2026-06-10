from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
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
    # lbr_ros2_control installs its plugin library (liblbr_ros2_control.so) under
    # lib/lbr_ros2_control/ (its CMakeLists uses `LIBRARY DESTINATION lib/${PROJECT_NAME}`),
    # not the standard lib/. Point the loader at that dir so the plugin's symbols resolve.
    lbr_ros2_control_lib_path = os.path.join(lbr_ros2_control_prefix, "lib", "lbr_ros2_control")

    # robot_description for robot_state_publisher, which publishes it on /robot_description.
    # The controller_manager (Jazzy) obtains the URDF by *subscribing to that topic*; it does
    # NOT read a robot_description node parameter. estimated_wrench_interface.on_init() parses
    # this URDF (WrenchEstimator), so it must not be spawned until the controller_manager has
    # received it — the spawner ordering below guarantees that.
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

    # joint_state_broadcaster needs no robot_description. Its spawner exits only once the
    # controller_manager has initialized the resource manager from the URDF (received over
    # the /robot_description topic), so it is a reliable "robot_description is ready" signal.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
        namespace=LaunchConfiguration("namespace"),
    )

    # These load after the broadcaster is up. estimated_wrench_interface (and the default
    # ctrl) need a non-empty robot_description in their on_init(); spawning them here avoids
    # the race that caused "Failed loading controller estimated_wrench_interface".
    robot_description_dependent_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "estimated_wrench_interface",
            "lbr_state_broadcaster",
            "force_torque_broadcaster",
            LaunchConfiguration("ctrl"),
            "--controller-manager",
            "controller_manager",
        ],
        namespace=LaunchConfiguration("namespace"),
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
                default_value="config/iiwa7_hardware_controllers.yaml",
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
                    {"use_sim_time": False},
                    PathSubstitution(
                        FindPackageShare(LaunchConfiguration("ctrl_cfg_pkg"))
                    )
                    / LaunchConfiguration("ctrl_cfg"),
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[robot_description_dependent_spawner],
                )
            ),
        ]
    )