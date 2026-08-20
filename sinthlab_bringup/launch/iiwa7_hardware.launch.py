from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathSubstitution,
    PythonExpression,
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
            "--controller-manager",
            "controller_manager",
        ],
        namespace=LaunchConfiguration("namespace"),
    )

    # The ACTIVE controller (ctrl) is spawned after an optional delay (ctrl_activation_delay, default 0),
    # kept as a general knob for holding a controller back until the hardware has settled.
    # NOTE: this is NOT how the effort controllers' empty-velocity-interface crash is handled — that was a
    # deadlock (FRI reaches commanding only once a controller is active, but the controller needed
    # commanding-populated velocity), so delaying activation could not fix it. It is fixed properly in
    # ros2_effort_controller/effort_controller_base (SINTHLAB PATCH #1; see that folder's README).
    delayed_ctrl_spawner = TimerAction(
        period=LaunchConfiguration("ctrl_activation_delay"),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    LaunchConfiguration("ctrl"),
                    "--controller-manager",
                    "controller_manager",
                ],
                namespace=LaunchConfiguration("namespace"),
            )
        ],
    )

    # Optionally load a second controller in the INACTIVE state (configured, not running). The
    # restricted-plane / maze experiments use this for the Cartesian impedance controller: the
    # joint-impedance controller (ctrl) is active for the start/recover moves, then the orchestrator
    # switches to this one for the fixture.
    inactive_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            LaunchConfiguration("extra_inactive_ctrl"),
            "--inactive",
            "--controller-manager",
            "controller_manager",
        ],
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("extra_inactive_ctrl"), "' != ''"])
        ),
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
                name="ctrl_overlay_cfg",
                default_value="config/ctrl_overlay_none.yaml",
                description="Generic controller-params OVERLAY, loaded AFTER ctrl_cfg so it wins. The "
                "torque fixtures point it at config/torque_overlay_*.yaml (per-experiment Cartesian "
                "stiffness + nullspace posture); the position experiments leave it at the harmless default.",
            ),
            DeclareLaunchArgument(
                name="ctrl",
                default_value="lbr_joint_position_command_controller",
                description="Desired default controller (spawned ACTIVE). Must be defined in the ctrl_cfg.",
            ),
            DeclareLaunchArgument(
                name="extra_inactive_ctrl",
                default_value="",
                description="Optional second controller spawned INACTIVE (e.g. the Cartesian impedance "
                "controller for the restricted-plane/maze experiments; the orchestrator switches to it "
                "for the fixture).",
            ),
            DeclareLaunchArgument(
                name="ctrl_activation_delay",
                default_value="0.0",
                description="Seconds to delay spawning the ACTIVE controller (ctrl) after the "
                "broadcasters. General-purpose knob; 0 for all shipped experiments.",
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
                    # Loaded LAST so it wins: the experiment's controller-params overlay.
                    PathSubstitution(
                        FindPackageShare(LaunchConfiguration("ctrl_cfg_pkg"))
                    )
                    / LaunchConfiguration("ctrl_overlay_cfg"),
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[robot_description_dependent_spawner, inactive_ctrl_spawner, delayed_ctrl_spawner],
                )
            ),
        ]
    )