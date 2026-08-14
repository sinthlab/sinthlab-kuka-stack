"""Maze-exploration experiment. Thin wrapper over experiment_base.launch.py."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("sinthlab_bringup"), "launch", "experiment_base.launch.py"]
                    )
                ),
                launch_arguments={
                    "params_file": PathJoinSubstitution(
                        [FindPackageShare("sinthlab_bringup"), "config", "maze_params.yaml"]
                    ),
                    "orchestrator": "maze_orchestrator.py",
                    # TORQUE mode (FRI joint-torque overlay + idra-lab impedance controllers):
                    # joint impedance active for the exact-posture start/recover moves; Cartesian
                    # impedance loaded INACTIVE and switched in by the orchestrator for the fixture.
                    "ctrl": "joint_impedance_controller",
                    "extra_inactive_ctrl": "cartesian_impedance_controller",
                    "sys_cfg_pkg": "sinthlab_bringup",
                    "sys_cfg": "config/lbr_system_config_torque.yaml",
                    "ctrl_cfg": "config/torque_controllers.yaml",
                    # Per-experiment overlay: Cartesian stiffness (X stiff, Y/Z soft) + the maze start
                    # posture as the nullspace bias (tool along +X).
                    "ctrl_overlay_cfg": "config/torque_overlay_maze.yaml",
                }.items(),
            ),
        ]
    )
