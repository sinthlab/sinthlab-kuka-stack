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
                    # Joint controller active for the exact-posture start/recover moves; CLIK loaded
                    # inactive and switched in by the orchestrator for the corridor fixtures.
                    "ctrl": "lbr_joint_position_command_controller",
                    "extra_inactive_ctrl": "kuka_clik_controller",
                    # The maze is the ONLY experiment whose tool points along +X, so the CLIK must
                    # resolve the arm's redundancy toward the maze start posture, not the tool-down one.
                    "clik_nullspace_cfg": "config/clik_nullspace_maze.yaml",
                }.items(),
            ),
        ]
    )
