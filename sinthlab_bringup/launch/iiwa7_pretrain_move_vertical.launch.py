"""Pre-training: move vertical (one vertical rail, out to a threshold and back). Thin wrapper over experiment_base.launch.py."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Names the recording folder: analysis/expt_<this file's name>_<timestamp>/
RUN_NAME = "iiwa7_pretrain_move_vertical"


def generate_launch_description():
    return LaunchDescription(
        [
            # Overridable so the dashboard (experiment_ctrl_gui) can run an edited copy; the
            # default is the package YAML.
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("sinthlab_bringup"), "config", "pretrain_move_vertical.yaml"]
                ),
                description="Experiment parameter YAML.",
            ),
            # Overridable for the same reason: the dashboard passes a posture matching an edited
            # start pose. Relative paths are under the package share; an absolute path is used as is.
            DeclareLaunchArgument(
                "clik_nullspace_cfg",
                default_value="config/clik_nullspace_maze.yaml",
                description="CLIK redundancy posture YAML (must equal move_to_start).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("sinthlab_bringup"), "launch", "experiment_base.launch.py"]
                    )
                ),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "run_name": RUN_NAME,
                    "orchestrator": "rail_training_orchestrator.py",
                    # Joint controller active for the exact-posture start/recover moves; CLIK loaded
                    # inactive and switched in by the orchestrator for the trial.
                    "ctrl": "lbr_joint_position_command_controller",
                    "extra_inactive_ctrl": "kuka_clik_controller",
                    # Starts at the maze start (tool along +X), so the CLIK resolves the arm's
                    # redundancy toward that posture, as in the maze.
                    "clik_nullspace_cfg": LaunchConfiguration("clik_nullspace_cfg"),
                }.items(),
            ),
        ]
    )
