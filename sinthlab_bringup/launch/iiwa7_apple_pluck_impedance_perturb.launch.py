"""Perturbation experiment. Thin wrapper over experiment_base.launch.py."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Names the recording folder: analysis/expt_<this file's name>_<timestamp>/
RUN_NAME = "iiwa7_apple_pluck_impedance_perturb"


def generate_launch_description():
    return LaunchDescription(
        [
            # Overridable so the dashboard (experiment_ctrl_gui) can run an edited copy; the
            # default is the package YAML, exactly as before.
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("sinthlab_bringup"), "config", "apple_pluck_impedance_perturb.yaml"]
                ),
                description="Experiment parameter YAML.",
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
                    "orchestrator": "perturb_orchestrator.py",
                    "ctrl": "lbr_joint_position_command_controller",
                }.items(),
            ),
        ]
    )
