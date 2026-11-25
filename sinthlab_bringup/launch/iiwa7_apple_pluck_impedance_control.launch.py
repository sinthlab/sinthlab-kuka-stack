"""Apple-pluck experiment -- pluck or perturb paradigm. Thin wrapper over experiment_base.launch.py.

    ros2 launch sinthlab_bringup iiwa7_apple_pluck_impedance_control.launch.py                     # YAML's paradigm
    ros2 launch sinthlab_bringup iiwa7_apple_pluck_impedance_control.launch.py paradigm:=perturb
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    return LaunchDescription(
        [
            # Overridable so the dashboard (experiment_ctrl_gui) can run an edited copy; the
            # default is the package YAML.
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("sinthlab_bringup"), "config", "apple_pluck_impedance.yaml"]
                ),
                description="Experiment parameter YAML.",
            ),
            DeclareLaunchArgument(
                "paradigm",
                default_value="",
                description="pluck | perturb. Empty = the `paradigm` in the params YAML.",
            ),
            DeclareLaunchArgument(
                "subject",
                default_value="",
                description="Subject ID, recorded with every trial (the dashboard asks for it at Start).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("sinthlab_bringup"), "launch", "experiment_base.launch.py"]
                    )
                ),
                launch_arguments={
                    "subject": LaunchConfiguration("subject"),
                    "params_file": LaunchConfiguration("params_file"),
                    "orchestrator": "apple_pluck_orchestrator.py",
                    "ctrl": "lbr_joint_position_command_controller",
                    "paradigm": LaunchConfiguration("paradigm"),
                }.items(),
            ),
        ]
    )
