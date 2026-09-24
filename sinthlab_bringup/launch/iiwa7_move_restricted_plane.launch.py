"""Restricted-plane (virtual fixtures) experiment. Thin wrapper over experiment_base.launch.py."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            # Overridable so the dashboard (experiment_ctrl_gui) can run an edited copy; the
            # default is the package YAML, exactly as before.
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("sinthlab_bringup"), "config", "virtual_fixtures_params.yaml"]
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
                    "orchestrator": "restricted_plane_orchestrator.py",
                    # Joint controller active for the exact-posture start/recover moves; CLIK loaded
                    # inactive and switched in by the orchestrator for the fixture phase.
                    "ctrl": "lbr_joint_position_command_controller",
                    "extra_inactive_ctrl": "kuka_clik_controller",
                }.items(),
            ),
        ]
    )
