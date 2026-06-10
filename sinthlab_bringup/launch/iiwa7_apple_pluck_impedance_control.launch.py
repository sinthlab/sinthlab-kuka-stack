"""Apple-pluck experiment. Thin wrapper over experiment_base.launch.py."""
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
                        [FindPackageShare("sinthlab_bringup"), "config", "apple_pluck_impedance.yaml"]
                    ),
                    "orchestrator": "apple_pluck_orchestrator.py",
                    "ctrl": "lbr_joint_position_command_controller",
                }.items(),
            ),
        ]
    )
