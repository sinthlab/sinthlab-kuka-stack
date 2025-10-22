from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Core timing/URDF
    robot_description = DeclareLaunchArgument("robot_description", default_value="")
    update_rate = DeclareLaunchArgument("update_rate", default_value="100")

    base_link = DeclareLaunchArgument("base_link", default_value="lbr_link_0")
    end_effector_link = DeclareLaunchArgument("end_effector_link", default_value="lbr_link_ee")
    exp_smooth = DeclareLaunchArgument("exp_smooth", default_value="0.95")
    pinv_rcond = DeclareLaunchArgument("pinv_rcond", default_value="0.1")

    stiffness = DeclareLaunchArgument(
        "stiffness", default_value="[1500.0, 700.0, 2500.0, 0.0, 0.0, 0.0]"
    )
    damping = DeclareLaunchArgument(
        "damping", default_value="[80.0, 60.0, 100.0, 0.0, 0.0, 0.0]"
    )
    max_wrench = DeclareLaunchArgument(
        "max_wrench", default_value="[100.0, 100.0, 150.0, 10.0, 10.0, 10.0]"
    )

    gamma_initial = DeclareLaunchArgument("gamma_initial", default_value="1.0")
    release_displacement = DeclareLaunchArgument("release_displacement", default_value="0.03")
    release_axis = DeclareLaunchArgument("release_axis", default_value="z")
    release_hold_time = DeclareLaunchArgument("release_hold_time", default_value="1.0")
    release_duration = DeclareLaunchArgument("release_duration", default_value="1.5")

    node = Node(
        package="sinthlab_bringup",
        executable="apple_pluck_impedance_control.py",
        name="apple_pluck_impedance_control",
        output="screen",
        parameters=[
            {"robot_description": LaunchConfiguration("robot_description")},
            {"update_rate": LaunchConfiguration("update_rate")},
            {"base_link": LaunchConfiguration("base_link")},
            {"end_effector_link": LaunchConfiguration("end_effector_link")},
            {"exp_smooth": LaunchConfiguration("exp_smooth")},
            {"pinv_rcond": LaunchConfiguration("pinv_rcond")},
            {"stiffness": LaunchConfiguration("stiffness")},
            {"damping": LaunchConfiguration("damping")},
            {"max_wrench": LaunchConfiguration("max_wrench")},
            {"gamma_initial": LaunchConfiguration("gamma_initial")},
            {"release_displacement": LaunchConfiguration("release_displacement")},
            {"release_axis": LaunchConfiguration("release_axis")},
            {"release_hold_time": LaunchConfiguration("release_hold_time")},
            {"release_duration": LaunchConfiguration("release_duration")},
        ],
    )

    return LaunchDescription(
        [
            robot_description,
            update_rate,
            base_link,
            end_effector_link,
            exp_smooth,
            pinv_rcond,
            stiffness,
            damping,
            max_wrench,
            gamma_initial,
            release_displacement,
            release_axis,
            release_hold_time,
            release_duration,
            node,
        ]
    )
