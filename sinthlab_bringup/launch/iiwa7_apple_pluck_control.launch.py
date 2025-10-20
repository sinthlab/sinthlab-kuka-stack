from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Common params to mirror the admittance demo style
    base_link = DeclareLaunchArgument("base_link", default_value="lbr_link_0")
    end_effector_link = DeclareLaunchArgument("end_effector_link", default_value="lbr_link_ee")
    f_ext_th = DeclareLaunchArgument(
        "f_ext_th", default_value="[2.0, 2.0, 2.0, 0.5, 0.5, 0.5]"
    )
    dq_gains = DeclareLaunchArgument(
        "dq_gains", default_value="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
    )
    dx_gains = DeclareLaunchArgument(
        "dx_gains", default_value="[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]"
    )
    exp_smooth = DeclareLaunchArgument("exp_smooth", default_value="0.95")
    dq_smooth = DeclareLaunchArgument("dq_smooth", default_value="0.95")
    pinv_rcond = DeclareLaunchArgument("pinv_rcond", default_value="0.1")

    gamma_initial = DeclareLaunchArgument("gamma_initial", default_value="0.8")
    release_trigger_force = DeclareLaunchArgument("release_trigger_force", default_value="10.0")
    release_hold_time = DeclareLaunchArgument("release_hold_time", default_value="1.0")
    release_duration = DeclareLaunchArgument("release_duration", default_value="1.5")

    node = Node(
        package="sinthlab_bringup",
        executable="apple_pluck_control.py",
        name="apple_pluck_control",
        output="screen",
        parameters=[
            {"base_link": LaunchConfiguration("base_link")},
            {"end_effector_link": LaunchConfiguration("end_effector_link")},
            {"f_ext_th": LaunchConfiguration("f_ext_th")},
            {"dq_gains": LaunchConfiguration("dq_gains")},
            {"dx_gains": LaunchConfiguration("dx_gains")},
            {"exp_smooth": LaunchConfiguration("exp_smooth")},
            {"dq_smooth": LaunchConfiguration("dq_smooth")},
            {"pinv_rcond": LaunchConfiguration("pinv_rcond")},
            {"gamma_initial": LaunchConfiguration("gamma_initial")},
            {"release_trigger_force": LaunchConfiguration("release_trigger_force")},
            {"release_hold_time": LaunchConfiguration("release_hold_time")},
            {"release_duration": LaunchConfiguration("release_duration")},
        ],
    )

    return LaunchDescription(
        [
            base_link,
            end_effector_link,
            f_ext_th,
            dq_gains,
            dx_gains,
            exp_smooth,
            dq_smooth,
            pinv_rcond,
            gamma_initial,
            release_trigger_force,
            release_hold_time,
            release_duration,
            node,
        ]
    )
