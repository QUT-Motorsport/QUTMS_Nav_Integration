import os

import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="qutms_nav2").find(
        "qutms_nav2"
    )

    localisation_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/dual_localisation_params.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    navsat_transform_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/dual_localisation_params.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[('gps/fix', 'imu/nav_sat_fix'), ('imu', 'imu/data')]
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="False",
                description="Flag to enable use_sim_time",
            ),
            localisation_node,
            navsat_transform_node,
        ]
    )
