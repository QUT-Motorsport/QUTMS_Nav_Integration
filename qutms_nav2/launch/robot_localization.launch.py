import os
from ament_index_python.packages import get_package_share_path

import launch
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_path("qutms_nav2")

    localisation_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/localisation_params.yaml"),
        ],
    )
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/localisation_params.yaml"),
        ],
        remappings=[('gps/fix', 'imu/nav_sat_fix'), ('imu', 'imu/data')]
    )

    return launch.LaunchDescription(
        [
            localisation_node,
            navsat_transform_node,
        ]
    )
