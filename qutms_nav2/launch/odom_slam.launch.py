import os

import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    pkg_share = FindPackageShare(package="qutms_nav2").find(
        "qutms_nav2"
    )

    # Community ROS 2 packages
    localisation_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/dual_localisation_params.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    navsat_transform_node = Node(
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

    slam_node = Node(
        package="py_slam",
        executable="odom_slam",
        name="odom_slam_node",
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            localisation_node,
            navsat_transform_node,
            slam_node,
        ]
    )
