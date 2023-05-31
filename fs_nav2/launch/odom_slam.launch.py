import os

import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="fs_nav2").find(
        "fs_nav2"
    )
    slam_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="py_slam").find("py_slam")

    odom_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/localisation_params.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    slam_node = launch_ros.actions.Node(
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
            odom_localization_node,
            slam_node,
        ]
    )
