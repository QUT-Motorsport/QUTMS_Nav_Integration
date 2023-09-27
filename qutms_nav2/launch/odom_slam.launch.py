import os
from ament_index_python.packages import get_package_share_path

import launch
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    pkg_share = get_package_share_path("qutms_nav2")

    # Community ROS 2 packages
    robot_localization_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=str(pkg_share / "launch" / "robot_localization.launch.py")
        ),
    )

    # Custom packages
    slam_node = Node(
        package="py_slam",
        executable="odom_slam",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/custom_params.yaml"),
        ],
    )
    boundary_map_node = Node(
        package="nav_interfaces",
        executable="boundary_interpolation",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/custom_params.yaml"),
        ],
    )
    return launch.LaunchDescription(
        [
            robot_localization_launch,
            slam_node,
            boundary_map_node,
        ]
    )
