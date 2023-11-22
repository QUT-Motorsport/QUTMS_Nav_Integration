import os
from ament_index_python.packages import get_package_share_path

import launch
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    pkg_share = get_package_share_path("qutms_nav2")

    # Community ROS 2 packages
    localisation_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/localisation_params.yaml"),
        ],
    )

    mapping = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        output='screen',
        remappings=[('/input_cloud','/velodyne_points')],
        parameters=[
            os.path.join(pkg_share, "config/lidar_slam_params.yaml"),
        ],
    )

    graphbasedslam = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, "config/lidar_slam_params.yaml"),
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
            localisation_node,
            mapping,
            graphbasedslam,
            # assocation_node,
            boundary_map_node,
        ]
    )
