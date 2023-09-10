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

    async_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/slam_params.yaml"),
        ],
        remappings=[
            ("/pose", "/slam/car_pose"),
        ],
    )

    # Custom packages
    assocation_node = Node(
        package="cone_association",
        executable="cone_placement",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/custom_params.yaml"),
        ],
    )
    pose_history_node = Node(
        package="evaluation",
        executable="pose_history",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/custom_params.yaml"),
        ],
    )
    boundary_map_node = Node(
        package="cone_association",
        executable="map_interpolation",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/custom_params.yaml"),
        ],
    )
    
    return launch.LaunchDescription(
        [
            robot_localization_launch,
            async_slam_toolbox_node,
            assocation_node,
            boundary_map_node,
        ]
    )
