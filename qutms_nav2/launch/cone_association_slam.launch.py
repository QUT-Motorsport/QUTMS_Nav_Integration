import os
from ament_index_python.packages import get_package_share_path

import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_path("qutms_nav2")

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
    async_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/slam_params.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
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
    )
    pose_history_node = Node(
        package="evaluation",
        executable="pose_history",
        output="screen",
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
            async_slam_toolbox_node,
            assocation_node,
            # pose_history_node,
        ]
    )
