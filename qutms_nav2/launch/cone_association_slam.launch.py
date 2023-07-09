import os
from ament_index_python.packages import get_package_share_path

import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_path("qutms_nav2")

    localisation_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/localisation_params.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
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
    assocation_node = Node(
        package="cone_association",
        executable="mapping2",
        name="cone_association_node",
        output="screen",
    )
    pose_history_node = Node(
        package="evaluation",
        executable="pose_history",
        name="pose_history_node",
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
            async_slam_toolbox_node,
            assocation_node,
            pose_history_node,
        ]
    )