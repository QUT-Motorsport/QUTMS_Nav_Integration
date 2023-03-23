import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='fs_nav2').find('fs_nav2')
    slam_pkg_share = launch_ros.substitutions.FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    nav2_pkg_share = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/localisation_params.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    odom_to_tf_node = launch_ros.actions.Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/odom2tf_params.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    slam_async_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': os.path.join(pkg_share, 'config/slam_params.yaml')}.items(),
    )

    nav2_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_share, 'launch', 'navigation_launch.py'),
        ),
        launch_arguments={'params_file': os.path.join(pkg_share, 'config/nav2_params.yaml')}.items(),
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                                    description='Flag to enable use_sim_time'),
        # odom_to_tf_node,
        robot_localization_node,
        slam_async_launch,
        # nav2_launch
    ])
