from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mission_controller",
                executable="mission_control",
            ),
            Node(
                package="canbus",
                executable="canbus_translator_node",
                parameters=[
                    get_package_share_path("canbus") / "config" / "canbus.yaml",
                ],
            ),
            Node(
                package="vehicle_supervisor",
                executable="vehicle_supervisor_node",
            ),
            Node(
                package="lidar_pipeline_3",
                executable="lidar_perception",
            ),
            Node(
                package="py_slam",
                executable="sbg_slam",
            ),
            Node(
                package="planners",
                executable="delaunay_planner",
            ),
            Node(
                package="vision_pipeline",
                executable="torch_detector",
            ),
            Node(
                package="steering_actuator",
                executable="steering_actuator_node",
                parameters=[
                    get_package_share_path("steering_actuator") / "config" / "steering.yaml",
                ],
            ),
            Node(
                package="velocity_controller",
                executable="velocity_controller_node",
                parameters=[
                    get_package_share_path("velocity_controller") / "config" / "velocity_controller.yaml",
                ],
            ),
            Node(
                package="controllers",
                executable="reactive_control",
            ),
            Node(
                package="path_follower",
                executable="pure_pursuit",
            ),
            # IncludeLaunchDescription(
            #     launch_description_source=PythonLaunchDescriptionSource(
            #         launch_file_path=str(
            #             get_package_share_path("roscube_machine") / "launch" / "sensor_drivers.launch.py"
            #         )
            #     ),
            # ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    launch_file_path=str(
                        get_package_share_path("vehicle_urdf") / "launch" / "robot_description.launch.py"
                    )
                ),
                launch_arguments=[
                    ("urdf_model", "qev3.urdf.xacro"),
                ],
            ),
        ]
    )
