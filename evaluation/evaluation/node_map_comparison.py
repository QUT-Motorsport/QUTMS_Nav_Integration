from math import sqrt
from pathlib import Path as OSPath

import pandas as pd
import rclpy
from driverless_msgs.msg import ConeDetectionStamped
from nav_msgs.msg import Path
from rclpy.node import Node

USE_ODOM = False


class MapComparison(Node):
    csv_folder = OSPath("./QUTMS_Nav_Integration/csv_data")
    slam_map = None
    slam_path = None
    gt_map = None
    gt_path = None

    def __init__(self) -> None:
        super().__init__("track_to_csv_node")

        # subscribe to topic
        if USE_ODOM:
            self.create_subscription(
                ConeDetectionStamped,
                "/odom_slam/global_map",
                self.slam_map_callback,
                10,
            )
            self.create_subscription(
                Path, "/odom_slam/car_pose_history", self.slam_path_callback, 10
            )
        else:
            self.create_subscription(
                ConeDetectionStamped, "/slam/global_map", self.slam_map_callback, 10
            )
            self.create_subscription(
                Path, "/slam/car_pose_history", self.slam_path_callback, 10
            )
        self.create_subscription(
            ConeDetectionStamped, "/ground_truth/global_map", self.gt_map_callback, 10
        )
        self.create_subscription(
            Path, "/ground_truth/car_pose_history", self.gt_path_callback, 10
        )

        # make a timer that periodically checks if we have received a map and path
        self.create_timer(1.0, self.check_received)

        self.get_logger().info("---Track Writer Initalised---")

    def slam_map_callback(self, msg: ConeDetectionStamped):
        if self.slam_path is None:
            self.slam_map = msg
            self.get_logger().info(f"Received slam map: {len(msg.cones)} cones")

    def slam_path_callback(self, msg: Path):
        if self.slam_path is None:
            self.slam_path = msg
            self.get_logger().info(f"Received slam path: {len(msg.poses)} poses")

    def gt_map_callback(self, msg: ConeDetectionStamped):
        if self.gt_map is None:
            self.gt_map = msg
            self.get_logger().info(f"Received gt map {len(msg.cones_with_cov)} cones")

    def gt_path_callback(self, msg: Path):
        if self.gt_path is None:
            self.gt_path = msg
            self.get_logger().info(f"Received gt path: {len(msg.poses)} poses")

    def check_received(self):
        if (
            self.slam_map is None
            or self.slam_path is None
            or self.gt_map is None
            or self.gt_path is None
        ):
            return

        self.get_logger().info("Comparing maps")
        # compare the two maps
        # iterate through each slam detection and find the closest gt detection to accumulate error
        map_error = 0
        for slam_cone in self.slam_map.cones:
            # find the closest gt cone
            closest_gt_cone = None
            closest_gt_cone_distance = 1000000
            for gt_cone in self.gt_map.cones_with_cov:
                distance = (slam_cone.location.x - gt_cone.cone.location.x) ** 2 + (
                    slam_cone.location.y - gt_cone.cone.location.y
                ) ** 2
                if distance < closest_gt_cone_distance:
                    closest_gt_cone = gt_cone
                    closest_gt_cone_distance = distance

            # add the error to the total error
            map_error += closest_gt_cone_distance

        # rmse
        map_rmse = sqrt(map_error / len(self.slam_map.cones))

        # compare the two paths
        # iterate through each slam pose and find the closest gt pose to accumulate error
        path_error = 0
        for slam_pose in self.slam_path.poses:
            # find the closest gt pose
            closest_gt_pose = None
            closest_gt_pose_distance = 1000000
            for gt_pose in self.gt_path.poses:
                distance = (
                    slam_pose.pose.position.x - gt_pose.pose.position.x
                ) ** 2 + (slam_pose.pose.position.y - gt_pose.pose.position.y) ** 2
                if distance < closest_gt_pose_distance:
                    closest_gt_pose = gt_pose
                    closest_gt_pose_distance = distance

            # add the error to the total error
            path_error += closest_gt_pose_distance

        # rmse
        path_rmse = sqrt(path_error / len(self.slam_path.poses))

        self.get_logger().info(f"Map error: {map_rmse}")
        self.get_logger().info(f"Path error: {path_rmse}")

        # write the error to a csv file, append to the file if it already exists
        csv_file = self.csv_folder / "map_comparison.csv"
        if csv_file.exists():
            df = pd.read_csv(csv_file)
        else:
            df = pd.DataFrame(columns=["id", "map_rmse", "path_rmse"])

        # get the id of the current run - whether it uses odom or not and number of runs with that config
        id = "odom" if USE_ODOM else "no_odom"
        id += f"_{len(df[df['id'] == id])}"

        df = df.append(
            {"id": id, "map_rmse": map_rmse, "path_rmse": path_rmse}, ignore_index=True
        )

        df.to_csv(csv_file, index=False)

        # exit the node
        self.get_logger().info("Finished writing to csv")
        exit(0)


def main():
    # begin ros node
    rclpy.init()
    node = MapComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
