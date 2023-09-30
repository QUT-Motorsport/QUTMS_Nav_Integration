from math import sqrt
import time
from pathlib import Path as OSPath
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt

import pandas as pd
import rclpy
from driverless_msgs.msg import ConeDetectionStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from driverless_common.common import QOS_LATEST, angle, dist, fast_dist, wrap_to_pi


class MapComparison(Node):
    csv_folder = OSPath("./QUTMS_Nav_Integration/csv_data")
    slam_map = []
    slam_path = []
    gt_map = []
    gt_path = []
    localisation_type = "ekf_slam"
    last_pos = [0.0, 0.0]
    started = False

    def __init__(self) -> None:
        super().__init__("track_to_csv_node")

        # subscribe to topic
        self.create_subscription(
            ConeDetectionStamped, "/slam/global_map", self.slam_map_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/slam/car_pose", self.slam_pose_callback, 10
        )
        self.create_subscription(
            ConeDetectionStamped, "/ground_truth/global_map", self.gt_map_callback, 10
        )
        self.create_subscription(
            Odometry, "/ground_truth/odom", self.gt_odom_callback, 10
        )

        self.get_logger().info("---SLAM accuracy node Initalised---")

    def slam_map_callback(self, msg: ConeDetectionStamped):
        self.get_logger().info(f"Received slam map: {len(msg.cones)} cones", once=True)
        for cone in msg.cones:
            self.slam_map.append([cone.position.x, cone.position.y])

    def gt_map_callback(self, msg: ConeDetectionStamped):
        if len(self.gt_map) == 0:
            self.get_logger().info(f"Received gt map {len(msg.cones_with_cov)} cones", once=True)
            for cone in msg.cones_with_cov:
                self.gt_map.append([cone.position.x, cone.position.y])

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        # convert to [x,y]
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.slam_path.append(pos)

        current_time = time.time()
        # only start calculating once we have moved forward a bit
        # check if we have moved forward a bit
        if fast_dist(pos, self.last_pos) < 0.1 and not self.started:
            self.get_logger().info(f"Not started - {pos}", throttle_duration_sec=1)
            return
        elif fast_dist(pos, self.last_pos) >= 0.1 and not self.started:
            # only is entered once
            self.started = True
            self.start_time = current_time
            self.get_logger().info(f"Car Pose Recieved - {pos}", once=True)

        # check if we have returned to the start line (some time after starting)
        if self.last_pos[0] <= 0 and pos[0] > 0 and current_time - self.start_time > 10:
            # crossed the start line
            self.get_logger().info(f"Crossed start line - {pos}", once=True)
            self.calculate_err()
        
        self.last_pos = pos

    def gt_odom_callback(self, msg: Odometry):
        # convert to [x,y]
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.gt_path.append(pos)

    def calculate_err(self):
        self.get_logger().info("Comparing maps")
        # compare the two maps
        # iterate through each slam detection and find the closest gt detection to accumulate error
        map_error = 0
        for slam_cone in self.slam_map:
            # find the closest gt cone
            closest_gt_cone_distance = 1000000
            for gt_cone in self.gt_map:
                distance = fast_dist(slam_cone, gt_cone)
                if distance < closest_gt_cone_distance:
                    closest_gt_cone_distance = distance

            # add the error to the total error
            map_error += closest_gt_cone_distance

        # rmse
        map_rmse = sqrt(map_error / len(self.slam_map.cones))

        # compare the two paths
        # iterate through each slam pose and find the closest gt pose to accumulate error
        path_error = 0
        for slam_pos in self.slam_path:
            # find the closest gt pose
            closest_gt_pos_distance = 1000000
            for gt_pos in self.gt_path:
                distance = fast_dist(slam_pos, gt_pos)
                if distance < closest_gt_pos_distance:
                    closest_gt_pos_distance = distance

            # add the error to the total error
            path_error += closest_gt_pos_distance

        self.get_logger().info(f"Path error: {path_error}, {len(self.slam_path)}, {len(self.gt_path)}")
        # rmse
        path_rmse = sqrt(path_error / len(self.slam_path))

        self.get_logger().info(f"Map error: {map_rmse}")
        self.get_logger().info(f"Path error: {path_rmse}")

        # write the error to a csv file, append to the file if it already exists
        csv_file = self.csv_folder / "map_comparison.csv"
        if csv_file.exists():
            df = pd.read_csv(csv_file)
        else:
            df = pd.DataFrame(columns=["id", "map_rmse", "path_rmse"])

        # get the id of the current run - whether it uses odom or not and number of runs with that config
        id = self.localisation_type
        if any(id in s for s in df["id"].values):
            # get the last number
            last_num = int(df["id"].values[-1].split("_")[-1])
            # increment it
            id = f"{id}_{last_num + 1}"
        else:
            id = f"{id}_0"                

        df = df._append(
            {"id": id, "map_rmse": map_rmse, "path_rmse": path_rmse}, ignore_index=True
        )

        df.to_csv(csv_file, index=False)

        # exit the node
        self.get_logger().info("Finished writing to csv")

        # plot with matplotlib
        plt.figure(figsize=(10, 10))
        plt.scatter(*zip(*self.slam_map), c="r", label="SLAM Map")
        plt.scatter(*zip(*self.gt_map), c="b", label="Ground Truth Map")
        plt.scatter(*zip(*self.slam_path), c="r", label="SLAM Pose Tracked")
        plt.scatter(*zip(*self.gt_path), c="b", label="Ground Truth Pose Tracked")
        plt.legend()
        plt.show()

        # exit(0)


def main():
    # begin ros node
    rclpy.init()
    node = MapComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
