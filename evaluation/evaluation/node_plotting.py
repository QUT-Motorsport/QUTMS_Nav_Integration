from math import sqrt
import time
from pathlib import Path as OSPath
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt

import pandas as pd
import rclpy
from driverless_msgs.msg import ConeDetectionStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from driverless_common.common import QOS_LATEST, angle, dist, fast_dist, wrap_to_pi

class MapComparison(Node):
    csv_folder = OSPath("./QUTMS_Nav_Integration/csv_data")
    slam_map = None
    slam_path = None
    gt_map = None
    gt_path = None
    blue_path = None
    yellow_path = None
    mid_path = None
    opt_path = None
    start_time = 0.0
    last_pos = [0.0, 0.0]
    started = False

    comparing = "slam"  # or "system"

    def __init__(self) -> None:
        super().__init__("track_to_csv_node")

        # subscribe to topic
        self.create_subscription(
            PoseWithCovarianceStamped, "/slam/car_pose", self.slam_pose_callback, 10
        )
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
            Path, "/ground_truth/odom_history", self.gt_path_callback, 10
        )
        self.create_subscription(
            Path, "/planning/midline_path", self.mid_path_callback, 10
        )
        self.create_subscription(
            Path, "/planning/global_path", self.opt_path_callback, 10
        )
        self.create_subscription(
            Path, "/planning/blue_bounds", self.blue_callback, 10
        )
        self.create_subscription(
            Path, "/planning/yellow_bounds", self.yellow_callback, 10
        )

        self.get_logger().info("---SLAM accuracy node Initalised---")

    def slam_map_callback(self, msg: ConeDetectionStamped):
        self.slam_map = msg

    def gt_map_callback(self, msg: ConeDetectionStamped):
        if self.gt_map is None:
            self.gt_map = msg

    def gt_path_callback(self, msg: Path):
        self.gt_path = msg

    def slam_path_callback(self, msg: Path):
        self.slam_path = msg

    def mid_path_callback(self, msg: Path):
        self.mid_path = msg

    def opt_path_callback(self, msg: Path):
        self.opt_path = msg

    def blue_callback(self, msg: Path):
        self.blue_path = msg

    def yellow_callback(self, msg: Path):
        self.yellow_path = msg

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        # convert to [x,y]
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]

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
            self.plot_stuff()
        
        self.last_pos = pos

    def plot_stuff(self):
        if self.comparing == "slam":
            self.plot_slam()
        elif self.comparing == "system":
            self.plot_system()

    def plot_slam(self):
        if self.slam_map is None or self.slam_path is None or self.gt_map is None or self.gt_path is None:
            self.get_logger().error("No SLAM or GT map or path recieved")
            return
        
        # extract msg data into arrays
        slam_map = []
        gt_map = []
        for cone in self.slam_map.cones:
            slam_map.append([cone.position.x, cone.position.y])

def main():
    # begin ros node
    rclpy.init()
    node = MapComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
