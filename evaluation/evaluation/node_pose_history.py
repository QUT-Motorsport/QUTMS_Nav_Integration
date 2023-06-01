import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

import time

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

class PoseHistory(Node):
    slam_path_history = Path()
    last_slam_path_time = time.time()
    gt_path_history = Path()
    last_gt_path_time = time.time()

    def __init__(self) -> None:
        super().__init__("pose_history_node")

        # subscribe to topic
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.slam_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/ground_truth/car_pose", self.gt_callback, 10)
        self.slam_history_pub: Publisher = self.create_publisher(Path, "/slam/car_pose_history", 10)
        self.gt_history_pub: Publisher = self.create_publisher(Path, "/ground_truth/car_pose_history", 10)

        self.get_logger().info("---Track Writer Initalised---")

    def slam_callback(self, msg: PoseWithCovarianceStamped):
        self.slam_path_history.header = msg.header
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        self.slam_path_history.poses.append(pose)

        if time.time() - self.last_slam_path_time > 1:
            self.slam_history_pub.publish(self.slam_path_history)
            self.last_slam_path_time = time.time()

    def gt_callback(self, msg: PoseWithCovarianceStamped):
        self.gt_path_history.header = msg.header
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        self.gt_path_history.poses.append(pose)

        if time.time() - self.last_gt_path_time > 1:
            self.gt_history_pub.publish(self.gt_path_history)
            self.last_gt_path_time = time.time()


def main():
    # begin ros node
    rclpy.init()
    node = PoseHistory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
