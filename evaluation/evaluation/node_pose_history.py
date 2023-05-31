import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

import time

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

class PoseHistory(Node):
    path_history = Path()
    last_path_time = time.time()

    def __init__(self) -> None:
        super().__init__("pose_history_node")

        # subscribe to topic
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.callback, 10)
        self.history_pub: Publisher = self.create_publisher(Path, "/slam/car_pose_history", 10)

        self.get_logger().info("---Track Writer Initalised---")

    def callback(self, msg: PoseWithCovarianceStamped):
        self.path_history.header = msg.header
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        self.path_history.poses.append(pose)

        if time.time() - self.last_path_time > 1:
            self.history_pub.publish(self.path_history)
            self.last_path_time = time.time()


def main():
    # begin ros node
    rclpy.init()
    node = PoseHistory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
