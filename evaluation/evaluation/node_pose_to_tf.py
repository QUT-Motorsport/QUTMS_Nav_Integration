import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class PoseToTf(Node):
    def __init__(self) -> None:
        super().__init__("pose_to_tf")

        # subscribe to topic
        self.create_subscription(
            Odometry, "/odometry/filtered", self.slam_callback, 10
        )

        self.sim_clock_pub = self.create_publisher(
            Clock, "/clock", 10
        )

        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("---TF publisher Initalised---")

    def slam_callback(self, msg: Odometry):
        # create tf
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # publish tf
        self.tf_broadcaster.sendTransform(t)

        # publish sim clock
        time_msg = Clock()
        time_msg.clock = msg.header.stamp
        self.sim_clock_pub.publish(time_msg)

def main():
    # begin ros node
    rclpy.init()
    node = PoseToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
