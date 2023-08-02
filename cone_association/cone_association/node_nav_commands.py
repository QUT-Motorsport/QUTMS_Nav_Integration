from math import atan2, cos, pi, sin, sqrt, atan

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import State
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped


def convert_trans_rot_vel_to_steering_angle(vel, omega, wheelbase):
    if omega == 0 or vel == 0:
        return 0

    radius = vel / omega
    return atan(wheelbase / radius)


class Nav2Commands(Node):
    wheelbase = 1.4  # taken from sim config - measured on car
    following = False
    driving = False
    published_poses = False

    def __init__(self):
        super().__init__('nav2_commands')
        
        self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_callback, 1)
        self.create_subscription(State, "/system/as_status", self.state_callback, 1)
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.pose_callback, 1)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        self.init_pose_pub = self.create_publisher(PoseStamped, "/initialpose", 1)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 1)

    def state_callback(self, msg: State):
        # we haven't started driving yet
        if msg.state == State.DRIVING and msg.lap_count == 0:
            self.driving = True

        # we have finished a lap
        if msg.lap_count > 0 and self.published_poses:
            self.get_logger().info("Path established, following commencing")
            self.following = True

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if not self.driving or self.published_poses:
            return
        
        init_pose_msg = PoseStamped()
        init_pose_msg.header = msg.header
        init_pose_msg.pose = msg.pose.pose
        self.init_pose_pub.publish(init_pose_msg)

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header = msg.header
        goal_pose_msg.pose = msg.pose.pose
        goal_pose_msg.pose.position.x = msg.pose.pose.position.x - 1
        self.goal_pose_pub.publish(goal_pose_msg)

        self.get_logger().info("Publishing initial and goal poses")
        self.published_poses = True

    def cmd_callback(self, twist_msg: Twist):       
        # must be all conditions to publish
        if not (self.driving and self.following):
            return

        vel = twist_msg.linear.x
        steering = convert_trans_rot_vel_to_steering_angle(
            vel, 
            twist_msg.angular.z, 
            self.wheelbase,
        )
        
        msg = AckermannDriveStamped()
        msg.header.stamp = rclpy.time.Time()
        msg.header.frame_id = "base_footprint"
        msg.drive.steering_angle = steering
        msg.drive.speed = vel
        
        self.drive_pub.publish(msg)

def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = Nav2Commands()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
