from math import atan2, cos, pi, sin, sqrt, atan

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
import numpy as np

from transforms3d.euler import euler2quat

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import State
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped


def convert_trans_rot_vel_to_steering_angle(vel, omega, wheelbase):
    if omega == 0 or vel == 0:
        return 0.0

    radius = vel / omega
    return atan(wheelbase / radius) * (180 / pi) * 5


class Nav2Commands(Node):
    wheelbase = 1.5  # taken from sim config - measured on car
    following = False
    driving = False
    planning = False

    def __init__(self):
        super().__init__('nav2_commands')
        
        self.create_subscription(Twist, "/control/nav_cmd_vel", self.cmd_callback, 1)
        self.create_subscription(State, "/system/as_status", self.state_callback, 1)

        self.create_timer((1 / 10), self.timer_callback)  # 50hz state 'prediction'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 1)

        self.get_logger().info("---Nav2 control interpreter initalised---")

    def state_callback(self, msg: State):
        # we haven't started driving yet
        if msg.state == State.DRIVING:
            self.driving = True

        # we have finished a lap
        if msg.lap_count > 0 and self.driving:
            self.planning = True

    def timer_callback(self):
        if not self.planning or self.following:
            return

        try:
            track_to_base = self.tf_buffer.lookup_transform("track", "base_footprint", rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn("Transform exception: " + str(e))
            return

        time.sleep(1)

        init_pose_msg = PoseWithCovarianceStamped()
        init_pose_msg.header.stamp = track_to_base.header.stamp
        init_pose_msg.header.frame_id = "track"
        # convert translation to pose
        init_pose_msg.pose.pose.position.x = track_to_base.transform.translation.x
        init_pose_msg.pose.pose.position.y = track_to_base.transform.translation.y
        init_pose_msg.pose.pose.position.z = track_to_base.transform.translation.z
        init_pose_msg.pose.pose.orientation = track_to_base.transform.rotation
        # cov diag to square
        diag = np.diag([0.25, 0.25, 0.0, 0.0, 0.0, 0.06]).astype(np.float32)
        init_pose_msg.pose.covariance = diag.flatten().tolist()
        self.init_pose_pub.publish(init_pose_msg)

        time.sleep(1)

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = track_to_base.header.stamp
        goal_pose_msg.header.frame_id = "track"
        goal_pose_msg.pose.position.x = -0.2
        goal_pose_msg.pose.position.y = -0.2
        goal_pose_msg.pose.position.z = 0.0

        # convert euler to quaternion
        quat = euler2quat(0.0, 0.0, 0.3)
        goal_pose_msg.pose.orientation.x = quat[1]
        goal_pose_msg.pose.orientation.y = quat[2]
        goal_pose_msg.pose.orientation.z = quat[3]
        goal_pose_msg.pose.orientation.w = quat[0]
        
        self.goal_pose_pub.publish(goal_pose_msg)

        self.get_logger().info("Publishing initial and goal poses")
        self.following = True

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
        # make time for msg id
        # msg.header.stamp = 
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
