from math import atan2, cos, pi, sin, sqrt, atan

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import State
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped


def convert_trans_rot_vel_to_steering_angle(vel, omega, wheelbase):
    if omega == 0 or vel == 0:
        return 0.0

    radius = vel / omega
    return atan(wheelbase / radius) * (180 / pi)


class Nav2Commands(Node):
    wheelbase = 1.5  # taken from sim config - measured on car
    following = False
    driving = False
    planning = False

    def __init__(self):
        super().__init__('nav2_commands')
        
        self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_callback, 1)
        self.create_subscription(State, "/system/as_status", self.state_callback, 1)

        self.create_timer((1 / 50), self.timer_callback)  # 50hz state 'prediction'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 1)

        self.get_logger().info("---Nav2 control interpreter initalised---")

    def state_callback(self, msg: State):
        # we haven't started driving yet
        if msg.state == State.DRIVING and msg.lap_count == 0:
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

        init_pose_msg = PoseWithCovarianceStamped()
        init_pose_msg.header = track_to_base.header
        # convert translation to pose
        init_pose_msg.pose.pose.position.x = track_to_base.transform.translation.x
        init_pose_msg.pose.pose.position.y = track_to_base.transform.translation.y
        init_pose_msg.pose.pose.orientation = track_to_base.transform.rotation
        self.init_pose_pub.publish(init_pose_msg)

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header = track_to_base.header
        goal_pose_msg.pose.position.x = 0.0
        goal_pose_msg.pose.position.y = 0.0
        self.goal_pose_pub.publish(goal_pose_msg)

        self.get_logger().info("Publishing initial and goal poses")
        self.following = True

    def cmd_callback(self, twist_msg: Twist):       
        # must be all conditions to publish
        if not (self.driving and self.following):
            return

        vel = twist_msg.linear.x * 6
        steering = convert_trans_rot_vel_to_steering_angle(
            vel, 
            twist_msg.angular.z, 
            self.wheelbase,
        )
        
        msg = AckermannDriveStamped()
        # make time for msg id
        # msg.header.stamp = 
        msg.header.frame_id = "base_footprint"
        msg.drive.steering_angle = steering * 2
        msg.drive.speed = vel
        
        self.drive_pub.publish(msg)

def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = Nav2Commands()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
