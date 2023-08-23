import time
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from driverless_msgs.msg import State
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses

DEBUG = True

class TrackdriveHandler(Node):
    mission_started = False
    sent_goal = False
    laps = 0
    last_lap_time = 0.0
    last_x = 0.0
    goal_offet = 5.0

    def __init__(self):
        super().__init__("trackdrive_logic_node")
        self.create_subscription(State, "system/as_status", self.state_callback, 10)

        self.create_timer((1 / 10), self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publishers
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 1)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "goal_pose", 1)

        # actions
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')

        self.get_logger().info("---Trackdrive handler node initialised---")

    def state_callback(self, msg: State):
        self.laps = msg.lap_count
        if not self.mission_started and msg.state == State.DRIVING and msg.mission == State.TRACKDRIVE:
            self.mission_started = True
            self.get_logger().info("Trackdrive mission started")

    def timer_callback(self):
        # check if car has crossed the finish line (0,0)
        # get distance from 0,0 and increment laps when within a certain threshold
        # and distance is increasing away from 0,0
        try:
            track_to_base = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn("Transform exception: " + str(e))
            return

        # publish initial pose on first lap
        if self.laps == 1 and self.mission_started and not self.sent_goal:
            init_pose_msg = PoseWithCovarianceStamped()
            init_pose_msg.header.frame_id = "map"
            # convert translation to pose
            init_pose_msg.pose.pose.position.x = track_to_base.transform.translation.x
            init_pose_msg.pose.pose.position.y = track_to_base.transform.translation.y
            init_pose_msg.pose.pose.position.z = track_to_base.transform.translation.z
            init_pose_msg.pose.pose.orientation = track_to_base.transform.rotation
            # cov diag to square
            # diag = np.diag([0.25, 0.25, 0.0, 0.0, 0.0, 0.06]).astype(np.float32)
            # init_pose_msg.pose.covariance = diag.flatten().tolist()
            self.init_pose_pub.publish(init_pose_msg)

            goal_poses = []
            for i in range(10):
                goal_pose_msg = PoseStamped()
                goal_pose_msg.header.frame_id = "map"
                goal_pose_msg.pose.position.x = self.goal_offet - (i * 0.1)
                goal_poses.append(goal_pose_msg)
            
            self.go_through_poses(goal_poses)
            # self.goal_pose_pub.publish(goal_pose_msg)
            self.sent_goal = True
            self.get_logger().info("Trackdrive mission goal sent")

        if self.laps == 10:
            self.get_logger().info("Trackdrive mission complete")

    def go_through_poses(self, poses: PoseStamped):
        # Sends a `NavThroughPoses` action request
        self.get_logger().info("Waiting for 'NavigateThroughPoses' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.get_logger().info('Navigating with ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        # while not send_goal_future.done():
        #     self.get_logger().info('Waiting for NavigateThroughPoses action to complete...')
        #     time.sleep(1)
        # self.goal_handle = send_goal_future.result()

        # if not self.goal_handle.accepted:
        #     self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
        #     return False

        # self.result_future = self.goal_handle.get_result_async()
        # return True


def main(args=None):
    rclpy.init(args=args)
    node = TrackdriveHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
