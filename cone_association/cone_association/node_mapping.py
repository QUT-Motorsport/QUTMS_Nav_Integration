from math import atan2, cos, hypot, pi, sin, sqrt
import time

import numpy as np
from sklearn.neighbors import KDTree
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import euler2quat, quat2euler
from sklearn.cluster import DBSCAN

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import ConeDetectionStamped, ConeWithCovariance, Reset, Cone
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Path

from driverless_common.common import wrap_to_pi
from cone_association.cone_props import ConeProps

from typing import Optional, Tuple

VIEW_X = 20
VIEW_Y = 10

class ConeAssociation(Node):
    state: Optional[np.ndarray] = None
    track_in_view: Optional[np.ndarray] = None
    confirmed_track: Optional[np.ndarray] = None
    properties = np.array([])

    last_time = time.time()
    
    detection_count = 0

    def __init__(self):
        super().__init__("sbg_slam_node")

        self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.callback, 1)
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)
        self.create_subscription(Reset, "/system/reset", self.reset_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # slam publisher
        self.slam_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/slam/global_map", 1)
        self.local_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/slam/local_map", 1)

        self.get_logger().info("---SLAM node initialised---")

    def reset_callback(self, msg):
        self.get_logger().info("Resetting Map")
        self.state = None
        self.track_in_view = None
        self.confirmed_track = None
        self.properties = np.array([])

    def callback(self, msg: ConeDetectionStamped):
        start: float = time.perf_counter()
        # skip if no transform received
        try:
            map_to_base = self.tf_buffer.lookup_transform("track", "base_footprint", rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn("Transform exception: " + str(e))
            return

        # get the delta from the previous state
        x = map_to_base.transform.translation.x
        y = map_to_base.transform.translation.y
        ang = quat2euler(
            [
                map_to_base.transform.rotation.w,
                map_to_base.transform.rotation.x,
                map_to_base.transform.rotation.y,
                map_to_base.transform.rotation.z,
            ]
        )[2]

        if self.state is None:
            self.state = np.array([x, y, ang])
            return
        # update the last pose
        self.state = np.array([x, y, ang])

        # flush the map of points that are outside the view of the car
        self.flush_map()

        # track array is in the form [id, x, y, colour]
        # ConeProps extracts the location of cones from the lidar to the car frame
        # also gets the location of cones in the map frame

        # process detected cones
        for cone in msg.cones:
            detection = ConeProps(cone, msg.header.frame_id, self.state)  # detection with properties
            if self.track_in_view is None:
                # initialise array
                self.track_in_view = np.array([[self.detection_count, detection.map_x, detection.map_y, detection.colour]])
                continue
            self.track_in_view = np.vstack([self.track_in_view, [self.detection_count, detection.map_x, detection.map_y, detection.colour]])
            self.detection_count += 1
        
        # cluster cones with DBSCAN
        clustering = DBSCAN(eps=1.5, min_samples=25).fit(self.track_in_view[:, 1:3])
        
        # get the center and number of pts in each cluster
        cluster_centers = []
        points_in_clusters = []
        for i in range(max(clustering.labels_)):
            center = np.mean(self.track_in_view[clustering.labels_ == i, 1:3], axis=0)
            points = len(self.track_in_view[clustering.labels_ == i, 1:3])
            points_in_clusters.append(points)
            cluster_centers.append(center)

            if self.confirmed_track is None:
                # initialise array
                if points > 25:
                    self.confirmed_track = np.array([[center[0], center[1], 1]])
                continue 
            self.confirmed_track = np.vstack([self.confirmed_track, [center[0], center[1], 1]])

        # publish slam msg
        if self.confirmed_track is not None:
            track_msg = ConeDetectionStamped()
            track_msg.header.stamp = msg.header.stamp
            track_msg.header.frame_id = "track"
            for cone in self.confirmed_track:
                track_msg.cones.append(Cone(location=Point(x=cone[0], y=cone[1], z=0.0), color=int(cone[2])))
                track_msg.cones_with_cov.append(
                    ConeWithCovariance(cone=Cone(location=Point(x=cone[0], y=cone[1], z=0.0), color=int(cone[2])), covariance=[0.0,0.0,0.0,0.0])
                )

            self.slam_publisher.publish(track_msg)

        # publish local map msg
        # local_map_msg = ConeDetectionStamped()
        # local_map_msg.header.stamp = msg.header.stamp
        # local_map_msg.header.frame_id = "base_footprint"
        # for detection in self.get_local_map(track_as_2d.reshape(-1, 2)):
        #     if detection.confirmed:
        #         local_map_msg.cones.append(detection.local_cone_as_msg)
        #         local_map_msg.cones_with_cov.append(
        #             ConeWithCovariance(cone=detection.local_cone_as_msg, covariance=detection.cov_as_msg)
        #         )
        # self.local_publisher.publish(local_map_msg)

        if time.time() - self.last_time > 1:
            self.get_logger().info(f"Wait time: {str(time.perf_counter()-start)}")  # log time
            self.last_time = time.time()


    # def get_local_map(self, rotation_mat) -> np.ndarray:
    #     """
    #     Get cones within view of the car
    #     * param rotation_mat: rotation matrix to rotate the map to the car's heading
    #     * return: array of (x, y) of these cones
    #     """
    #     # transform detection to map
    #     rotation_mat = np.array(
    #         [
    #             [cos(self.state[2]), -sin(self.state[2])],
    #             [sin(self.state[2]), cos(self.state[2])],
    #         ]
    #     )

    #     local_coords = np.array([])
    #     for i in range(len(self.properties)):
    #         local = np.linalg.inv(rotation_mat) @ (self.properties[i].map_coords - self.state[0:2])
    #         local_coords = np.append(local_coords, [local])
    #         self.properties[i].local_x = local[0]
    #         self.properties[i].local_y = local[1]
    #     local_coords = local_coords.reshape(-1, 2)

    #     # get any cones that are within -10m to 10m beside car
    #     side_idxs = np.where(np.logical_and(local_coords[:, 1] > -Y_RANGE, local_coords[:, 1] < Y_RANGE))[0]
    #     # get any cones that are within 15m in front of car
    #     forward_idxs = np.where(np.logical_and(local_coords[:, 0] > 0, local_coords[:, 0] < X_RANGE))[0]
    #     # combine indexes
    #     idxs = np.intersect1d(side_idxs, forward_idxs)

    #     return self.properties[idxs]

    def flush_map(self):
        """
        Remove landmarks outside of the view of the car
        """

        # if no landmarks, return
        if self.track_in_view is None:
            return
        
        # get the indexes of landmarks that are outside of the view of the car
        # rotation matrix to rotate the map to the car's heading
        rotation_mat = np.array(
            [
                [cos(self.state[2]), -sin(self.state[2])],
                [sin(self.state[2]), cos(self.state[2])],
            ]
        )

        local_coords = np.array([])
        for i in range(len(self.track_in_view)):
            local = np.linalg.inv(rotation_mat) @ (self.track_in_view[i, 1:3] - self.state[0:2])
            local_coords = np.append(local_coords, [local])
        local_coords = local_coords.reshape(-1, 2)

        # get any cones that are within -10m to 10m beside car
        side_idxs = np.where(np.logical_and(local_coords[:, 1] > -VIEW_Y, local_coords[:, 1] < VIEW_Y))[0]
        # get any cones that are within 15m in front of car
        forward_idxs = np.where(np.logical_and(local_coords[:, 0] > 0, local_coords[:, 0] < VIEW_X))[0]
        # combine indexes
        idxs = np.intersect1d(side_idxs, forward_idxs)

        # remove landmarks outside of the view of the car
        self.track_in_view = self.track_in_view[idxs]
        

def main(args=None):
    rclpy.init(args=args)
    node = ConeAssociation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
