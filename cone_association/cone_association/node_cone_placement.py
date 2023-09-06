import time
from math import atan2, cos, hypot, pi, sin, sqrt
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from driverless_msgs.msg import (Cone, ConeDetectionStamped,
                                 ConeWithCovariance, State)
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.publisher import Publisher
from sklearn.neighbors import KDTree
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import euler2quat, quat2euler

from cone_association.cone_props import ConeProps

VIEW_X = 20
VIEW_Y = 10
RADIUS = 1
LEAF_SIZE = 10
MIN_DETECTIONS = 20


class ConeAssociation(Node):
    state: Optional[np.ndarray] = None
    track: Optional[np.ndarray] = None
    last_time = time.time()

    detection_count = 0
    mapping = False

    def __init__(self):
        super().__init__("cone_association_node")

        self.create_subscription(
            ConeDetectionStamped, "/lidar/cone_detection", self.callback, 1
        )
        self.create_subscription(
            ConeDetectionStamped, "/vision/cone_detection", self.callback, 1
        )
        self.create_subscription(State, "/system/as_status", self.state_callback, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # slam publisher
        self.slam_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, "/slam/global_map", 1
        )
        self.local_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, "/slam/local_map", 1
        )

        self.get_logger().info("---SLAM node initialised---")

    def state_callback(self, msg: State):
        # we haven't started driving yet
        if msg.state == State.DRIVING and msg.lap_count == 0:
            self.mapping = True

        # we have finished mapping
        if msg.lap_count > 0 and self.mapping:
            self.get_logger().info("Lap completed, mapping completed")
            self.mapping = False

    def callback(self, msg: ConeDetectionStamped):
        if not self.mapping:
            return

        start: float = time.perf_counter()

        # skip if no transform received
        try:
            map_to_base = self.tf_buffer.lookup_transform(
                "track", "base_footprint", Time(), Duration(seconds=10)
            )
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

        # track array is in the form [id, x, y, colour, count, yellow_count, blue_count, orange_count]
        # ConeProps extracts the location of cones from the lidar to the car frame
        # also gets the location of cones in the map frame

        # process detected cones
        for cone in msg.cones:
            detection = ConeProps(
                cone, msg.header.frame_id, self.state
            )  # detection with properties
            if self.track is None:
                if msg.header.frame_id == "velodyne":
                    # initialise array
                    yellow_count = 1 if detection.colour == Cone.YELLOW else 0
                    blue_count = 1 if detection.colour == Cone.BLUE else 0
                    orange_count = 1 if detection.colour == Cone.ORANGE_BIG else 0

                    self.track = np.array(
                        [
                            [
                                detection.map_x,
                                detection.map_y,
                                detection.colour,
                                1,
                                yellow_count,
                                blue_count,
                                orange_count,
                            ]
                        ]
                    )
                    print("Initialised track: ", self.track)
                continue

            # search a KD tree for the closest cone, if it is within 1m, update the location
            # otherwise, add it to the array
            tree = KDTree(self.track[:, :2], leaf_size=LEAF_SIZE)
            ind = tree.query_radius([[detection.map_x, detection.map_y]], r=RADIUS)
            if ind[0].size != 0:
                idx = ind[0][0]
                current_cone = self.track[idx]

                # update position with lidar
                if msg.header.frame_id == "velodyne":
                    # update the location of the cone using the average of the previous location and the new location
                    detections = current_cone[3]
                    self.track[idx, :2] = (
                        current_cone[:2] * detections
                        + [detection.map_x, detection.map_y]
                    ) / (detections + 1)
                    # update the count of the cone
                    self.track[idx, 3] += 1

                # update colour with camera
                else:
                    # update the colour weight of the cone
                    yellow_count = (
                        current_cone[4] + 1
                        if detection.colour == Cone.YELLOW
                        else current_cone[4]
                    )
                    blue_count = (
                        current_cone[5] + 1
                        if detection.colour == Cone.BLUE
                        else current_cone[5]
                    )
                    orange_count = (
                        current_cone[6] + 1
                        if detection.colour == Cone.ORANGE_BIG
                        else current_cone[6]
                    )
                    if yellow_count > blue_count and yellow_count > orange_count:
                        colour = Cone.YELLOW
                    elif blue_count > yellow_count and blue_count > orange_count:
                        colour = Cone.BLUE
                    else:
                        colour = Cone.ORANGE_BIG
                    self.track[idx, 2] = colour
                    self.track[idx, 4] = yellow_count
                    self.track[idx, 5] = blue_count
                    self.track[idx, 6] = orange_count

            else:
                if msg.header.frame_id == "velodyne":
                    yellow_count = 1 if detection.colour == Cone.YELLOW else 0
                    blue_count = 1 if detection.colour == Cone.BLUE else 0
                    orange_count = 1 if detection.colour == Cone.ORANGE_BIG else 0
                    self.track = np.vstack(
                        [
                            self.track,
                            [
                                detection.map_x,
                                detection.map_y,
                                detection.colour,
                                1,
                                yellow_count,
                                blue_count,
                                orange_count,
                            ],
                        ]
                    )

        # if no cones were detected, return
        if self.track is None:
            return

        # publish slam msg
        track_msg = ConeDetectionStamped()
        track_msg.header.stamp = msg.header.stamp
        track_msg.header.frame_id = "track"
        for cone in self.track:
            # ensure that the cone has been detected enough
            if cone[3] < MIN_DETECTIONS:
                continue

            cone_msg = Cone(
                location=Point(x=cone[0], y=cone[1], z=0.0), color=int(cone[2])
            )
            track_msg.cones.append(cone_msg)
            # make covariance based on the number of detections
            covariance = [5.0 / cone[3], 0.0, 0.0, 5.0 / cone[3]]
            track_msg.cones_with_cov.append(
                ConeWithCovariance(cone=cone_msg, covariance=covariance)
            )

        self.slam_publisher.publish(track_msg)

        # publish local map msg
        local_map_msg = self.get_local_map(msg)
        self.local_publisher.publish(local_map_msg)

        if time.time() - self.last_time > 1:
            self.get_logger().debug(
                f"Wait time: {str(time.perf_counter()-start)}"
            )  # log time
            self.last_time = time.time()

    def get_local_map(self, msg: ConeDetectionStamped) -> np.ndarray:
        """
        Get cones within view of the car
        * param rotation_mat: rotation matrix to rotate the map to the car's heading
        * return: array of (x, y) of these cones
        """
        # transform detection to map
        rotation_mat = np.array(
            [
                [cos(self.state[2]), -sin(self.state[2])],
                [sin(self.state[2]), cos(self.state[2])],
            ]
        )

        local_map_msg = ConeDetectionStamped()
        local_map_msg.header.stamp = msg.header.stamp
        local_map_msg.header.frame_id = "base_footprint"

        # get the location of the cones in the car frame
        for cone in self.track:
            local = np.linalg.inv(rotation_mat) @ (cone[:2] - self.state[0:2])
            if local[0] < 0:
                continue
            if local[0] > VIEW_X:
                continue
            if local[1] < -VIEW_Y:
                continue
            if local[1] > VIEW_Y:
                continue

            cone_msg = Cone(
                location=Point(x=local[0], y=local[1], z=0.0), color=int(cone[2])
            )
            local_map_msg.cones.append(cone_msg)

            covariance = [10.0 / cone[3], 0.0, 0.0, 10.0 / cone[3]]
            local_map_msg.cones_with_cov.append(
                ConeWithCovariance(cone=cone_msg, covariance=covariance)
            )

        return local_map_msg


def main(args=None):
    rclpy.init(args=args)
    node = ConeAssociation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
