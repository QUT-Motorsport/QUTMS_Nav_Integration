from dataclasses import dataclass
from math import atan2, cos, hypot, sin
from typing import Tuple

import numpy as np
from driverless_msgs.msg import Cone
from geometry_msgs.msg import Point


@dataclass
class ConeProps:
    """Creates a datatype for storing and converting cones recorded"""

    local_x: float
    local_y: float
    map_x: float
    map_y: float
    cov: np.ndarray
    colour: int
    range: float
    bearing: float
    sensor: str
    tracked: bool = False
    confirmed: bool = False
    frame_count: int = 0
    yellow_count: int = 0
    blue_count: int = 0
    orange_count: int = 0

    def __init__(self, cone: Cone, frame_id: str, state: np.ndarray):
        if frame_id == "velodyne":
            self.local_x = cone.location.x + 1.65
            self.sensor = "lidar"
        else:
            self.local_x = cone.location.x - 0.1
            self.sensor = "camera"
        self.local_y = cone.location.y
        self.colour = cone.color

        self.range = hypot(self.local_x, self.local_y)
        self.bearing = atan2(self.local_y, self.local_x)

        # transform detection to map
        rotation_mat = np.array(
            [
                [cos(state[2]), -sin(state[2])],
                [sin(state[2]), cos(state[2])],
            ]
        )
        map_coords = rotation_mat @ np.array([self.local_x, self.local_y]).T + state[:2]
        self.map_x = map_coords[0]
        self.map_y = map_coords[1]

    def update(self, state: np.ndarray, cov: np.ndarray, colour: int, FRAME_COUNT: int):
        """Updates the state and colour of the cone"""
        self.map_coords = state
        self.cov = cov

        if colour == Cone.UNKNOWN:
            self.frame_count += 1

        if self.frame_count > FRAME_COUNT and not self.confirmed:
            if abs(self.cov[0, 0]) < 0.5 and abs(self.cov[1, 1]) < 0.5:
                self.confirmed = True  # mark as confirmed

        if self.sensor == "lidar":
            increment = 1
        else:
            increment = 3

        if colour == Cone.YELLOW:
            self.yellow_count += increment
        elif colour == Cone.BLUE:
            self.blue_count += increment
        elif colour == Cone.ORANGE_BIG:
            self.orange_count += increment

        if (
            self.yellow_count > self.blue_count
            and self.yellow_count > self.orange_count
        ):
            self.colour = Cone.YELLOW
        elif (
            self.blue_count > self.yellow_count and self.blue_count > self.orange_count
        ):
            self.colour = Cone.BLUE

        if (
            self.orange_count > 10
            or self.orange_count > self.yellow_count
            or self.orange_count > self.blue_count
        ):
            self.colour = Cone.ORANGE_BIG

        self.tracked = True

    @property
    def sense_rb(self) -> Tuple[float, float]:
        """Returns the range and bearing of the cone"""
        return self.range, self.bearing

    @property
    def cov_as_msg(self) -> list:
        """Returns the covariance of the cone for a message"""
        cov = self.cov.flatten().tolist()
        return cov

    @property
    def cone_as_msg(self) -> Cone:
        """Returns the state of the cone for a message"""
        cone = Cone(
            location=Point(x=self.map_coords[0], y=self.map_coords[1], z=0.0),
            color=self.colour,
        )
        return cone

    @property
    def local_cone_as_msg(self) -> Cone:
        """Returns the local state of the cone for a message"""
        cone = Cone(
            location=Point(x=self.local_x, y=self.local_y, z=0.0), color=self.colour
        )
        return cone
