from math import atan2, cos, pi, sin, sqrt

import numpy as np
import scipy.interpolate as scipy_interpolate

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from driverless_msgs.msg import Cone, ConeDetectionStamped, State
from nav_msgs.msg import OccupancyGrid

from typing import List, Tuple

# for colour gradient based on intensity
MAX_ANGLE = 0.15


def approximate_b_spline_path(x: list, y: list, n_path_points: int, degree=3, s=0) -> Tuple[list, list]:
    """
    ADAPTED FROM: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/BSplinePath/bspline_path.py \n
    Approximate points with a B-Spline path
    * param x: x position list of approximated points
    * param y: y position list of approximated points
    * param n_path_points: number of path points
    * param degree: (Optional) B Spline curve degree
    * return: x and y position list of the result path
    """

    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
    distances = np.concatenate(([0.0], distances))
    distances /= distances[-1]

    spl_i_x = scipy_interpolate.UnivariateSpline(distances, x, k=degree, s=s)
    spl_i_y = scipy_interpolate.UnivariateSpline(distances, y, k=degree, s=s)

    sampled = np.linspace(0.0, distances[-1], n_path_points)

    spline_x = spl_i_x(sampled)
    spline_y = spl_i_y(sampled)

    # curvature
    dx = spl_i_x.derivative(1)(sampled)
    dy = spl_i_y.derivative(1)(sampled)
    heading = np.arctan2(dy, dx)
    ddx = spl_i_x.derivative(2)(sampled)
    ddy = spl_i_y.derivative(2)(sampled)
    curvature = (ddy * dx - ddx * dy) / np.power(dx * dx + dy * dy, 2.0 / 3.0)
    
    return spline_x, spline_y


def sort_cones(cones, start_index=None, end_index=None):
    """
    This is a function that calculates the nearest-neighbor path between a start and end index for a list of cones.
    The cones are represented as 2D points in the form of (x, y).

    The function takes 3 parameters:
    cones: a list of the cones to be sorted.
    start_index: the index of the starting cone.
    end_index: the index of the ending cone.

    The function uses a matrix "mat" to store the Euclidean distances between each pair of cones, and uses these
    distances to calculate the nearest-neighbor path between the start and end indices.

    The path is found by starting at the start_index, finding the index of the closest cone that hasn't already
    been visited, and adding that index to the "order" list. This process is repeated until the end_index is reached.

    The final path is returned as the "order" list, which contains the indices of the cones in the order they should
    be visited.
    """
    if start_index is None:
        start_index = 0
    if end_index is None:
        end_index = len(cones) - 1

    cone_count = len(cones)
    mat = [[0] * cone_count for i in range(cone_count)]
    for i, a in enumerate(cones):
        for j, b in enumerate(cones):
            if i == j:
                continue

            # Ensures we sort in the direction that the car is pointing
            # If the current cone is the far orange cone, we ignore all cones where the x coord is less than the
            # selected cone's x coord.
            # I can't image a track that this would cause problems on.
            if i == start_index and b[0] < a[0]:
                continue

            # Prevent unnecessary square root calculations. We don't need to
            # calculate distance if the distance is going to be greater than 5m (sqrt(25)).
            # So don't square root, or store the distance in the mat.
            # Note: Doubled the accepted distance to account for human error and camera noise.
            diff = [a[0] - b[0], a[1] - b[1]]
            dist = diff[0] * diff[0] + diff[1] * diff[1]
            if dist <= 50:
                mat[i][j] = sqrt(dist)

    order = [start_index]
    # Loop for each cone that needs to be ordered.
    # -2 accounts for the start and end indices that are manually handled.
    for unused in range(len(mat) - 2):
        min_index = -1
        min_value = 10000
        # Get the latest ordered cone's index from the 'order' list, and loop through that cone's row in 'mat'.
        # That cone's row in 'mat' contains the distance between the cone, and every other cone on the track.
        # We can then easily find the closest unused cone and append it to the 'order' list.
        for i, dist in enumerate(mat[order[-1]]):
            if dist >= min_value or dist == 0 or i == end_index or i in order:
                continue

            min_index = i
            min_value = dist
        order.append(min_index)
    order.append(end_index)

    # Return the ordered cones.
    return [cones[order[i]] for i in range(len(order))]


def parse_orange_cones(logger, orange_cones: List[List[float]]) -> List[List[float]]:
    """
    Breaks the big orange starting cones into their position relative to the other blue/yellow cones.
    Returns format close_blue, far_blue, close_yellow, far_yellow.
    """
    if len(orange_cones) != 4:
        logger.fatal("parse_orange_cones called with less than 4 visible cones. Requires 4 cones.")
        return []

    blue_cones = [cone for cone in orange_cones if cone[1] > 0]
    yellow_cones = [cone for cone in orange_cones if cone[1] < 0]

    if blue_cones[0][0] > blue_cones[1][0]:
        blue_cones[0], blue_cones[1] = blue_cones[1], blue_cones[0]

    if yellow_cones[0][0] > yellow_cones[1][0]:
        yellow_cones[0], yellow_cones[1] = yellow_cones[1], yellow_cones[0]

    return [blue_cones[0], blue_cones[1], yellow_cones[0], yellow_cones[1]]


class OrderedMapSpline(Node):
    spline_const = 10  # number of points per cone
    segment = int(spline_const * 0.1)  # percentage of PPC
    planning = False
    final_path_published = False
    current_track = None
    interp_cone_num = 3  # number of points interpolated between each pair of cones
    current_map = None

    def __init__(self):
        super().__init__("ordered_map_spline_node")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(ConeDetectionStamped, "/slam/global_map", self.map_callback, 10)
        self.create_subscription(State, "/system/as_status", self.state_callback, 1)
        self.create_timer(1/20, self.map_processing_callback)

        # publishers
        # rclcpp::KeepLast(1)).transient_local().reliable()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", qos_profile)

        self.get_logger().info("---Ordered path planner node initalised---")

    def state_callback(self, msg: State):
        if msg.state == State.DRIVING and msg.lap_count > 0 and not self.planning:
            self.planning = True
            self.get_logger().info("Lap completed, planning commencing")

    def map_callback(self, track_msg: ConeDetectionStamped):
        if self.final_path_published:
            return

        self.get_logger().debug("Received map")
        self.current_track = track_msg

    def get_occupancy_grid(self, bx, by, yx, yy):
        # turn spline boundary points into an occupancy grid map
        map = OccupancyGrid()
        map.header = self.current_track.header
        map.info.resolution = 0.1
        map.info.map_load_time = self.current_track.header.stamp

        # get max and min x and y values for each side of the track
        x_offset = min(bx)
        y_offset = min(by)

        # offset track so minimum is 0
        bx_cells = [int((x - x_offset)/map.info.resolution) for x in bx]
        by_cells = [int((y - y_offset)/map.info.resolution) for y in by]
        yx_cells = [int((x - x_offset)/map.info.resolution) for x in yx]
        yy_cells = [int((y - y_offset)/map.info.resolution) for y in yy]

        # get track properties
        map.info.width = max(bx_cells)+1
        map.info.height = max(by_cells)+1
        map.info.origin.position.x = x_offset
        map.info.origin.position.y = y_offset

        # turn points into a 2D np array
        bounds = np.array([[x_cells, y_cells] for x_cells, y_cells in zip(bx_cells+yx_cells, by_cells+yy_cells)])
        grid = np.zeros((map.info.height, map.info.width), dtype=np.int8)
        for x, y in bounds:
            # pad around each point a bit
            if (x == 0) or (x == map.info.width-1) or (y == 0) or (y == map.info.height-1):
                # cant pad if on edge of map
                grid[y:y+1, x:x+1] = 100
            else:
                grid[y-1:y+1, x-1:x+1] = 100

        # make a wall of points at 0,0 to prevent the car from driving backwards
        x_offset_cell = int((0 - x_offset)/map.info.resolution)
        y_offset_cell = int((0 - y_offset)/map.info.resolution)
        grid[y_offset_cell-20:y_offset_cell+20, x_offset_cell+30:x_offset_cell+31] = 100

        map.data = grid.ravel().tolist()
        return map

    def map_processing_callback(self):
        # skip if we haven't completed a lap yet
        if (not self.planning) or (self.current_track is None):
            return

        self.get_logger().debug("Planning")

        # extract data out of message
        cones = self.current_track.cones

        yellows: List[List[float]] = []
        blues: List[List[float]] = []
        oranges: List[List[float]] = []

        for cone in cones:
            if cone.color == Cone.YELLOW:
                yellows.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.BLUE:
                blues.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.ORANGE_BIG:
                oranges.append([cone.location.x, cone.location.y])

        # place orange cones on their respective sides of the track
        parsed_orange_cones = parse_orange_cones(self.get_logger(), oranges)
        if len(parsed_orange_cones) == 0:
            return
        blues.insert(0, parsed_orange_cones[1])
        blues.append(parsed_orange_cones[0])
        yellows.insert(0, parsed_orange_cones[3])
        yellows.append(parsed_orange_cones[2])

        # Sort the blue and yellow cones starting from the far orange cone, and ending at the close orange cone.
        ordered_blues = sort_cones(blues)
        ordered_yellows = sort_cones(yellows)
        ordered_blues.append(ordered_blues[0])
        ordered_yellows.append(ordered_yellows[0])

        ## Spline smoothing
        # make number of pts based on length of path
        spline_len = self.spline_const * len(ordered_blues)
        # retrieves spline lists (x,y)
        yx, yy = approximate_b_spline_path(
            [cone[0] for cone in ordered_yellows], [cone[1] for cone in ordered_yellows], spline_len
        )
        bx, by = approximate_b_spline_path(
            [cone[0] for cone in ordered_blues], [cone[1] for cone in ordered_blues], spline_len
        )

        ## Create occupancy grid of interpolated bounds
        map = self.get_occupancy_grid(bx, by, yx, yy)
        self.current_map = map
        self.map_pub.publish(self.current_map)
        self.final_path_published = True

def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = OrderedMapSpline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
