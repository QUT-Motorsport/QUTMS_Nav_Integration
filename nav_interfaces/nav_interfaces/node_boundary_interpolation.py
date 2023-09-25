from math import atan2, cos, pi, sin, sqrt

import numpy as np
import scipy.interpolate as scipy_interpolate
from transforms3d.euler import euler2quat

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped
from driverless_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData

from driverless_common.common import QOS_LATEST, angle, dist, midpoint, fast_dist, wrap_to_pi

from typing import List, Tuple
import time

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


def make_path_msg(points, spline_len):
    poses: List[PoseStamped] = []  # target spline poses
    for i in range(spline_len):
        # get angle between current point and next point
        if i < spline_len - 1:
            th_change = angle(points[i], points[i+1])
        elif i == spline_len - 1:
            th_change = angle(points[i], points[0])
        # keep between 360
        if th_change > pi:
            th_change = th_change - 2 * pi
        elif th_change < -pi:
            th_change = th_change + 2 * pi

        pose = PoseStamped()
        pose.header.frame_id = "track"
        pose.pose.position.x = points[i][0]
        pose.pose.position.y = points[i][1]
        pose.pose.position.z = 0.0
        quat = euler2quat(0.0, 0.0, th_change)
        pose.pose.orientation.w = quat[0]
        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]
        poses.append(pose)

    # Add the first path point to the end of the list to complete the loop
    # poses.append(poses[0])
    path_msg = Path()
    path_msg.header.frame_id = "track"
    path_msg.poses = poses
    return path_msg


def get_closest_cone(cones: list, dir: int=1, start_dist: int=3, pos: float=1.5) -> List[float]:
    """
    Gets the position of the nearest cone to the car.
    * param cones: [x,y] coords of all current cones
    * param dir: 1 for blue cones, -1 for yellow cones
    * return: [x,y] of nearest cone to the car
    """
    # iterate through all cones and find the closest one
    nearest_cone = None
    last_dist = float("inf")
    for cone in cones:
        current_dist = fast_dist([start_dist, dir*pos], cone)
        if current_dist < last_dist:
            nearest_cone = cone
            last_dist = current_dist

    return nearest_cone


def get_next_cone(cones: list, current_cone: list, last_angle: float, search_range=5, search_angle=pi/2):
    """
    Gets the next cone in the direction of travel. searches within view cone of 90 degrees.
    * param cones: [x,y] coords of all current cones
    * param current_cone: [x,y] coords of current cone
    * param angle: angle cone of view in degrees
    * param range: range of cone of view in metres
    """

    # iterate through all cones and find the closest one
    nearest_cone = None
    nearest_angle = None
    last_dist = float("inf")

    # rotate search area by last angle
    for cone in cones:
        dist = fast_dist(current_cone, cone)
        # get angle between current point and next point
        cone_angle = angle(current_cone, cone)
        error = wrap_to_pi(last_angle - cone_angle)
        if search_angle > error > -search_angle and dist < search_range ** 2:
            if dist < last_dist:
                nearest_cone = cone
                nearest_angle = cone_angle
                last_dist = dist
    return nearest_cone, nearest_angle


def discovery_cones(cones):
    """
    Extracts cones from message including unknown cones, which can be interpreted
    """
    # extract data out of message
    unsearched_cones = []
    for cone in cones:
        unsearched_cones.append([cone.location.x, cone.location.y])
    return unsearched_cones


def mapped_cones(cones):
    """
    Stable map of cones, only colour identified cones are included
    """

    # extract data out of message
    unsearched_cones = []
    for cone in cones:
        if cone.color == Cone.YELLOW:
            unsearched_cones.append([cone.location.x, cone.location.y])
        elif cone.color == Cone.BLUE:
            unsearched_cones.append([cone.location.x, cone.location.y])
        elif cone.color == Cone.ORANGE_BIG:
            unsearched_cones.append([cone.location.x, cone.location.y])
    return unsearched_cones


def get_occupancy_grid(blue_points, yellow_points, header):
    # turn spline boundary points into an occupancy grid map
    map = OccupancyGrid()
    map.header = header
    map.info.resolution = 0.1
    map.info.map_load_time = header.stamp

    # we have blue and yellow interpolated points. 
    # turn them into an occupancy grid map with the specified resolution
    # set origin which has an origin at 0,0 in world coords
    # find the min and max x and y values
    bounds = blue_points + yellow_points
    bounds = np.array(bounds)
    bounds = bounds / map.info.resolution
    bounds = bounds.astype(int)

    # find the min and max x and y values
    min_x = int(np.min(bounds[:, 0]))
    max_x = int(np.max(bounds[:, 0]))
    min_y = int(np.min(bounds[:, 1]))
    max_y = int(np.max(bounds[:, 1]))

    # set map size
    if min_x > 0:
        min_x = 0

    # set origin to be the 0,0 point
    map.info.origin.position.x = min_x * map.info.resolution
    map.info.origin.position.y = min_y * map.info.resolution

    map.info.width = max_x - min_x + 1
    map.info.height = max_y - min_y + 1

    # shift all points to be relative to the origin
    bounds = bounds - [min_x, min_y]

    # turn points into a 2D np array
    grid = np.zeros((map.info.height, map.info.width), dtype=np.int8)
    for x, y in bounds:
        grid[y, x] = 100
    map.data = grid.ravel().tolist()
    return map
    

class OrderedMapSpline(Node):
    spline_const = 10  # number of points per cone
    segment = int(spline_const * 0.1)  # percentage of PPC
    following = False
    current_track = None
    interp_cone_num = 3  # number of points interpolated between each pair of cones
    final_path_published = False

    def __init__(self):
        super().__init__("ordered_map_spline_node")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(ConeDetectionStamped, "/slam/global_map", self.map_callback, QOS_LATEST)
        self.create_subscription(State, "/system/as_status", self.state_callback, QOS_LATEST)
        self.create_timer(1/10, self.planning_callback)

        self.declare_parameter("start_following", True)

        # publishers
        self.blue_bound_pub = self.create_publisher(Path, "/planning/blue_bounds", 1)
        self.yellow_bound_pub = self.create_publisher(Path, "/planning/yellow_bounds", 1)
        self.planned_path_pub = self.create_publisher(Path, "/planning/midline_path", 1)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.map_meta_pub = self.create_publisher(MapMetaData, "/map_metadata", map_qos)

        if self.get_parameter("start_following").value:
            self.following = True
            self.get_logger().warn("---DEBUG MODE ENABLED---")

        self.get_logger().info("---Ordered path planner node initalised---")

    def state_callback(self, msg: State):
        if msg.state == State.DRIVING and msg.lap_count > 0 and not self.following:
            self.following = True
            self.get_logger().info("Lap completed, planning commencing")

    def map_callback(self, track_msg: ConeDetectionStamped):
        if self.final_path_published:
            return

        self.get_logger().debug("Received map")
        self.current_track = track_msg

    def planning_callback(self):
        # skip if we haven't completed a lap yet
        if self.current_track is None or len(self.current_track.cones) == 0:
            return

        if not self.following:
            self.get_logger().info("Discovering", once=True)
            unsearched_cones = discovery_cones(self.current_track.cones)
        else:
            self.get_logger().info("Following", once=True)
            unsearched_cones = mapped_cones(self.current_track.cones)

        # find closest left and right cones to car
        closest_blue = get_closest_cone(unsearched_cones, dir=1, start_dist=1)
        # remove closest cones from list
        if closest_blue is None:
            self.get_logger().warn("No blue cones found")
            return
        unsearched_cones.remove(closest_blue)

        closest_yellow = get_closest_cone(unsearched_cones, dir=-1, start_dist=1)
        if closest_yellow is None:
            self.get_logger().warn("No yellow cones found")
            return
        unsearched_cones.remove(closest_yellow)

        # sort cones into order by finding the next cone in the direction of travel
        ordered_blues = [closest_blue]
        ordered_yellows = [closest_yellow]
        last_blue = closest_blue
        last_yellow = closest_yellow
        last_blue_angle = 0
        last_yellow_angle = 0
        while len(unsearched_cones) > 0:
            next_blue, last_blue_angle = get_next_cone(
                unsearched_cones, last_blue, last_blue_angle,
                search_range=7, search_angle=pi/4
            )
            if next_blue is None:
                break
            ordered_blues.append(next_blue)
            unsearched_cones.remove(next_blue)
            last_blue = next_blue

        while len(unsearched_cones) > 0:
            next_yellow, last_yellow_angle = get_next_cone(
                unsearched_cones, last_yellow, last_yellow_angle,
                search_range=7, search_angle=pi/4
            )
            if next_yellow is None:
                break
            ordered_yellows.append(next_yellow)
            unsearched_cones.remove(next_yellow)
            last_yellow = next_yellow

        # Spline smoothing
        # make number of pts based on length of path
        spline_len = self.spline_const * len(ordered_blues)

        # specify degree of spline if less than 3 cones        
        blue_degree = len(ordered_blues) - 1 if len(ordered_blues) <= 3 else 3
        yellow_degree = len(ordered_yellows) - 1 if len(ordered_yellows) <= 3 else 3
        if blue_degree <= 1 or yellow_degree <= 1:
            self.get_logger().warn("Not enough cones to spline")
            return

        if self.following:
            # add extra points to the end of the spline to make sure it doesn't end too early
            ordered_blues.append(ordered_blues[0])
            ordered_yellows.append(ordered_yellows[0])

        yx, yy = approximate_b_spline_path(
            [cone[0] for cone in ordered_yellows], 
            [cone[1] for cone in ordered_yellows], 
            spline_len, yellow_degree, 0.01
        )
        bx, by = approximate_b_spline_path(
            [cone[0] for cone in ordered_blues], 
            [cone[1] for cone in ordered_blues], 
            spline_len, blue_degree, 0.01
        )
        # turn individual x,y lists into points lists
        blue_points = []
        yellow_points = []
        mid_points = []
        for i in range(spline_len):
            blue_points.append([bx[i], by[i]])
            yellow_points.append([yx[i], yy[i]])
            mid_points.append(midpoint([yx[i], yy[i]], [bx[i], by[i]]))

        # publish bounds
        blue_bound_msg = make_path_msg(blue_points, spline_len)
        self.blue_bound_pub.publish(blue_bound_msg)

        yellow_bound_msg = make_path_msg(yellow_points, spline_len)
        self.yellow_bound_pub.publish(yellow_bound_msg)

        # publish midpoints
        mid_bound_msg = make_path_msg(mid_points, spline_len)
        self.planned_path_pub.publish(mid_bound_msg)

        ## Create occupancy grid of interpolated bounds
        map = get_occupancy_grid(blue_points, yellow_points, self.current_track.header)
        self.current_map = map
        self.map_pub.publish(self.current_map)
        self.map_meta_pub.publish(self.current_map.info)

def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = OrderedMapSpline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
