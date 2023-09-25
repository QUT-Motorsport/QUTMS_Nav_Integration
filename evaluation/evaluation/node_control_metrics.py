from math import sqrt
from pathlib import Path as OSPath
import time
import numpy as np
from sklearn.neighbors import KDTree
import pandas as pd

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from driverless_common.common import QOS_LATEST, angle, dist, fast_dist, wrap_to_pi

class PoseHistory(Node):
    csv_folder = OSPath("./QUTMS_Nav_Integration/csv_data")
    midline = False
    nav2 = True
    path = None
    start_time = 0.0
    last_pos = [0.0, 0.0]
    square_error = 0.0
    started = False

    def __init__(self):
        super().__init__("pose_history_node")

        # subscribe to topic
        self.create_subscription(
            PoseWithCovarianceStamped, "/slam/car_pose", self.pose_callback, 10
        )
        # subscribe to topic
        if self.midline:
            self.create_subscription(
                Path, "/planning/midline_path", self.path_callback, 10
            )
        else:
            self.create_subscription(
                Path, "/planning/global_path", self.path_callback, 10
            )

        self.get_logger().info("---CTE Initalised---")

    def path_callback(self, msg: Path):
        # receive path and convert to numpy array
        if self.path is not None:
            return

        self.get_logger().info(f"Spline Path Recieved - length: {len(msg.poses)}", once=True)
        self.path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # compare pose to closest point on path and add to cumulative error
        if self.path is None:
            return

        current_time = time.time()
        # get car position
        car_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # only start calculating once we have moved forward a bit
        # check if we have moved forward a bit
        if fast_dist(car_pos, self.last_pos) < 0.1 and not self.started:
            self.get_logger().info(f"Not started - {car_pos}", throttle_duration_sec=1)
            return
        elif fast_dist(car_pos, self.last_pos) >= 0.1 and not self.started:
            # only is entered once
            self.started = True
            self.start_time = current_time
            self.get_logger().info(f"Car Pose Recieved - {car_pos}", once=True)

        # get distance to path
        distance = self.get_distance_err(car_pos)
        # add to cumulative error
        self.square_error += distance

        # check if we have returned to the start line (some time after starting)
        if self.last_pos[0] <= 0 and car_pos[0] > 0 and current_time - self.start_time > 5:
            # get lap time
            lap_time = current_time - self.start_time
            # get final RMSE
            cross_track_error = sqrt(self.square_error / len(self.path))
            self.get_logger().info(f"Lap Completed {lap_time} - CTE: {cross_track_error}")

            # write the error to a csv file, append to the file if it already exists
            csv_file = self.csv_folder / "control_comparison.csv"
            if csv_file.exists():
                df = pd.read_csv(csv_file)
            else:
                df = pd.DataFrame(columns=["id", "path_type", "cross_track_error", "lap_time"])

            # get the id of the current run - whether it uses midline or not and number of runs with that config
            id = "regulatedPP" if self.nav2 else "customPP"
            # if there are multiple runs with the same config, add a number to the end of the id
            # check if "id" contains "id"
            if any(id in s for s in df["id"].values):
                # get the last number
                last_num = int(df["id"].values[-1].split("_")[-1])
                # increment it
                id = f"{id}_{last_num + 1}"
            else:
                id = f"{id}_0"                

            path_type = "midline" if self.midline else "optimal"
            df = df._append(
                {"id": id, "path_type": path_type, "cross_track_error": cross_track_error, "lap_time": lap_time}, ignore_index=True
            )

            df.to_csv(csv_file, index=False)

        self.last_pos = car_pos

    def get_distance_err(self, car_pos):
        # gets a perpendicular distance from the car to the path
        # uses two points on the path either side of the car to calculate the distance
        # returns a distance at the tangent

        path_xy = [[p[0], p[1]] for p in self.path]
        # get closest point on path
        kdtree = KDTree(path_xy)
        close_index = kdtree.query([[car_pos[0], car_pos[1]]], return_distance=False)[0][0]
        # get the closest point and the point before it
        closest_point = path_xy[close_index]
        # get the point before it
        if close_index == 0:
            # if the closest point is the first point, use the second point
            second_point = path_xy[1]
        else:
            second_point = path_xy[close_index - 1]

        # get the tangent of the path at the closest point
        tangent = angle(closest_point, second_point)
        # get the perpendicular distance from the car to the path
        distance = dist(car_pos, closest_point) * np.sin(wrap_to_pi(tangent - angle(car_pos, closest_point)))
        return distance

def main():
    # begin ros node
    rclpy.init()
    node = PoseHistory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
