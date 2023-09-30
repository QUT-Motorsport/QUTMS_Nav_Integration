from math import sqrt
import time
from pathlib import Path as OSPath
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt

import pandas as pd
import rclpy
from driverless_msgs.msg import ConeDetectionStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from driverless_msgs.msg import Cone
from rclpy.node import Node
from driverless_common.common import QOS_LATEST, angle, dist, fast_dist, wrap_to_pi


NEW_STACK = True

class MapComparison(Node):
    csv_folder = OSPath("./QUTMS_Nav_Integration/csv_data")
    blue_cones = []
    yellow_cones = []
    slam_poses = []
    gt_cones = []
    gt_path = []
    blue_bounds = []
    yellow_bounds = []
    mid_path = []
    route_path = []
    start_time = 0.0
    last_pos = [0.0, 0.0]
    laps_complete = -1

    comparing = "system"  # or "system"

    def __init__(self) -> None:
        super().__init__("track_to_csv_node")

        # subscribe to topic
        self.create_subscription(
            PoseWithCovarianceStamped, "/slam/car_pose", self.slam_pose_callback, 10
        )
        self.create_subscription(
            ConeDetectionStamped, "/slam/global_map", self.slam_map_callback, 10
        )
        self.create_subscription(
            ConeDetectionStamped, "/ground_truth/global_map", self.gt_map_callback, 10
        )
        self.create_subscription(
            Path, "/planning/blue_bounds", self.blue_callback, 10
        )
        self.create_subscription(
            Path, "/planning/yellow_bounds", self.yellow_callback, 10
        )

        self.create_subscription(
            Path, "/planning/midline_path", self.path_callback, 10
        )
        # self.create_subscription(
        #     Path, "/planning/global_path", self.path_callback, 10
        # )

        self.get_logger().info("---SLAM accuracy node Initalised---")

    def gt_map_callback(self, msg: ConeDetectionStamped):
        if self.laps_complete == 0 and len(self.gt_cones) == 0:
            gt_map = []
            for cone in msg.cones:
                gt_map.append([cone.location.x, cone.location.y])
            self.gt_cones = gt_map
            self.get_logger().info(f"Ground Truth Map Recieved - {len(self.gt_cones)}", once=True)

    def slam_map_callback(self, msg: ConeDetectionStamped):
        if self.laps_complete == 1 and len(self.blue_cones) == 0 and len(self.yellow_cones) == 0:
            blue_cones = []
            yellow_cones = []
            for cone in msg.cones:
                if cone.color == Cone.BLUE:
                    blue_cones.append([cone.location.x, cone.location.y])
                elif cone.color == Cone.YELLOW:
                    yellow_cones.append([cone.location.x, cone.location.y])
            # overwrite the cones
            self.blue_cones = blue_cones
            self.yellow_cones = yellow_cones
            self.get_logger().info(f"SLAM Map Recieved - {len(self.blue_cones)} Blue, {len(self.yellow_cones)} Yellow", once=True)

    def path_callback(self, msg: Path):
        if self.laps_complete == 1 and len(self.route_path) == 0:
            for pose in msg.poses:
                self.route_path.append([pose.pose.position.x, pose.pose.position.y])
            self.get_logger().info(f"Optimal Path Recieved - {len(self.route_path)}", once=True)

    def blue_callback(self, msg: Path):
        blue_path = []
        if self.laps_complete == 0:
            for pose in msg.poses:
                blue_path.append([pose.pose.position.x, pose.pose.position.y])
            self.blue_bounds = blue_path
            self.get_logger().info(f"Blue Bounds Recieved - {len(self.blue_bounds)}", once=True)

    def yellow_callback(self, msg: Path):
        yellow_path = []
        if self.laps_complete == 0:
            for pose in msg.poses:
                yellow_path.append([pose.pose.position.x, pose.pose.position.y])
            self.yellow_bounds = yellow_path
            self.get_logger().info(f"Yellow Bounds Recieved - {len(self.yellow_bounds)}", once=True)

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        # convert to [x,y]
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]

        current_time = time.time()
        # only start calculating once we have moved forward a bit
        # check if we have moved forward a bit
        if fast_dist(pos, self.last_pos) < 0.1 and self.laps_complete == -1:
            self.get_logger().info(f"Not started - {pos}", throttle_duration_sec=1)
            return
        elif fast_dist(pos, self.last_pos) >= 0.1 and self.laps_complete == -1:
            # only is entered once
            self.laps_complete += 1
            self.start_time = current_time
            self.get_logger().info(f"Car Pose Recieved, Starting - {pos}", once=True)

        if self.laps_complete == 1:
            self.slam_poses.append(pos)

        # check if we have returned to the start line (some time after starting)
        if self.last_pos[0] <= 0 and pos[0] > 0 and current_time - self.start_time > 10:
            # crossed the start line
            self.get_logger().info(f"Crossed finish line - {pos}", once=True)
            self.laps_complete += 1
            self.start_time = current_time

            # plot at the end of the 2nd lap
            if self.laps_complete == 2:
                self.plot_system()
        
        self.last_pos = pos

    def plot_system(self, csv_file: OSPath = None):
        if csv_file is None:
            self.get_logger().info(
                f"Lengths - SLAM: {len(self.slam_poses)}, Blue: {len(self.blue_bounds)},\
                Yellow: {len(self.yellow_bounds)}, Poses: {len(self.slam_poses)}, \
                Path: {len(self.route_path)}", once=True)

            blue_cones_x, blue_cones_y = zip(*self.blue_cones)
            yellow_cones_x, yellow_cones_y = zip(*self.yellow_cones)
            blue_bounds_x, blue_bounds_y = zip(*self.blue_bounds)
            yellow_bounds_x, yellow_bounds_y = zip(*self.yellow_bounds)
            slam_poses_x, slam_poses_y = zip(*self.slam_poses)
            route_path_x, route_path_y = zip(*self.route_path)
            gt_cones_x, gt_cones_y = zip(*self.gt_cones)

            # not all columns have the same length, so we need to pad them
            # pad the shorter lists with nans
            max_len = max(len(blue_cones_x), len(yellow_cones_x), 
                          len(blue_bounds_x), len(yellow_bounds_x), 
                          len(slam_poses_x), len(route_path_x), 
                          len(gt_cones_x))
            blue_cones_x_df = list(blue_cones_x) + [float("nan")] * (max_len - len(blue_cones_x))
            blue_cones_y_df = list(blue_cones_y) + [float("nan")] * (max_len - len(blue_cones_y))
            yellow_cones_x_df = list(yellow_cones_x) + [float("nan")] * (max_len - len(yellow_cones_x))
            yellow_cones_y_df = list(yellow_cones_y) + [float("nan")] * (max_len - len(yellow_cones_y))
            blue_bounds_x_df = list(blue_bounds_x) + [float("nan")] * (max_len - len(blue_bounds_x))
            blue_bounds_y_df = list(blue_bounds_y) + [float("nan")] * (max_len - len(blue_bounds_y))
            yellow_bounds_x_df = list(yellow_bounds_x) + [float("nan")] * (max_len - len(yellow_bounds_x))
            yellow_bounds_y_df = list(yellow_bounds_y) + [float("nan")] * (max_len - len(yellow_bounds_y))
            slam_poses_x_df = list(slam_poses_x) + [float("nan")] * (max_len - len(slam_poses_x))
            slam_poses_y_df = list(slam_poses_y) + [float("nan")] * (max_len - len(slam_poses_y))
            route_path_x_df = list(route_path_x) + [float("nan")] * (max_len - len(route_path_x))
            route_path_y_df = list(route_path_y) + [float("nan")] * (max_len - len(route_path_y))
            gt_cones_x_df = list(gt_cones_x) + [float("nan")] * (max_len - len(gt_cones_x))
            gt_cones_y_df = list(gt_cones_y) + [float("nan")] * (max_len - len(gt_cones_y))

            # save all this data to csv
            # write the error to a csv file, append to the file if it already exists
            # only take time to the second
            current_time = time.time().__int__()
            csv_file = self.csv_folder / f"run_{current_time}.csv"
            df = pd.DataFrame(columns=["pose_x", "pose_x", "blue_cone_x", "blue_cone_y", 
                                       "yellow_cone_x", "yellow_cone_y", "route_path_x", "route_path_y",
                                       "blue_bounds_x", "blue_bounds_y", "yellow_bounds_x", "yellow_bounds_y"])
            df["pose_x"] = slam_poses_x_df
            df["pose_y"] = slam_poses_y_df
            df["blue_cone_x"] = blue_cones_x_df
            df["blue_cone_y"] = blue_cones_y_df
            df["yellow_cone_x"] = yellow_cones_x_df
            df["yellow_cone_y"] = yellow_cones_y_df
            df["route_path_x"] = route_path_x_df
            df["route_path_y"] = route_path_y_df
            df["blue_bounds_x"] = blue_bounds_x_df
            df["blue_bounds_y"] = blue_bounds_y_df
            df["yellow_bounds_x"] = yellow_bounds_x_df
            df["yellow_bounds_y"] = yellow_bounds_y_df
            df["gt_cones_x"] = gt_cones_x_df
            df["gt_cones_y"] = gt_cones_y_df

            df.to_csv(csv_file, index=False)
            self.get_logger().info(f"Saved to {csv_file}", once=True)

        else:
            # load the csv file
            df = pd.read_csv(csv_file)
            # get the data
            blue_cones_x = df["blue_cone_x"].values
            blue_cones_y = df["blue_cone_y"].values
            yellow_cones_x = df["yellow_cone_x"].values
            yellow_cones_y = df["yellow_cone_y"].values
            blue_bounds_x = df["blue_bounds_x"].values
            blue_bounds_y = df["blue_bounds_y"].values
            yellow_bounds_x = df["yellow_bounds_x"].values
            yellow_bounds_y = df["yellow_bounds_y"].values
            slam_poses_x = df["pose_x"].values
            slam_poses_y = df["pose_y"].values
            route_path_x = df["route_path_x"].values
            route_path_y = df["route_path_y"].values
            gt_cones_x = df["gt_cones_x"].values
            gt_cones_y = df["gt_cones_y"].values

        # plot the map, bounds, optimal line, and path on one plot
        # plot with matplotlib as figure
        plt.figure(figsize=(10, 10))
        # make axis equal
        plt.axis("equal")
        # plt.plot(list(zip(*self.mid_path), c="k", label="Midline Path", linestyle="dashed")
        plt.scatter(gt_cones_x, gt_cones_y, c="k", label="Ground Truth Map")
        plt.scatter(blue_cones_x, blue_cones_y, c="b", label="Blue Cone Estimates")
        plt.scatter(yellow_cones_x, yellow_cones_y, c="y", label="Yellow Cone Estimates")
        plt.plot(blue_bounds_x, blue_bounds_y, c="b", label="Blue Bounds")
        plt.plot(yellow_bounds_x, yellow_bounds_y, c="y", label="Yellow Bounds")
        plt.plot(slam_poses_x, slam_poses_y, c="r", label="Pose Tracked", linestyle="dotted")
        if NEW_STACK:
            plt.plot(route_path_x, route_path_y, c="g", label="Optimal Path")
        else:
            plt.plot(route_path_x, route_path_y, c="g", label="Midline Path")
        plt.yticks(fontsize=15)
        plt.xticks(fontsize=15)
        plt.ylabel("Track Width [m]", fontsize=15)
        plt.xlabel("Track Length [m]", fontsize=15)
        plt.legend(loc="upper left", fontsize=18)
        plt.tight_layout()
        # save as pdf
        if NEW_STACK:
            plt.savefig(f"new_stack.pdf", bbox_inches="tight", format="pdf")
        else:
            plt.savefig(f"old_stack.pdf", bbox_inches="tight", format="pdf")
        plt.show()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)

def main():
    # begin ros node
    rclpy.init()
    node = MapComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    if NEW_STACK:
        csv_file = OSPath("./QUTMS_Nav_Integration/csv_data/run_1695785482.csv")
    else:
        csv_file = OSPath("./QUTMS_Nav_Integration/csv_data/run_1695803134.csv")

    # only need to create the node so we can use the plot
    rclpy.init()
    node = MapComparison()
    node.plot_system(csv_file)
