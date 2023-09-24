from math import sqrt
from pathlib import Path as OSPath

from transforms3d.euler import quat2euler

import pandas as pd
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

USE_ODOM = False


class RacingLineComparison(Node):
    csv_folder = OSPath("./QUTMS_Nav_Integration/csv_data")

    def __init__(self) -> None:
        super().__init__("racing_line_comparison_node")

        # subscribe to topic
        self.create_subscription(
            Path, "/planning/global_path", self.path_callback, 10
        )

        self.get_logger().info("---Path evaluation initialised---")

    def path_callback(self, msg: Path):
        # get path
        path: [PoseStamped] = msg.poses

        # we want to sum the distance between each pose to get total track length
        # and sum the angle between each pose to get a cumulative curvature.
        # the shorter the distance and the lower the curvature, the better.
        total_distance = 0
        total_curvature = 0
        for i in range(len(path) - 1):
            current_pose: PoseStamped = path[i]
            next_pose: PoseStamped = path[i + 1]

            # get the distance between the two poses
            distance = sqrt(
                (current_pose.pose.position.x - next_pose.pose.position.x) ** 2
                + (current_pose.pose.position.y - next_pose.pose.position.y) ** 2
            )
            total_distance += distance

            # convert from quaternion to euler
            current_euler = quat2euler(
                [
                    current_pose.pose.orientation.w,
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                ]
            )
            next_euler = quat2euler(
                [
                    next_pose.pose.orientation.w,
                    next_pose.pose.orientation.x,
                    next_pose.pose.orientation.y,
                    next_pose.pose.orientation.z,
                ]
            )
            
            # get the angle between the two poses
            angle = abs(current_euler[2] - next_euler[2])
            total_curvature += angle

        self.get_logger().info(f"Total distance: {total_distance}")
        self.get_logger().info(f"Total curvature: {total_curvature}")

        # write the error to a csv file, append to the file if it already exists
        csv_file = self.csv_folder / "path_comparison.csv"
        if csv_file.exists():
            df = pd.read_csv(csv_file)
        else:
            df = pd.DataFrame(columns=["id", "sum_dist", "sum_angle"])

        # get the id of the current run - whether it uses odom or not and number of runs with that config
        id = f"path_{len(df[df['id'] == id])}"

        df.to_csv(csv_file, index=False)

        # exit the node
        self.get_logger().info("Finished writing to csv")
        exit(0)


def main():
    # begin ros node
    rclpy.init()
    node = RacingLineComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
