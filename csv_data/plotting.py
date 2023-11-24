from pathlib import Path as OSPath
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.sans-serif": ["Helvetica"],
    "text.latex.preamble": r"\usepackage{amsmath}",
})

sns.set_context("paper", font_scale=1.2)

def plot_system(csv_file):

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
    plt.figure(figsize=(4, 4))
    # make axis equal
    plt.axis("equal")
    # plt.plot(list(zip(*self.mid_path), c="k", label="Midline Path", linestyle="dashed")
    plt.scatter(gt_cones_x, gt_cones_y, c="k", label="Ground Truth Map")
    plt.scatter(blue_cones_x, blue_cones_y, c="b", label="Blue Cone Estimates")
    plt.scatter(yellow_cones_x, yellow_cones_y, c="y", label="Yellow Cone Estimates")
    plt.plot(blue_bounds_x, blue_bounds_y, c="b", label="Blue Bounds")
    plt.plot(yellow_bounds_x, yellow_bounds_y, c="y", label="Yellow Bounds")
    plt.plot(slam_poses_x, slam_poses_y, c="r", label="Pose Tracked", linestyle="dotted")
    plt.plot(route_path_x, route_path_y, c="g", label="Planned Path")

    plt.ylabel("Track Width [m]")
    plt.xlabel("Track Length [m]")
    plt.legend(loc="upper left")
    plt.xticks([-20, -10, 0, 10, 20, 30])
    plt.tight_layout()
    # save as pdf
    if NEW_STACK:
        plt.savefig(f"new_stack.pdf", bbox_inches="tight", format="pdf")
    else:
        plt.savefig(f"old_stack.pdf", bbox_inches="tight", format="pdf")
    plt.show()


if __name__ == "__main__":
    NEW_STACK = True

    if NEW_STACK:
        csv_file = OSPath("./run_1695785482.csv")
    else:
        csv_file = OSPath("./run_1695803134.csv")

    plot_system(csv_file)