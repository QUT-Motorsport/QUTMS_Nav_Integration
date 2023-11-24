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

def plot_system(csv_file, fig, axis):

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

    # Plot first subplot
    axis.scatter(gt_cones_x, gt_cones_y, c="k", label="Ground Truth Map")
    axis.scatter(blue_cones_x, blue_cones_y, c="b", label="Blue Cone Estimates")
    axis.scatter(yellow_cones_x, yellow_cones_y, c="y", label="Yellow Cone Estimates")
    axis.plot(blue_bounds_x, blue_bounds_y, c="b", label="Blue Bounds")
    axis.plot(yellow_bounds_x, yellow_bounds_y, c="y", label="Yellow Bounds")
    axis.plot(route_path_x, route_path_y, c="g", label="Planned Path")
    axis.plot(slam_poses_x, slam_poses_y, c="r", label="Pose Tracked", linestyle="dotted")
    axis.set_xlabel("X coordinates [m]")
    axis.set_xticks([-20, -10, 0, 10, 20, 30])

if __name__ == "__main__":
    csv_file2 = OSPath("./run_1695785482.csv")
    csv_file1 = OSPath("./run_1695803134.csv")

    # Create subplots
    fig, axs = plt.subplots(1, 2, figsize=(12, 5), sharey=True, sharex=True, gridspec_kw={'wspace': 0})
    # axs[0].axis("equal")
    # axs[1].axis("equal")

    plot_system(csv_file1, fig, axs[0])
    plot_system(csv_file2, fig, axs[1])

    axs[0].set_ylabel("Y coordinates [m]")
    axs[0].legend(loc="upper left")

    # Save or show the plot
    plt.tight_layout()
    plt.savefig("track_comparison.pdf", bbox_inches="tight", format="pdf")
    plt.show()
