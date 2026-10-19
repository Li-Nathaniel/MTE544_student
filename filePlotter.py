
import matplotlib.pyplot as plt
import numpy as np
import ast
import argparse


def plot_laserscan(filename, row=0):

    # Read LaserScan data
    with open(filename, "r") as file:
        lines = file.readlines()

    # Skip header and empty lines
    data = [line.strip() for line in lines[1:] if line.strip()]

    if row < 0 or row >= len(data):
        raise ValueError(f"Row must be between 0 and {len(data)-1}")

    # Extract one row of LaserScan data
    line = data[row]

    # Find the end of the ranges array
    end = line.index("]") + 1

    # Parse ranges array
    ranges = np.array(
        ast.literal_eval(
            line[:end].replace("inf", "float('inf')")
        ) if False else
        [float(x) for x in line[1:end-1].split(",")],
        dtype=float
    )

    # Extract angle increment and timestamp
    remaining = line[end:].strip().lstrip(",").split(",")

    angle_increment = float(remaining[0])
    stamp = int(remaining[1])

    # Generate angles for each range measurement
    # Assumes angle_min = 0 radians
    angles = np.arange(len(ranges)) * angle_increment

    # Remove NaN, Inf, and invalid ranges
    valid = np.isfinite(ranges) & (ranges > 0)

    ranges = ranges[valid]
    angles = angles[valid]

    # Convert polar coordinates to Cartesian coordinates
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot LaserScan points
    ax.scatter(x, y, s=5, label="LaserScan Data")

    # Mark robot position
    ax.scatter(0, 0, color="red", s=100,
               marker="x", label="Robot Position")

    # Set title and axis labels
    ax.set_title("Spiral Maneuver LaserScan Data at Time = 0 s",
             fontsize=18, fontweight="bold", pad=20)

    ax.set_xlabel("X Position (m)", fontsize=14, labelpad=10)
    ax.set_ylabel("Y Position (m)", fontsize=14, labelpad=10)

    # Formatting
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(axis="both", labelsize=12)
    ax.legend(fontsize=12)
    ax.grid(True)

    # Ensure labels are visible
    fig.tight_layout(pad=2.0)

    plt.show()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Plot LaserScan data"
    )

    parser.add_argument(
        "--files", nargs="+", required=True,
        help="List of files to process"
    )

    parser.add_argument(
        "--row", type=int, default=0,
        help="LaserScan row to plot (default: 0)"
    )

    args = parser.parse_args()

    print("Plotting files:", args.files)

    for filename in args.files:
        plot_laserscan(filename, args.row)
