
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


# ---------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------

BASE_DIR = Path(__file__).resolve().parent
RAW_DIR = BASE_DIR / "Raw Data"
OUTPUT_DIR = BASE_DIR / "Plots"

CONFIGS = [
    ("0.01", "0.01"),
    ("0.01", "0.5"),
    ("0.5", "0.01"),
    ("0.5", "0.5"),
]

# Note: Your filenames spell Spiral as "Sprial".
SPIRAL_PREFIX = "Sprial"


# ---------------------------------------------------------
# CSV READING
# ---------------------------------------------------------

def read_csv(filename):

    filepath = RAW_DIR / filename

    if not filepath.exists():
        print(f"WARNING: File not found: {filepath}")
        return None

    with open(filepath, "r", newline="") as file:

        reader = csv.reader(file)

        headers = [h.strip() for h in next(reader)]

        data = []

        for row in reader:

            if not row:
                continue

            row = [v.strip() for v in row]

            try:
                values = {
                    headers[i]: float(row[i])
                    for i in range(min(len(headers), len(row)))
                    if headers[i] != ""
                }

                data.append(values)

            except (ValueError, IndexError):
                continue

    if not data:
        print(f"WARNING: No valid data in {filename}")
        return None

    return data


def get_time(data):

    stamps = [row["stamp"] for row in data]

    # ROS timestamps are in nanoseconds.
    first_stamp = stamps[0]

    return [(stamp - first_stamp) / 1e9 for stamp in stamps]


def get_column(data, name):

    return [row[name] for row in data]


# ---------------------------------------------------------
# PLOT FORMATTING
# ---------------------------------------------------------

def format_axis(ax, title, ylabel):

    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)

    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend()


def save_figure(fig, filename):

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    fig.tight_layout()

    filepath = OUTPUT_DIR / filename

    fig.savefig(filepath, dpi=300, bbox_inches="tight")

    print(f"Saved: {filepath}")


# ---------------------------------------------------------
# INDIVIDUAL ROBOT POSE PLOTS
# ---------------------------------------------------------

def plot_robot_pose(filename, motion, q, r):

    data = read_csv(filename)

    if data is None:
        return

    required = [
        "stamp",
        "imu_ax", "imu_ay",
        "kf_ax", "kf_ay",
        "kf_vx", "kf_w",
        "kf_x", "kf_y"
    ]

    if not all(key in data[0] for key in required):
        print(f"WARNING: Unexpected columns in {filename}")
        return

    t = get_time(data)

    fig, axes = plt.subplots(2, 3, figsize=(17, 9))

    fig.suptitle(
        f"{motion} Motion: EKF Results (Q={q}, R={r})",
        fontsize=16
    )

    # 1. XY trajectory
    ax = axes[0, 0]

    ax.plot(
        get_column(data, "kf_x"),
        get_column(data, "kf_y"),
        label="EKF estimated trajectory"
    )

    ax.set_title("Robot Trajectory")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend()

    # 2. X acceleration
    ax = axes[0, 1]

    ax.plot(
        t,
        get_column(data, "imu_ax"),
        label="IMU measured ax",
        linestyle="--"
    )

    ax.plot(
        t,
        get_column(data, "kf_ax"),
        label="EKF estimated ax"
    )

    format_axis(ax, "X Acceleration", "Acceleration (m/s²)")

    # 3. Y acceleration
    ax = axes[0, 2]

    ax.plot(
        t,
        get_column(data, "imu_ay"),
        label="IMU measured ay",
        linestyle="--"
    )

    ax.plot(
        t,
        get_column(data, "kf_ay"),
        label="EKF estimated ay"
    )

    format_axis(ax, "Y Acceleration", "Acceleration (m/s²)")

    # 4. Estimated X position
    ax = axes[1, 0]

    ax.plot(
        t,
        get_column(data, "kf_x"),
        label="EKF estimated x"
    )

    format_axis(ax, "Estimated X Position", "Position (m)")

    # 5. Estimated Y position
    ax = axes[1, 1]

    ax.plot(
        t,
        get_column(data, "kf_y"),
        label="EKF estimated y"
    )

    format_axis(ax, "Estimated Y Position", "Position (m)")

    # 6. Estimated linear and angular velocity
    ax = axes[1, 2]

    ax.plot(
        t,
        get_column(data, "kf_vx"),
        label="Estimated linear velocity"
    )

    ax.set_title("Estimated Linear Velocity")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend()

    save_figure(
        fig,
        f"{motion}_Q{q}_R{r}_robotPose.png"
    )

    plt.close(fig)

    # Separate angular velocity figure
    fig, ax = plt.subplots(figsize=(11, 5))

    ax.plot(
        t,
        get_column(data, "kf_w"),
        label="EKF estimated angular velocity"
    )

    format_axis(
        ax,
        f"{motion}: Estimated Angular Velocity (Q={q}, R={r})",
        "Angular Velocity (rad/s)"
    )

    save_figure(
        fig,
        f"{motion}_Q{q}_R{r}_angular_velocity.png"
    )

    plt.close(fig)


# ---------------------------------------------------------
# CONTROLLER ERROR PLOTS
# ---------------------------------------------------------

def plot_controller_error(filename, motion, q, r, controller):

    data = read_csv(filename)

    if data is None:
        return

    t = get_time(data)

    fig, ax = plt.subplots(figsize=(11, 5))

    for column in ["e", "e_dot", "e_int"]:

        if column in data[0]:

            ax.plot(
                t,
                get_column(data, column),
                label=column
            )

    format_axis(
        ax,
        f"{motion}: {controller.capitalize()} Controller Errors "
        f"(Q={q}, R={r})",
        "Error / Error Derivative / Integrated Error"
    )

    save_figure(
        fig,
        f"{motion}_Q{q}_R{r}_{controller}_errors.png"
    )

    plt.close(fig)


# ---------------------------------------------------------
# SPIRAL Q/R COMPARISON
# ---------------------------------------------------------

def plot_spiral_comparison():

    datasets = []

    for q, r in CONFIGS:

        filename = (
            f"{SPIRAL_PREFIX}_Q{q}_R{r}_robotPose.csv"
        )

        data = read_csv(filename)

        if data is not None:
            datasets.append((q, r, data))

    if not datasets:
        print("No spiral datasets found.")
        return

    # Compare EKF estimated trajectories
    fig, ax = plt.subplots(figsize=(9, 8))

    for q, r, data in datasets:

        ax.plot(
            get_column(data, "kf_x"),
            get_column(data, "kf_y"),
            label=f"Q={q}, R={r}"
        )

    ax.set_title("Spiral Motion: EKF Trajectory Comparison")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.axis("equal")
    ax.legend()

    save_figure(fig, "Spiral_QR_Trajectory_Comparison.png")
    plt.close(fig)

    # Compare individual estimated states
    states = [
        ("kf_x", "X Position", "Position (m)"),
        ("kf_y", "Y Position", "Position (m)"),
        ("kf_vx", "Linear Velocity", "Velocity (m/s)"),
        ("kf_w", "Angular Velocity", "Angular Velocity (rad/s)")
    ]

    for column, title, ylabel in states:

        fig, ax = plt.subplots(figsize=(12, 5))

        for q, r, data in datasets:

            t = get_time(data)

            ax.plot(
                t,
                get_column(data, column),
                label=f"Q={q}, R={r}"
            )

        format_axis(
            ax,
            f"Spiral Motion: {title} Comparison",
            ylabel
        )

        save_figure(
            fig,
            f"Spiral_QR_{column}_Comparison.png"
        )

        plt.close(fig)


# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------

def main():

    parser = argparse.ArgumentParser(
        description="MTE544 EKF plotting script"
    )

    parser.add_argument(
        "--type",
        choices=["point", "spiral", "all"],
        default="all",
        help="Select which motion to plot"
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures instead of only saving them"
    )

    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print("\nMTE544 EKF Plotting")
    print("-------------------")

    if args.type in ["point", "all"]:

        for q, r in CONFIGS:

            prefix = f"Point_Q{q}_R{r}"

            print(f"\nProcessing {prefix}")

            plot_robot_pose(
                f"{prefix}_robotPose.csv",
                "Point",
                q,
                r
            )

            plot_controller_error(
                f"{prefix}_linear.csv",
                "Point",
                q,
                r,
                "linear"
            )

            plot_controller_error(
                f"{prefix}_angular.csv",
                "Point",
                q,
                r,
                "angular"
            )

    if args.type in ["spiral", "all"]:

        for q, r in CONFIGS:

            prefix = f"{SPIRAL_PREFIX}_Q{q}_R{r}"

            print(f"\nProcessing {prefix}")

            plot_robot_pose(
                f"{prefix}_robotPose.csv",
                "Spiral",
                q,
                r
            )

        print("\nGenerating combined spiral comparisons...")

        plot_spiral_comparison()

    print("\nPlot generation complete.")
    print(f"Figures saved in: {OUTPUT_DIR}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
