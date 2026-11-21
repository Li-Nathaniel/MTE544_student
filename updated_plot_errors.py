
import argparse
import csv
from pathlib import Path

import numpy as np
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

# Note: The filenames spell Spiral as "Sprial".
SPIRAL_PREFIX = "Sprial"


# ---------------------------------------------------------
# CSV READING
# ---------------------------------------------------------

def read_csv(filename):
    """
    Read a CSV file into a list of dictionaries.

    Preserves ROS nanosecond timestamps as integers to avoid
    losing precision from conversion to floating point.
    """

    filepath = RAW_DIR / filename

    if not filepath.exists():
        print(f"WARNING: File not found: {filepath}")
        return None

    data = []

    with open(filepath, "r", newline="") as file:
        reader = csv.reader(file)

        try:
            headers = [h.strip() for h in next(reader)]
        except StopIteration:
            print(f"WARNING: Empty file: {filename}")
            return None

        for row in reader:
            if not row:
                continue

            row = [v.strip() for v in row]
            values = {}

            try:
                for i in range(min(len(headers), len(row))):
                    header = headers[i]
                    value = row[i]

                    # Ignore trailing empty CSV columns.
                    if not header or not value:
                        continue

                    if header == "stamp":
                        values[header] = int(value)
                    else:
                        values[header] = float(value)

                if values:
                    data.append(values)

            except (ValueError, IndexError):
                continue

    if not data:
        print(f"WARNING: No valid data in {filename}")
        return None

    return data


def get_time(data):
    """Convert nanosecond ROS timestamps to elapsed seconds."""

    stamps = np.array(
        [row["stamp"] for row in data],
        dtype=np.int64
    )

    # Subtract while still integers to preserve timestamp precision.
    return (stamps - stamps[0]).astype(np.float64) / 1e9


def get_column(data, name):
    return [row[name] for row in data]


# ---------------------------------------------------------
# ODOMETRY RECONSTRUCTION
# ---------------------------------------------------------

def reconstruct_odometry(data):
    """
    Reconstruct robot x, y, and heading by integrating the
    EKF linear and angular velocity estimates.

    Initial conditions:
        x = 0 m
        y = 0 m
        theta = 0 rad

    Integration:
        - Trapezoidal integration for angular velocity.
        - Trapezoidal integration for linear velocity.
        - Midpoint heading for position integration.
        - Actual timestamp intervals.

    Note:
        This trajectory is reconstructed from EKF velocity
        estimates, not from independently logged odometry.
    """

    stamps = np.array(
        [row["stamp"] for row in data],
        dtype=np.int64
    )

    time = (stamps - stamps[0]).astype(np.float64) / 1e9
    dt = np.diff(time)

    if np.any(dt < 0):
        raise ValueError("Timestamps are not in chronological order.")

    vx = np.array(get_column(data, "kf_vx"), dtype=float)
    omega = np.array(get_column(data, "kf_w"), dtype=float)

    n = len(time)

    x = np.zeros(n)
    y = np.zeros(n)
    theta = np.zeros(n)

    for i in range(1, n):

        # Average angular velocity over the interval.
        omega_avg = 0.5 * (omega[i - 1] + omega[i])

        # Integrate heading.
        theta[i] = theta[i - 1] + omega_avg * dt[i - 1]

        # Average linear velocity over the interval.
        vx_avg = 0.5 * (vx[i - 1] + vx[i])

        # Midpoint heading.
        theta_mid = 0.5 * (theta[i - 1] + theta[i])

        # Integrate position.
        x[i] = (
            x[i - 1]
            + vx_avg * np.cos(theta_mid) * dt[i - 1]
        )

        y[i] = (
            y[i - 1]
            + vx_avg * np.sin(theta_mid) * dt[i - 1]
        )

    return time, x, y, theta


def save_reconstructed_odometry(filename, data, time, x, y, theta):
    """Save reconstructed odometry data to a separate CSV."""

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    output_file = (
        OUTPUT_DIR /
        f"{filename.replace('_robotPose.csv', '_reconstructedOdometry.csv')}"
    )

    columns = [
        "time_s",
        "reconstructed_x",
        "reconstructed_y",
        "reconstructed_theta",
        "kf_x",
        "kf_y",
        "kf_vx",
        "kf_w"
    ]

    with open(output_file, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(columns)

        for i in range(len(time)):
            writer.writerow([
                time[i],
                x[i],
                y[i],
                theta[i],
                data[i]["kf_x"],
                data[i]["kf_y"],
                data[i]["kf_vx"],
                data[i]["kf_w"]
            ])

    print(f"Saved reconstructed odometry: {output_file}")


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

    # Original timestamps and EKF data.
    t = get_time(data)

    # Reconstruct odometry from EKF velocity estimates.
    odom_time, odom_x, odom_y, odom_theta = reconstruct_odometry(data)

    # Save reconstructed data for further analysis.
    save_reconstructed_odometry(
        filename,
        data,
        odom_time,
        odom_x,
        odom_y,
        odom_theta
    )

    fig, axes = plt.subplots(2, 3, figsize=(17, 9))

    fig.suptitle(
        f"{motion} Motion: EKF Results (Q={q}, R={r})",
        fontsize=16
    )

    # -----------------------------------------------------
    # 1. XY trajectory
    # -----------------------------------------------------

    ax = axes[0, 0]

    ax.plot(
        get_column(data, "kf_x"),
        get_column(data, "kf_y"),
        label="EKF estimated trajectory",
        linewidth=2
    )

    ax.plot(
        odom_x,
        odom_y,
        label="Measured Odometry Trajectory",
        linestyle="--",
        linewidth=2
    )

    ax.set_title("Robot Trajectory")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.axis("equal")
    ax.legend()

    # -----------------------------------------------------
    # 2. X acceleration
    # -----------------------------------------------------

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

    # -----------------------------------------------------
    # 3. Y acceleration
    # -----------------------------------------------------

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

    # -----------------------------------------------------
    # 4. Estimated X position
    # -----------------------------------------------------

    ax = axes[1, 0]

    ax.plot(
        t,
        get_column(data, "kf_x"),
        label="EKF estimated x"
    )

    format_axis(ax, "Estimated X Position", "Position (m)")

    # -----------------------------------------------------
    # 5. Estimated Y position
    # -----------------------------------------------------

    ax = axes[1, 1]

    ax.plot(
        t,
        get_column(data, "kf_y"),
        label="EKF estimated y"
    )

    format_axis(ax, "Estimated Y Position", "Position (m)")

    # -----------------------------------------------------
    # 6. Estimated linear velocity
    # -----------------------------------------------------

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

    # -----------------------------------------------------
    # Separate angular velocity figure
    # -----------------------------------------------------

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

    # -----------------------------------------------------
    # Reconstructed heading figure
    # -----------------------------------------------------

    fig, ax = plt.subplots(figsize=(11, 5))

    ax.plot(
        odom_time,
        odom_theta,
        label="Reconstructed heading"
    )

    format_axis(
        ax,
        f"{motion}: Reconstructed Heading (Q={q}, R={r})",
        "Heading (rad)"
    )

    save_figure(
        fig,
        f"{motion}_Q{q}_R{r}_reconstructed_heading.png"
    )

    plt.close(fig)

    print(
        f"Reconstructed final pose for {filename}: "
        f"x={odom_x[-1]:.3f} m, "
        f"y={odom_y[-1]:.3f} m, "
        f"theta={odom_theta[-1]:.3f} rad"
    )


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

    # -----------------------------------------------------
    # Compare EKF estimated trajectories
    # -----------------------------------------------------

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

    # -----------------------------------------------------
    # Compare individual estimated states
    # -----------------------------------------------------

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
