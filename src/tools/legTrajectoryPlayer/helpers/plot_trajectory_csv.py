#!/usr/bin/env python3

import argparse
import csv
import math
import sys
from pathlib import Path


LEG_JOINTS = {
    "left": [
        "l_hip_pitch",
        "l_hip_roll",
        "l_hip_yaw",
        "l_knee",
        "l_ankle_pitch",
        "l_ankle_roll",
    ],
    "right": [
        "r_hip_pitch",
        "r_hip_roll",
        "r_hip_yaw",
        "r_knee",
        "r_ankle_pitch",
        "r_ankle_roll",
    ],
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot legTrajectoryPlayer CSV dumps for a visual feasibility check."
    )
    parser.add_argument("csv_file", type=Path, help="CSV file generated with legTrajectoryPlayer --dump")
    parser.add_argument("--legs", nargs="+", choices=["left", "right"], default=["left", "right"])
    parser.add_argument(
        "--joints",
        nargs="+",
        help="Joint names to plot, for example l_knee r_knee. Overrides --legs.",
    )
    parser.add_argument("--position-limit", type=float, help="Draw +/- position limit in degrees")
    parser.add_argument("--velocity-limit", type=float, help="Draw +/- velocity limit in degrees/s")
    parser.add_argument("--output", type=Path, help="Save the figure to this path instead of only showing it")
    parser.add_argument("--no-show", action="store_true", help="Do not open an interactive plot window")
    return parser.parse_args()


def read_csv(csv_file):
    with csv_file.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or "time" not in reader.fieldnames:
            raise ValueError("CSV does not contain a 'time' column")

        rows = list(reader)
        if not rows:
            raise ValueError("CSV is empty")

    time = [float(row["time"]) for row in rows]
    return reader.fieldnames, rows, time


def selected_joints(fieldnames, legs, explicit_joints):
    available = []
    for name in fieldnames:
        if name.endswith("_pos_deg"):
            available.append(name[: -len("_pos_deg")])

    if explicit_joints:
        missing = [joint for joint in explicit_joints if joint not in available]
        if missing:
            raise ValueError("Unknown joint(s): " + ", ".join(missing))
        return explicit_joints

    wanted = []
    for leg in legs:
        wanted.extend(LEG_JOINTS[leg])
    return [joint for joint in wanted if joint in available]


def series(rows, joint, suffix):
    key = f"{joint}_{suffix}"
    return [float(row[key]) if row[key] else math.nan for row in rows]


def print_summary(rows, joints):
    print("Trajectory summary")
    for joint in joints:
        pos = series(rows, joint, "pos_deg")
        vel = series(rows, joint, "vel_deg_s")
        print(
            f"  {joint:16s} "
            f"pos [{min(pos):8.3f}, {max(pos):8.3f}] deg   "
            f"vel [{min(vel):8.3f}, {max(vel):8.3f}] deg/s"
        )


def main():
    args = parse_args()
    try:
        fieldnames, rows, time = read_csv(args.csv_file)
        joints = selected_joints(fieldnames, args.legs, args.joints)
        if not joints:
            raise ValueError("No matching joints found in the CSV")
    except (OSError, ValueError) as exc:
        print(f"plot_trajectory_csv: {exc}", file=sys.stderr)
        return 1

    print_summary(rows, joints)

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("plot_trajectory_csv: matplotlib is required for plotting", file=sys.stderr)
        return 1

    fig, (pos_ax, vel_ax) = plt.subplots(2, 1, sharex=True, figsize=(13, 8))
    for joint in joints:
        pos_ax.plot(time, series(rows, joint, "pos_deg"), label=joint)
        vel_ax.plot(time, series(rows, joint, "vel_deg_s"), label=joint)

    if args.position_limit is not None:
        pos_ax.axhline(args.position_limit, color="black", linestyle="--", linewidth=0.8)
        pos_ax.axhline(-args.position_limit, color="black", linestyle="--", linewidth=0.8)
    if args.velocity_limit is not None:
        vel_ax.axhline(args.velocity_limit, color="black", linestyle="--", linewidth=0.8)
        vel_ax.axhline(-args.velocity_limit, color="black", linestyle="--", linewidth=0.8)

    pos_ax.set_title(args.csv_file.name)
    pos_ax.set_ylabel("position [deg]")
    pos_ax.grid(True, alpha=0.3)
    pos_ax.legend(loc="upper right", ncol=2, fontsize="small")

    vel_ax.set_ylabel("velocity [deg/s]")
    vel_ax.set_xlabel("time [s]")
    vel_ax.grid(True, alpha=0.3)
    vel_ax.legend(loc="upper right", ncol=2, fontsize="small")

    fig.tight_layout()
    if args.output:
        fig.savefig(args.output, dpi=150)
        print(f"Saved plot to {args.output}")
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
