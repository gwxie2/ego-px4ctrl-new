#!/usr/bin/env python3

import argparse
import glob
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


@dataclass
class RunSeries:
    label: str
    frame: pd.DataFrame


REQUIRED_COLUMNS = {
    "ros_time_sec",
    "drone_id",
    "x",
    "y",
    "z",
    "p_des_x",
    "p_des_y",
    "p_des_z",
    "v_des_x",
    "v_des_y",
    "v_des_z",
    "vx",
    "vy",
    "vz",
    "tracking_error_m",
    "jerk_norm",
    "planner_latency_ms",
}


def _load_csv(path: str) -> pd.DataFrame:
    frame = pd.read_csv(path)
    missing = sorted(REQUIRED_COLUMNS - set(frame.columns))
    if missing:
        raise ValueError(f"{path} is missing required columns: {missing}")
    for column in ["ros_time_sec", "x", "y", "z", "p_des_x", "p_des_y", "p_des_z", "vx", "vy", "vz", "v_des_x", "v_des_y", "v_des_z", "tracking_error_m", "jerk_norm", "planner_latency_ms"]:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.sort_values(["drone_id", "ros_time_sec"]).reset_index(drop=True)
    return frame


def _load_inputs(inputs: Sequence[str]) -> List[RunSeries]:
    runs: List[RunSeries] = []
    for item in inputs:
        matched = glob.glob(os.path.expanduser(item))
        if not matched and os.path.isfile(os.path.expanduser(item)):
            matched = [os.path.expanduser(item)]
        for path in sorted(matched):
            runs.append(RunSeries(label=os.path.splitext(os.path.basename(path))[0], frame=_load_csv(path)))
    if not runs:
        raise FileNotFoundError("no CSV inputs matched the requested patterns")
    return runs


def _ensure_output_dir(path: str):
    os.makedirs(path, exist_ok=True)


def _save_figure(fig: plt.Figure, output_dir: str, name: str):
    path = os.path.join(output_dir, name)
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


def _compute_ate(frame: pd.DataFrame) -> pd.Series:
    dx = frame["x"] - frame["p_des_x"]
    dy = frame["y"] - frame["p_des_y"]
    dz = frame["z"] - frame["p_des_z"]
    return pd.Series((dx * dx + dy * dy + dz * dz) ** 0.5, index=frame.index)


def _compute_speed(frame: pd.DataFrame) -> pd.Series:
    return pd.Series((frame["vx"] ** 2 + frame["vy"] ** 2 + frame["vz"] ** 2) ** 0.5, index=frame.index)


def _compute_desired_speed(frame: pd.DataFrame) -> pd.Series:
    return pd.Series((frame["v_des_x"] ** 2 + frame["v_des_y"] ** 2 + frame["v_des_z"] ** 2) ** 0.5, index=frame.index)


def plot_ate(runs: Sequence[RunSeries], output_dir: str):
    fig, ax = plt.subplots(figsize=(10, 5))
    for run in runs:
        for drone_id, group in run.frame.groupby("drone_id"):
            ate = _compute_ate(group)
            ax.plot(group["ros_time_sec"], ate, label=f"{run.label} / drone_{drone_id}")
    ax.set_title("ATE Over Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("ATE (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    return _save_figure(fig, output_dir, "ate_over_time.png")


def plot_velocity_profile(runs: Sequence[RunSeries], output_dir: str):
    fig, ax = plt.subplots(figsize=(10, 5))
    for run in runs:
        for drone_id, group in run.frame.groupby("drone_id"):
            desired = _compute_desired_speed(group)
            actual = _compute_speed(group)
            ax.plot(group["ros_time_sec"], desired, linestyle="--", linewidth=1.5, label=f"{run.label} drone_{drone_id} v_des")
            ax.plot(group["ros_time_sec"], actual, linewidth=1.8, label=f"{run.label} drone_{drone_id} v_actual")
    ax.axhline(10.0, color="red", linestyle=":", linewidth=1.5, label="10 m/s target")
    ax.set_title("Velocity Profile")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    return _save_figure(fig, output_dir, "velocity_profile.png")


def plot_planner_latency_histogram(runs: Sequence[RunSeries], output_dir: str):
    fig, ax = plt.subplots(figsize=(9, 5))
    all_latencies: List[float] = []
    for run in runs:
        series = pd.to_numeric(run.frame["planner_latency_ms"], errors="coerce").dropna().tolist()
        all_latencies.extend(series)
        if series:
            ax.hist(series, bins=30, alpha=0.35, label=run.label)
    ax.axvline(10.0, color="red", linestyle="--", linewidth=2, label="10 ms warning")
    ax.set_title("Planner Latency Histogram")
    ax.set_xlabel("Latency (ms)")
    ax.set_ylabel("Count")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    return _save_figure(fig, output_dir, "planner_latency_histogram.png")


def plot_jerk_integral(runs: Sequence[RunSeries], output_dir: str):
    fig, ax = plt.subplots(figsize=(8, 5))
    labels = []
    values = []
    for run in runs:
        total = 0.0
        valid = False
        for drone_id, group in run.frame.groupby("drone_id"):
            t = pd.to_numeric(group["ros_time_sec"], errors="coerce").fillna(method="ffill")
            jerk = pd.to_numeric(group["jerk_norm"], errors="coerce").fillna(0.0)
            if len(group) >= 2:
                dt = t.diff().fillna(0.0).clip(lower=0.0)
                total += float((jerk * dt).sum())
                valid = True
        if valid:
            labels.append(run.label)
            values.append(total)
    ax.bar(labels, values, color=["#4C78A8", "#F58518", "#54A24B", "#E45756"][: len(labels)])
    ax.set_title("Jerk Integral")
    ax.set_ylabel("Integral of |jerk|")
    ax.grid(True, axis="y", alpha=0.3)
    return _save_figure(fig, output_dir, "jerk_integral.png")


def main():
    parser = argparse.ArgumentParser(description="Offline performance plots for swarm benchmark CSVs")
    parser.add_argument("inputs", nargs="+", help="CSV files or glob patterns")
    parser.add_argument("--output-dir", default="./benchmark_plots", help="Directory for generated figures")
    args = parser.parse_args()

    runs = _load_inputs(args.inputs)
    _ensure_output_dir(args.output_dir)

    outputs = [
        plot_ate(runs, args.output_dir),
        plot_velocity_profile(runs, args.output_dir),
        plot_planner_latency_histogram(runs, args.output_dir),
        plot_jerk_integral(runs, args.output_dir),
    ]

    summary_path = os.path.join(args.output_dir, "summary.txt")
    with open(summary_path, "w", encoding="utf-8") as handle:
        handle.write("Generated plots:\n")
        for path in outputs:
            handle.write(f"- {path}\n")

    print("\n".join(outputs))
    print(summary_path)


if __name__ == "__main__":
    main()
