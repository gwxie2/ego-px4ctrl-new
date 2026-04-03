#!/usr/bin/env python3

import argparse
import glob
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd


MAIN_REQUIRED_COLUMNS = {
    "ros_time_sec",
    "drone_id",
    "tracking_error_m",
    "speed_mps",
    "goal_distance_m",
    "safety_margin_m",
    "control_lag_ms",
    "actuator_ratio",
    "planner_latency_ms",
    "planner_replan_interval_ms",
    "planner_iter_count",
}


@dataclass
class RunData:
    label: str
    frame: pd.DataFrame
    replan_frame: pd.DataFrame
    summary: Optional[dict]


def _p95(series: pd.Series) -> float:
    values = pd.to_numeric(series, errors="coerce").dropna().sort_values()
    if values.empty:
        return float("nan")
    index = max(0, min(len(values) - 1, int(math.ceil(0.95 * len(values))) - 1))
    return float(values.iloc[index])


def _correlation(xs: pd.Series, ys: pd.Series) -> float:
    pair = pd.DataFrame({"x": pd.to_numeric(xs, errors="coerce"), "y": pd.to_numeric(ys, errors="coerce")}).dropna()
    if len(pair) < 2:
        return float("nan")
    return float(pair["x"].corr(pair["y"]))


def _compute_stress_score(replan_frame: pd.DataFrame) -> float:
    if replan_frame.empty:
        return 0.0
    latency_p95 = _p95(replan_frame["time_total_ms"])
    interval_mean = float(pd.to_numeric(replan_frame["replan_interval_ms"], errors="coerce").dropna().mean())
    corr = _correlation(replan_frame["iter_count"], replan_frame["time_total_ms"])
    failure_rate = 1.0 - float(pd.to_numeric(replan_frame["success"], errors="coerce").fillna(0.0).mean())
    parts = []
    if math.isfinite(latency_p95):
        parts.append(min(1.0, latency_p95 / 80.0))
    if math.isfinite(interval_mean) and interval_mean > 1e-6:
        parts.append(min(1.0, 600.0 / max(interval_mean, 1.0)))
    if math.isfinite(corr):
        parts.append(min(1.0, abs(corr)))
    parts.append(min(1.0, failure_rate * 2.0))
    return float(sum(parts) / len(parts)) if parts else 0.0


def _predict_failure(tracking_error_p95: float, control_lag_p95: float, latency_p95: float, actuator_mean: float, actuator_max: float, max_speed: float, vmax: float, stress_score: float) -> str:
    predictions = []
    if (math.isfinite(tracking_error_p95) and tracking_error_p95 > max(1.0, 0.15 * vmax)) or (math.isfinite(control_lag_p95) and control_lag_p95 > 200.0):
        predictions.append("定位漂移")
    if (math.isfinite(latency_p95) and latency_p95 > 50.0) or stress_score > 0.75:
        predictions.append("计算超时")
    if (math.isfinite(actuator_mean) and actuator_mean > 0.7) or (math.isfinite(actuator_max) and actuator_max > 0.9) or (math.isfinite(max_speed) and max_speed > 1.15 * vmax):
        predictions.append("动力学过载")
    return "正常" if not predictions else "/".join(predictions)


def _discover_session_files(item: str) -> Tuple[List[str], List[str], Optional[str]]:
    if os.path.isdir(item):
        main_csvs = sorted(glob.glob(os.path.join(item, "drone_*_*.csv")))
        main_csvs = [path for path in main_csvs if not path.endswith("_replan.csv") and not path.endswith("_event.csv")]
        replan_csvs = sorted(glob.glob(os.path.join(item, "drone_*_*_replan.csv")))
        summary_matches = sorted(glob.glob(os.path.join(item, "*_summary.json")))
        return main_csvs, replan_csvs, summary_matches[0] if summary_matches else None
    expanded = sorted(glob.glob(os.path.expanduser(item)))
    main_csvs = [path for path in expanded if path.endswith(".csv") and not path.endswith("_replan.csv") and not path.endswith("_event.csv")]
    replan_csvs = [path for path in expanded if path.endswith("_replan.csv")]
    return main_csvs, replan_csvs, None


def _load_frame(path: str, required_columns: Sequence[str]) -> pd.DataFrame:
    frame = pd.read_csv(path)
    missing = sorted(set(required_columns) - set(frame.columns))
    if missing:
        raise ValueError(f"{path} is missing required columns: {missing}")
    return frame


def _load_runs(inputs: Sequence[str]) -> List[RunData]:
    runs: List[RunData] = []
    for item in inputs:
        main_csvs, replan_csvs, summary_path = _discover_session_files(item)
        if not main_csvs:
            continue
        main_frames = []
        for path in main_csvs:
            frame = _load_frame(path, MAIN_REQUIRED_COLUMNS)
            frame["source_file"] = os.path.basename(path)
            main_frames.append(frame)
        replan_frames = []
        for path in replan_csvs:
            frame = pd.read_csv(path)
            frame["source_file"] = os.path.basename(path)
            replan_frames.append(frame)
        label = os.path.basename(os.path.dirname(main_csvs[0])) or os.path.splitext(os.path.basename(main_csvs[0]))[0]
        summary = None
        if summary_path and os.path.isfile(summary_path):
            with open(summary_path, "r", encoding="utf-8") as handle:
                summary = json.load(handle)
        runs.append(
            RunData(
                label=label,
                frame=pd.concat(main_frames, ignore_index=True),
                replan_frame=pd.concat(replan_frames, ignore_index=True) if replan_frames else pd.DataFrame(),
                summary=summary,
            )
        )
    if not runs:
        raise FileNotFoundError("no benchmark CSV inputs matched the provided paths")
    return runs


def _ensure_output_dir(path: str):
    os.makedirs(path, exist_ok=True)


def _save_png(fig: plt.Figure, output_dir: str, name: str) -> str:
    path = os.path.join(output_dir, name)
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    return path


def plot_safety_margin(runs: Sequence[RunData], output_dir: str) -> Tuple[plt.Figure, str]:
    fig, ax = plt.subplots(figsize=(10, 5))
    for run in runs:
        for drone_id, group in run.frame.groupby("drone_id"):
            group = group.sort_values("ros_time_sec")
            ax.plot(group["ros_time_sec"], pd.to_numeric(group["safety_margin_m"], errors="coerce"), label=f"{run.label} / drone_{drone_id}")
    ax.axhline(0.5, color="#C0392B", linestyle="--", linewidth=1.5, label="0.5 m threshold")
    ax.set_title("Swarm Safety Margin")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Safety Margin (m)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    return fig, _save_png(fig, output_dir, "advanced_safety_margin.png")


def plot_actuator_ratio(runs: Sequence[RunData], output_dir: str) -> Tuple[plt.Figure, str]:
    fig, ax = plt.subplots(figsize=(10, 5))
    for run in runs:
        for drone_id, group in run.frame.groupby("drone_id"):
            group = group.sort_values("ros_time_sec")
            ax.plot(group["ros_time_sec"], pd.to_numeric(group["actuator_ratio"], errors="coerce"), label=f"{run.label} / drone_{drone_id}")
    ax.axhline(0.9, color="#E67E22", linestyle="--", linewidth=1.5, label="Saturation Threshold")
    ax.set_title("Actuator Saturation Ratio")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Normalized Thrust")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    return fig, _save_png(fig, output_dir, "advanced_actuator_ratio.png")


def plot_control_lag(runs: Sequence[RunData], output_dir: str) -> Tuple[plt.Figure, str]:
    fig, ax = plt.subplots(figsize=(9, 5))
    for run in runs:
        values = pd.to_numeric(run.frame["control_lag_ms"], errors="coerce").dropna()
        if not values.empty:
            ax.hist(values, bins=30, alpha=0.35, label=run.label)
    ax.axvline(200.0, color="#8E44AD", linestyle="--", linewidth=1.5, label="200 ms Warning")
    ax.set_title("Command/Odom Control Lag")
    ax.set_xlabel("Lag (ms)")
    ax.set_ylabel("Count")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    return fig, _save_png(fig, output_dir, "advanced_control_lag_histogram.png")


def plot_iter_latency_scatter(runs: Sequence[RunData], output_dir: str) -> Tuple[plt.Figure, str]:
    fig, ax = plt.subplots(figsize=(9, 5))
    for run in runs:
        replan_frame = run.replan_frame
        if replan_frame.empty:
            continue
        ax.scatter(
            pd.to_numeric(replan_frame["iter_count"], errors="coerce"),
            pd.to_numeric(replan_frame["time_total_ms"], errors="coerce"),
            alpha=0.55,
            s=18,
            label=f"{run.label} (corr={_correlation(replan_frame['iter_count'], replan_frame['time_total_ms']):.2f})",
        )
    ax.set_title("Planner Iterations vs Total Latency")
    ax.set_xlabel("Iterations")
    ax.set_ylabel("Latency (ms)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    return fig, _save_png(fig, output_dir, "advanced_iter_latency_scatter.png")


def plot_tracking_error(runs: Sequence[RunData], output_dir: str) -> Tuple[plt.Figure, str]:
    fig, ax = plt.subplots(figsize=(10, 5))
    for run in runs:
        for drone_id, group in run.frame.groupby("drone_id"):
            group = group.sort_values("ros_time_sec")
            ax.plot(group["ros_time_sec"], pd.to_numeric(group["tracking_error_m"], errors="coerce"), label=f"{run.label} / drone_{drone_id}")
    ax.set_title("Tracking Error Over Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    return fig, _save_png(fig, output_dir, "advanced_tracking_error.png")


def build_summary_rows(runs: Sequence[RunData], default_vmax: float) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for run in runs:
        for drone_id, group in run.frame.groupby("drone_id"):
            group = group.copy()
            replan_group = run.replan_frame[run.replan_frame["drone_id"] == drone_id] if not run.replan_frame.empty else pd.DataFrame()
            tracking_error_p95 = _p95(group["tracking_error_m"])
            control_lag_p95 = _p95(group["control_lag_ms"])
            latency_p95 = _p95(replan_group["time_total_ms"]) if not replan_group.empty else _p95(group["planner_latency_ms"])
            actuator_mean = float(pd.to_numeric(group["actuator_ratio"], errors="coerce").dropna().mean())
            actuator_max = float(pd.to_numeric(group["actuator_ratio"], errors="coerce").dropna().max())
            max_speed = float(pd.to_numeric(group["speed_mps"], errors="coerce").dropna().max())
            stress_score = _compute_stress_score(replan_group if not replan_group.empty else group.rename(columns={"planner_latency_ms": "time_total_ms", "planner_replan_interval_ms": "replan_interval_ms", "planner_iter_count": "iter_count"}))
            failure_prediction = _predict_failure(tracking_error_p95, control_lag_p95, latency_p95, actuator_mean, actuator_max, max_speed, default_vmax, stress_score)
            rows.append(
                {
                    "run": run.label,
                    "drone_id": int(drone_id),
                    "tracking_error_p95_m": tracking_error_p95,
                    "control_lag_p95_ms": control_lag_p95,
                    "planner_latency_p95_ms": latency_p95,
                    "planner_iter_latency_corr": _correlation(replan_group["iter_count"], replan_group["time_total_ms"]) if not replan_group.empty else float("nan"),
                    "safety_margin_min_m": float(pd.to_numeric(group["safety_margin_m"], errors="coerce").dropna().min()),
                    "actuator_ratio_mean": actuator_mean,
                    "actuator_ratio_max": actuator_max,
                    "max_speed_mps": max_speed,
                    "computational_stress_score": stress_score,
                    "failure_prediction": failure_prediction,
                }
            )
    return rows


def plot_summary_table(rows: Sequence[Dict[str, object]], output_dir: str) -> Tuple[plt.Figure, str]:
    fig, ax = plt.subplots(figsize=(12, max(3, 0.45 * len(rows) + 1.5)))
    ax.axis("off")
    columns = ["run", "drone_id", "tracking_error_p95_m", "planner_latency_p95_ms", "safety_margin_min_m", "computational_stress_score", "failure_prediction"]
    table_rows = []
    for row in rows:
        table_rows.append([
            row["run"],
            row["drone_id"],
            f"{row['tracking_error_p95_m']:.2f}" if math.isfinite(row["tracking_error_p95_m"]) else "",
            f"{row['planner_latency_p95_ms']:.1f}" if math.isfinite(row["planner_latency_p95_ms"]) else "",
            f"{row['safety_margin_min_m']:.2f}" if math.isfinite(row["safety_margin_min_m"]) else "",
            f"{row['computational_stress_score']:.2f}",
            row["failure_prediction"],
        ])
    table = ax.table(cellText=table_rows, colLabels=columns, loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.35)
    ax.set_title("Advanced Benchmark Summary", pad=16)
    return fig, _save_png(fig, output_dir, "advanced_summary_table.png")


def main():
    parser = argparse.ArgumentParser(description="Advanced offline plots and PDF report for swarm benchmark sessions")
    parser.add_argument("inputs", nargs="+", help="Benchmark session directories, CSV files, or glob patterns")
    parser.add_argument("--output-dir", default="./advanced_benchmark_report", help="Directory for generated figures and reports")
    parser.add_argument("--default-vmax", type=float, default=10.0, help="Fallback vmax used by failure prediction")
    args = parser.parse_args()

    runs = _load_runs(args.inputs)
    _ensure_output_dir(args.output_dir)

    figures_and_paths = [
        plot_tracking_error(runs, args.output_dir),
        plot_safety_margin(runs, args.output_dir),
        plot_actuator_ratio(runs, args.output_dir),
        plot_control_lag(runs, args.output_dir),
        plot_iter_latency_scatter(runs, args.output_dir),
    ]
    summary_rows = build_summary_rows(runs, args.default_vmax)
    figures_and_paths.append(plot_summary_table(summary_rows, args.output_dir))

    pdf_path = os.path.join(args.output_dir, "advanced_benchmark_report.pdf")
    with PdfPages(pdf_path) as pdf:
        for fig, _ in figures_and_paths:
            fig.tight_layout()
            pdf.savefig(fig, bbox_inches="tight")
            plt.close(fig)

    summary_json_path = os.path.join(args.output_dir, "advanced_benchmark_summary.json")
    with open(summary_json_path, "w", encoding="utf-8") as handle:
        json.dump(summary_rows, handle, indent=2, ensure_ascii=False)

    print(pdf_path)
    print(summary_json_path)
    for _, path in figures_and_paths:
        print(path)


if __name__ == "__main__":
    main()