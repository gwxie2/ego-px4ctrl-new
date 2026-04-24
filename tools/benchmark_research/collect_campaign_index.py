#!/usr/bin/env python3

import argparse
import csv
import json
import math
import shutil
import subprocess
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]


def repo_path(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else REPO_ROOT / path


def safe_mean(values):
    numbers = [value for value in values if value is not None]
    return sum(numbers) / len(numbers) if numbers else None


def safe_float(value):
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def safe_int(value):
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def load_json(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_json_if_exists(path: Path):
    if not path.exists():
        return None
    try:
        return load_json(path)
    except (OSError, json.JSONDecodeError):
        return None


def repo_relative(path: Path):
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def scan_session_dirs(run_spec):
    session_dirs = [repo_path(path_text) for path_text in run_spec.get("session_dirs", []) if path_text]
    if session_dirs:
        return [path for path in session_dirs if path.exists()]

    output_dir_text = run_spec.get("output_dir")
    if not output_dir_text:
        return []

    output_dir = repo_path(output_dir_text)
    if not output_dir.exists():
        return []
    return sorted(path for path in output_dir.iterdir() if path.is_dir())


def iter_candidate_session_dirs(run_spec):
    seen = set()
    summary_path_text = run_spec.get("summary_path")
    if summary_path_text:
        summary_path = repo_path(summary_path_text)
        if summary_path.exists():
            session_dir = summary_path.parent
            if session_dir.exists():
                seen.add(session_dir)
                yield session_dir

    for session_dir in reversed(scan_session_dirs(run_spec)):
        if session_dir in seen:
            continue
        seen.add(session_dir)
        yield session_dir


def inspect_rosbag(bag_path: Path):
    rosbag_executable = shutil.which("rosbag")
    if not rosbag_executable:
        return None
    try:
        result = subprocess.run(
            [rosbag_executable, "info", "--yaml", str(bag_path)],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None

    payload = yaml.safe_load(result.stdout) or {}
    topics = [item.get("topic") for item in payload.get("topics", []) if isinstance(item, dict) and item.get("topic")]
    size_bytes = payload.get("size")
    duration_sec = payload.get("duration")
    messages = payload.get("messages")
    return {
        "path": str(bag_path.relative_to(REPO_ROOT)),
        "duration_sec": float(duration_sec) if isinstance(duration_sec, (int, float)) else None,
        "size_mb": float(size_bytes) / (1024.0 * 1024.0) if isinstance(size_bytes, (int, float)) else None,
        "messages": int(messages) if isinstance(messages, int) else None,
        "topics": sorted(set(topics)),
    }


def summarise_rosbags(run_spec):
    per_bag = []
    for session_dir in scan_session_dirs(run_spec):
        for bag_path in sorted(session_dir.glob("*.bag")):
            info = inspect_rosbag(bag_path)
            if info:
                per_bag.append(info)

    if not per_bag:
        return {
            "file_count": 0,
            "total_duration_sec": None,
            "max_duration_sec": None,
            "total_size_mb": None,
            "total_messages": None,
            "topics": [],
            "bags": [],
        }

    durations = [item["duration_sec"] for item in per_bag if item["duration_sec"] is not None]
    sizes = [item["size_mb"] for item in per_bag if item["size_mb"] is not None]
    messages = [item["messages"] for item in per_bag if item["messages"] is not None]
    topics = sorted({topic for item in per_bag for topic in item.get("topics", [])})
    return {
        "file_count": len(per_bag),
        "total_duration_sec": sum(durations) if durations else None,
        "max_duration_sec": max(durations) if durations else None,
        "total_size_mb": sum(sizes) if sizes else None,
        "total_messages": sum(messages) if messages else None,
        "topics": topics,
        "bags": per_bag,
    }


def summarise_coverage(run_spec):
    empty_summary = {
        "coverage_artifacts_present": 0,
        "coverage_session_id": None,
        "coverage_metrics_path": "",
        "coverage_target_detection_path": "",
        "coverage_history_path": "",
        "coverage_summary_path": "",
        "coverage_plot_path": "",
        "coverage_history_samples": None,
        "coverage_total_targets": None,
        "coverage_detected_targets": None,
        "coverage_target_detected": 0,
        "coverage_time_to_first_detection_sec": None,
        "coverage_detection_count": 0,
        "coverage_first_detection_target_index": None,
        "coverage_first_detection_drone_id": None,
        "coverage_first_detection_ros_time_sec": None,
        "coverage_first_detection_wall_time_sec": None,
        "coverage_cumulative_coverage_pct": None,
        "coverage_explored_area_m2": None,
        "coverage_total_area_m2": None,
        "coverage_exploration_rate_m2_s": None,
        "coverage_redundancy_factor": None,
        "coverage_trajectory_jerk_integral_mean": None,
        "coverage_trajectory_jerk_integral_max": None,
        "coverage_replan_count_total": None,
        "coverage_replan_frequency_hz": None,
        "coverage_replan_latency_ms_mean": None,
        "coverage_replan_latency_ms_p95": None,
        "coverage_replan_interval_ms_mean": None,
        "coverage_stop_requested": 0,
        "coverage_stop_reason": "",
        "coverage_finished_wall_time_sec": None,
        "coverage_finished_ros_time_sec": None,
    }

    for session_dir in iter_candidate_session_dirs(run_spec):
        metrics_path = session_dir / "coverage_metrics.json"
        metrics = load_json_if_exists(metrics_path)
        if not isinstance(metrics, dict):
            continue

        target_path = session_dir / "target_detection.json"
        history_path = session_dir / "coverage_history.json"
        summary_path = session_dir / "coverage_memory_summary.json"
        plot_path = session_dir / "coverage_vs_time.png"

        coverage = metrics.get("coverage") if isinstance(metrics.get("coverage"), dict) else {}
        targets = metrics.get("targets") if isinstance(metrics.get("targets"), dict) else {}
        trajectory = metrics.get("trajectory") if isinstance(metrics.get("trajectory"), dict) else {}
        replanning = metrics.get("replanning") if isinstance(metrics.get("replanning"), dict) else {}

        detections = targets.get("detections", [])
        if not isinstance(detections, list):
            detections = []

        target_history = load_json_if_exists(target_path)
        if isinstance(target_history, list) and target_history and not detections:
            detections = target_history

        history = load_json_if_exists(history_path)
        if not isinstance(history, list):
            history = []

        first_detection = detections[0] if detections and isinstance(detections[0], dict) else {}

        summary = dict(empty_summary)
        summary.update(
            {
                "coverage_artifacts_present": 1,
                "coverage_session_id": metrics.get("session_id") or run_spec.get("case_name") or run_spec.get("label") or "",
                "coverage_metrics_path": repo_relative(metrics_path),
                "coverage_target_detection_path": repo_relative(target_path) if target_path.exists() else "",
                "coverage_history_path": repo_relative(history_path) if history_path.exists() else "",
                "coverage_summary_path": repo_relative(summary_path) if summary_path.exists() else "",
                "coverage_plot_path": repo_relative(plot_path) if plot_path.exists() else "",
                "coverage_history_samples": safe_int(metrics.get("history_samples")) if metrics.get("history_samples") is not None else len(history),
                "coverage_total_targets": safe_int(targets.get("total_targets")),
                "coverage_detected_targets": safe_int(targets.get("detected_targets")),
                "coverage_target_detected": int(bool(targets.get("target_detected"))),
                "coverage_time_to_first_detection_sec": safe_float(targets.get("time_to_first_detection_sec")),
                "coverage_detection_count": len(detections),
                "coverage_first_detection_target_index": safe_int(first_detection.get("target_index")) if first_detection else None,
                "coverage_first_detection_drone_id": safe_int(first_detection.get("drone_id")) if first_detection else None,
                "coverage_first_detection_ros_time_sec": safe_float(first_detection.get("ros_time_sec")) if first_detection else None,
                "coverage_first_detection_wall_time_sec": safe_float(first_detection.get("wall_time_sec")) if first_detection else None,
                "coverage_cumulative_coverage_pct": safe_float(coverage.get("cumulative_coverage_pct")),
                "coverage_explored_area_m2": safe_float(coverage.get("explored_area_m2")),
                "coverage_total_area_m2": safe_float(coverage.get("total_area_m2")),
                "coverage_exploration_rate_m2_s": safe_float(coverage.get("exploration_rate_m2_s")),
                "coverage_redundancy_factor": safe_float(coverage.get("redundancy_factor")),
                "coverage_trajectory_jerk_integral_mean": safe_float(trajectory.get("jerk_integral_mean")),
                "coverage_trajectory_jerk_integral_max": safe_float(trajectory.get("jerk_integral_max")),
                "coverage_replan_count_total": safe_int(replanning.get("replan_count_total")),
                "coverage_replan_frequency_hz": safe_float(replanning.get("replan_frequency_hz")),
                "coverage_replan_latency_ms_mean": safe_float(replanning.get("optimization_latency_ms_mean")),
                "coverage_replan_latency_ms_p95": safe_float(replanning.get("optimization_latency_ms_p95")),
                "coverage_replan_interval_ms_mean": safe_float(replanning.get("replan_interval_ms_mean")),
                "coverage_stop_requested": int(bool(metrics.get("stop_requested"))),
                "coverage_stop_reason": str(metrics.get("stop_reason") or ""),
                "coverage_finished_wall_time_sec": safe_float(metrics.get("finished_wall_time_sec")),
                "coverage_finished_ros_time_sec": safe_float(metrics.get("finished_ros_time_sec")),
            }
        )
        return summary

    return dict(empty_summary)


def summarise_run(run_spec_path: Path):
    run_spec = load_json(run_spec_path)
    summary_path_text = run_spec.get("summary_path")
    summary_payload = load_json(repo_path(summary_path_text)) if summary_path_text else None
    rosbag_summary = summarise_rosbags(run_spec)
    coverage_summary = summarise_coverage(run_spec)

    metrics = []
    total_flight_time_sec = None
    if summary_payload:
        started = summary_payload.get("started_wall_time_sec")
        stopped = summary_payload.get("stopped_wall_time_sec")
        if isinstance(started, (int, float)) and isinstance(stopped, (int, float)):
            total_flight_time_sec = float(stopped) - float(started)
        drone_metrics = summary_payload.get("drone_metrics", {})
        for drone_id, data in drone_metrics.items():
            if not isinstance(data, dict):
                continue
            metrics.append(
                {
                    "drone_id": int(drone_id),
                    "tracking_error_p95": data.get("tracking_error_p95"),
                    "control_lag_p95_ms": data.get("control_lag_p95_ms"),
                    "planner_latency_p95_ms": data.get("planner_latency_p95_ms"),
                    "replan_success_rate": data.get("replan_success_rate"),
                    "min_safety_margin_m": data.get("min_safety_margin_m"),
                    "safety_violation_count": data.get("safety_violation_count"),
                    "jerk_integral": data.get("jerk_integral"),
                    "stop_reason": data.get("terminal_reason") or summary_payload.get("stop_reason") or "",
                }
            )

    aggregated = {
        "phase_id": run_spec.get("phase_id"),
        "case_name": run_spec.get("case_name"),
        "label": run_spec.get("label"),
        "version": run_spec.get("version"),
        "profile": run_spec.get("profile"),
        "run_index": run_spec.get("run_index"),
        "status": run_spec.get("status"),
        "output_dir": run_spec.get("output_dir"),
        "summary_path": run_spec.get("summary_path"),
        "target_speed_mps": run_spec.get("resolved_parameters", {}).get("benchmark.default_vmax"),
        "total_flight_time_sec": total_flight_time_sec,
        "avg_tracking_error_p95_m": safe_mean([item["tracking_error_p95"] for item in metrics]),
        "avg_control_lag_p95_ms": safe_mean([item["control_lag_p95_ms"] for item in metrics]),
        "avg_planner_latency_p95_ms": safe_mean([item["planner_latency_p95_ms"] for item in metrics]),
        "avg_replan_success_rate": safe_mean([item["replan_success_rate"] for item in metrics]),
        "min_safety_margin_m": min((item["min_safety_margin_m"] for item in metrics if item["min_safety_margin_m"] is not None), default=None),
        "total_safety_violation_count": sum(int(item["safety_violation_count"] or 0) for item in metrics),
        "avg_jerk_integral": safe_mean([item["jerk_integral"] for item in metrics]),
        "stop_reasons": sorted({item["stop_reason"] for item in metrics if item["stop_reason"]}),
        "rosbag_file_count": rosbag_summary["file_count"],
        "rosbag_total_duration_sec": rosbag_summary["total_duration_sec"],
        "rosbag_max_duration_sec": rosbag_summary["max_duration_sec"],
        "rosbag_total_size_mb": rosbag_summary["total_size_mb"],
        "rosbag_total_messages": rosbag_summary["total_messages"],
        "rosbag_topics": rosbag_summary["topics"],
        **coverage_summary,
        "rosbag": rosbag_summary,
        "resolved_parameters": run_spec.get("resolved_parameters", {}),
    }
    return aggregated


def write_csv(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "phase_id",
        "case_name",
        "label",
        "version",
        "profile",
        "run_index",
        "status",
        "target_speed_mps",
        "total_flight_time_sec",
        "avg_tracking_error_p95_m",
        "avg_control_lag_p95_ms",
        "avg_planner_latency_p95_ms",
        "avg_replan_success_rate",
        "min_safety_margin_m",
        "total_safety_violation_count",
        "avg_jerk_integral",
        "coverage_artifacts_present",
        "coverage_session_id",
        "coverage_cumulative_coverage_pct",
        "coverage_explored_area_m2",
        "coverage_total_area_m2",
        "coverage_exploration_rate_m2_s",
        "coverage_redundancy_factor",
        "coverage_total_targets",
        "coverage_detected_targets",
        "coverage_target_detected",
        "coverage_time_to_first_detection_sec",
        "coverage_detection_count",
        "coverage_first_detection_target_index",
        "coverage_first_detection_drone_id",
        "coverage_first_detection_ros_time_sec",
        "coverage_first_detection_wall_time_sec",
        "coverage_trajectory_jerk_integral_mean",
        "coverage_trajectory_jerk_integral_max",
        "coverage_replan_count_total",
        "coverage_replan_frequency_hz",
        "coverage_replan_latency_ms_mean",
        "coverage_replan_latency_ms_p95",
        "coverage_replan_interval_ms_mean",
        "coverage_history_samples",
        "coverage_stop_requested",
        "coverage_stop_reason",
        "coverage_finished_wall_time_sec",
        "coverage_finished_ros_time_sec",
        "coverage_metrics_path",
        "coverage_target_detection_path",
        "coverage_history_path",
        "coverage_summary_path",
        "coverage_plot_path",
        "rosbag_file_count",
        "rosbag_total_duration_sec",
        "rosbag_max_duration_sec",
        "rosbag_total_size_mb",
        "rosbag_total_messages",
        "rosbag_topics",
        "stop_reasons",
        "summary_path",
        "output_dir",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            serialised = dict(row)
            serialised["stop_reasons"] = ";".join(row.get("stop_reasons", []))
            serialised["rosbag_topics"] = ";".join(row.get("rosbag_topics", []))
            writer.writerow({key: serialised.get(key, "") for key in fieldnames})


def main():
    parser = argparse.ArgumentParser(description="Collect run_spec.json files into a campaign index")
    parser.add_argument("--root", default="benchmark_artifacts/research_profiles", help="Campaign output root")
    parser.add_argument("--output-json", default="benchmark_artifacts/research_profiles/campaign_index.json")
    parser.add_argument("--output-csv", default="benchmark_artifacts/research_profiles/campaign_index.csv")
    args = parser.parse_args()

    root = repo_path(args.root)
    rows = []
    for run_spec_path in sorted(root.rglob("run_spec.json")):
        rows.append(summarise_run(run_spec_path))

    output_json = repo_path(args.output_json)
    output_csv = repo_path(args.output_csv)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    with output_json.open("w", encoding="utf-8") as handle:
        json.dump(rows, handle, indent=2, ensure_ascii=False)
    write_csv(output_csv, rows)

    print(output_json)
    print(output_csv)


if __name__ == "__main__":
    main()