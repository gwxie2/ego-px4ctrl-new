#!/usr/bin/env python3

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[1]


def repo_path(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else REPO_ROOT / path


def load_rows(index_path: Path):
    with index_path.open("r", encoding="utf-8") as handle:
        rows = json.load(handle)
    return [row for row in rows if row.get("status") == "completed"]


def classify(row):
    if (row.get("total_safety_violation_count") or 0) > 0 or (row.get("avg_replan_success_rate") or 0.0) < 0.8:
        return "high"
    if (row.get("avg_tracking_error_p95_m") or 0.0) > 0.5 or (row.get("avg_control_lag_p95_ms") or 0.0) > 150.0:
        return "medium"
    return "low"


def aggregate(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[classify(row)].append(row)
    result = []
    for level in ("high", "medium", "low"):
        bucket = groups.get(level, [])
        count = len(bucket)
        summary = {
            "risk_level": level,
            "run_count": count,
            "avg_total_flight_time_sec": sum((item.get("total_flight_time_sec") or 0.0) for item in bucket) / count if count else 0.0,
            "avg_jerk_integral": sum((item.get("avg_jerk_integral") or 0.0) for item in bucket) / count if count else 0.0,
            "avg_replan_success_rate": sum((item.get("avg_replan_success_rate") or 0.0) for item in bucket) / count if count else 0.0,
            "total_safety_violation_count": sum(int(item.get("total_safety_violation_count") or 0) for item in bucket),
            "speed_contexts": sorted({str(item.get("target_speed_mps")) for item in bucket if item.get("target_speed_mps") is not None}),
        }
        result.append(summary)
    return result


def write_csv(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            serialised = dict(row)
            serialised["speed_contexts"] = ";".join(row.get("speed_contexts", []))
            writer.writerow(serialised)


def plot(rows, output_png: Path):
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.bar([row["risk_level"] for row in rows], [row["total_safety_violation_count"] for row in rows], color=["#E74C3C", "#F5B041", "#52BE80"])
    ax.set_title("Safety Violation Count by Risk Level")
    ax.set_xlabel("Risk Level")
    ax.set_ylabel("Safety Violation Count")
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=200, bbox_inches="tight")


def main():
    parser = argparse.ArgumentParser(description="Aggregate campaign risk statistics")
    parser.add_argument("--index", default="benchmark_artifacts/research_profiles/campaign_index.json")
    parser.add_argument("--output-json", default="benchmark_artifacts/research_profiles/risk_statistics.json")
    parser.add_argument("--output-csv", default="benchmark_artifacts/research_profiles/risk_statistics.csv")
    parser.add_argument("--output-png", default="benchmark_artifacts/research_profiles/risk_statistics.png")
    args = parser.parse_args()

    rows = load_rows(repo_path(args.index))
    aggregated = aggregate(rows)
    output_json = repo_path(args.output_json)
    output_csv = repo_path(args.output_csv)
    output_png = repo_path(args.output_png)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    with output_json.open("w", encoding="utf-8") as handle:
        json.dump(aggregated, handle, indent=2, ensure_ascii=False)
    write_csv(output_csv, aggregated)
    plot(aggregated, output_png)
    print(output_json)
    print(output_csv)
    print(output_png)


if __name__ == "__main__":
    main()