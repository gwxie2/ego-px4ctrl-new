#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PARAMETERS = [
    "mission_profile.max_vel",
    "mission_profile.max_acc",
    "mission_profile.replan_time",
    "mission_profile.planning_horizon",
    "mission_profile.lambda_smooth",
    "mission_profile.swarm_clearance",
    "mission_profile.grid_map_obstacles_inflation",
    "physical_uav.pid_gain.Kv",
    "planning_base.local_update_range_x",
    "coverage.map_size_m",
    "coverage.grid_resolution_m",
    "coverage.fov_deg",
    "coverage.publish_hz",
    "coverage.projection_range_scale",
    "coverage.min_projection_range_m",
    "coverage.target_distance_threshold_m",
]


def repo_path(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else REPO_ROOT / path


def load_rows(index_path: Path):
    with index_path.open("r", encoding="utf-8") as handle:
        rows = json.load(handle)
    return [row for row in rows if row.get("status") == "completed"]


def correlation(xs, ys):
    pairs = [(float(x), float(y)) for x, y in zip(xs, ys) if x is not None and y is not None]
    if len(pairs) < 2:
        return None
    mean_x = sum(pair[0] for pair in pairs) / len(pairs)
    mean_y = sum(pair[1] for pair in pairs) / len(pairs)
    cov = sum((pair[0] - mean_x) * (pair[1] - mean_y) for pair in pairs)
    var_x = sum((pair[0] - mean_x) ** 2 for pair in pairs)
    var_y = sum((pair[1] - mean_y) ** 2 for pair in pairs)
    if var_x <= 1e-12 or var_y <= 1e-12:
        return None
    return cov / math.sqrt(var_x * var_y)


def analyse(rows, metric_name, parameters):
    results = []
    for parameter in parameters:
        xs = [row.get("resolved_parameters", {}).get(parameter) for row in rows]
        ys = [row.get(metric_name) for row in rows]
        corr = correlation(xs, ys)
        if corr is None:
            continue
        results.append({"parameter": parameter, "metric": metric_name, "correlation": corr})
    results.sort(key=lambda item: abs(item["correlation"]), reverse=True)
    return results


def plot(results, output_png: Path):
    fig, ax = plt.subplots(figsize=(10, max(4, 0.5 * len(results))))
    if not results:
        ax.text(0.5, 0.5, "No completed runs available", ha="center", va="center")
        ax.set_axis_off()
        fig.tight_layout()
        output_png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_png, dpi=200, bbox_inches="tight")
        return
    ax.barh([row["parameter"] for row in results], [row["correlation"] for row in results], color="#5B8FF9")
    ax.set_title("Parameter Sensitivity Correlation")
    ax.set_xlabel("Pearson Correlation")
    ax.set_ylabel("Parameter")
    ax.grid(True, axis="x", alpha=0.25)
    fig.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=200, bbox_inches="tight")


def main():
    parser = argparse.ArgumentParser(description="Analyse parameter sensitivity from campaign index")
    parser.add_argument("--index", default="benchmark_artifacts/research_profiles/campaign_index.json")
    parser.add_argument("--metric", default="avg_replan_success_rate")
    parser.add_argument("--output-json", default="benchmark_artifacts/research_profiles/parameter_sensitivity.json")
    parser.add_argument("--output-png", default="benchmark_artifacts/research_profiles/parameter_sensitivity.png")
    parser.add_argument("--parameters", nargs="*", default=DEFAULT_PARAMETERS)
    args = parser.parse_args()

    rows = load_rows(repo_path(args.index))
    results = analyse(rows, args.metric, args.parameters)
    output_json = repo_path(args.output_json)
    output_png = repo_path(args.output_png)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    with output_json.open("w", encoding="utf-8") as handle:
        json.dump(results, handle, indent=2, ensure_ascii=False)
    plot(results, output_png)
    print(output_json)
    print(output_png)


if __name__ == "__main__":
    main()