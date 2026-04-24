#!/usr/bin/env python3

import argparse
import json
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
    if not isinstance(rows, list):
        raise ValueError("campaign index must be a list")
    filtered = []
    for row in rows:
        if row.get("status") != "completed":
            continue
        if row.get("total_flight_time_sec") in (None, ""):
            continue
        if row.get("avg_jerk_integral") in (None, ""):
            continue
        filtered.append(row)
    return filtered


def dominates(lhs, rhs):
    return (
        lhs["total_flight_time_sec"] <= rhs["total_flight_time_sec"]
        and lhs["avg_jerk_integral"] <= rhs["avg_jerk_integral"]
        and (
            lhs["total_flight_time_sec"] < rhs["total_flight_time_sec"]
            or lhs["avg_jerk_integral"] < rhs["avg_jerk_integral"]
        )
    )


def pareto_front(rows):
    frontier = []
    for candidate in rows:
        if any(dominates(other, candidate) for other in rows if other is not candidate):
            continue
        frontier.append(candidate)
    frontier.sort(key=lambda item: (item["total_flight_time_sec"], item["avg_jerk_integral"]))
    return frontier


def plot(rows, frontier, output_png: Path):
    fig, ax = plt.subplots(figsize=(9, 6))
    if not rows:
        ax.text(0.5, 0.5, "No completed runs available", ha="center", va="center")
        ax.set_axis_off()
        fig.tight_layout()
        output_png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_png, dpi=200, bbox_inches="tight")
        return
    ax.scatter(
        [row["total_flight_time_sec"] for row in rows],
        [row["avg_jerk_integral"] for row in rows],
        alpha=0.55,
        color="#5B8FF9",
        label="All runs",
    )
    ax.plot(
        [row["total_flight_time_sec"] for row in frontier],
        [row["avg_jerk_integral"] for row in frontier],
        color="#E74C3C",
        linewidth=2.0,
        marker="o",
        label="Pareto frontier",
    )
    for row in frontier:
        ax.annotate(row.get("label", row.get("case_name", "run")), (row["total_flight_time_sec"], row["avg_jerk_integral"]), fontsize=8)
    ax.set_title("Pareto Frontier: Flight Time vs Average Jerk Integral")
    ax.set_xlabel("Total Flight Time (s)")
    ax.set_ylabel("Average Jerk Integral")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=200, bbox_inches="tight")


def main():
    parser = argparse.ArgumentParser(description="Build Pareto frontier plot from campaign index")
    parser.add_argument("--index", default="benchmark_artifacts/research_profiles/campaign_index.json")
    parser.add_argument("--output-png", default="benchmark_artifacts/research_profiles/pareto_frontier.png")
    parser.add_argument("--output-json", default="benchmark_artifacts/research_profiles/pareto_frontier.json")
    args = parser.parse_args()

    rows = load_rows(repo_path(args.index))
    frontier = pareto_front(rows)
    output_png = repo_path(args.output_png)
    output_json = repo_path(args.output_json)
    plot(rows, frontier, output_png)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    with output_json.open("w", encoding="utf-8") as handle:
        json.dump(frontier, handle, indent=2, ensure_ascii=False)
    print(output_png)
    print(output_json)


if __name__ == "__main__":
    main()