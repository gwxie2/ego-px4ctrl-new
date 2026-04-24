#!/usr/bin/env python3

import argparse
import json
import subprocess
import sys
from copy import deepcopy
from datetime import datetime
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATOR_PATH = REPO_ROOT / "src/clean_uav_core/scripts/swarm_launch_generator_yaml.py"
RUN_SINGLE_PROFILE = REPO_ROOT / "tools/benchmark_research/run_single_profile.sh"


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def resolve_repo_path(text: str) -> Path:
    path = Path(text)
    return path if path.is_absolute() else REPO_ROOT / path


def value_to_launch_text(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def launch_text_to_value(text: str):
    lowered = text.strip().lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        if any(marker in text for marker in (".", "e", "E")):
            return float(text)
        return int(text)
    except ValueError:
        return text


def merge_launch_args(*arg_lists):
    ordered_pairs = []
    index_by_key = {}
    passthrough = []
    for arg_list in arg_lists:
        for item in arg_list or []:
            key, separator, value = item.partition(":=")
            if not separator:
                passthrough.append(item)
                continue
            if key in index_by_key:
                ordered_pairs[index_by_key[key]] = (key, value)
            else:
                index_by_key[key] = len(ordered_pairs)
                ordered_pairs.append((key, value))
    return [f"{key}:={value}" for key, value in ordered_pairs] + passthrough


def apply_cli_launch_args(runtime_overrides, launch_args):
    resolved = dict(runtime_overrides or {})
    for item in launch_args or []:
        key, separator, value = item.partition(":=")
        if separator:
            resolved[key] = launch_text_to_value(value)
    return resolved


def merge_case_overrides(defaults, overrides):
    result = dict(defaults or {})
    result.update(overrides or {})
    return result


def extract_resolved_parameters(swarm_config_path: Path, profile_name: str, runtime_overrides):
    config = load_yaml(swarm_config_path)
    simulation = config.get("simulation", {})
    planning_base = config.get("planning_base", {})
    physical_uav = config.get("physical_uav", {})
    mission_profiles = config.get("mission_profiles", {})
    profile = mission_profiles.get(profile_name, {})
    pid_gain = physical_uav.get("pid_gain", {})

    resolved = {
        "simulation.world_path": simulation.get("world_path"),
        "simulation.position_scale": simulation.get("position_scale", 1.0),
        "simulation.enable_vins": simulation.get("enable_vins"),
        "simulation.use_truth_odom_runtime": simulation.get("use_truth_odom_runtime"),
        "planning_base.map_size_x": planning_base.get("map_size_x"),
        "planning_base.map_size_y": planning_base.get("map_size_y"),
        "planning_base.map_size_z": planning_base.get("map_size_z"),
        "planning_base.resolution": planning_base.get("resolution"),
        "planning_base.ground_height": planning_base.get("ground_height"),
        "planning_base.odom_depth_timeout": planning_base.get("odom_depth_timeout"),
        "planning_base.grid_map_depth_filter_mindist": planning_base.get("grid_map_depth_filter_mindist"),
        "planning_base.local_update_range_x": planning_base.get("local_update_range_x"),
        "planning_base.local_update_range_y": planning_base.get("local_update_range_y"),
        "planning_base.local_update_range_z": planning_base.get("local_update_range_z"),
        "physical_uav.mass": physical_uav.get("mass"),
        "physical_uav.hover_percent": physical_uav.get("hover_percent"),
        "physical_uav.pid_gain.Kp": pid_gain.get("Kp"),
        "physical_uav.pid_gain.Kv": pid_gain.get("Kv"),
        "mission_profile.name": profile_name,
        "mission_profile.max_vel": profile.get("max_vel"),
        "mission_profile.max_acc": profile.get("max_acc"),
        "mission_profile.max_jerk": profile.get("max_jerk"),
        "mission_profile.planning_horizon": profile.get("planning_horizon"),
        "mission_profile.replan_time": profile.get("replan_time"),
        "mission_profile.emergency_time": profile.get("emergency_time"),
        "mission_profile.weight_time": profile.get("weight_time"),
        "mission_profile.lambda_smooth": profile.get("lambda_smooth"),
        "mission_profile.grid_map_obstacles_inflation": profile.get("grid_map_obstacles_inflation"),
        "mission_profile.swarm_clearance": profile.get("swarm_clearance"),
        "mission_profile.swarm_collision_weight": profile.get("swarm_collision_weight"),
        "mission_profile.swarm_weight": profile.get("swarm_weight"),
        "mission_profile.swarm_symmetry_gain": profile.get("swarm_symmetry_gain"),
    }

    override_to_resolved = {
        "planner_max_vel": "mission_profile.max_vel",
        "planner_max_acc": "mission_profile.max_acc",
        "planner_max_jerk": "mission_profile.max_jerk",
        "planner_planning_horizon": "mission_profile.planning_horizon",
        "planner_replan_time": "mission_profile.replan_time",
        "planner_emergency_time": "mission_profile.emergency_time",
        "planner_weight_time": "mission_profile.weight_time",
        "planner_lambda_smooth": "mission_profile.lambda_smooth",
        "grid_map_obstacles_inflation": "mission_profile.grid_map_obstacles_inflation",
        "swarm_clearance": "mission_profile.swarm_clearance",
        "swarm_collision_weight": "mission_profile.swarm_collision_weight",
        "swarm_weight": "mission_profile.swarm_weight",
        "swarm_symmetry_gain": "mission_profile.swarm_symmetry_gain",
        "grid_map_local_update_range_x": "planning_base.local_update_range_x",
        "grid_map_local_update_range_y": "planning_base.local_update_range_y",
        "grid_map_local_update_range_z": "planning_base.local_update_range_z",
        "mass": "physical_uav.mass",
        "hover_percent": "physical_uav.hover_percent",
        "px4ctrl_kp": "physical_uav.pid_gain.Kp",
        "px4ctrl_kv": "physical_uav.pid_gain.Kv",
        "coverage_map_size_m": "coverage.map_size_m",
        "coverage_grid_resolution": "coverage.grid_resolution_m",
        "coverage_fov_deg": "coverage.fov_deg",
        "coverage_publish_hz": "coverage.publish_hz",
        "coverage_projection_range_scale": "coverage.projection_range_scale",
        "coverage_min_projection_range_m": "coverage.min_projection_range_m",
        "target_distance_threshold": "coverage.target_distance_threshold_m",
        "target_positions": "coverage.target_positions",
        "UAV_NUM": "coverage.uav_num",
        "benchmark_default_vmax": "benchmark.default_vmax",
    }
    for key, value in (runtime_overrides or {}).items():
        resolved[override_to_resolved.get(key, f"launch_override.{key}")] = value

    if "benchmark.default_vmax" not in resolved or resolved["benchmark.default_vmax"] is None:
        resolved["benchmark.default_vmax"] = resolved.get("mission_profile.max_vel")

    return resolved


def run_command(command, dry_run=False):
    printable = " ".join(str(item) for item in command)
    print(f"[campaign] {printable}")
    if not dry_run:
        subprocess.run(command, check=True)


def scan_session_dirs(output_dir: Path):
    if not output_dir.exists():
        return []
    return sorted(path for path in output_dir.iterdir() if path.is_dir())


def choose_run_index(case_output_root: Path, requested_index: int) -> int:
    run_index = requested_index
    while (case_output_root / f"run_{run_index:02d}").exists():
        run_index += 1
    return run_index


def scan_summary_file(session_dirs):
    for session_dir in reversed(session_dirs):
        matches = sorted(session_dir.glob("*_summary.json"))
        if matches:
            return matches[-1]
    return None


def write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)


def select_cases(campaign, phase_filter, case_filter):
    selected = []
    for phase in campaign.get("phases", []):
        if phase_filter and phase.get("id") != phase_filter:
            continue
        for case in phase.get("cases", []):
            if case_filter and case.get("name") != case_filter:
                continue
            selected.append((phase, case))
    return selected


def main():
    parser = argparse.ArgumentParser(description="Run YAML-driven swarm benchmark research campaign")
    parser.add_argument(
        "--campaign",
        default="src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml",
        help="Path to campaign YAML",
    )
    parser.add_argument("--phase", default="", help="Run only one phase id")
    parser.add_argument("--case", default="", help="Run only one case name")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without executing")
    parser.add_argument("--generate-only", action="store_true", help="Generate launch files and run specs only")
    parser.add_argument("--launch-arg", action="append", default=[], help="Additional roslaunch args in key:=value form")
    parser.add_argument("--visualize", action="store_true", help="Force gui:=true and use_rviz:=true for this run")
    args = parser.parse_args()

    campaign_path = resolve_repo_path(args.campaign)
    campaign = load_yaml(campaign_path)
    output_root = resolve_repo_path(campaign.get("output_root", "benchmark_artifacts/research_profiles"))
    default_repetitions = int(campaign.get("default_repetitions", 1))
    cleanup_before_each_run = bool(campaign.get("cleanup_before_each_run", True))
    wait_for_session_complete = bool(campaign.get("wait_for_session_complete", True))
    default_runtime_overrides = campaign.get("default_runtime_overrides", {})

    selected_cases = select_cases(campaign, args.phase.strip(), args.case.strip())
    if not selected_cases:
        raise SystemExit("no matching campaign cases")

    visual_launch_args = ["gui:=true", "use_rviz:=true"] if args.visualize else []
    cli_launch_args = merge_launch_args(visual_launch_args, args.launch_arg)

    for phase, case in selected_cases:
        phase_id = phase["id"]
        case_name = case["name"]
        version = case["version"]
        repetitions = int(case.get("repetitions", default_repetitions))
        swarm_config_path = resolve_repo_path(case["swarm_config"])
        launch_output_path = resolve_repo_path(case["launch_output"])
        runtime_overrides = merge_case_overrides(default_runtime_overrides, case.get("runtime_overrides", {}))
        effective_runtime_overrides = apply_cli_launch_args(runtime_overrides, cli_launch_args)
        case_output_root = output_root / phase_id / case_name

        generator_command = [
            sys.executable,
            str(GENERATOR_PATH),
            "--version",
            version,
            "--swarm-config",
            str(swarm_config_path),
            "--profile",
            str(case["profile"]),
            "--output",
            str(launch_output_path),
        ]
        _resolved_for_scale = extract_resolved_parameters(swarm_config_path, str(case["profile"]), {})
        _position_scale = _resolved_for_scale.get("simulation.position_scale", 1.0)
        if _position_scale != 1.0:
            generator_command.extend(["--scale", str(_position_scale)])
        run_command(generator_command, dry_run=args.dry_run)

        for run_index in range(1, repetitions + 1):
            actual_run_index = choose_run_index(case_output_root, run_index)
            run_dir = case_output_root / f"run_{actual_run_index:02d}"
            run_dir.mkdir(parents=True, exist_ok=True)
            run_spec_path = run_dir / "run_spec.json"
            run_spec = {
                "phase_id": phase_id,
                "phase_title": phase.get("title"),
                "case_name": case_name,
                "label": case.get("label", case_name),
                "description": phase.get("description"),
                "version": version,
                "profile": case["profile"],
                "swarm_config": str(swarm_config_path.relative_to(REPO_ROOT)),
                "launch_output": str(launch_output_path.relative_to(REPO_ROOT)),
                "launch_file": case.get("launch_file", launch_output_path.name),
                "run_index": actual_run_index,
                "requested_at": datetime.utcnow().isoformat() + "Z",
                "output_dir": str(run_dir.relative_to(REPO_ROOT)),
                "runtime_overrides": deepcopy(effective_runtime_overrides),
                "cli_launch_args": list(cli_launch_args),
                "resolved_parameters": extract_resolved_parameters(swarm_config_path, case["profile"], effective_runtime_overrides),
                "status": "planned",
            }
            write_json(run_spec_path, run_spec)

            if args.generate_only:
                run_spec["status"] = "generated"
                write_json(run_spec_path, run_spec)
                continue

            runner_command = [
                str(RUN_SINGLE_PROFILE),
                "--version",
                version,
                "--launch-file",
                case.get("launch_file", launch_output_path.name),
                "--output-dir",
                str(run_dir),
            ]
            if not wait_for_session_complete:
                runner_command.append("--no-wait")
            if not cleanup_before_each_run:
                runner_command.append("--no-cleanup")
            for key, value in runtime_overrides.items():
                runner_command.extend(["--launch-arg", f"{key}:={value_to_launch_text(value)}"])
            for item in cli_launch_args:
                runner_command.extend(["--launch-arg", item])
            if args.dry_run:
                runner_command.append("--dry-run")

            try:
                run_command(runner_command, dry_run=False)
            except subprocess.CalledProcessError as exc:
                run_spec["status"] = "failed"
                run_spec["exit_code"] = exc.returncode
                write_json(run_spec_path, run_spec)
                raise

            session_dirs = scan_session_dirs(run_dir)
            summary_path = scan_summary_file(session_dirs)
            run_spec["status"] = "completed"
            run_spec["session_dirs"] = [str(path.relative_to(REPO_ROOT)) for path in session_dirs]
            run_spec["summary_path"] = str(summary_path.relative_to(REPO_ROOT)) if summary_path else ""
            write_json(run_spec_path, run_spec)


if __name__ == "__main__":
    main()