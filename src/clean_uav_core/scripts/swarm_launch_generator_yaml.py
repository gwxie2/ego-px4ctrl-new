#!/usr/bin/env python3
"""Generate swarm top-level launch files from YAML mission profiles."""

import argparse
import math
import re
import sys
from collections import OrderedDict
from pathlib import Path

import rospkg
import yaml


SUPPORTED_VERSIONS = ("v1", "v2")
DEFAULT_SWARM_CONFIG = Path("src/clean_uav_core/config/swarm_config.yaml")
DEFAULT_POSITION_CONFIG = Path("docs/uav_position_goal.md")
LEGACY_VARIANT_COUNTS = (3, 4, 5, 6)


class SwarmConfigError(ValueError):
    """Raised when the shared swarm YAML is invalid."""


def get_clean_uav_core_path():
    return Path(rospkg.RosPack().get_path("clean_uav_core"))


def get_workspace_root():
    return get_clean_uav_core_path().parent.parent


def resolve_path(path_text, default_path):
    workspace_root = get_workspace_root()
    clean_uav_core_path = get_clean_uav_core_path()

    candidate = Path(path_text) if path_text else default_path
    if candidate.is_absolute():
        return candidate

    workspace_candidate = workspace_root / candidate
    if workspace_candidate.exists():
        return workspace_candidate

    package_candidate = clean_uav_core_path / candidate
    if package_candidate.exists():
        return package_candidate

    return workspace_candidate


def resolve_output_path(output_text, version, num_uavs=None):
    workspace_root = get_workspace_root()
    if output_text:
        candidate = Path(output_text)
        if candidate.is_absolute():
            return candidate
        return workspace_root / candidate

    launch_dir = get_clean_uav_core_path() / "launch"
    suffix = f"_{num_uavs}UAV" if num_uavs else ""
    return launch_dir / f"swarm_top_level_{version}{suffix}.launch"


def normalize_version(version):
    normalized = str(version).strip().lower()
    if normalized not in SUPPORTED_VERSIONS:
        raise SwarmConfigError(
            f"unsupported version '{version}', expected one of {', '.join(SUPPORTED_VERSIONS)}"
        )
    return normalized


def parse_uav_ids(config_path):
    drone_ids = []
    pattern = re.compile(r"`?drone_(\d+)`?", re.IGNORECASE)

    try:
        with config_path.open("r", encoding="utf-8") as handle:
            for line in handle:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue

                match = pattern.search(stripped)
                if match:
                    drone_ids.append(int(match.group(1)))
    except FileNotFoundError as error:
        raise FileNotFoundError(f"position config file not found: {config_path}") from error

    if not drone_ids:
        raise SwarmConfigError(f"no drone configuration found in {config_path}")

    actual_ids = sorted(set(drone_ids))
    expected_ids = list(range(len(actual_ids)))
    if actual_ids != expected_ids:
        raise SwarmConfigError(f"drone ids must be contiguous starting from 0, got {actual_ids}")

    return actual_ids


def _build_phase1_uav_configs_from_ids(drone_ids, radius=15.0, start_z=0.10, goal_z=1.50, scale=1.0):
    num_uavs = len(drone_ids)

    if num_uavs == 1:
        angles_deg = [0.0]
    else:
        span_deg = min(160.0, 30.0 * (num_uavs - 1))
        step_deg = span_deg / (num_uavs - 1)
        angles_deg = [-span_deg / 2.0 + step_deg * index for index in range(num_uavs)]

    uav_configs = OrderedDict()
    for drone_id, angle_deg in zip(drone_ids, angles_deg):
        angle_rad = math.radians(angle_deg)
        start_x = radius * math.cos(angle_rad) * scale
        start_y = radius * math.sin(angle_rad) * scale
        start_yaw = math.atan2(-start_y, -start_x)

        uav_configs[drone_id] = {
            "start": {
                "x": start_x,
                "y": start_y,
                "z": start_z,
                "yaw": start_yaw,
            },
            "goal": {
                "x": -start_x,
                "y": -start_y,
                "z": goal_z,
                "yaw": start_yaw,
            },
        }

    return uav_configs


def _build_track_uav_configs_from_ids(
    drone_ids,
    start_x=2.0,
    delta_y=3.0,
    start_z=0.20,
    goal_x=115.0,
    goal_z=1.50,
    goal_mode="parallel",
    start_yaw=0.0,
):
    goal_mode = str(goal_mode).strip().lower()
    if goal_mode not in {"parallel", "cross"}:
        raise SwarmConfigError(f"track.goal_mode must be either 'parallel' or 'cross', got '{goal_mode}'")

    num_uavs = len(drone_ids)
    uav_configs = OrderedDict()
    for index, drone_id in enumerate(drone_ids):
        start_y = (index - (num_uavs - 1) / 2.0) * float(delta_y)
        goal_y = start_y if goal_mode == "parallel" else -start_y
        uav_configs[drone_id] = {
            "start": {
                "x": float(start_x),
                "y": start_y,
                "z": float(start_z),
                "yaw": float(start_yaw),
            },
            "goal": {
                "x": float(goal_x),
                "y": goal_y,
                "z": float(goal_z),
                "yaw": float(start_yaw),
            },
        }

    return uav_configs


def build_phase1_uav_configs(config_path, radius=15.0, start_z=0.10, goal_z=1.50, scale=1.0):
    drone_ids = parse_uav_ids(config_path)
    return _build_phase1_uav_configs_from_ids(drone_ids, radius=radius, start_z=start_z, goal_z=goal_z, scale=scale)


def build_phase1_uav_configs_for_count(num_uavs, radius=15.0, start_z=0.10, goal_z=1.50, scale=1.0):
    if num_uavs <= 0:
        raise SwarmConfigError(f"num_uavs must be positive, got {num_uavs}")

    drone_ids = list(range(num_uavs))
    return _build_phase1_uav_configs_from_ids(drone_ids, radius=radius, start_z=start_z, goal_z=goal_z, scale=scale)


def build_uav_configs_from_shared_config(drone_ids, shared_config, scale=1.0):
    simulation = shared_config.get("simulation", {})
    if not isinstance(simulation, dict):
        raise SwarmConfigError("simulation must be a mapping")

    position_layout = str(simulation.get("position_layout", "circular")).strip().lower()
    if position_layout in {"circular", "circle", "phase1", "legacy"}:
        return _build_phase1_uav_configs_from_ids(drone_ids, scale=scale)

    if position_layout in {"track", "track_line", "high_speed_track"}:
        track = simulation.get("track", {})
        if not isinstance(track, dict):
            raise SwarmConfigError("simulation.track must be a mapping when position_layout is track_line")
        return _build_track_uav_configs_from_ids(
            drone_ids,
            start_x=track.get("start_x", 2.0),
            delta_y=track.get("delta_y", 3.0),
            start_z=track.get("start_z", 0.20),
            goal_x=track.get("goal_x", 115.0),
            goal_z=track.get("goal_z", 1.50),
            goal_mode=track.get("goal_mode", "parallel"),
            start_yaw=track.get("start_yaw", 0.0),
        )

    raise SwarmConfigError(
        f"unsupported simulation.position_layout '{position_layout}', expected circular or track_line"
    )


def load_yaml_config(config_path):
    try:
        with config_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except FileNotFoundError as error:
        raise FileNotFoundError(f"swarm config file not found: {config_path}") from error
    except yaml.YAMLError as error:
        raise SwarmConfigError(f"failed to parse YAML config {config_path}: {error}") from error

    if not isinstance(data, dict):
        raise SwarmConfigError(f"swarm config root must be a mapping, got {type(data).__name__}")
    return data


def require_keys(mapping, dotted_prefix, keys):
    missing = [key for key in keys if key not in mapping]
    if missing:
        raise SwarmConfigError(f"missing required keys under {dotted_prefix}: {', '.join(missing)}")


def resolve_world_launch_path(world_path):
    text = str(world_path).strip()
    if not text:
        raise SwarmConfigError("simulation.world_path must not be empty")
    if text.startswith("$(find "):
        return text
    if text.startswith("/"):
        raise SwarmConfigError("simulation.world_path must be relative to a ROS package, not an absolute path")
    if text.startswith("clean_uav_core/"):
        return f"$(find clean_uav_core)/{text.split('/', 1)[1]}"
    return f"$(find px4)/Tools/sitl_gazebo/worlds/{text}"


def build_legacy_variant_world_path(num_uavs):
    return f"$(find px4)/Tools/sitl_gazebo/worlds/swarm_benchmark_forest_phase2_{int(num_uavs)}UAV.world"


def bool_text(value):
    return "true" if bool(value) else "false"


def float_text(value):
    return str(float(value))


def int_text(value):
    return str(int(value))


def format_launch_config_path(config_path):
    workspace_root = get_workspace_root()
    try:
        relative = config_path.relative_to(workspace_root)
        return relative.as_posix()
    except ValueError:
        return str(config_path)


def build_runtime_defaults(shared_config, profile_name, version):
    simulation = shared_config.get("simulation", {})
    planning_base = shared_config.get("planning_base", {})
    physical_uav = shared_config.get("physical_uav", {})
    mission_profiles = shared_config.get("mission_profiles", {})

    if not isinstance(simulation, dict):
        raise SwarmConfigError("simulation must be a mapping")
    if not isinstance(planning_base, dict):
        raise SwarmConfigError("planning_base must be a mapping")
    if not isinstance(physical_uav, dict):
        raise SwarmConfigError("physical_uav must be a mapping")
    if not isinstance(mission_profiles, dict):
        raise SwarmConfigError("mission_profiles must be a mapping")

    require_keys(
        simulation,
        "simulation",
        ["world_path", "gui", "use_rviz", "enable_vins", "use_truth_odom_runtime"],
    )
    require_keys(
        planning_base,
        "planning_base",
        [
            "map_size_x",
            "map_size_y",
            "map_size_z",
            "resolution",
            "ground_height",
            "odom_depth_timeout",
            "grid_map_depth_filter_mindist",
        ],
    )
    require_keys(physical_uav, "physical_uav", ["mass", "hover_percent", "pid_gain"])

    available_profiles = sorted(mission_profiles.keys())
    if profile_name not in mission_profiles:
        raise SwarmConfigError(
            f"unknown profile '{profile_name}', available profiles: {', '.join(available_profiles)}"
        )

    profile = mission_profiles[profile_name]
    if not isinstance(profile, dict):
        raise SwarmConfigError(f"mission_profiles.{profile_name} must be a mapping")
    require_keys(
        profile,
        f"mission_profiles.{profile_name}",
        [
            "max_vel",
            "max_acc",
            "planning_horizon",
            "replan_time",
            "weight_time",
            "lambda_smooth",
            "grid_map_obstacles_inflation",
            "swarm_clearance",
        ],
    )

    pid_gain = physical_uav.get("pid_gain")
    if not isinstance(pid_gain, dict):
        raise SwarmConfigError("physical_uav.pid_gain must be a mapping")
    require_keys(pid_gain, "physical_uav.pid_gain", ["Kp", "Kv"])

    defaults = OrderedDict()
    defaults["world"] = resolve_world_launch_path(simulation["world_path"])
    defaults["gui"] = bool_text(simulation["gui"])
    defaults["use_rviz"] = bool_text(simulation["use_rviz"])
    defaults["enable_vins"] = bool_text(simulation["enable_vins"])
    defaults["use_truth_odom_runtime"] = bool_text(simulation["use_truth_odom_runtime"])

    sensor_degradation = simulation.get("sensor_degradation", {})
    if sensor_degradation is None:
        sensor_degradation = {}
    if not isinstance(sensor_degradation, dict):
        raise SwarmConfigError("simulation.sensor_degradation must be a mapping")
    sensor_degradation_enabled = bool(sensor_degradation.get("enabled", False))
    sensor_degradation_mode = str(sensor_degradation.get("mode", "clean")).strip().lower()
    _valid_modes = ("clean", "nominal", "mild_no_latency", "mild", "extreme")
    if sensor_degradation_mode not in _valid_modes:
        raise SwarmConfigError(
            f"simulation.sensor_degradation.mode must be one of {_valid_modes}, got '{sensor_degradation_mode}'"
        )
    depth_median_filter_size = int(sensor_degradation.get("depth_median_filter_size", 0))
    defaults["sensor_degradation_enabled"] = bool_text(sensor_degradation_enabled)
    defaults["sensor_degradation_mode"] = sensor_degradation_mode
    defaults["depth_median_filter_size"] = str(depth_median_filter_size)

    defaults["grid_map_resolution"] = float_text(planning_base["resolution"])
    defaults["map_size_x"] = float_text(planning_base["map_size_x"])
    defaults["map_size_y"] = float_text(planning_base["map_size_y"])
    defaults["map_size_z"] = float_text(planning_base["map_size_z"])
    defaults["grid_map_ground_height"] = float_text(planning_base["ground_height"])
    defaults["grid_map_odom_depth_timeout"] = float_text(planning_base["odom_depth_timeout"])
    defaults["grid_map_use_depth_filter"] = bool_text(planning_base.get("grid_map_use_depth_filter", True))
    defaults["grid_map_depth_filter_tolerance"] = float_text(
        planning_base.get("grid_map_depth_filter_tolerance", 0.15)
    )
    defaults["grid_map_depth_filter_maxdist"] = float_text(planning_base.get("grid_map_depth_filter_maxdist", 25.0))
    defaults["grid_map_depth_filter_mindist"] = float_text(planning_base["grid_map_depth_filter_mindist"])
    defaults["grid_map_depth_filter_margin"] = int_text(planning_base.get("grid_map_depth_filter_margin", 2))
    defaults["grid_map_k_depth_scaling_factor"] = float_text(
        planning_base.get("grid_map_k_depth_scaling_factor", 1000.0)
    )
    defaults["grid_map_skip_pixel"] = int_text(planning_base.get("grid_map_skip_pixel", 2))
    defaults["grid_map_local_update_range_x"] = float_text(planning_base.get("local_update_range_x", 15.0))
    defaults["grid_map_local_update_range_y"] = float_text(planning_base.get("local_update_range_y", 15.0))
    defaults["grid_map_local_update_range_z"] = float_text(planning_base.get("local_update_range_z", 4.5))
    defaults["grid_map_obstacles_inflation"] = float_text(profile["grid_map_obstacles_inflation"])

    defaults["planner_max_vel"] = float_text(profile["max_vel"])
    defaults["planner_max_acc"] = float_text(profile["max_acc"])
    defaults["planner_max_jerk"] = float_text(profile.get("max_jerk", float(profile["max_acc"]) * 2.0))
    defaults["planner_planning_horizon"] = float_text(profile["planning_horizon"])
    defaults["planner_replan_time"] = float_text(profile["replan_time"])
    defaults["planner_emergency_time"] = float_text(profile.get("emergency_time", profile["replan_time"]))
    defaults["planner_weight_obstacle"] = float_text(profile.get("weight_obstacle", -1.0))
    defaults["planner_astar_latency_predict"] = float_text(profile.get("astar_latency_predict", 0.0))
    defaults["planner_weight_time"] = float_text(profile["weight_time"])
    defaults["planner_lambda_smooth"] = float_text(profile["lambda_smooth"])
    defaults["swarm_clearance"] = float_text(profile["swarm_clearance"])
    defaults["swarm_collision_weight"] = float_text(profile.get("swarm_collision_weight", 0.5))
    defaults["swarm_weight"] = float_text(profile.get("swarm_weight", 10000.0))
    defaults["swarm_symmetry_gain"] = float_text(profile.get("swarm_symmetry_gain", 0.0))

    defaults["mass"] = float_text(physical_uav["mass"])
    defaults["hover_percent"] = float_text(physical_uav["hover_percent"])
    defaults["px4ctrl_kp"] = float_text(pid_gain["Kp"])
    defaults["px4ctrl_kv"] = float_text(pid_gain["Kv"])

    defaults["planner_fail_safe"] = "true"
    defaults["planner_flight_type"] = "2" if version == "v2" else "1"
    defaults["planner_realworld_experiment"] = "false"
    defaults["use_multitopology_trajs"] = "false"
    defaults["swarm_acceptance_radius"] = "-1.0"
    defaults["swarm_filter_far_trajectories"] = "false"
    defaults["swarm_time_warn_threshold"] = "0.25"
    defaults["swarm_time_reject_threshold"] = "10.0"

    return defaults


class SwarmLaunchGeneratorYaml:
    def __init__(
        self,
        position_config_path,
        swarm_config_path,
        output_path,
        version,
        profile,
        uav_count=None,
        world_override=None,
        emit_legacy_variants=False,
        scale=1.0,
    ):
        self.version = normalize_version(version)
        self.profile = str(profile).strip() or "original"
        self.position_config_path = position_config_path
        self.swarm_config_path = swarm_config_path
        self.output_path = output_path
        self.uav_count = uav_count
        self.emit_legacy_variants = bool(emit_legacy_variants)
        self.scale = float(scale)

        self.shared_config = load_yaml_config(swarm_config_path)
        self.defaults = build_runtime_defaults(self.shared_config, self.profile, self.version)

        if self.uav_count is None:
            drone_ids = parse_uav_ids(position_config_path)
        else:
            drone_ids = list(range(self.uav_count))

        self.uav_configs = build_uav_configs_from_shared_config(drone_ids, self.shared_config, scale=self.scale)
        self.num_uavs = len(self.uav_configs)
        if world_override:
            self.defaults["world"] = world_override
        self.position_config_launch_value = format_launch_config_path(position_config_path)

    @staticmethod
    def _indent(level, text=""):
        return "  " * level + text

    def _emit_arg_block(self, title, args_map, level=1):
        lines = [
            self._indent(level, "<!-- ========================================== -->"),
            self._indent(level, f"<!-- {title:<42} -->"),
            self._indent(level, "<!-- ========================================== -->"),
            self._indent(level, ""),
        ]
        for name, default in args_map.items():
            lines.append(self._indent(level, f'<arg name="{name}" default="{default}"/>'))
        lines.append(self._indent(level, ""))
        return lines

    def _generate_global_args(self):
        common_args = OrderedDict(
            [
                ("planner_version", self.version),
                ("planner_node_name", "ego_planner_v2" if self.version == "v2" else "ego_planner"),
                ("world", self.defaults["world"]),
                ("gui", self.defaults["gui"]),
                ("paused", "false"),
                ("debug", "false"),
                ("use_rviz", self.defaults["use_rviz"]),
                (
                    "rviz_config",
                    "$(find clean_uav_core)/rviz/swarm_rviz_v2.rviz"
                    if self.version == "v2"
                    else "$(find clean_uav_core)/rviz/swarm_rviz.rviz",
                ),
                ("enable_vins", self.defaults["enable_vins"]),
                ("use_truth_odom_runtime", self.defaults["use_truth_odom_runtime"]),
                ("sensor_degradation_enabled", self.defaults["sensor_degradation_enabled"]),
                ("sensor_degradation_mode", self.defaults["sensor_degradation_mode"]),
                ("depth_median_filter_size", self.defaults["depth_median_filter_size"]),
            ]
        )
        mission_args = OrderedDict(
            [
                ("planner_max_vel", self.defaults["planner_max_vel"]),
                ("planner_max_acc", self.defaults["planner_max_acc"]),
                ("planner_max_jerk", self.defaults["planner_max_jerk"]),
                ("planner_planning_horizon", self.defaults["planner_planning_horizon"]),
                ("planner_replan_time", self.defaults["planner_replan_time"]),
                ("planner_emergency_time", self.defaults["planner_emergency_time"]),
                ("planner_weight_time", self.defaults["planner_weight_time"]),
                ("planner_astar_pool_size", "100"),
                ("planner_astar_debug_logging", "false"),
                ("planner_astar_step_factor", "1.0"),
                ("planner_weight_obstacle", self.defaults["planner_weight_obstacle"]),
                ("planner_astar_latency_predict", self.defaults["planner_astar_latency_predict"]),
                ("planner_lambda_smooth", self.defaults["planner_lambda_smooth"]),
                ("planner_fail_safe", self.defaults["planner_fail_safe"]),
                ("planner_flight_type", self.defaults["planner_flight_type"]),
                ("planner_realworld_experiment", self.defaults["planner_realworld_experiment"]),
                ("use_multitopology_trajs", self.defaults["use_multitopology_trajs"]),
                ("swarm_acceptance_radius", self.defaults["swarm_acceptance_radius"]),
                ("swarm_filter_far_trajectories", self.defaults["swarm_filter_far_trajectories"]),
                ("swarm_time_warn_threshold", self.defaults["swarm_time_warn_threshold"]),
                ("swarm_time_reject_threshold", self.defaults["swarm_time_reject_threshold"]),
                ("swarm_clearance", self.defaults["swarm_clearance"]),
                ("swarm_collision_weight", self.defaults["swarm_collision_weight"]),
                ("swarm_weight", self.defaults["swarm_weight"]),
                ("swarm_symmetry_gain", self.defaults["swarm_symmetry_gain"]),
            ]
        )
        map_args = OrderedDict(
            [
                ("grid_map_resolution", self.defaults["grid_map_resolution"]),
                ("map_size_x", self.defaults["map_size_x"]),
                ("map_size_y", self.defaults["map_size_y"]),
                ("map_size_z", self.defaults["map_size_z"]),
                ("grid_map_ground_height", self.defaults["grid_map_ground_height"]),
                ("grid_map_odom_depth_timeout", self.defaults["grid_map_odom_depth_timeout"]),
                ("grid_map_use_depth_filter", self.defaults["grid_map_use_depth_filter"]),
                ("grid_map_depth_filter_tolerance", self.defaults["grid_map_depth_filter_tolerance"]),
                ("grid_map_depth_filter_maxdist", self.defaults["grid_map_depth_filter_maxdist"]),
                ("grid_map_depth_filter_mindist", self.defaults["grid_map_depth_filter_mindist"]),
                ("grid_map_depth_filter_margin", self.defaults["grid_map_depth_filter_margin"]),
                ("grid_map_k_depth_scaling_factor", self.defaults["grid_map_k_depth_scaling_factor"]),
                ("grid_map_skip_pixel", self.defaults["grid_map_skip_pixel"]),
                ("grid_map_local_update_range_x", self.defaults["grid_map_local_update_range_x"]),
                ("grid_map_local_update_range_y", self.defaults["grid_map_local_update_range_y"]),
                ("grid_map_local_update_range_z", self.defaults["grid_map_local_update_range_z"]),
                ("grid_map_obstacles_inflation", self.defaults["grid_map_obstacles_inflation"]),
            ]
        )
        physical_args = OrderedDict(
            [
                ("mass", self.defaults["mass"]),
                ("hover_percent", self.defaults["hover_percent"]),
                ("px4ctrl_kp", self.defaults["px4ctrl_kp"]),
                ("px4ctrl_kv", self.defaults["px4ctrl_kv"]),
            ]
        )
        timing_args = OrderedDict(
            [
                ("takeoff_delay", "8.0"),
                ("planner_start_z_threshold", "0.8"),
                ("planner_start_stable_duration", "1.5"),
                ("planner_start_timeout", "60.0"),
                ("planner_post_takeoff_delay", "1.0"),
                ("swarm_wait_for_all_uavs", "true" if self.version == "v2" else "false"),
                ("swarm_num_uavs", str(self.num_uavs)),
                (
                    "swarm_odom_topic_template",
                    "/drone_%d/truth_odom" if self.version == "v2" else "/drone_%d/odom",
                ),
                ("traj_trigger_delay", "20.0"),
                ("traj_trigger_repeat", "1"),
                ("traj_trigger_rate", "3.0"),
                (
                    "goal_start_delay",
                    "$(eval float(arg('takeoff_delay')) + 12.0)" if self.version == "v2" else "1.0",
                ),
                ("goal_publish_rate", "0.2"),
                ("goal_period_sec", "30.0"),
                ("enable_oscillation", "true"),
                ("planner_log_to_file", "false"),
                ("planner_log_root_dir", "$(arg benchmark_output_dir)/planner_logs"),
                ("planner_log_session", "manual_run"),
            ]
        )
        mission_manager_args = OrderedDict(
            [
                ("enable_mission_manager", "false"),
                ("mission_manager_mode", "search"),
                ("mission_use_v1", "true" if self.version == "v1" else "false"),
                ("mission_use_v2", "true" if self.version == "v2" else "false"),
                ("mission_goal_topic", "/goal_with_id"),
                ("mission_goal_topic_template", "/drone_%d/goal"),
                ("mission_feedback_topic", "/mission_manager/feedback"),
                ("mission_benchmark_track", "$(arg benchmark_enable)"),
                ("mission_goal_reached_radius_m", "1.0"),
                ("mission_target_confirmation_radius_m", "1.5"),
                ("mission_support_radius_m", "8.0"),
                ("mission_search_altitude_m", "1.5"),
                ("mission_waypoint_spacing_m", "12.0"),
                ("mission_initial_escape_distance_m", "3.0"),
                ("mission_initial_heading_weight", "1.0"),
                ("mission_search_area_min_x", "-20.0"),
                ("mission_search_area_max_x", "20.0"),
                ("mission_search_area_min_y", "-12.0"),
                ("mission_search_area_max_y", "12.0"),
                ("mission_search_sector_count", "3"),
                ("mission_explore_speed_mps", "0.8"),
                ("mission_track_speed_mps", "2.5"),
                ("mission_support_speed_mps", "1.8"),
                ("mission_finish_speed_mps", "1.0"),
            ]
        )
        benchmark_args = OrderedDict(
            [
                ("benchmark_enable", "false"),
                ("benchmark_output_dir", "$(env HOME)/swarm_benchmark/data"),
                ("benchmark_record_rosbag", "true"),
                ("benchmark_low_speed_threshold", "0.1"),
                ("benchmark_low_speed_duration", "2.0"),
                ("benchmark_goal_distance_threshold", "0.5"),
                ("benchmark_command_age_warn_ms", "150.0"),
                ("benchmark_command_interval_warn_ms", "150.0"),
                ("benchmark_stop_after_terminal_sec", "2.0"),
                ("benchmark_min_session_duration_sec", "0.0"),
                ("benchmark_max_session_duration_sec", "0.0"),
                ("benchmark_safety_margin_threshold", "$(arg swarm_clearance)"),
                ("benchmark_actuator_saturation_threshold", "0.9"),
                ("benchmark_default_vmax", self.defaults["planner_max_vel"]),
                ("benchmark_goal_stop_enabled", "false"),
            ]
        )

        lines = []
        lines.extend(self._emit_arg_block("Global Arguments", common_args))
        lines.extend(self._emit_arg_block("Mission Profile Arguments", mission_args))
        lines.extend(self._emit_arg_block("Cooperative Search Arguments", mission_manager_args))
        lines.extend(self._emit_arg_block("Benchmark Arguments", benchmark_args))
        lines.extend(self._emit_arg_block("Grid Map Arguments", map_args))
        lines.extend(self._emit_arg_block("Physical UAV Arguments", physical_args))
        lines.extend(self._emit_arg_block("Timing Arguments", timing_args))

        if self.version == "v2":
            v2_aux_args = OrderedDict(
                [
                    ("planner_goal_topic", "/goal_with_id"),
                    ("traj_server_enable_stale_traj_extrapolation", "false"),
                    ("traj_server_stale_traj_extrapolation_timeout", "0.35"),
                    ("traj_server_retime_position_cmd_now", "true"),
                    ("planning_broadcast_topic_v2", "/swarm/v2/broadcast_traj"),
                    ("enable_goal_tooling_v2", "false"),
                    ("enable_assign_goals_v2", "true"),
                    ("enable_random_goals_v2", "false"),
                    ("enable_moving_obstacles_v2", "false"),
                    ("enable_manual_take_over_v2", "false"),
                    ("enable_manual_take_over_station_v2", "false"),
                    ("enable_manual_take_over_joy_node_v2", "true"),
                    ("enable_odom_visualization_v2", "false"),
                    ("v2_selected_drones_topic", "/swarm/v2/rviz_selected_drones"),
                    ("v2_pose_goal_topic", "/swarm/v2/goal_pose"),
                    ("v2_goal_arrow_topic", "/swarm/v2/new_goals_arrow"),
                    ("enable_moving_obstacles_joy_node_v2", "false"),
                    ("moving_obstacles_joy_topic_v2", "/joy0"),
                    ("manual_take_over_joy_input_topic_v2", "/joy"),
                    ("manual_take_over_joy_topic_v2", "/swarm/v2/manual_take_over/joystick"),
                    ("manual_take_over_joy_dev_v2", "/dev/input/js0"),
                    ("odom_visualization_scale_v2", "0.35"),
                ]
            )
            lines.extend(self._emit_arg_block("V2 Optional Arguments", v2_aux_args))

        return lines

    def _generate_gazebo_launch(self):
        return [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- Gazebo Simulation                           -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
            self._indent(1, '<include file="$(find gazebo_ros)/launch/empty_world.launch">'),
            self._indent(2, '<arg name="world_name" value="$(arg world)"/>'),
            self._indent(2, '<arg name="gui" value="$(arg gui)"/>'),
            self._indent(2, '<arg name="paused" value="$(arg paused)"/>'),
            self._indent(2, '<arg name="debug" value="$(arg debug)"/>'),
            self._indent(1, "</include>"),
            self._indent(1, ""),
        ]

    def _generate_uav_instances(self):
        lines = [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- UAV Simulation Layer (iris_X namespace)   -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
        ]

        for drone_id, config in self.uav_configs.items():
            init = config["start"]
            lines.append(self._indent(1, f"<!-- drone_{drone_id} sim -->"))
            lines.append(self._indent(1, '<include file="$(find clean_uav_core)/launch/swarm_uav_sim_instance.launch">'))
            lines.append(self._indent(2, f'<arg name="drone_id" value="{drone_id}"/>'))
            lines.append(self._indent(2, f'<arg name="init_x" value="{init["x"]}"/>'))
            lines.append(self._indent(2, f'<arg name="init_y" value="{init["y"]}"/>'))
            lines.append(self._indent(2, f'<arg name="init_z" value="{init["z"]}"/>'))
            lines.append(self._indent(2, f'<arg name="init_yaw" value="{init["yaw"]}"/>'))
            lines.append(self._indent(1, "</include>"))
            lines.append(self._indent(1, ""))

        lines.extend(
            [
                self._indent(1, "<!-- ========================================== -->"),
                self._indent(1, "<!-- UAV Runtime Layer (drone_X namespace)     -->"),
                self._indent(1, "<!-- ========================================== -->"),
                self._indent(1, ""),
            ]
        )

        common_runtime_args = [
            "enable_vins",
            "use_truth_odom_runtime",
            "sensor_degradation_enabled",
            "sensor_degradation_mode",
            "planner_max_vel",
            "planner_max_acc",
            "planner_max_jerk",
            "planner_planning_horizon",
            "planner_replan_time",
            "planner_emergency_time",
            "planner_weight_time",
            "planner_fail_safe",
            "planner_flight_type",
            "planner_realworld_experiment",
            "use_multitopology_trajs",
            "grid_map_resolution",
            "map_size_x",
            "map_size_y",
            "map_size_z",
            "grid_map_ground_height",
            "grid_map_odom_depth_timeout",
            "grid_map_use_depth_filter",
            "grid_map_depth_filter_tolerance",
            "grid_map_depth_filter_maxdist",
            "grid_map_depth_filter_mindist",
            "grid_map_depth_filter_margin",
            "grid_map_k_depth_scaling_factor",
            "grid_map_skip_pixel",
            "grid_map_local_update_range_x",
            "grid_map_local_update_range_y",
            "grid_map_local_update_range_z",
            "grid_map_obstacles_inflation",
            "hover_percent",
            "mass",
            "px4ctrl_kp",
            "px4ctrl_kv",
            "takeoff_delay",
            "planner_start_z_threshold",
            "planner_start_stable_duration",
            "planner_start_timeout",
            "planner_post_takeoff_delay",
            "swarm_wait_for_all_uavs",
            "swarm_num_uavs",
            "swarm_odom_topic_template",
            "planner_log_to_file",
            "swarm_acceptance_radius",
            "swarm_filter_far_trajectories",
            "swarm_time_warn_threshold",
            "swarm_time_reject_threshold",
            "swarm_clearance",
            "depth_median_filter_size",
        ]
        if self.version == "v2":
            runtime_launch = "swarm_uav_runtime_instance_v2.launch"
            runtime_specific_args = [
                "planner_astar_pool_size",
                "planner_astar_debug_logging",
                "planner_astar_step_factor",
                "planner_weight_obstacle",
                "planner_astar_latency_predict",
                "swarm_weight",
                "swarm_symmetry_gain",
                "planner_goal_topic",
                "traj_server_enable_stale_traj_extrapolation",
                "traj_server_stale_traj_extrapolation_timeout",
                "traj_server_retime_position_cmd_now",
                "traj_server_startup_yaw_hold_time",
                "traj_server_startup_yaw_release_speed",
                "traj_server_startup_yaw_release_distance",
            ]
            broadcast_value_name = "planning_broadcast_topic_v2"
        else:
            runtime_launch = "swarm_uav_runtime_instance.launch"
            runtime_specific_args = [
                "planner_lambda_smooth",
                "swarm_collision_weight",
            ]
            broadcast_value_name = None

        for drone_id, config in self.uav_configs.items():
            init = config["start"]
            goal = config["goal"]
            lines.append(self._indent(1, f"<!-- drone_{drone_id} runtime -->"))
            lines.append(self._indent(1, f'<include file="$(find clean_uav_core)/launch/{runtime_launch}">'))
            lines.append(self._indent(2, f'<arg name="drone_id" value="{drone_id}"/>'))
            lines.append(self._indent(2, f'<arg name="init_x" value="{init["x"]}"/>'))
            lines.append(self._indent(2, f'<arg name="init_y" value="{init["y"]}"/>'))
            lines.append(self._indent(2, f'<arg name="init_z" value="{init["z"]}"/>'))
            lines.append(self._indent(2, '<arg name="goal_mode" value="$(arg mission_manager_mode)"/>'))
            lines.append(self._indent(2, f'<arg name="target_x" value="{init["x"]}"/>'))
            lines.append(self._indent(2, f'<arg name="target_y" value="{init["y"]}"/>'))
            lines.append(self._indent(2, f'<arg name="target_z" value="{init["z"]}"/>'))
            for arg_name in common_runtime_args + runtime_specific_args:
                if arg_name == "planner_flight_type":
                    lines.append(
                        self._indent(
                            2,
                            '<arg name="planner_flight_type" value="$(eval 1 if str(arg(\'enable_mission_manager\')).lower() in [\'true\', \'1\', \'yes\'] and str(arg(\'mission_manager_mode\')).lower() == \'search\' else arg(\'planner_flight_type\'))"/>',
                        )
                    )
                else:
                    lines.append(self._indent(2, f'<arg name="{arg_name}" value="$(arg {arg_name})"/>'))
            if self.version == "v2":
                lines.append(self._indent(2, f'<arg name="planner_log_file" value="$(eval \'%s/%s/drone_%s_ego_planner_v2.log\' % (arg(\'planner_log_root_dir\'), arg(\'planner_log_session\'), \'{drone_id}\'))"/>'))
            if broadcast_value_name is not None:
                lines.append(self._indent(2, f'<arg name="planning_broadcast_topic" value="$(arg {broadcast_value_name})"/>'))
            lines.append(self._indent(1, "</include>"))
            lines.append(self._indent(1, ""))
        return lines

    def _generate_benchmark_suite(self):
        planner_node_name = "ego_planner_v2" if self.version == "v2" else "ego_planner"
        planner_name_segment = planner_node_name
        drone_ids = ",".join(str(drone_id) for drone_id in self.uav_configs.keys())
        goal_xs = ",".join(str(config["goal"]["x"]) for config in self.uav_configs.values())
        goal_ys = ",".join(str(config["goal"]["y"]) for config in self.uav_configs.values())
        goal_zs = ",".join(str(config["goal"]["z"]) for config in self.uav_configs.values())
        start_xs = ",".join(str(config["start"]["x"]) for config in self.uav_configs.values())
        start_ys = ",".join(str(config["start"]["y"]) for config in self.uav_configs.values())
        start_zs = ",".join(str(config["start"]["z"]) for config in self.uav_configs.values())

        return [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- Benchmark Suite                           -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
            self._indent(1, '<group if="$(arg benchmark_enable)" ns="benchmark">'),
            self._indent(2, '<node pkg="clean_uav_core" type="benchmark_manager.py" name="benchmark_manager" output="screen">'),
            self._indent(3, f'<param name="planner_node_name" value="{planner_node_name}"/>'),
            self._indent(3, f'<param name="drone_count" value="{self.num_uavs}"/>'),
            self._indent(3, '<param name="output_dir" value="$(arg benchmark_output_dir)"/>'),
            self._indent(3, '<param name="record_rosbag" value="$(arg benchmark_record_rosbag)"/>'),
            self._indent(3, '<param name="low_speed_threshold" value="$(arg benchmark_low_speed_threshold)"/>'),
            self._indent(3, '<param name="low_speed_duration" value="$(arg benchmark_low_speed_duration)"/>'),
            self._indent(3, '<param name="goal_distance_threshold" value="$(arg benchmark_goal_distance_threshold)"/>'),
            self._indent(3, '<param name="command_age_warn_ms" value="$(arg benchmark_command_age_warn_ms)"/>'),
            self._indent(3, '<param name="command_interval_warn_ms" value="$(arg benchmark_command_interval_warn_ms)"/>'),
            self._indent(3, '<param name="stop_after_terminal_sec" value="$(arg benchmark_stop_after_terminal_sec)"/>'),
            self._indent(3, '<param name="min_session_duration_sec" value="$(arg benchmark_min_session_duration_sec)"/>'),
            self._indent(3, '<param name="max_session_duration_sec" value="$(arg benchmark_max_session_duration_sec)"/>'),
            self._indent(3, '<param name="safety_margin_threshold" value="$(arg benchmark_safety_margin_threshold)"/>'),
            self._indent(3, '<param name="actuator_saturation_threshold" value="$(arg benchmark_actuator_saturation_threshold)"/>'),
            self._indent(3, '<param name="default_vmax" value="$(arg benchmark_default_vmax)"/>'),
            self._indent(3, f'<param name="default_drone_ids" value="{drone_ids}"/>'),
            self._indent(3, '<param name="goal_stop_enabled" value="$(eval str(arg(\'mission_manager_mode\')).lower() != \'search\')"/>'),
            self._indent(3, f'<param name="default_goal_xs" value="{start_xs}"/>'),
            self._indent(3, f'<param name="default_goal_ys" value="{start_ys}"/>'),
            self._indent(3, f'<param name="default_goal_zs" value="{start_zs}"/>'),
            self._indent(3, '<rosparam command="load" file="$(find clean_uav_core)/config/benchmark_research/search_hidden_target_v2.yaml"/>'),
            self._indent(3, '<param name="odom_topic_template" value="/drone_%d/odom"/>'),
            self._indent(3, '<param name="position_cmd_topic_template" value="/drone_%d/position_cmd"/>'),
            self._indent(3, f'<param name="replan_info_topic_template" value="/drone_%d/{planner_name_segment}/planning/replan_info"/>'),
            self._indent(3, f'<param name="planner_event_topic_template" value="/drone_%d/{planner_name_segment}/planning/benchmark_event"/>'),
            self._indent(3, f'<param name="safety_topic_template" value="/drone_%d/{planner_name_segment}/grid_map/occupancy_inflate"/>'),
            self._indent(3, '<param name="attitude_topic_template" value="/iris_%d/mavros/setpoint_raw/attitude"/>'),
            self._indent(3, '<param name="mavros_state_topic_template" value="/iris_%d/mavros/state"/>'),
            self._indent(3, '<param name="extrinsic_topic_template" value="/drone_%d/vins_estimator/extrinsic"/>'),
            self._indent(3, '<param name="px4_debug_topic_template" value="/drone_%d/px4ctrl/debugPx4ctrl"/>'),
            self._indent(2, '</node>'),
            self._indent(1, '</group>'),
            self._indent(1, ""),
        ]

    def _generate_swarm_trigger(self):
        if self.version != "v1":
            return []
        return [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- Synchronized Trajectory Trigger            -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
            self._indent(1, '<node pkg="clean_uav_core" type="swarm_traj_trigger.py" name="swarm_traj_trigger" output="screen">'),
            self._indent(2, f'<param name="num_uavs" value="{self.num_uavs}"/>'),
            self._indent(2, '<param name="delay" value="$(arg traj_trigger_delay)"/>'),
            self._indent(2, '<param name="repeat" value="$(arg traj_trigger_repeat)"/>'),
            self._indent(2, '<param name="rate" value="$(arg traj_trigger_rate)"/>'),
            self._indent(2, '<param name="frame_id" value="world"/>'),
            self._indent(1, "</node>"),
            self._indent(1, ""),
        ]

    def _generate_mission_manager(self):
        target_positions = ";".join(
            f"{config['goal']['x']},{config['goal']['y']},{config['goal']['z']}"
            for config in self.uav_configs.values()
        )
        default_goal_xs = ",".join(str(config["goal"]["x"]) for config in self.uav_configs.values())
        default_goal_ys = ",".join(str(config["goal"]["y"]) for config in self.uav_configs.values())
        default_goal_zs = ",".join(str(config["goal"]["z"]) for config in self.uav_configs.values())
        default_drone_ids = ",".join(str(drone_id) for drone_id in self.uav_configs.keys())
        planner_node_name = "ego_planner_v2" if self.version == "v2" else "ego_planner"

        lines = [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- Cooperative Mission Manager               -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
            self._indent(1, '<group if="$(arg enable_mission_manager)">'),
            self._indent(2, '<node pkg="clean_uav_core" type="swarm_mission_manager.py" name="swarm_mission_manager" output="screen">'),
            self._indent(3, f'<param name="config_file" value="{self.position_config_launch_value}"/>'),
            self._indent(3, f'<param name="planner_node_name" value="{planner_node_name}"/>'),
            self._indent(3, '<param name="mission_manager_mode" value="$(arg mission_manager_mode)"/>'),
            self._indent(3, '<param name="mission_use_v1" value="$(arg mission_use_v1)"/>'),
            self._indent(3, '<param name="mission_use_v2" value="$(arg mission_use_v2)"/>'),
            self._indent(3, '<param name="mission_goal_topic" value="$(arg mission_goal_topic)"/>'),
            self._indent(3, '<param name="mission_goal_topic_template" value="$(arg mission_goal_topic_template)"/>'),
            self._indent(3, '<param name="mission_feedback_topic" value="$(arg mission_feedback_topic)"/>'),
            self._indent(3, '<param name="mission_benchmark_track" value="$(arg mission_benchmark_track)"/>'),
            self._indent(3, '<param name="output_dir" value="$(arg benchmark_output_dir)/0_mission_manager"/>'),
            self._indent(3, f'<param name="default_drone_ids" value="{default_drone_ids}"/>'),
            self._indent(3, '<param name="odom_topic_template" value="$(arg swarm_odom_topic_template)"/>'),
            self._indent(3, '<param name="replan_info_topic_template" value="/drone_%d/$(arg planner_node_name)/planning/replan_info"/>'),
            self._indent(3, '<param name="planner_event_topic_template" value="/drone_%d/$(arg planner_node_name)/planning/benchmark_event"/>'),
            self._indent(3, '<param name="safety_topic_template" value="/drone_%d/$(arg planner_node_name)/grid_map/occupancy_inflate"/>'),
            self._indent(3, '<param name="coverage_metrics_topic" value="/benchmark/coverage_metrics"/>'),
            self._indent(3, '<param name="target_detected_topic" value="/benchmark/target_detected"/>'),
            self._indent(3, '<param name="benchmark_stop_service" value="/benchmark/stop_session"/>'),
            self._indent(3, '<param name="publish_rate" value="$(arg goal_publish_rate)"/>'),
            self._indent(3, '<param name="start_delay" value="$(arg goal_start_delay)"/>'),
            self._indent(3, '<param name="wait_for_all_uavs" value="$(arg swarm_wait_for_all_uavs)"/>'),
            self._indent(3, '<param name="ready_z_threshold" value="$(arg planner_start_z_threshold)"/>'),
            self._indent(3, '<param name="ready_stable_duration" value="$(arg planner_start_stable_duration)"/>'),
            self._indent(3, '<param name="ready_timeout" value="$(arg planner_start_timeout)"/>'),
            self._indent(3, '<param name="ready_post_delay" value="0.0"/>'),
            self._indent(3, '<param name="goal_reached_radius_m" value="$(arg mission_goal_reached_radius_m)"/>'),
            self._indent(3, '<param name="target_confirmation_radius_m" value="$(arg mission_target_confirmation_radius_m)"/>'),
            self._indent(3, '<param name="support_radius_m" value="$(arg mission_support_radius_m)"/>'),
            self._indent(3, '<param name="search_altitude_m" value="$(arg mission_search_altitude_m)"/>'),
            self._indent(3, '<param name="waypoint_spacing_m" value="$(arg mission_waypoint_spacing_m)"/>'),
            self._indent(3, '<param name="search_area_min_x" value="$(arg mission_search_area_min_x)"/>'),
            self._indent(3, '<param name="search_area_max_x" value="$(arg mission_search_area_max_x)"/>'),
            self._indent(3, '<param name="search_area_min_y" value="$(arg mission_search_area_min_y)"/>'),
            self._indent(3, '<param name="search_area_max_y" value="$(arg mission_search_area_max_y)"/>'),
            self._indent(3, '<param name="search_sector_count" value="$(arg mission_search_sector_count)"/>'),
            self._indent(3, '<param name="explore_speed_mps" value="$(arg mission_explore_speed_mps)"/>'),
            self._indent(3, '<param name="track_speed_mps" value="$(arg mission_track_speed_mps)"/>'),
            self._indent(3, '<param name="support_speed_mps" value="$(arg mission_support_speed_mps)"/>'),
            self._indent(3, '<param name="finish_speed_mps" value="$(arg mission_finish_speed_mps)"/>'),
            self._indent(2, '</node>'),
            self._indent(1, '</group>'),
            self._indent(1, ""),
        ]
        return lines

    def _generate_dynamic_commander(self):
        lines = [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- Dynamic Goal Commander                     -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
            self._indent(1, '<group unless="$(arg enable_mission_manager)">'),
        ]
        drone_ids = ",".join(str(drone_id) for drone_id in self.uav_configs.keys())
        start_xs = ",".join(str(config["start"]["x"]) for config in self.uav_configs.values())
        start_ys = ",".join(str(config["start"]["y"]) for config in self.uav_configs.values())
        start_zs = ",".join(str(config["start"]["z"]) for config in self.uav_configs.values())
        if self.version == "v2":
            lines.extend(
                [
                    self._indent(1, '<node pkg="clean_uav_core" type="swarm_dynamic_commander_v2.py" name="swarm_dynamic_commander_v2" output="screen">'),
                    self._indent(2, f'<param name="config_file" value="{self.position_config_launch_value}"/>'),
                    self._indent(2, f'<param name="default_drone_ids" value="{drone_ids}"/>'),
                    self._indent(2, f'<param name="default_goal_xs" value="{start_xs}"/>'),
                    self._indent(2, f'<param name="default_goal_ys" value="{start_ys}"/>'),
                    self._indent(2, f'<param name="default_goal_zs" value="{start_zs}"/>'),
                    self._indent(2, '<param name="start_delay" value="$(arg goal_start_delay)"/>'),
                    self._indent(2, '<param name="publish_rate" value="$(arg goal_publish_rate)"/>'),
                    self._indent(2, '<param name="enable_oscillation" value="$(arg enable_oscillation)"/>'),
                    self._indent(2, '<param name="period_sec" value="$(arg goal_period_sec)"/>'),
                    self._indent(2, '<param name="goal_topic" value="$(arg planner_goal_topic)"/>'),
                    self._indent(2, '<param name="wait_for_all_uavs" value="$(arg swarm_wait_for_all_uavs)"/>'),
                    self._indent(2, '<param name="odom_topic_template" value="$(arg swarm_odom_topic_template)"/>'),
                    self._indent(2, '<param name="ready_z_threshold" value="$(arg planner_start_z_threshold)"/>'),
                    self._indent(2, '<param name="ready_stable_duration" value="$(arg planner_start_stable_duration)"/>'),
                    self._indent(2, '<param name="ready_timeout" value="$(arg planner_start_timeout)"/>'),
                    self._indent(2, '<param name="ready_post_delay" value="0.0"/>'),
                    self._indent(1, "</node>"),
                    self._indent(1, "</group>"),
                    self._indent(1, ""),
                ]
            )
        else:
            lines.extend(
                [
                    self._indent(1, '<node pkg="clean_uav_core" type="swarm_dynamic_commander.py" name="swarm_dynamic_commander" output="screen">'),
                    self._indent(2, f'<param name="config_file" value="{self.position_config_launch_value}"/>'),
                    self._indent(2, f'<param name="default_drone_ids" value="{drone_ids}"/>'),
                    self._indent(2, f'<param name="default_goal_xs" value="{start_xs}"/>'),
                    self._indent(2, f'<param name="default_goal_ys" value="{start_ys}"/>'),
                    self._indent(2, f'<param name="default_goal_zs" value="{start_zs}"/>'),
                    self._indent(2, '<param name="frame_id" value="world"/>'),
                    self._indent(2, '<param name="start_delay" value="$(arg goal_start_delay)"/>'),
                    self._indent(2, '<param name="publish_rate" value="$(arg goal_publish_rate)"/>'),
                    self._indent(2, '<param name="enable_oscillation" value="$(arg enable_oscillation)"/>'),
                    self._indent(2, '<param name="period_sec" value="$(arg goal_period_sec)"/>'),
                    self._indent(1, "</node>"),
                    self._indent(1, "</group>"),
                    self._indent(1, ""),
                ]
            )
        return lines

    def _generate_v2_optional_includes(self):
        if self.version != "v2":
            return []

        lines = [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- Optional V2 Tooling                        -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
            self._indent(1, '<include if="$(arg enable_goal_tooling_v2)" file="$(find clean_uav_core)/launch/swarm_goal_tooling_v2.launch">'),
            self._indent(2, '<arg name="rviz_node_name" value="swarm_rviz"/>'),
            self._indent(2, '<arg name="enable_assign_goals" value="$(arg enable_assign_goals_v2)"/>'),
            self._indent(2, '<arg name="enable_random_goals" value="$(arg enable_random_goals_v2)"/>'),
            self._indent(2, '<arg name="selected_drones_topic" value="$(arg v2_selected_drones_topic)"/>'),
            self._indent(2, '<arg name="pose_goal_topic" value="$(arg v2_pose_goal_topic)"/>'),
            self._indent(2, '<arg name="goalset_topic" value="$(arg planner_goal_topic)"/>'),
            self._indent(2, '<arg name="arrow_topic" value="$(arg v2_goal_arrow_topic)"/>'),
            self._indent(2, f'<arg name="drone_num" value="{self.num_uavs}"/>'),
            self._indent(1, "</include>"),
            self._indent(1, ""),
            self._indent(1, '<include if="$(arg enable_moving_obstacles_v2)" file="$(find clean_uav_core)/launch/swarm_moving_obstacles_v2.launch">'),
            self._indent(2, '<arg name="enable_joy_node" value="$(arg enable_moving_obstacles_joy_node_v2)"/>'),
            self._indent(2, '<arg name="joy_topic" value="$(arg moving_obstacles_joy_topic_v2)"/>'),
            self._indent(2, '<arg name="broadcast_traj_topic" value="$(arg planning_broadcast_topic_v2)"/>'),
            self._indent(1, "</include>"),
            self._indent(1, ""),
            self._indent(1, '<include if="$(arg enable_manual_take_over_station_v2)" file="$(find clean_uav_core)/launch/swarm_manual_take_over_station_v2.launch">'),
            self._indent(2, '<arg name="enable_joy_node" value="$(arg enable_manual_take_over_joy_node_v2)"/>'),
            self._indent(2, '<arg name="joy_input_topic" value="$(arg manual_take_over_joy_input_topic_v2)"/>'),
            self._indent(2, '<arg name="joystick_topic" value="$(arg manual_take_over_joy_topic_v2)"/>'),
            self._indent(2, '<arg name="joy_dev" value="$(arg manual_take_over_joy_dev_v2)"/>'),
            self._indent(1, "</include>"),
            self._indent(1, ""),
        ]

        for drone_id in self.uav_configs.keys():
            lines.extend(
                [
                    self._indent(1, '<include if="$(arg enable_manual_take_over_v2)" file="$(find clean_uav_core)/launch/swarm_manual_take_over_instance_v2.launch">'),
                    self._indent(2, f'<arg name="drone_id" value="{drone_id}"/>'),
                    self._indent(2, '<arg name="joystick_topic" value="$(arg manual_take_over_joy_topic_v2)"/>'),
                    self._indent(1, "</include>"),
                    self._indent(1, ""),
                    self._indent(1, '<include if="$(arg enable_odom_visualization_v2)" file="$(find clean_uav_core)/launch/swarm_odom_visualization_instance_v2.launch">'),
                    self._indent(2, f'<arg name="drone_id" value="{drone_id}"/>'),
                    self._indent(2, '<arg name="robot_scale" value="$(arg odom_visualization_scale_v2)"/>'),
                    self._indent(1, "</include>"),
                    self._indent(1, ""),
                ]
            )
        return lines

    def _generate_rviz(self):
        return [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- RViz Visualization                         -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
            self._indent(1, '<node if="$(arg use_rviz)" pkg="rviz" type="rviz" name="swarm_rviz" args="-d $(arg rviz_config)" output="screen"/>'),
            self._indent(1, ""),
        ]

    def generate_launch_xml(self):
        lines = [
            "<launch>",
            self._indent(0, "<!-- ============================================================= -->"),
            self._indent(0, "<!-- Auto-generated by swarm_launch_generator_yaml.py            -->"),
            self._indent(0, f"<!-- Planner version: {self.version:<44}-->"),
            self._indent(0, f"<!-- Mission profile: {self.profile:<43}-->"),
            self._indent(0, f"<!-- Number of UAVs: {self.num_uavs:<46}-->"),
            self._indent(0, f"<!-- Position config: {self.position_config_launch_value} -->"),
            self._indent(0, f"<!-- Swarm config: {format_launch_config_path(self.swarm_config_path)} -->"),
            self._indent(0, "<!-- ============================================================= -->"),
            self._indent(0, ""),
        ]
        lines.extend(self._generate_global_args())
        lines.extend(self._generate_gazebo_launch())
        lines.extend(self._generate_uav_instances())
        lines.extend(self._generate_swarm_trigger())
        lines.extend(self._generate_mission_manager())
        lines.extend(self._generate_dynamic_commander())
        lines.extend(self._generate_benchmark_suite())
        lines.extend(self._generate_v2_optional_includes())
        lines.extend(self._generate_rviz())
        lines.append("</launch>")
        return "\n".join(lines)

    def _save_xml(self, output_path):
        xml_content = self.generate_launch_xml()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(xml_content, encoding="utf-8")
        return output_path

    def _generate_legacy_variant_paths(self):
        if not self.emit_legacy_variants:
            return []

        paths = []
        for variant_count in LEGACY_VARIANT_COUNTS:
            variant_generator = SwarmLaunchGeneratorYaml(
                position_config_path=self.position_config_path,
                swarm_config_path=self.swarm_config_path,
                output_path=resolve_output_path("", self.version, variant_count),
                version=self.version,
                profile=self.profile,
                uav_count=variant_count,
                world_override=build_legacy_variant_world_path(variant_count),
                emit_legacy_variants=False,
            )
            paths.append(variant_generator._save_xml(variant_generator.output_path))
        return paths

    def save(self):
        generated_paths = [self._save_xml(self.output_path)]
        if self.version == "v1" and self.output_path.name == "swarm_top_level_v1.launch":
            compatibility_path = self.output_path.parent / "swarm_top_level.launch"
            compatibility_xml = "\n".join(
                [
                    "<launch>",
                    '  <include file="$(find clean_uav_core)/launch/swarm_top_level_v1.launch"/>',
                    "</launch>",
                ]
            )
            compatibility_path.write_text(compatibility_xml, encoding="utf-8")
            generated_paths.append(compatibility_path)
        generated_paths.extend(self._generate_legacy_variant_paths())
        return generated_paths


def main():
    parser = argparse.ArgumentParser(description="Generate swarm launch files from YAML mission profiles")
    parser.add_argument("--config", type=str, default="", help="Path to the drone position markdown file")
    parser.add_argument(
        "--swarm-config",
        type=str,
        default="",
        help="Path to the shared swarm YAML config file",
    )
    parser.add_argument("--output", type=str, default="", help="Path to the generated launch file")
    parser.add_argument("--version", type=str, default="v1", choices=SUPPORTED_VERSIONS, help="Planner stack version")
    parser.add_argument("--profile", type=str, default="original", help="Mission profile name from mission_profiles")
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="XY position scale factor applied to all spawn and goal coordinates (default: 1.0)",
    )
    args = parser.parse_args()

    try:
        position_config_path = resolve_path(args.config, DEFAULT_POSITION_CONFIG)
        swarm_config_path = resolve_path(args.swarm_config, DEFAULT_SWARM_CONFIG)
        output_path = resolve_output_path(args.output, args.version)

        generator = SwarmLaunchGeneratorYaml(
            position_config_path=position_config_path,
            swarm_config_path=swarm_config_path,
            output_path=output_path,
            version=args.version,
            profile=args.profile,
            emit_legacy_variants=not args.output,
            scale=args.scale,
        )
        generated_paths = generator.save()
    except (FileNotFoundError, SwarmConfigError) as error:
        print(f"[swarm_launch_generator_yaml] error: {error}", file=sys.stderr)
        sys.exit(1)

    print("=" * 70)
    print("Swarm Launch Generator (YAML)")
    print("=" * 70)
    print(f"Version: {generator.version}")
    print(f"Profile: {generator.profile}")
    print(f"Position config: {position_config_path}")
    print(f"Swarm config: {swarm_config_path}")
    print(f"UAV count: {generator.num_uavs}")
    for path in generated_paths:
        print(f"Generated: {path}")
    print("=" * 70)


if __name__ == "__main__":
    main()