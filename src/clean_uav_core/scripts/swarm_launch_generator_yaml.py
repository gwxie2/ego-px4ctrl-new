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


def _build_phase1_uav_configs_from_ids(drone_ids, radius=15.0, start_z=0.10, goal_z=1.50):
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
        start_x = radius * math.cos(angle_rad)
        start_y = radius * math.sin(angle_rad)
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


def build_phase1_uav_configs(config_path, radius=15.0, start_z=0.10, goal_z=1.50):
    drone_ids = parse_uav_ids(config_path)
    return _build_phase1_uav_configs_from_ids(drone_ids, radius=radius, start_z=start_z, goal_z=goal_z)


def build_phase1_uav_configs_for_count(num_uavs, radius=15.0, start_z=0.10, goal_z=1.50):
    if num_uavs <= 0:
        raise SwarmConfigError(f"num_uavs must be positive, got {num_uavs}")

    drone_ids = list(range(num_uavs))
    return _build_phase1_uav_configs_from_ids(drone_ids, radius=radius, start_z=start_z, goal_z=goal_z)


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

    defaults["grid_map_resolution"] = float_text(planning_base["resolution"])
    defaults["map_size_x"] = float_text(planning_base["map_size_x"])
    defaults["map_size_y"] = float_text(planning_base["map_size_y"])
    defaults["map_size_z"] = float_text(planning_base["map_size_z"])
    defaults["grid_map_ground_height"] = float_text(planning_base["ground_height"])
    defaults["grid_map_odom_depth_timeout"] = float_text(planning_base["odom_depth_timeout"])
    defaults["grid_map_depth_filter_mindist"] = float_text(planning_base["grid_map_depth_filter_mindist"])
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
    ):
        self.version = normalize_version(version)
        self.profile = str(profile).strip() or "original"
        self.position_config_path = position_config_path
        self.swarm_config_path = swarm_config_path
        self.output_path = output_path
        self.uav_count = uav_count
        self.emit_legacy_variants = bool(emit_legacy_variants)

        if self.uav_count is None:
            self.uav_configs = build_phase1_uav_configs(position_config_path)
        else:
            self.uav_configs = build_phase1_uav_configs_for_count(self.uav_count)
        self.num_uavs = len(self.uav_configs)
        self.shared_config = load_yaml_config(swarm_config_path)
        self.defaults = build_runtime_defaults(self.shared_config, self.profile, self.version)
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
                ("planner_lambda_smooth", self.defaults["planner_lambda_smooth"]),
                ("planner_fail_safe", self.defaults["planner_fail_safe"]),
                ("planner_flight_type", self.defaults["planner_flight_type"]),
                ("planner_realworld_experiment", self.defaults["planner_realworld_experiment"]),
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
                ("grid_map_depth_filter_mindist", self.defaults["grid_map_depth_filter_mindist"]),
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
                ("traj_trigger_delay", "20.0"),
                ("traj_trigger_repeat", "1"),
                ("traj_trigger_rate", "3.0"),
                ("goal_start_delay", "$(eval float(arg('takeoff_delay')) + 12.0)"),
                ("goal_publish_rate", "0.2"),
                ("goal_period_sec", "30.0"),
                ("enable_oscillation", "true"),
            ]
        )

        lines = []
        lines.extend(self._emit_arg_block("Global Arguments", common_args))
        lines.extend(self._emit_arg_block("Mission Profile Arguments", mission_args))
        lines.extend(self._emit_arg_block("Grid Map Arguments", map_args))
        lines.extend(self._emit_arg_block("Physical UAV Arguments", physical_args))
        lines.extend(self._emit_arg_block("Timing Arguments", timing_args))

        if self.version == "v2":
            v2_aux_args = OrderedDict(
                [
                    ("planner_goal_topic", "/goal_with_id"),
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
            self._indent(1, "<!-- UAV Full-stack Instances                   -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
        ]

        forwarded_args = [
            "planner_version",
            "enable_vins",
            "use_truth_odom_runtime",
            "planner_max_vel",
            "planner_max_acc",
            "planner_max_jerk",
            "planner_planning_horizon",
            "planner_replan_time",
            "planner_emergency_time",
            "planner_weight_time",
            "planner_lambda_smooth",
            "planner_fail_safe",
            "planner_flight_type",
            "planner_realworld_experiment",
            "grid_map_resolution",
            "map_size_x",
            "map_size_y",
            "map_size_z",
            "grid_map_ground_height",
            "grid_map_odom_depth_timeout",
            "grid_map_depth_filter_mindist",
            "grid_map_local_update_range_x",
            "grid_map_local_update_range_y",
            "grid_map_local_update_range_z",
            "grid_map_obstacles_inflation",
            "mass",
            "hover_percent",
            "px4ctrl_kp",
            "px4ctrl_kv",
            "takeoff_delay",
            "planner_start_z_threshold",
            "planner_start_stable_duration",
            "planner_start_timeout",
            "planner_post_takeoff_delay",
            "swarm_acceptance_radius",
            "swarm_filter_far_trajectories",
            "swarm_time_warn_threshold",
            "swarm_time_reject_threshold",
            "swarm_clearance",
            "swarm_collision_weight",
            "swarm_weight",
            "swarm_symmetry_gain",
        ]
        if self.version == "v2":
            forwarded_args.extend(["planner_goal_topic", "planning_broadcast_topic_v2"])

        for drone_id, config in self.uav_configs.items():
            init = config["start"]
            goal = config["goal"]
            lines.append(self._indent(1, f"<!-- drone_{drone_id} -->"))
            lines.append(self._indent(1, '<include file="$(find clean_uav_core)/launch/swarm_uav_instance.launch">'))
            lines.append(self._indent(2, f'<arg name="drone_id" value="{drone_id}"/>'))
            lines.append(self._indent(2, f'<arg name="init_x" value="{init["x"]}"/>'))
            lines.append(self._indent(2, f'<arg name="init_y" value="{init["y"]}"/>'))
            lines.append(self._indent(2, f'<arg name="init_z" value="{init["z"]}"/>'))
            lines.append(self._indent(2, f'<arg name="init_yaw" value="{init["yaw"]}"/>'))
            lines.append(self._indent(2, f'<arg name="target_x" value="{goal["x"]}"/>'))
            lines.append(self._indent(2, f'<arg name="target_y" value="{goal["y"]}"/>'))
            lines.append(self._indent(2, f'<arg name="target_z" value="{goal["z"]}"/>'))
            for arg_name in forwarded_args:
                value_name = "planning_broadcast_topic_v2" if arg_name == "planning_broadcast_topic_v2" else arg_name
                pass_name = "planning_broadcast_topic" if arg_name == "planning_broadcast_topic_v2" else arg_name
                lines.append(self._indent(2, f'<arg name="{pass_name}" value="$(arg {value_name})"/>'))
            lines.append(self._indent(1, "</include>"))
            lines.append(self._indent(1, ""))
        return lines

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

    def _generate_dynamic_commander(self):
        lines = [
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, "<!-- Dynamic Goal Commander                     -->"),
            self._indent(1, "<!-- ========================================== -->"),
            self._indent(1, ""),
        ]
        if self.version == "v2":
            lines.extend(
                [
                    self._indent(1, '<node pkg="clean_uav_core" type="swarm_dynamic_commander_v2.py" name="swarm_dynamic_commander_v2" output="screen">'),
                    self._indent(2, f'<param name="config_file" value="{self.position_config_launch_value}"/>'),
                    self._indent(2, '<param name="start_delay" value="$(arg goal_start_delay)"/>'),
                    self._indent(2, '<param name="publish_rate" value="$(arg goal_publish_rate)"/>'),
                    self._indent(2, '<param name="enable_oscillation" value="$(arg enable_oscillation)"/>'),
                    self._indent(2, '<param name="period_sec" value="$(arg goal_period_sec)"/>'),
                    self._indent(2, '<param name="goal_topic" value="$(arg planner_goal_topic)"/>'),
                    self._indent(1, "</node>"),
                    self._indent(1, ""),
                ]
            )
        else:
            lines.extend(
                [
                    self._indent(1, '<node pkg="clean_uav_core" type="swarm_dynamic_commander.py" name="swarm_dynamic_commander" output="screen">'),
                    self._indent(2, f'<param name="config_file" value="{self.position_config_launch_value}"/>'),
                    self._indent(2, '<param name="frame_id" value="world"/>'),
                    self._indent(2, '<param name="start_delay" value="$(arg goal_start_delay)"/>'),
                    self._indent(2, '<param name="publish_rate" value="$(arg goal_publish_rate)"/>'),
                    self._indent(2, '<param name="enable_oscillation" value="$(arg enable_oscillation)"/>'),
                    self._indent(2, '<param name="period_sec" value="$(arg goal_period_sec)"/>'),
                    self._indent(1, "</node>"),
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
        lines.extend(self._generate_dynamic_commander())
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