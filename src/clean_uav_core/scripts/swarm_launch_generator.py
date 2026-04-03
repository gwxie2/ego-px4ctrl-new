#!/usr/bin/env python3
"""Generate versioned top-level swarm launch files from the Phase 1 position rules."""

import argparse
import math
import re
import sys
from pathlib import Path

import rospkg


SUPPORTED_VERSIONS = ("v1", "v2")


def get_clean_uav_core_path():
    return rospkg.RosPack().get_path("clean_uav_core")


def get_workspace_root():
    clean_uav_core_path = Path(get_clean_uav_core_path())
    return clean_uav_core_path.parent.parent


def resolve_config_path(config_path):
    workspace_root = get_workspace_root()
    if not config_path:
        return workspace_root / "docs" / "uav_position_goal.md"

    candidate = Path(config_path)
    if candidate.is_absolute():
        return candidate

    workspace_candidate = workspace_root / candidate
    if workspace_candidate.exists():
        return workspace_candidate

    package_candidate = Path(get_clean_uav_core_path()) / candidate
    if package_candidate.exists():
        return package_candidate

    return workspace_candidate


def normalize_version(version):
    normalized = str(version).strip().lower()
    if normalized not in SUPPORTED_VERSIONS:
        raise ValueError(f"unsupported version '{version}', expected one of {SUPPORTED_VERSIONS}")
    return normalized


def resolve_output_path(output_path, version, num_uavs=None):
    workspace_root = get_workspace_root()
    if not output_path:
        suffix = f"_{num_uavs}UAV" if num_uavs else ""
        return workspace_root / "src" / "clean_uav_core" / "launch" / f"swarm_top_level_{version}{suffix}.launch"

    candidate = Path(output_path)
    if candidate.is_absolute():
        return candidate

    return workspace_root / candidate


def ensure_v2_requirements():
    clean_uav_core_path = Path(get_clean_uav_core_path())
    workspace_root = get_workspace_root()
    missing = []

    try:
        rospkg.RosPack().get_path("ego_planner_v2")
    except rospkg.ResourceNotFound:
        missing.append("ROS package ego_planner_v2")

    runtime_template = clean_uav_core_path / "launch" / "swarm_uav_runtime_instance_v2.launch"
    if not runtime_template.exists():
        missing.append(str(runtime_template))

    goalset_msg = workspace_root / "src" / "quadrotor_msgs" / "msg" / "GoalSet.msg"
    if not goalset_msg.exists():
        missing.append(str(goalset_msg))

    if missing:
        raise FileNotFoundError("missing V2 prerequisites: " + ", ".join(missing))


def parse_uav_ids(config_path):
    drone_ids = []
    pattern = re.compile(r"`?drone_(\d+)`?", re.IGNORECASE)

    try:
        with config_path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue

                match = pattern.search(stripped)
                if not match:
                    continue

                drone_ids.append(int(match.group(1)))
    except FileNotFoundError as error:
        raise FileNotFoundError(f"config file not found: {config_path}") from error

    if not drone_ids:
        raise ValueError(f"no drone configuration found in {config_path}")

    actual_ids = sorted(set(drone_ids))
    expected_ids = list(range(len(actual_ids)))
    if actual_ids != expected_ids:
        raise ValueError(f"drone ids must be contiguous starting from 0, got {actual_ids}")

    return actual_ids


def _build_phase1_uav_configs_from_ids(drone_ids, radius=15.0, start_z=0.10, goal_z=1.50):
    num_uavs = len(drone_ids)

    if num_uavs == 1:
        angles_deg = [0.0]
    else:
        span_deg = min(160.0, 30.0 * (num_uavs - 1))
        step_deg = span_deg / (num_uavs - 1)
        angles_deg = [-span_deg / 2.0 + step_deg * index for index in range(num_uavs)]

    uav_configs = {}
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
        raise ValueError(f"num_uavs must be positive, got {num_uavs}")

    drone_ids = list(range(num_uavs))
    return _build_phase1_uav_configs_from_ids(drone_ids, radius=radius, start_z=start_z, goal_z=goal_z)


class SwarmLaunchGenerator:
    """Generate the composed top-level swarm launch file."""

    def __init__(self, config_path, output_path, version, uav_count=None):
        self.config_path = config_path
        self.output_path = output_path
        self.version = normalize_version(version)
        self.uav_count = uav_count
        self.runtime_launch = "swarm_uav_runtime_instance.launch"
        self.commander_script = "swarm_dynamic_commander.py"
        self.commander_name = "swarm_dynamic_commander"
        self.include_swarm_trigger = True

        if self.version == "v2":
            ensure_v2_requirements()
            self.runtime_launch = "swarm_uav_runtime_instance_v2.launch"
            self.commander_script = "swarm_dynamic_commander_v2.py"
            self.commander_name = "swarm_dynamic_commander_v2"
            self.include_swarm_trigger = False

        self.benchmark_planner_node_name = "ego_planner_v2" if self.version == "v2" else "ego_planner"

        if self.uav_count is None:
            self.uav_configs = build_phase1_uav_configs(config_path)
        else:
            self.uav_configs = build_phase1_uav_configs_for_count(self.uav_count)
        self.num_uavs = len(self.uav_configs)
        self.world_name = f"swarm_benchmark_forest_phase2_{self.num_uavs}UAV.world"
        self.swarm_defaults = self._build_swarm_defaults()

        print(
            f"[Generator] Built {self.num_uavs} UAV configs for {self.version} from Phase 1 rules in {self.config_path}"
        )

    def _indent(self, level, text=""):
        """生成缩进"""
        return "  " * level + text

    def _build_swarm_defaults(self):
          defaults = {
              "swarm_acceptance_radius": "-1.0",
              "swarm_filter_far_trajectories": "false",
              "swarm_time_warn_threshold": "0.25",
              "swarm_time_reject_threshold": "10.0",
          }

          if self.version == "v2":
              defaults["swarm_clearance"] = "0.35"
              defaults["swarm_weight"] = "10000.0"
              defaults["swarm_symmetry_gain"] = "0.0"
              if self.num_uavs >= 6:
                  defaults["swarm_clearance"] = "0.55"
                  defaults["swarm_weight"] = "25000.0"
                  defaults["swarm_symmetry_gain"] = "0.03"
          else:
              defaults["swarm_clearance"] = "0.5"
              defaults["swarm_collision_weight"] = "0.5"
              if self.num_uavs >= 6:
                  defaults["swarm_clearance"] = "0.65"
                  defaults["swarm_collision_weight"] = "1.0"

          return defaults

    def _generate_global_args(self):
          lines = []
          lines.append(self._indent(1, "<!-- ========================================== -->"))
          lines.append(self._indent(1, "<!-- Global Arguments                            -->"))
          lines.append(self._indent(1, "<!-- ========================================== -->"))
          lines.append(self._indent(1, ""))

          lines.append(self._indent(1, f"<arg name=\"world\" default=\"$(find px4)/Tools/sitl_gazebo/worlds/{self.world_name}\"/>"))
          lines.append(self._indent(1, "<arg name=\"gui\" default=\"false\"/>"))
          lines.append(self._indent(1, "<arg name=\"paused\" default=\"false\"/>"))
          lines.append(self._indent(1, "<arg name=\"debug\" default=\"false\"/>"))
          lines.append(self._indent(1, ""))

          lines.append(self._indent(1, "<arg name=\"use_rviz\" default=\"true\"/>"))
          rviz_config_default = "$(find clean_uav_core)/rviz/swarm_rviz_v2.rviz" if self.version == "v2" else "$(find clean_uav_core)/rviz/swarm_rviz.rviz"
          lines.append(self._indent(1, f"<arg name=\"rviz_config\" default=\"{rviz_config_default}\"/>"))
          lines.append(self._indent(1, ""))

          lines.append(self._indent(1, "<arg name=\"enable_vins\" default=\"false\"/>"))
          lines.append(self._indent(1, "<arg name=\"use_truth_odom_runtime\" default=\"true\"/>"))
          lines.append(self._indent(1, ""))

          lines.append(self._indent(1, "<!-- ===== 规划器参数 ===== -->"))
          lines.append(self._indent(1, "<arg name=\"planner_max_vel\" default=\"1.2\"/>"))
          lines.append(self._indent(1, "<arg name=\"planner_max_acc\" default=\"2.8\"/>"))
          lines.append(self._indent(1, "<arg name=\"planner_max_jerk\" default=\"4.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"planner_fail_safe\" default=\"true\"/>"))
          planner_flight_type_default = "2" if self.version == "v2" else "1"
          lines.append(self._indent(1, f"<arg name=\"planner_flight_type\" default=\"{planner_flight_type_default}\"/>"))
          lines.append(self._indent(1, "<arg name=\"planner_realworld_experiment\" default=\"false\"/>"))
          lines.append(self._indent(1, f"<arg name=\"swarm_acceptance_radius\" default=\"{self.swarm_defaults['swarm_acceptance_radius']}\"/>"))
          lines.append(self._indent(1, f"<arg name=\"swarm_filter_far_trajectories\" default=\"{self.swarm_defaults['swarm_filter_far_trajectories']}\"/>"))
          lines.append(self._indent(1, f"<arg name=\"swarm_time_warn_threshold\" default=\"{self.swarm_defaults['swarm_time_warn_threshold']}\"/>"))
          lines.append(self._indent(1, f"<arg name=\"swarm_time_reject_threshold\" default=\"{self.swarm_defaults['swarm_time_reject_threshold']}\"/>"))
          lines.append(self._indent(1, f"<arg name=\"swarm_clearance\" default=\"{self.swarm_defaults['swarm_clearance']}\"/>"))
          if self.version == "v2":
              lines.append(self._indent(1, f"<arg name=\"swarm_weight\" default=\"{self.swarm_defaults['swarm_weight']}\"/>"))
              lines.append(self._indent(1, f"<arg name=\"swarm_symmetry_gain\" default=\"{self.swarm_defaults['swarm_symmetry_gain']}\"/>"))
          else:
              lines.append(self._indent(1, f"<arg name=\"swarm_collision_weight\" default=\"{self.swarm_defaults['swarm_collision_weight']}\"/>"))
          lines.append(self._indent(1, ""))

          if self.version == "v2":
              lines.append(self._indent(1, "<!-- ===== V2 sandbox topics ===== -->"))
              lines.append(self._indent(1, "<arg name=\"planner_goal_topic\" default=\"/goal_with_id\"/>"))
              lines.append(self._indent(1, "<arg name=\"planning_broadcast_topic_v2\" default=\"/swarm/v2/broadcast_traj\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_goal_tooling_v2\" default=\"false\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_assign_goals_v2\" default=\"true\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_random_goals_v2\" default=\"false\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_moving_obstacles_v2\" default=\"false\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_manual_take_over_v2\" default=\"false\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_manual_take_over_station_v2\" default=\"false\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_manual_take_over_joy_node_v2\" default=\"true\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_odom_visualization_v2\" default=\"false\"/>"))
              lines.append(self._indent(1, "<arg name=\"v2_selected_drones_topic\" default=\"/swarm/v2/rviz_selected_drones\"/>"))
              lines.append(self._indent(1, "<arg name=\"v2_pose_goal_topic\" default=\"/swarm/v2/goal_pose\"/>"))
              lines.append(self._indent(1, "<arg name=\"v2_goal_arrow_topic\" default=\"/swarm/v2/new_goals_arrow\"/>"))
              lines.append(self._indent(1, "<arg name=\"enable_moving_obstacles_joy_node_v2\" default=\"false\"/>"))
              lines.append(self._indent(1, "<arg name=\"moving_obstacles_joy_topic_v2\" default=\"/joy0\"/>"))
              lines.append(self._indent(1, "<arg name=\"manual_take_over_joy_input_topic_v2\" default=\"/joy\"/>"))
              lines.append(self._indent(1, "<arg name=\"manual_take_over_joy_topic_v2\" default=\"/swarm/v2/manual_take_over/joystick\"/>"))
              lines.append(self._indent(1, "<arg name=\"manual_take_over_joy_dev_v2\" default=\"/dev/input/js0\"/>"))
              lines.append(self._indent(1, "<arg name=\"odom_visualization_scale_v2\" default=\"0.35\"/>"))
              lines.append(self._indent(1, ""))

          lines.append(self._indent(1, "<!-- ===== Benchmark Suite ===== -->"))
          lines.append(self._indent(1, "<arg name=\"benchmark_enable\" default=\"false\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_output_dir\" default=\"$(env HOME)/swarm_benchmark/data\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_record_rosbag\" default=\"true\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_low_speed_threshold\" default=\"0.1\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_low_speed_duration\" default=\"2.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_goal_distance_threshold\" default=\"0.5\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_stop_after_terminal_sec\" default=\"2.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_safety_margin_threshold\" default=\"$(arg swarm_clearance)\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_actuator_saturation_threshold\" default=\"0.9\"/>"))
          lines.append(self._indent(1, "<arg name=\"benchmark_default_vmax\" default=\"10.0\"/>"))
          lines.append(self._indent(1, ""))

          lines.append(self._indent(1, "<!-- ===== Grid Map 参数 ===== -->"))
          lines.append(self._indent(1, "<arg name=\"map_size_x\" default=\"80.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"map_size_y\" default=\"80.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"map_size_z\" default=\"5.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"grid_map_ground_height\" default=\"-0.01\"/>"))
          lines.append(self._indent(1, "<arg name=\"grid_map_odom_depth_timeout\" default=\"2.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"grid_map_depth_filter_mindist\" default=\"0.1\"/>"))
          lines.append(self._indent(1, "<arg name=\"grid_map_obstacles_inflation\" default=\"0.45\"/>"))
          lines.append(self._indent(1, ""))

          lines.append(self._indent(1, "<!-- ===== 飞控器参数（核心调试参数） ===== -->"))
          lines.append(self._indent(1, "<arg name=\"hover_percent\" default=\"0.58\"/>"))
          lines.append(self._indent(1, "<arg name=\"mass\" default=\"1.2\"/>"))
          lines.append(self._indent(1, ""))

          lines.append(self._indent(1, "<!-- ===== 触发器时序 ===== -->"))
          lines.append(self._indent(1, "<arg name=\"takeoff_delay\" default=\"8.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"planner_start_z_threshold\" default=\"0.8\"/>"))
          lines.append(self._indent(1, "<arg name=\"planner_start_stable_duration\" default=\"1.5\"/>"))
          lines.append(self._indent(1, "<arg name=\"planner_start_timeout\" default=\"60.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"planner_post_takeoff_delay\" default=\"1.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"traj_trigger_delay\" default=\"20.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"traj_trigger_repeat\" default=\"1\"/>"))
          lines.append(self._indent(1, "<arg name=\"traj_trigger_rate\" default=\"3.0\"/>"))
          lines.append(self._indent(1, ""))

          lines.append(self._indent(1, "<!-- ===== 动态目标参数 ===== -->"))
          lines.append(self._indent(1, "<arg name=\"goal_start_delay\" default=\"$(eval float(arg('takeoff_delay')) + 12.0)\"/>"))
          lines.append(self._indent(1, "<arg name=\"goal_publish_rate\" default=\"0.2\"/>"))
          lines.append(self._indent(1, "<arg name=\"goal_period_sec\" default=\"30.0\"/>"))
          lines.append(self._indent(1, "<arg name=\"enable_oscillation\" default=\"true\"/>"))
          lines.append(self._indent(1, ""))

          return lines

    def _generate_gazebo_launch(self):
        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- Gazebo Simulation                             -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))
        lines.append(self._indent(1, "<include file=\"$(find gazebo_ros)/launch/empty_world.launch\">"))
        lines.append(self._indent(2, "<arg name=\"world_name\" value=\"$(arg world)\"/>"))
        lines.append(self._indent(2, "<arg name=\"gui\" value=\"$(arg gui)\"/>"))
        lines.append(self._indent(2, "<arg name=\"paused\" value=\"$(arg paused)\"/>"))
        lines.append(self._indent(2, "<arg name=\"debug\" value=\"$(arg debug)\"/>"))
        lines.append(self._indent(1, "</include>"))
        lines.append(self._indent(1, ""))
        return lines

    def _generate_uav_instances(self):
        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- UAV Simulation Layer (iris_X namespace)       -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))

        for drone_id in sorted(self.uav_configs.keys()):
            config = self.uav_configs[drone_id]
            init = config["start"]
            goal = config["goal"]

            lines.append(self._indent(1, f"<!-- drone_{drone_id} sim -->"))
            lines.append(self._indent(1, "<include file=\"$(find clean_uav_core)/launch/swarm_uav_sim_instance.launch\">"))

            lines.append(self._indent(2, f"<arg name=\"drone_id\" value=\"{drone_id}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"init_x\" value=\"{init['x']}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"init_y\" value=\"{init['y']}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"init_z\" value=\"{init['z']}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"init_yaw\" value=\"{init['yaw']}\"/>"))
            lines.append(self._indent(1, "</include>"))
            lines.append(self._indent(1, ""))

        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- UAV Runtime Layer (drone_X namespace)       -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))

        for drone_id in sorted(self.uav_configs.keys()):
            config = self.uav_configs[drone_id]
            init = config["start"]
            goal = config["goal"]

            lines.append(self._indent(1, f"<!-- drone_{drone_id} runtime -->"))
            lines.append(self._indent(1, f"<include file=\"$(find clean_uav_core)/launch/{self.runtime_launch}\">"))

            lines.append(self._indent(2, f"<arg name=\"drone_id\" value=\"{drone_id}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"init_x\" value=\"{init['x']}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"init_y\" value=\"{init['y']}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"init_z\" value=\"{init['z']}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"target_x\" value=\"{goal['x']}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"target_y\" value=\"{goal['y']}\"/>"))
            lines.append(self._indent(2, f"<arg name=\"target_z\" value=\"{goal['z']}\"/>"))
            lines.append(self._indent(2, "<arg name=\"enable_vins\" value=\"$(arg enable_vins)\"/>"))
            lines.append(self._indent(2, "<arg name=\"use_truth_odom_runtime\" value=\"$(arg use_truth_odom_runtime)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_max_vel\" value=\"$(arg planner_max_vel)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_max_acc\" value=\"$(arg planner_max_acc)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_max_jerk\" value=\"$(arg planner_max_jerk)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_fail_safe\" value=\"$(arg planner_fail_safe)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_flight_type\" value=\"$(arg planner_flight_type)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_realworld_experiment\" value=\"$(arg planner_realworld_experiment)\"/>"))
            lines.append(self._indent(2, "<arg name=\"map_size_x\" value=\"$(arg map_size_x)\"/>"))
            lines.append(self._indent(2, "<arg name=\"map_size_y\" value=\"$(arg map_size_y)\"/>"))
            lines.append(self._indent(2, "<arg name=\"map_size_z\" value=\"$(arg map_size_z)\"/>"))
            lines.append(self._indent(2, "<arg name=\"grid_map_ground_height\" value=\"$(arg grid_map_ground_height)\"/>"))
            lines.append(self._indent(2, "<arg name=\"grid_map_odom_depth_timeout\" value=\"$(arg grid_map_odom_depth_timeout)\"/>"))
            lines.append(self._indent(2, "<arg name=\"grid_map_depth_filter_mindist\" value=\"$(arg grid_map_depth_filter_mindist)\"/>"))
            lines.append(self._indent(2, "<arg name=\"grid_map_obstacles_inflation\" value=\"$(arg grid_map_obstacles_inflation)\"/>"))
            lines.append(self._indent(2, "<arg name=\"hover_percent\" value=\"$(arg hover_percent)\"/>"))
            lines.append(self._indent(2, "<arg name=\"mass\" value=\"$(arg mass)\"/>"))
            lines.append(self._indent(2, "<arg name=\"takeoff_delay\" value=\"$(arg takeoff_delay)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_start_z_threshold\" value=\"$(arg planner_start_z_threshold)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_start_stable_duration\" value=\"$(arg planner_start_stable_duration)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_start_timeout\" value=\"$(arg planner_start_timeout)\"/>"))
            lines.append(self._indent(2, "<arg name=\"planner_post_takeoff_delay\" value=\"$(arg planner_post_takeoff_delay)\"/>"))
            lines.append(self._indent(2, "<arg name=\"swarm_acceptance_radius\" value=\"$(arg swarm_acceptance_radius)\"/>"))
            lines.append(self._indent(2, "<arg name=\"swarm_filter_far_trajectories\" value=\"$(arg swarm_filter_far_trajectories)\"/>"))
            lines.append(self._indent(2, "<arg name=\"swarm_time_warn_threshold\" value=\"$(arg swarm_time_warn_threshold)\"/>"))
            lines.append(self._indent(2, "<arg name=\"swarm_time_reject_threshold\" value=\"$(arg swarm_time_reject_threshold)\"/>"))
            lines.append(self._indent(2, "<arg name=\"swarm_clearance\" value=\"$(arg swarm_clearance)\"/>"))
            if self.version == "v2":
                lines.append(self._indent(2, "<arg name=\"swarm_weight\" value=\"$(arg swarm_weight)\"/>"))
                lines.append(self._indent(2, "<arg name=\"swarm_symmetry_gain\" value=\"$(arg swarm_symmetry_gain)\"/>"))
                lines.append(self._indent(2, "<arg name=\"planner_goal_topic\" value=\"$(arg planner_goal_topic)\"/>"))
                lines.append(self._indent(2, "<arg name=\"planning_broadcast_topic\" value=\"$(arg planning_broadcast_topic_v2)\"/>"))
            else:
                lines.append(self._indent(2, "<arg name=\"swarm_collision_weight\" value=\"$(arg swarm_collision_weight)\"/>"))

            lines.append(self._indent(1, "</include>"))
            lines.append(self._indent(1, ""))

        return lines

    def _generate_swarm_trigger(self):
        if not self.include_swarm_trigger:
            return []

        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- Synchronized Trajectory Trigger               -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))
        lines.append(self._indent(1, "<node pkg=\"clean_uav_core\" type=\"swarm_traj_trigger.py\" name=\"swarm_traj_trigger\" output=\"screen\">"))
        lines.append(self._indent(2, f"<param name=\"num_uavs\" value=\"{self.num_uavs}\"/>"))
        lines.append(self._indent(2, "<param name=\"delay\" value=\"$(arg traj_trigger_delay)\"/>"))
        lines.append(self._indent(2, "<param name=\"repeat\" value=\"$(arg traj_trigger_repeat)\"/>"))
        lines.append(self._indent(2, "<param name=\"rate\" value=\"$(arg traj_trigger_rate)\"/>"))
        lines.append(self._indent(2, "<param name=\"frame_id\" value=\"world\"/>"))
        lines.append(self._indent(1, "</node>"))
        lines.append(self._indent(1, ""))
        return lines

    def _generate_dynamic_commander(self):
        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- Dynamic Goal Commander                        -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))
        lines.append(self._indent(1, f"<node pkg=\"clean_uav_core\" type=\"{self.commander_script}\" name=\"{self.commander_name}\" output=\"screen\">"))
        lines.append(self._indent(2, f"<param name=\"config_file\" value=\"{self.config_path}\"/>"))
        lines.append(self._indent(2, "<param name=\"start_delay\" value=\"$(arg goal_start_delay)\"/>"))
        lines.append(self._indent(2, "<param name=\"publish_rate\" value=\"$(arg goal_publish_rate)\"/>"))
        lines.append(self._indent(2, "<param name=\"enable_oscillation\" value=\"$(arg enable_oscillation)\"/>"))
        lines.append(self._indent(2, "<param name=\"period_sec\" value=\"$(arg goal_period_sec)\"/>"))
        if self.version == "v1":
            lines.append(self._indent(2, "<param name=\"frame_id\" value=\"world\"/>"))
        else:
            lines.append(self._indent(2, "<param name=\"goal_topic\" value=\"$(arg planner_goal_topic)\"/>"))
        lines.append(self._indent(1, "</node>"))
        lines.append(self._indent(1, ""))
        return lines

    def _generate_benchmark_manager(self):
        lines = []
        goal_xs = ",".join(f"{self.uav_configs[drone_id]['goal']['x']}" for drone_id in sorted(self.uav_configs.keys()))
        goal_ys = ",".join(f"{self.uav_configs[drone_id]['goal']['y']}" for drone_id in sorted(self.uav_configs.keys()))
        goal_zs = ",".join(f"{self.uav_configs[drone_id]['goal']['z']}" for drone_id in sorted(self.uav_configs.keys()))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- Benchmark Suite                             -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))
        lines.append(self._indent(1, "<group if=\"$(arg benchmark_enable)\" ns=\"benchmark\">"))
        lines.append(self._indent(2, "<node pkg=\"clean_uav_core\" type=\"benchmark_manager.py\" name=\"benchmark_manager\" output=\"screen\">"))
        lines.append(self._indent(3, f"<param name=\"planner_node_name\" value=\"{self.benchmark_planner_node_name}\"/>"))
        lines.append(self._indent(3, f"<param name=\"drone_count\" value=\"{self.num_uavs}\"/>"))
        lines.append(self._indent(3, "<param name=\"output_dir\" value=\"$(arg benchmark_output_dir)\"/>"))
        lines.append(self._indent(3, "<param name=\"record_rosbag\" value=\"$(arg benchmark_record_rosbag)\"/>"))
        lines.append(self._indent(3, "<param name=\"low_speed_threshold\" value=\"$(arg benchmark_low_speed_threshold)\"/>"))
        lines.append(self._indent(3, "<param name=\"low_speed_duration\" value=\"$(arg benchmark_low_speed_duration)\"/>"))
        lines.append(self._indent(3, "<param name=\"goal_distance_threshold\" value=\"$(arg benchmark_goal_distance_threshold)\"/>"))
        lines.append(self._indent(3, "<param name=\"stop_after_terminal_sec\" value=\"$(arg benchmark_stop_after_terminal_sec)\"/>"))
        lines.append(self._indent(3, "<param name=\"safety_margin_threshold\" value=\"$(arg benchmark_safety_margin_threshold)\"/>"))
        lines.append(self._indent(3, "<param name=\"actuator_saturation_threshold\" value=\"$(arg benchmark_actuator_saturation_threshold)\"/>"))
        lines.append(self._indent(3, "<param name=\"default_vmax\" value=\"$(arg benchmark_default_vmax)\"/>"))
        lines.append(self._indent(3, f"<param name=\"default_drone_ids\" value=\"{','.join(str(drone_id) for drone_id in sorted(self.uav_configs.keys()))}\"/>"))
        lines.append(self._indent(3, f"<param name=\"default_goal_xs\" value=\"{goal_xs}\"/>"))
        lines.append(self._indent(3, f"<param name=\"default_goal_ys\" value=\"{goal_ys}\"/>"))
        lines.append(self._indent(3, f"<param name=\"default_goal_zs\" value=\"{goal_zs}\"/>"))
        lines.append(self._indent(3, "<param name=\"odom_topic_template\" value=\"/drone_%d/odom\"/>"))
        lines.append(self._indent(3, "<param name=\"position_cmd_topic_template\" value=\"/drone_%d/position_cmd\"/>"))
        lines.append(self._indent(3, f"<param name=\"replan_info_topic_template\" value=\"/drone_%d/{self.benchmark_planner_node_name}/planning/replan_info\"/>"))
        lines.append(self._indent(3, f"<param name=\"planner_event_topic_template\" value=\"/drone_%d/{self.benchmark_planner_node_name}/planning/benchmark_event\"/>"))
        lines.append(self._indent(3, f"<param name=\"safety_topic_template\" value=\"/drone_%d/{self.benchmark_planner_node_name}/grid_map/occupancy_inflate\"/>"))
        lines.append(self._indent(3, "<param name=\"attitude_topic_template\" value=\"/iris_%d/mavros/setpoint_raw/attitude\"/>"))
        lines.append(self._indent(3, "<param name=\"mavros_state_topic_template\" value=\"/iris_%d/mavros/state\"/>"))
        lines.append(self._indent(3, "<param name=\"extrinsic_topic_template\" value=\"/drone_%d/vins_estimator/extrinsic\"/>"))
        lines.append(self._indent(3, "<param name=\"px4_debug_topic_template\" value=\"/drone_%d/px4ctrl/debugPx4ctrl\"/>"))
        lines.append(self._indent(2, "</node>"))
        lines.append(self._indent(1, "</group>"))
        lines.append(self._indent(1, ""))
        return lines

    def _generate_v2_goal_tooling(self):
        if self.version != "v2":
            return []

        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- Optional V2 Goal Tooling                     -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))
        lines.append(self._indent(1, "<include if=\"$(arg enable_goal_tooling_v2)\" file=\"$(find clean_uav_core)/launch/swarm_goal_tooling_v2.launch\">"))
        lines.append(self._indent(2, "<arg name=\"rviz_node_name\" value=\"swarm_rviz\"/>"))
        lines.append(self._indent(2, "<arg name=\"enable_assign_goals\" value=\"$(arg enable_assign_goals_v2)\"/>"))
        lines.append(self._indent(2, "<arg name=\"enable_random_goals\" value=\"$(arg enable_random_goals_v2)\"/>"))
        lines.append(self._indent(2, "<arg name=\"selected_drones_topic\" value=\"$(arg v2_selected_drones_topic)\"/>"))
        lines.append(self._indent(2, "<arg name=\"pose_goal_topic\" value=\"$(arg v2_pose_goal_topic)\"/>"))
        lines.append(self._indent(2, "<arg name=\"goalset_topic\" value=\"$(arg planner_goal_topic)\"/>"))
        lines.append(self._indent(2, "<arg name=\"arrow_topic\" value=\"$(arg v2_goal_arrow_topic)\"/>"))
        lines.append(self._indent(2, f"<arg name=\"drone_num\" value=\"{self.num_uavs}\"/>"))
        lines.append(self._indent(1, "</include>"))
        lines.append(self._indent(1, ""))
        return lines

    def _generate_v2_moving_obstacles(self):
        if self.version != "v2":
            return []

        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- Optional V2 Moving Obstacles                -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))
        lines.append(self._indent(1, "<include if=\"$(arg enable_moving_obstacles_v2)\" file=\"$(find clean_uav_core)/launch/swarm_moving_obstacles_v2.launch\">"))
        lines.append(self._indent(2, "<arg name=\"enable_joy_node\" value=\"$(arg enable_moving_obstacles_joy_node_v2)\"/>"))
        lines.append(self._indent(2, "<arg name=\"joy_topic\" value=\"$(arg moving_obstacles_joy_topic_v2)\"/>"))
        lines.append(self._indent(2, "<arg name=\"broadcast_traj_topic\" value=\"$(arg planning_broadcast_topic_v2)\"/>"))
        lines.append(self._indent(1, "</include>"))
        lines.append(self._indent(1, ""))
        return lines

    def _generate_v2_manual_take_over(self):
        if self.version != "v2":
            return []

        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- Optional V2 Manual Take Over                -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))

        lines.append(self._indent(1, "<include if=\"$(arg enable_manual_take_over_station_v2)\" file=\"$(find clean_uav_core)/launch/swarm_manual_take_over_station_v2.launch\">"))
        lines.append(self._indent(2, "<arg name=\"enable_joy_node\" value=\"$(arg enable_manual_take_over_joy_node_v2)\"/>"))
        lines.append(self._indent(2, "<arg name=\"joy_input_topic\" value=\"$(arg manual_take_over_joy_input_topic_v2)\"/>"))
        lines.append(self._indent(2, "<arg name=\"joystick_topic\" value=\"$(arg manual_take_over_joy_topic_v2)\"/>"))
        lines.append(self._indent(2, "<arg name=\"joy_dev\" value=\"$(arg manual_take_over_joy_dev_v2)\"/>"))
        lines.append(self._indent(1, "</include>"))
        lines.append(self._indent(1, ""))

        for drone_id in sorted(self.uav_configs.keys()):
            lines.append(self._indent(1, f"<include if=\"$(arg enable_manual_take_over_v2)\" file=\"$(find clean_uav_core)/launch/swarm_manual_take_over_instance_v2.launch\">"))
            lines.append(self._indent(2, f"<arg name=\"drone_id\" value=\"{drone_id}\"/>"))
            lines.append(self._indent(2, "<arg name=\"joystick_topic\" value=\"$(arg manual_take_over_joy_topic_v2)\"/>"))
            lines.append(self._indent(1, "</include>"))
            lines.append(self._indent(1, ""))

        return lines

    def _generate_v2_odom_visualization(self):
        if self.version != "v2":
            return []

        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- Optional V2 Odom Visualization              -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))

        for drone_id in sorted(self.uav_configs.keys()):
            lines.append(self._indent(1, f"<include if=\"$(arg enable_odom_visualization_v2)\" file=\"$(find clean_uav_core)/launch/swarm_odom_visualization_instance_v2.launch\">"))
            lines.append(self._indent(2, f"<arg name=\"drone_id\" value=\"{drone_id}\"/>"))
            lines.append(self._indent(2, "<arg name=\"robot_scale\" value=\"$(arg odom_visualization_scale_v2)\"/>"))
            lines.append(self._indent(1, "</include>"))
            lines.append(self._indent(1, ""))

        return lines

    def _generate_rviz(self):
        lines = []
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, "<!-- RViz Visualization                            -->"))
        lines.append(self._indent(1, "<!-- ========================================== -->"))
        lines.append(self._indent(1, ""))
        lines.append(self._indent(1, "<node if=\"$(arg use_rviz)\" pkg=\"rviz\" type=\"rviz\" name=\"swarm_rviz\" args=\"-d $(arg rviz_config)\" output=\"screen\"/>"))
        lines.append(self._indent(1, ""))
        return lines

    def generate_launch_xml(self):
        lines = []
        lines.append('<launch>')
        lines.append(self._indent(0, '<!-- ============================================================= -->'))
        lines.append(self._indent(0, '<!-- Auto-generated by swarm_launch_generator.py                     -->'))
        lines.append(self._indent(0, f'<!-- Planner version: {self.version}                                        -->'))
        lines.append(self._indent(0, f'<!-- Number of UAVs: {self.num_uavs}                                           -->'))
        lines.append(self._indent(0, f'<!-- Config file: {self.config_path} -->'))
        lines.append(self._indent(0, '<!-- ============================================================= -->'))
        lines.append(self._indent(0, ''))

        lines.extend(self._generate_global_args())
        lines.extend(self._generate_gazebo_launch())
        lines.extend(self._generate_uav_instances())
        lines.extend(self._generate_swarm_trigger())
        lines.extend(self._generate_dynamic_commander())
        lines.extend(self._generate_benchmark_manager())
        lines.extend(self._generate_v2_goal_tooling())
        lines.extend(self._generate_v2_moving_obstacles())
        lines.extend(self._generate_v2_manual_take_over())
        lines.extend(self._generate_v2_odom_visualization())
        lines.extend(self._generate_rviz())

        lines.append('</launch>')

        return '\n'.join(lines)

    def save(self):
        xml_content = self.generate_launch_xml()

        if "topic_tools/relay" in xml_content:
            raise ValueError("generated launch unexpectedly contains topic_tools/relay")
        if re.search(r"<group\s+ns=\"iris_", xml_content):
            raise ValueError("generated launch unexpectedly contains iris_* runtime namespaces")

        output_path = Path(resolve_output_path(self.output_path, self.version, self.num_uavs))
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with output_path.open("w", encoding="utf-8") as handle:
            handle.write(xml_content)

        print(f"[Generator] Generated {self.version} launch file: {output_path}")
        print(f"[Generator] Total UAVs: {self.num_uavs}")
        print(f"[Generator] File size: {len(xml_content)} bytes")


def main():
    parser = argparse.ArgumentParser(description="Generate the top-level swarm launch file")
    parser.add_argument("--config", type=str, default="", help="Path to the drone configuration markdown file")
    parser.add_argument("--output", type=str, default="", help="Path to the generated launch file")
    parser.add_argument("--version", type=str, default="v1", choices=SUPPORTED_VERSIONS, help="Planner stack version to generate")
    parser.add_argument("--uav-count", type=int, default=0, help="Override the UAV count and synthesize configs directly")

    args = parser.parse_args()

    config_path = resolve_config_path(args.config)
    requested_uav_count = args.uav_count if args.uav_count > 0 else None
    output_path = resolve_output_path(args.output, args.version, requested_uav_count)

    print("=" * 70)
    print("Swarm Launch Generator")
    print("=" * 70)
    print(f"Workspace root: {get_workspace_root()}")
    print(f"Planner version: {args.version}")
    if requested_uav_count is not None:
        print(f"UAV count override: {requested_uav_count}")
    print(f"Config file: {config_path}")
    print(f"Output file: {output_path}")
    print("=" * 70)

    generator = SwarmLaunchGenerator(config_path, output_path, args.version, requested_uav_count)
    generator.save()

    print("=" * 70)
    print("Generation complete")
    print("=" * 70)
    print(f"roslaunch clean_uav_core {Path(output_path).name}")


if __name__ == "__main__":
    main()
