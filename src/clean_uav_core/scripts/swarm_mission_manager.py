#!/usr/bin/env python3

"""Cooperative swarm mission manager.

This node owns the higher-level search mission state, keeps a process-local
blackboard, publishes goals for V1/V2 planner stacks, and optionally stops the
benchmark session when the mission reaches its terminal condition.
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from quadrotor_msgs.msg import GoalSet, PlannerBenchmarkEvent, PlannerReplanInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from swarm_dynamic_commander import build_phase1_uav_configs as build_phase1_layout_v1
from swarm_dynamic_commander import resolve_config_path, yaw_to_quaternion
from swarm_dynamic_commander_v2 import build_goal_configs
from swarm_dynamic_commander_v2 import build_phase1_uav_configs as build_phase1_layout_v2


ANSI = {
    "reset": "\033[0m",
    "dim": "\033[2m",
    "red": "\033[31m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "blue": "\033[34m",
    "magenta": "\033[35m",
    "cyan": "\033[36m",
    "white": "\033[37m",
    "bold": "\033[1m",
}


def _now_wall() -> float:
    return time.time()


def _now_ros() -> float:
    try:
        return rospy.Time.now().to_sec()
    except Exception:
        return _now_wall()


def _workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _resolve_workspace_path(path_text: str) -> Path:
    text = str(path_text or "").strip()
    if not text:
        return _workspace_root()
    expanded = os.path.expanduser(text)
    candidate = Path(expanded)
    if candidate.is_absolute():
        return candidate
    return _workspace_root() / candidate


def _coerce_float_list(value) -> List[float]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    text = str(value).strip()
    if not text:
        return []
    return [float(item) for item in text.replace(";", ",").split(",") if item.strip()]


def _coerce_int_list(value) -> List[int]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [int(item) for item in value]
    text = str(value).strip()
    if not text:
        return []
    return [int(item) for item in text.replace(";", ",").split(",") if item.strip()]


def _coerce_triplets(value) -> List[Tuple[float, float, float]]:
    if value is None:
        return []
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return []
        triplets = []
        for chunk in text.replace("|", ";").split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            parts = [item.strip() for item in chunk.split(",") if item.strip()]
            if len(parts) >= 3:
                triplets.append((float(parts[0]), float(parts[1]), float(parts[2])))
        return triplets
    if isinstance(value, (list, tuple)):
        if value and isinstance(value[0], dict):
            return [(float(item["x"]), float(item["y"]), float(item["z"])) for item in value]
        if value and isinstance(value[0], (list, tuple)) and len(value[0]) >= 3:
            return [(float(item[0]), float(item[1]), float(item[2])) for item in value]
        flat = [float(item) for item in value]
        return [(flat[index], flat[index + 1], flat[index + 2]) for index in range(0, len(flat) - 2, 3)]
    return []


def _mean(values: Sequence[float]) -> float:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return sum(finite) / float(len(finite)) if finite else float("nan")


def _distance(lhs: Tuple[float, float, float], rhs: Tuple[float, float, float]) -> float:
    return math.sqrt((lhs[0] - rhs[0]) ** 2 + (lhs[1] - rhs[1]) ** 2 + (lhs[2] - rhs[2]) ** 2)


def _color(text: str, color_name: str, enabled: bool) -> str:
    if not enabled:
        return text
    return f"{ANSI.get(color_name, '')}{text}{ANSI['reset']}"


def _format_bool(value: bool) -> str:
    return "true" if value else "false"


@dataclass
class SearchWaypoint:
    waypoint_id: str
    position: Tuple[float, float, float]
    kind: str
    preferred_drone_id: Optional[int] = None
    claimed_by: Optional[int] = None
    visited_by: Optional[int] = None
    visited_ros_sec: Optional[float] = None


@dataclass
class TargetRecord:
    target_id: str
    position: Tuple[float, float, float]
    detected: bool = False
    detected_by: Optional[int] = None
    first_detection_ros_sec: Optional[float] = None
    first_detection_wall_sec: Optional[float] = None
    confirmed: bool = False


@dataclass
class DroneMissionState:
    drone_id: int
    phase: str = "IDLE"
    health: str = "UNKNOWN"
    pose: Optional[Tuple[float, float, float]] = None
    yaw: float = 0.0
    odom_ros_sec: Optional[float] = None
    ready_since_ros_sec: Optional[float] = None
    current_goal_id: str = ""
    current_goal_kind: str = ""
    current_goal_position: Optional[Tuple[float, float, float]] = None
    current_goal_queue: List[Tuple[float, float, float]] = field(default_factory=list)
    current_route_start_position: Optional[Tuple[float, float, float]] = None
    current_goal_yaw: float = 0.0
    current_goal_distance_m: float = float("inf")
    assigned_speed_mps: float = 0.0
    support_anchor: Optional[Tuple[float, float, float]] = None
    last_command_ros_sec: Optional[float] = None
    last_transition_ros_sec: Optional[float] = None
    planner_failed: bool = False
    emergency_stop_seen: bool = False
    waypoint_history: List[str] = field(default_factory=list)


@dataclass
class GlobalBlackboard:
    mission_phase: str = "WAIT_READY"
    mission_mode: str = "search"
    mission_started_ros_sec: Optional[float] = None
    mission_started_wall_sec: Optional[float] = None
    mission_finished_ros_sec: Optional[float] = None
    mission_finished_wall_sec: Optional[float] = None
    stop_reason: str = ""
    reassign_count: int = 0
    emergency_recoveries: int = 0
    active_target_id: str = ""
    active_target_drone_id: Optional[int] = None
    active_target_position: Optional[Tuple[float, float, float]] = None
    target_registry: Dict[str, TargetRecord] = field(default_factory=dict)
    search_waypoints: Dict[str, SearchWaypoint] = field(default_factory=dict)
    visited_nodes: Dict[str, Dict[str, object]] = field(default_factory=dict)
    coverage_metrics: Dict[str, object] = field(default_factory=dict)
    task_coverage_pct: float = 0.0
    global_coverage_pct: float = 0.0


class SwarmMissionManager:
    def __init__(self):
        self._lock = threading.RLock()
        self._use_color = sys.stdout.isatty()

        self.config_file = resolve_config_path(rospy.get_param("~config_file", ""))
        self.output_dir = _resolve_workspace_path(rospy.get_param("~output_dir", "~/swarm_benchmark/mission_manager"))
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.session_name = str(rospy.get_param("~mission_session_name", "")).strip()
        if not self.session_name:
            self.session_name = datetime.now().strftime("mission_%Y%m%d_%H%M%S")

        self.mission_manager_mode = str(rospy.get_param("~mission_manager_mode", "search")).strip().lower()
        if self.mission_manager_mode not in ("search", "relay"):
            rospy.logwarn(
                "[clean_uav_core] mission_manager_mode=%s is unsupported, falling back to search",
                self.mission_manager_mode,
            )
            self.mission_manager_mode = "search"

        self.mission_use_v1 = bool(rospy.get_param("~mission_use_v1", False))
        self.mission_use_v2 = bool(rospy.get_param("~mission_use_v2", True))
        if self.mission_use_v1 and self.mission_use_v2:
            rospy.logwarn("[clean_uav_core] both mission_use_v1 and mission_use_v2 are true, preferring V2 goals")
            self.mission_use_v1 = False

        self.goal_topic = str(rospy.get_param("~mission_goal_topic", "/goal_with_id")).strip()
        self.goal_topic_template = str(rospy.get_param("~mission_goal_topic_template", "/drone_%d/goal")).strip()
        self.feedback_topic = str(rospy.get_param("~mission_feedback_topic", "/mission_manager/feedback")).strip()
        self.coverage_metrics_topic = str(rospy.get_param("~coverage_metrics_topic", "/benchmark/coverage_metrics")).strip()
        self.target_detected_topic = str(rospy.get_param("~target_detected_topic", "/benchmark/target_detected")).strip()
        self.benchmark_stop_service = str(rospy.get_param("~benchmark_stop_service", "/benchmark/stop_session")).strip()
        self.mission_benchmark_track = bool(rospy.get_param("~mission_benchmark_track", True))

        self.planner_node_name = str(rospy.get_param("~planner_node_name", "ego_planner_v2" if self.mission_use_v2 else "ego_planner")).strip()
        self.safety_topic_template = str(
            rospy.get_param(
                "~safety_topic_template",
                f"/drone_%d/{self.planner_node_name}/grid_map/occupancy_inflate",
            )
        ).strip()
        self.odom_topic_template = str(
            rospy.get_param("~odom_topic_template", "/drone_%d/truth_odom" if self.mission_use_v2 else "/drone_%d/odom")
        ).strip()
        self.replan_info_topic_template = str(
            rospy.get_param(
                "~replan_info_topic_template",
                f"/drone_%d/{self.planner_node_name}/planning/replan_info",
            )
        ).strip()
        self.planner_event_topic_template = str(
            rospy.get_param(
                "~planner_event_topic_template",
                f"/drone_%d/{self.planner_node_name}/planning/benchmark_event",
            )
        ).strip()

        self.tick_rate_hz = max(0.5, float(rospy.get_param("~publish_rate", 2.0)))
        self.start_delay_sec = max(0.0, float(rospy.get_param("~start_delay", 0.0)))
        self.wait_for_all_uavs = bool(rospy.get_param("~wait_for_all_uavs", True))
        self.ready_z_threshold = float(rospy.get_param("~ready_z_threshold", 0.8))
        self.ready_stable_duration_sec = float(rospy.get_param("~ready_stable_duration", 1.5))
        self.ready_timeout_sec = float(rospy.get_param("~ready_timeout", 60.0))
        self.ready_post_delay_sec = max(0.0, float(rospy.get_param("~ready_post_delay", 0.0)))

        self.goal_reached_radius_m = max(0.1, float(rospy.get_param("~goal_reached_radius_m", 1.0)))
        self.target_confirmation_radius_m = max(0.1, float(rospy.get_param("~target_confirmation_radius_m", 1.5)))
        self.support_radius_m = max(0.5, float(rospy.get_param("~support_radius_m", 8.0)))
        self.search_goal_z_min_m = max(1.0, float(rospy.get_param("~search_goal_z_min_m", 1.0)))
        self.search_goal_z_max_m = max(self.search_goal_z_min_m, float(rospy.get_param("~search_goal_z_max_m", 4.0)))
        self.search_altitude_m = self._clamp_search_goal_z(float(rospy.get_param("~search_altitude_m", 1.5)))
        self.search_bend_offset_m = max(1.0, float(rospy.get_param("~search_bend_offset_m", max(3.0, self.support_radius_m * 0.5))))
        self.search_initial_escape_distance_m = max(
            1.0,
            float(rospy.get_param("~search_initial_escape_distance_m", max(2.0, self.search_bend_offset_m))),
        )
        self.search_initial_heading_weight = max(0.0, float(rospy.get_param("~search_initial_heading_weight", 1.0)))
        self.search_boundary_margin_m = max(
            0.0,
            float(
                rospy.get_param(
                    "~search_boundary_margin_m",
                    max(1.5, 0.5 * self.search_initial_escape_distance_m),
                )
            ),
        )
        self.search_corridor_half_width_m = max(0.5, float(rospy.get_param("~search_corridor_half_width_m", 1.5)))
        self.search_memory_resolution_m = max(0.5, float(rospy.get_param("~search_memory_resolution_m", 2.0)))
        self.search_route_failure_penalty = max(1.0, float(rospy.get_param("~search_route_failure_penalty", 5.0)))
        self.waypoint_spacing_m = max(0.5, float(rospy.get_param("~waypoint_spacing_m", 12.0)))
        self.search_lane_spacing_m = max(0.5, float(rospy.get_param("~search_lane_spacing_m", self.waypoint_spacing_m / 2.0)))
        self.search_lane_margin_m = max(0.0, float(rospy.get_param("~search_lane_margin_m", 1.5)))
        self.search_area_min_x = float(rospy.get_param("~search_area_min_x", -20.0))
        self.search_area_max_x = float(rospy.get_param("~search_area_max_x", 20.0))
        self.search_area_min_y = float(rospy.get_param("~search_area_min_y", -12.0))
        self.search_area_max_y = float(rospy.get_param("~search_area_max_y", 12.0))
        self.search_sector_count = max(1, int(rospy.get_param("~search_sector_count", 3)))
        self.hidden_target_count = max(1, int(rospy.get_param("~hidden_target_count", 1)))

        self.explore_speed_mps = max(0.1, float(rospy.get_param("~explore_speed_mps", 1.2)))
        self.track_speed_mps = max(0.1, float(rospy.get_param("~track_speed_mps", 2.5)))
        self.support_speed_mps = max(0.1, float(rospy.get_param("~support_speed_mps", 1.8)))
        self.finish_speed_mps = max(0.1, float(rospy.get_param("~finish_speed_mps", 1.0)))

        self.default_drone_ids = _coerce_int_list(rospy.get_param("~default_drone_ids", []))
        self.default_goal_xs = _coerce_float_list(rospy.get_param("~default_goal_xs", []))
        self.default_goal_ys = _coerce_float_list(rospy.get_param("~default_goal_ys", []))
        self.default_goal_zs = _coerce_float_list(rospy.get_param("~default_goal_zs", []))
        self.target_positions = _coerce_triplets(rospy.get_param("~target_positions", []))

        self.phase1_layout = self._build_phase1_layout()
        self.goal_layout = self._build_goal_layout()
        self.drone_ids = sorted(self.goal_layout.keys()) if self.goal_layout else sorted(self.phase1_layout.keys())
        if not self.drone_ids:
            rospy.logerr("[clean_uav_core] mission manager could not resolve any drone ids")
            rospy.signal_shutdown("no drone ids")
            return

        self.home_positions = {
            drone_id: tuple(float(self.phase1_layout[drone_id]["start"][axis]) for axis in ("x", "y", "z"))
            for drone_id in self.drone_ids
            if drone_id in self.phase1_layout
        }
        self.target_positions = self._resolve_target_positions()
        rospy.loginfo(
            "[clean_uav_core] search z policy band=[%.1f, %.1f] search_altitude=%.2f",
            self.search_goal_z_min_m,
            self.search_goal_z_max_m,
            self.search_altitude_m,
        )

        self.blackboard = GlobalBlackboard(mission_mode=self.mission_manager_mode)
        self.drone_states: Dict[int, DroneMissionState] = {
            drone_id: DroneMissionState(drone_id=drone_id) for drone_id in self.drone_ids
        }
        self._latest_safety_points: Dict[int, List[Tuple[float, float, float]]] = {drone_id: [] for drone_id in self.drone_ids}
        self._failed_segment_memory: Dict[str, int] = {}
        self.task_waypoints: List[SearchWaypoint] = self._build_task_waypoints()
        self.home_waypoints: Dict[int, SearchWaypoint] = self._build_home_waypoints()
        self.blackboard.search_waypoints = {waypoint.waypoint_id: waypoint for waypoint in self.task_waypoints}

        self._feedback_pub = rospy.Publisher(self.feedback_topic, String, queue_size=20, latch=True)
        self._goal_publishers = self._build_goal_publishers()
        self._csv_path = self.output_dir / f"{self.session_name}_mission.csv"
        self.summary_path = self.output_dir / f"{self.session_name}_summary.json"
        self._csv_handle = self._csv_path.open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_handle, fieldnames=self._csv_fieldnames())
        self._csv_writer.writeheader()

        self._stop_proxy = None
        self._benchmark_stop_requested = False
        if self.mission_benchmark_track:
            try:
                self._stop_proxy = rospy.ServiceProxy(self.benchmark_stop_service, Trigger)
            except Exception:
                self._stop_proxy = None

        self._subscribers = []
        self._setup_subscribers()
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.tick_rate_hz), self._tick)

        self._last_render_wall_sec = 0.0
        self._last_feedback_hash = ""
        self._finalized = False

        rospy.loginfo(
            "[clean_uav_core] swarm_mission_manager ready mode=%s version=%s drones=%d output=%s",
            self.mission_manager_mode,
            "v2" if self.mission_use_v2 else "v1",
            len(self.drone_ids),
            self.output_dir,
        )

    def _build_phase1_layout(self):
        if self.mission_use_v2:
            return build_phase1_layout_v2(self.config_file)
        return build_phase1_layout_v1(self.config_file)

    def _build_goal_layout(self) -> Dict[int, Dict[str, float]]:
        if self.mission_manager_mode == "search":
            return {}
        if self.mission_use_v2:
            explicit_layout = build_goal_configs(
                self.config_file,
                self.default_drone_ids,
                self.default_goal_xs,
                self.default_goal_ys,
                self.default_goal_zs,
            )
            if explicit_layout:
                return explicit_layout
        if not self.phase1_layout:
            return {}
        return {
            drone_id: {"x": float(config["goal"]["x"]), "y": float(config["goal"]["y"]), "z": float(config["goal"]["z"])}
            for drone_id, config in self.phase1_layout.items()
        }

    def _resolve_target_positions(self) -> List[Tuple[float, float, float]]:
        if self.mission_manager_mode == "search":
            return []
        if self.target_positions:
            return [self._normalize_search_position(position) for position in self.target_positions]
        ordered_ids = sorted(self.goal_layout.keys())
        return [
            (
                float(self.goal_layout[drone_id]["x"]),
                float(self.goal_layout[drone_id]["y"]),
                self._clamp_search_goal_z(float(self.goal_layout[drone_id]["z"])),
            )
            for drone_id in ordered_ids
        ]

    def _current_target_count(self) -> int:
        return len(self.blackboard.target_registry) or len(self.target_positions) or self.hidden_target_count

    def _clamp_search_goal_z(self, value: float) -> float:
        return max(self.search_goal_z_min_m, min(self.search_goal_z_max_m, float(value)))

    def _normalize_search_position(self, position: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return float(position[0]), float(position[1]), self._clamp_search_goal_z(position[2])

    def _build_search_waypoints(self) -> List[SearchWaypoint]:
        waypoints: List[SearchWaypoint] = []
        seen = set()

        def add_waypoint(waypoint_id: str, position: Tuple[float, float, float], kind: str, preferred_drone_id: Optional[int] = None):
            signature = (round(position[0], 3), round(position[1], 3), round(position[2], 3), kind)
            if signature in seen:
                return
            seen.add(signature)
            waypoints.append(
                SearchWaypoint(
                    waypoint_id=waypoint_id,
                    position=(float(position[0]), float(position[1]), float(position[2])),
                    kind=kind,
                    preferred_drone_id=preferred_drone_id,
                )
            )

        drone_ids = sorted(self.drone_ids)
        if not drone_ids:
            return waypoints

        x_min = min(self.search_area_min_x, self.search_area_max_x)
        x_max = max(self.search_area_min_x, self.search_area_max_x)
        y_min = min(self.search_area_min_y, self.search_area_max_y)
        y_max = max(self.search_area_min_y, self.search_area_max_y)
        x_min = x_min + self.search_boundary_margin_m
        x_max = x_max - self.search_boundary_margin_m
        y_min = y_min + self.search_boundary_margin_m
        y_max = y_max - self.search_boundary_margin_m
        if x_max <= x_min:
            midpoint_x = 0.5 * (self.search_area_min_x + self.search_area_max_x)
            x_min = midpoint_x - self.search_boundary_margin_m
            x_max = midpoint_x + self.search_boundary_margin_m
        if y_max <= y_min:
            midpoint_y = 0.5 * (self.search_area_min_y + self.search_area_max_y)
            y_min = midpoint_y - self.search_boundary_margin_m
            y_max = midpoint_y + self.search_boundary_margin_m
        sector_count = min(max(1, self.search_sector_count), len(drone_ids))
        sector_height = (y_max - y_min) / float(sector_count)
        lane_spacing = max(0.5, self.search_lane_spacing_m)
        lane_margin = min(self.search_lane_margin_m, max(0.0, 0.25 * sector_height))
        search_z = self._clamp_search_goal_z(self.search_altitude_m)

        for index, drone_id in enumerate(drone_ids):
            sector_index = index % sector_count
            sector_y_min = y_min + sector_index * sector_height
            sector_y_max = sector_y_min + sector_height
            lane_start = sector_y_min + lane_margin
            lane_end = sector_y_max - lane_margin
            if lane_end < lane_start:
                lane_start = sector_y_min
                lane_end = sector_y_max

            lane_positions: List[float] = []
            cursor = lane_start
            while cursor <= lane_end + 1e-6:
                lane_positions.append(cursor)
                cursor += lane_spacing
            if not lane_positions:
                lane_positions = [0.5 * (sector_y_min + sector_y_max)]

            for row_index, lane_y in enumerate(lane_positions):
                x_first, x_second = (x_min, x_max) if row_index % 2 == 0 else (x_max, x_min)
                add_waypoint(
                    f"search_s{sector_index}_r{row_index}_a",
                    (x_first, lane_y, search_z),
                    "search_lane",
                    preferred_drone_id=drone_id,
                )
                add_waypoint(
                    f"search_s{sector_index}_r{row_index}_b",
                    (x_second, lane_y, search_z),
                    "search_lane",
                    preferred_drone_id=drone_id,
                )

        return waypoints

    def _build_task_waypoints(self) -> List[SearchWaypoint]:
        if self.mission_manager_mode == "search":
            return self._build_search_waypoints()

        waypoints: List[SearchWaypoint] = []
        seen = set()

        def add_waypoint(waypoint_id: str, position: Tuple[float, float, float], kind: str, preferred_drone_id: Optional[int] = None):
            signature = (round(position[0], 3), round(position[1], 3), round(position[2], 3), kind)
            if signature in seen:
                return
            seen.add(signature)
            waypoints.append(
                SearchWaypoint(
                    waypoint_id=waypoint_id,
                    position=(float(position[0]), float(position[1]), float(position[2])),
                    kind=kind,
                    preferred_drone_id=preferred_drone_id,
                )
            )

        for drone_id in sorted(self.goal_layout.keys()):
            goal = self.goal_layout[drone_id]
            add_waypoint(
                f"anchor_{drone_id}",
                (goal["x"], goal["y"], self._clamp_search_goal_z(goal["z"])),
                "anchor",
                preferred_drone_id=drone_id,
            )

        if not waypoints:
            return waypoints

        centroid_x = _mean([wp.position[0] for wp in waypoints])
        centroid_y = _mean([wp.position[1] for wp in waypoints])
        centroid_z = self._clamp_search_goal_z(max(self.search_altitude_m, _mean([wp.position[2] for wp in waypoints])))
        add_waypoint("center", (centroid_x, centroid_y, centroid_z), "center")

        ring_offsets = [
            (self.support_radius_m, 0.0),
            (-self.support_radius_m, 0.0),
            (0.0, self.support_radius_m),
            (0.0, -self.support_radius_m),
        ]
        for index, (offset_x, offset_y) in enumerate(ring_offsets):
            add_waypoint(
                f"ring_{index}",
                (centroid_x + offset_x, centroid_y + offset_y, centroid_z),
                "ring",
            )

        if self.mission_manager_mode == "relay":
            return waypoints[: len(self.goal_layout)]

        if len(self.drone_ids) > 1:
            for index, drone_id in enumerate(sorted(self.drone_ids)):
                home = self.home_positions.get(drone_id)
                if home is None:
                    continue
                midpoint = (
                    (home[0] + centroid_x) / 2.0,
                    (home[1] + centroid_y) / 2.0,
                    self._clamp_search_goal_z(max(centroid_z, home[2])),
                )
                add_waypoint(f"mid_{index}", midpoint, "midpoint", preferred_drone_id=drone_id)

        return waypoints

    def _build_home_waypoints(self) -> Dict[int, SearchWaypoint]:
        waypoints: Dict[int, SearchWaypoint] = {}
        for drone_id in self.drone_ids:
            home = self.home_positions.get(drone_id)
            if home is None:
                continue
            waypoints[drone_id] = SearchWaypoint(
                waypoint_id=f"home_{drone_id}",
                position=home,
                kind="home",
                preferred_drone_id=drone_id,
            )
        return waypoints

    def _build_goal_publishers(self):
        if self.mission_use_v2:
            return {"goalset": rospy.Publisher(self.goal_topic, GoalSet, queue_size=20)}
        publishers = {}
        for drone_id in self.drone_ids:
            topic_name = self.goal_topic_template % drone_id
            publishers[drone_id] = rospy.Publisher(topic_name, PoseStamped, queue_size=20)
        return publishers

    def _csv_fieldnames(self) -> List[str]:
        return [
            "wall_time_sec",
            "ros_time_sec",
            "mission_phase",
            "mission_mode",
            "drone_id",
            "drone_phase",
            "health",
            "goal_kind",
            "goal_id",
            "goal_x",
            "goal_y",
            "goal_z",
            "goal_distance_m",
            "speed_mps",
            "coverage_global_pct",
            "coverage_task_pct",
            "target_count",
            "target_detected_count",
            "active_target_id",
            "active_target_drone_id",
            "reassign_count",
            "emergency_recoveries",
            "stop_reason",
            "waypoint_visited_count",
            "waypoint_total_count",
            "ready_count",
        ]

    def _setup_subscribers(self):
        for drone_id in self.drone_ids:
            odom_topic = self.odom_topic_template % drone_id
            replan_topic = self.replan_info_topic_template % drone_id
            event_topic = self.planner_event_topic_template % drone_id
            safety_topic = self.safety_topic_template % drone_id
            self._subscribers.append(rospy.Subscriber(odom_topic, Odometry, self._make_odom_callback(drone_id), queue_size=20))
            self._subscribers.append(rospy.Subscriber(replan_topic, PlannerReplanInfo, self._make_replan_callback(drone_id), queue_size=20))
            self._subscribers.append(rospy.Subscriber(event_topic, PlannerBenchmarkEvent, self._make_event_callback(drone_id), queue_size=20))
            self._subscribers.append(rospy.Subscriber(safety_topic, PointCloud2, self._make_safety_callback(drone_id), queue_size=5))

        self._subscribers.append(rospy.Subscriber(self.coverage_metrics_topic, String, self._coverage_metrics_callback, queue_size=10))
        self._subscribers.append(rospy.Subscriber(self.target_detected_topic, String, self._target_detected_callback, queue_size=10))

    def _make_odom_callback(self, drone_id: int):
        def _callback(msg: Odometry):
            with self._lock:
                state = self.drone_states[drone_id]
                state.pose = (
                    float(msg.pose.pose.position.x),
                    float(msg.pose.pose.position.y),
                    float(msg.pose.pose.position.z),
                )
                state.odom_ros_sec = self._stamp_to_sec(msg)
                state.health = "OK"
                state.yaw = self._yaw_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                if state.pose[2] >= self.ready_z_threshold:
                    if state.ready_since_ros_sec is None:
                        state.ready_since_ros_sec = state.odom_ros_sec
                else:
                    state.ready_since_ros_sec = None

        return _callback

    def _make_replan_callback(self, drone_id: int):
        def _callback(msg: PlannerReplanInfo):
            with self._lock:
                state = self.drone_states[drone_id]
                if bool(getattr(msg, "success", True)):
                    state.planner_failed = False
                    if state.health != "OK":
                        state.health = "OK"
                    if int(getattr(msg, "failure_reason", 0)) == PlannerReplanInfo.FAILURE_EMERGENCY_STOP:
                        self.blackboard.emergency_recoveries += 1
                    return

                state.planner_failed = True
                failure_reason = int(getattr(msg, "failure_reason", PlannerReplanInfo.FAILURE_UNKNOWN))
                if failure_reason == PlannerReplanInfo.FAILURE_EMERGENCY_STOP:
                    state.emergency_stop_seen = True
                    self.blackboard.emergency_recoveries += 1
                state.health = "DEGRADED"
                self._remember_failed_segment(drone_id)
                self._release_waypoint_if_needed(drone_id)

        return _callback

    def _make_event_callback(self, drone_id: int):
        def _callback(msg: PlannerBenchmarkEvent):
            with self._lock:
                state = self.drone_states[drone_id]
                current_state = int(getattr(msg, "current_state", -1))
                if current_state == PlannerBenchmarkEvent.STATE_EMERGENCY_STOP:
                    state.emergency_stop_seen = True
                    state.health = "DEGRADED"
                    self.blackboard.emergency_recoveries += 1
                    self._remember_failed_segment(drone_id)
                    self._release_waypoint_if_needed(drone_id)

        return _callback

    def _make_safety_callback(self, drone_id: int):
        def _callback(msg: PointCloud2):
            with self._lock:
                sampled_points: List[Tuple[float, float, float]] = []
                try:
                    for index, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
                        if index % 6 != 0:
                            continue
                        sampled_points.append((float(point[0]), float(point[1]), float(point[2])))
                        if len(sampled_points) >= 2000:
                            break
                except Exception:
                    sampled_points = []
                self._latest_safety_points[drone_id] = sampled_points

        return _callback

    @staticmethod
    def _stamp_to_sec(msg) -> float:
        try:
            value = msg.header.stamp.to_sec()
            if value > 0.0:
                return float(value)
        except Exception:
            pass
        return _now_ros()

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _coverage_metrics_callback(self, msg: String):
        with self._lock:
            try:
                payload = json.loads(msg.data)
            except Exception:
                return
            self.blackboard.coverage_metrics = payload
            coverage = payload.get("coverage") if isinstance(payload.get("coverage"), dict) else {}
            self.blackboard.global_coverage_pct = float(coverage.get("cumulative_coverage_pct", self.blackboard.global_coverage_pct or 0.0))
            targets = payload.get("targets") if isinstance(payload.get("targets"), dict) else {}
            detections = targets.get("detections") if isinstance(targets.get("detections"), list) else []
            self._refresh_target_registry_from_payload(detections)

    def _target_detected_callback(self, msg: String):
        with self._lock:
            try:
                payload = json.loads(msg.data)
            except Exception:
                return
            target_index = int(payload.get("target_index", -1))
            target_id = f"target_{target_index}"
            position = payload.get("position") or []
            if len(position) >= 3:
                target_position = self._normalize_search_position((float(position[0]), float(position[1]), float(position[2])))
            else:
                target_position = self._resolve_active_target_position()
            record = self.blackboard.target_registry.get(target_id)
            if record is None:
                record = TargetRecord(target_id=target_id, position=target_position)
                self.blackboard.target_registry[target_id] = record
            record.detected = True
            record.detected_by = int(payload.get("drone_id", -1))
            record.first_detection_ros_sec = float(payload.get("ros_time_sec", _now_ros()))
            record.first_detection_wall_sec = float(payload.get("wall_time_sec", _now_wall()))
            self.blackboard.active_target_id = target_id
            self.blackboard.active_target_drone_id = record.detected_by
            self.blackboard.active_target_position = record.position
            self._promote_tracking_state(record.position)

    def _refresh_target_registry_from_payload(self, detections: Sequence[dict]):
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            target_index = int(detection.get("target_index", -1))
            target_id = f"target_{target_index}"
            position = detection.get("position") or []
            if len(position) < 3:
                continue
            target_position = self._normalize_search_position((float(position[0]), float(position[1]), float(position[2])))
            record = self.blackboard.target_registry.get(target_id)
            if record is None:
                record = TargetRecord(target_id=target_id, position=target_position)
                self.blackboard.target_registry[target_id] = record
            record.position = target_position
            record.detected = True
            if record.detected_by is None:
                record.detected_by = int(detection.get("drone_id", -1))
            if record.first_detection_ros_sec is None:
                record.first_detection_ros_sec = float(detection.get("ros_time_sec", _now_ros()))
            if record.first_detection_wall_sec is None:
                record.first_detection_wall_sec = float(detection.get("wall_time_sec", _now_wall()))

    def _resolve_active_target_position(self) -> Tuple[float, float, float]:
        if self.blackboard.active_target_position is not None:
            return self._normalize_search_position(self.blackboard.active_target_position)
        if self.target_positions:
            return self._normalize_search_position(self.target_positions[0])
        if self.mission_manager_mode != "search" and self.goal_layout:
            first_drone_id = sorted(self.goal_layout.keys())[0]
            goal = self.goal_layout[first_drone_id]
            return float(goal["x"]), float(goal["y"]), self._clamp_search_goal_z(float(goal["z"]))
        return 0.0, 0.0, self.search_altitude_m

    def _all_drones_ready(self, now_ros_sec: float) -> bool:
        if not self.wait_for_all_uavs:
            return True
        ready_count = 0
        for state in self.drone_states.values():
            if state.ready_since_ros_sec is None:
                continue
            if (now_ros_sec - state.ready_since_ros_sec) >= self.ready_stable_duration_sec:
                ready_count += 1
        if ready_count == len(self.drone_states):
            return True
        if self.ready_timeout_sec > 0.0 and self.blackboard.mission_started_ros_sec is not None:
            if (now_ros_sec - self.blackboard.mission_started_ros_sec) > self.ready_timeout_sec:
                rospy.logwarn("[clean_uav_core] mission manager readiness timeout reached")
                return True
        return False

    def _start_mission_if_needed(self, now_ros_sec: float):
        if self.blackboard.mission_phase != "WAIT_READY":
            return
        if self.blackboard.mission_started_ros_sec is None:
            self.blackboard.mission_started_ros_sec = now_ros_sec
            self.blackboard.mission_started_wall_sec = _now_wall()
        if self.start_delay_sec > 0.0 and (now_ros_sec - self.blackboard.mission_started_ros_sec) < self.start_delay_sec:
            return
        if not self._all_drones_ready(now_ros_sec):
            return
        if self.ready_post_delay_sec > 0.0 and (now_ros_sec - self.blackboard.mission_started_ros_sec) < (self.start_delay_sec + self.ready_post_delay_sec):
            return
        self.blackboard.mission_phase = "SEARCHING"
        for drone_id, state in self.drone_states.items():
            state.phase = "EXPLORING"
            state.last_transition_ros_sec = now_ros_sec
            self._set_speed_profile(drone_id, self.explore_speed_mps)
        rospy.loginfo("[clean_uav_core] mission manager entered SEARCHING phase")

    def _desired_speed_for_phase(self, phase: str) -> float:
        if phase == "TRACKING":
            return self.track_speed_mps
        if phase == "SUPPORT":
            return self.support_speed_mps
        if phase == "FINISHED":
            return self.finish_speed_mps
        return self.explore_speed_mps

    def _set_speed_profile(self, drone_id: int, speed_mps: float):
        state = self.drone_states[drone_id]
        if abs(state.assigned_speed_mps - speed_mps) < 1e-3:
            return
        state.assigned_speed_mps = float(speed_mps)
        planner_namespace = f"/drone_{drone_id}/{self.planner_node_name}"
        for param_name in ("manager/max_vel", "optimization/max_vel", "bspline/limit_vel"):
            try:
                rospy.set_param(f"{planner_namespace}/{param_name}", float(speed_mps))
            except Exception:
                pass

    def _within_search_area(self, position: Tuple[float, float, float]) -> bool:
        x_min = min(self.search_area_min_x, self.search_area_max_x) + self.search_boundary_margin_m
        x_max = max(self.search_area_min_x, self.search_area_max_x) - self.search_boundary_margin_m
        y_min = min(self.search_area_min_y, self.search_area_max_y) + self.search_boundary_margin_m
        y_max = max(self.search_area_min_y, self.search_area_max_y) - self.search_boundary_margin_m
        if x_max <= x_min:
            x_min = min(self.search_area_min_x, self.search_area_max_x)
            x_max = max(self.search_area_min_x, self.search_area_max_x)
        if y_max <= y_min:
            y_min = min(self.search_area_min_y, self.search_area_max_y)
            y_max = max(self.search_area_min_y, self.search_area_max_y)
        return x_min <= position[0] <= x_max and y_min <= position[1] <= y_max

    def _clamp_to_search_area(self, position: Tuple[float, float, float]) -> Tuple[float, float, float]:
        x_min = min(self.search_area_min_x, self.search_area_max_x) + self.search_boundary_margin_m
        x_max = max(self.search_area_min_x, self.search_area_max_x) - self.search_boundary_margin_m
        y_min = min(self.search_area_min_y, self.search_area_max_y) + self.search_boundary_margin_m
        y_max = max(self.search_area_min_y, self.search_area_max_y) - self.search_boundary_margin_m
        if x_max <= x_min:
            x_min = min(self.search_area_min_x, self.search_area_max_x)
            x_max = max(self.search_area_min_x, self.search_area_max_x)
        if y_max <= y_min:
            y_min = min(self.search_area_min_y, self.search_area_max_y)
            y_max = max(self.search_area_min_y, self.search_area_max_y)
        return (
            max(x_min, min(x_max, float(position[0]))),
            max(y_min, min(y_max, float(position[1]))),
            self._clamp_search_goal_z(float(position[2])),
        )

    def _segment_key(self, start: Tuple[float, float, float], end: Tuple[float, float, float]) -> str:
        midpoint_x = 0.5 * (start[0] + end[0])
        midpoint_y = 0.5 * (start[1] + end[1])
        heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0])) if _distance(start, end) > 1e-6 else 0.0
        cell_x = int(round(midpoint_x / self.search_memory_resolution_m))
        cell_y = int(round(midpoint_y / self.search_memory_resolution_m))
        heading_bin = int(round(heading / 30.0))
        return f"{cell_x}:{cell_y}:{heading_bin}"

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _angle_difference(self, lhs: float, rhs: float) -> float:
        return abs(self._normalize_angle(lhs - rhs))

    def _heading_from_points(self, start: Tuple[float, float, float], end: Tuple[float, float, float]) -> float:
        return math.atan2(end[1] - start[1], end[0] - start[0])

    def _startup_escape_candidates(self, state: DroneMissionState, start: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        if state.pose is None:
            return []
        base_distance = self.search_initial_escape_distance_m
        headings = [state.yaw, state.yaw + math.pi, state.yaw + math.pi / 2.0, state.yaw - math.pi / 2.0]
        candidates: List[Tuple[float, float, float]] = []
        for heading in headings:
            candidate = self._clamp_to_search_area(
                (
                    start[0] + base_distance * math.cos(heading),
                    start[1] + base_distance * math.sin(heading),
                    self._clamp_search_goal_z(max(start[2], self.search_altitude_m)),
                )
            )
            if self._within_search_area(candidate):
                candidates.append(candidate)
        return candidates

    @staticmethod
    def _point_segment_distance_2d(point: Tuple[float, float, float], start: Tuple[float, float, float], end: Tuple[float, float, float]) -> Tuple[float, float]:
        ax, ay = start[0], start[1]
        bx, by = end[0], end[1]
        px, py = point[0], point[1]
        dx = bx - ax
        dy = by - ay
        norm_sq = dx * dx + dy * dy
        if norm_sq <= 1e-9:
            return math.hypot(px - ax, py - ay), 0.0
        t = ((px - ax) * dx + (py - ay) * dy) / norm_sq
        t = max(0.0, min(1.0, t))
        closest_x = ax + t * dx
        closest_y = ay + t * dy
        return math.hypot(px - closest_x, py - closest_y), t

    def _segment_risk_score(self, drone_id: int, start: Tuple[float, float, float], end: Tuple[float, float, float]) -> float:
        points = self._latest_safety_points.get(drone_id, [])
        if not points:
            return 0.0
        corridor = self.search_corridor_half_width_m
        z_low = min(start[2], end[2]) - self.search_goal_z_min_m
        z_high = max(start[2], end[2]) + self.search_goal_z_min_m
        min_distance = float("inf")
        hit_count = 0
        for point in points:
            if point[2] < z_low or point[2] > z_high:
                continue
            distance, ratio = self._point_segment_distance_2d(point, start, end)
            if distance < min_distance:
                min_distance = distance
            if 0.0 <= ratio <= 1.0 and distance <= corridor:
                hit_count += 1
        if hit_count == 0:
            if math.isfinite(min_distance):
                return max(0.0, corridor - min_distance)
            return 0.0
        return float(hit_count) * 10.0 + max(0.0, corridor - min_distance)

    def _route_memory_penalty(self, start: Tuple[float, float, float], end: Tuple[float, float, float]) -> float:
        return float(self._failed_segment_memory.get(self._segment_key(start, end), 0)) * self.search_route_failure_penalty

    def _candidate_bend_points(self, start: Tuple[float, float, float], end: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        dx = float(end[0] - start[0])
        dy = float(end[1] - start[1])
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            return []
        perp_x = -dy / norm
        perp_y = dx / norm
        mid_x = 0.5 * (start[0] + end[0])
        mid_y = 0.5 * (start[1] + end[1])
        bend_z = self._clamp_search_goal_z(max(self.search_altitude_m, start[2], end[2]))
        candidates = []
        for side in (1.0, -1.0):
            for multiplier in (1.0, 1.5, 2.0):
                candidates.append(
                    self._clamp_to_search_area(
                        (
                            mid_x + side * perp_x * self.search_bend_offset_m * multiplier,
                            mid_y + side * perp_y * self.search_bend_offset_m * multiplier,
                            bend_z,
                        )
                    )
                )
        return candidates

    def _build_search_route(self, drone_id: int, state: DroneMissionState, start: Tuple[float, float, float], goal: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        start = self._normalize_search_position(start)
        goal = self._normalize_search_position(goal)
        direct_score = self._segment_risk_score(drone_id, start, goal) + self._route_memory_penalty(start, goal)
        if state.pose is not None:
            direct_heading_penalty = self.search_initial_heading_weight * (self._angle_difference(state.yaw, self._heading_from_points(start, goal)) / math.pi)
            direct_score += direct_heading_penalty
        startup_escape_candidates = self._startup_escape_candidates(state, start) if state.current_goal_position is None else []
        if direct_score <= 1e-6 and not startup_escape_candidates:
            return [goal]

        scored_candidates: List[Tuple[float, Tuple[float, float, float]]] = []
        for escape_point in startup_escape_candidates:
            score = (
                self._segment_risk_score(drone_id, start, escape_point)
                + self._route_memory_penalty(start, escape_point)
                + self._segment_risk_score(drone_id, escape_point, goal)
                + self._route_memory_penalty(escape_point, goal)
                + 0.12 * _distance(start, escape_point)
                + 0.05 * _distance(escape_point, goal)
                + self.search_initial_heading_weight * (self._angle_difference(state.yaw, self._heading_from_points(start, escape_point)) / math.pi)
            )
            scored_candidates.append((score, escape_point))

        for bend in self._candidate_bend_points(start, goal):
            if not self._within_search_area(bend):
                continue
            score = (
                self._segment_risk_score(drone_id, start, bend)
                + self._segment_risk_score(drone_id, bend, goal)
                + self._route_memory_penalty(start, bend)
                + self._route_memory_penalty(bend, goal)
                + 0.15 * _distance(start, bend)
                + 0.05 * _distance(bend, goal)
            )
            if state.pose is not None and state.current_goal_position is None:
                score += self.search_initial_heading_weight * (self._angle_difference(state.yaw, self._heading_from_points(start, bend)) / math.pi)
            scored_candidates.append((score, bend))

        if not scored_candidates:
            return [goal]

        scored_candidates.sort(key=lambda item: item[0])
        best_bend = scored_candidates[0][1]
        if self._segment_risk_score(drone_id, start, best_bend) > self._segment_risk_score(drone_id, start, goal) + 1.0:
            return [goal]
        return [best_bend, goal]

    def _remember_failed_segment(self, drone_id: int):
        state = self.drone_states[drone_id]
        if state.current_route_start_position is None or state.current_goal_position is None:
            return
        segment_key = self._segment_key(state.current_route_start_position, state.current_goal_position)
        self._failed_segment_memory[segment_key] = self._failed_segment_memory.get(segment_key, 0) + 1

    def _score_search_waypoint(self, drone_id: int, state: DroneMissionState, waypoint: SearchWaypoint) -> float:
        reference_position = state.pose or self.home_positions.get(drone_id) or waypoint.position
        route = self._build_search_route(drone_id, state, reference_position, waypoint.position)
        route_score = 0.0
        route_start = self._normalize_search_position(reference_position)
        for route_goal in route:
            route_score += self._segment_risk_score(drone_id, route_start, route_goal)
            route_score += self._route_memory_penalty(route_start, route_goal)
            route_score += 0.10 * _distance(route_start, route_goal)
            if state.pose is not None and route_start == self._normalize_search_position(reference_position):
                route_score += self.search_initial_heading_weight * (self._angle_difference(state.yaw, self._heading_from_points(route_start, route_goal)) / math.pi)
            route_start = route_goal
        preferred_bonus = -2.0 if waypoint.preferred_drone_id == drone_id else 0.0
        claim_penalty = 0.0 if waypoint.claimed_by is None else 500.0
        return route_score + preferred_bonus + claim_penalty

    def _assign_search_waypoint(self, drone_id: int) -> Optional[SearchWaypoint]:
        state = self.drone_states[drone_id]
        candidates = [wp for wp in self.task_waypoints if wp.claimed_by is None and wp.visited_by is None]
        if not candidates:
            return None

        preferred = [wp for wp in candidates if wp.preferred_drone_id == drone_id]
        pool = preferred or candidates
        waypoint = min(pool, key=lambda item: self._score_search_waypoint(drone_id, state, item))
        waypoint.claimed_by = drone_id
        state.waypoint_history.append(waypoint.waypoint_id)
        return waypoint

    def _assign_home_waypoint(self, drone_id: int) -> Optional[SearchWaypoint]:
        state = self.drone_states[drone_id]
        waypoint = self.home_waypoints.get(drone_id)
        if waypoint is None:
            return None
        waypoint.claimed_by = drone_id
        state.waypoint_history.append(waypoint.waypoint_id)
        return waypoint

    def _release_waypoint_if_needed(self, drone_id: int):
        state = self.drone_states[drone_id]
        if not state.current_goal_id:
            return
        waypoint = self.blackboard.search_waypoints.get(state.current_goal_id)
        if waypoint is not None and waypoint.visited_by is None and waypoint.claimed_by == drone_id:
            waypoint.claimed_by = None
            self.blackboard.reassign_count += 1
        state.current_goal_id = ""
        state.current_goal_kind = ""
        state.current_goal_position = None
        state.current_goal_queue.clear()
        state.current_goal_distance_m = float("inf")
        state.support_anchor = None
        state.current_route_start_position = None

    def _promote_tracking_state(self, target_position: Tuple[float, float, float]):
        target_position = self._normalize_search_position(target_position)
        tracking_drone_id = None
        best_distance = float("inf")
        for drone_id, state in self.drone_states.items():
            if state.pose is None:
                continue
            if state.health not in ("OK", "DEGRADED"):
                continue
            distance = _distance(state.pose, target_position)
            if distance < best_distance:
                best_distance = distance
                tracking_drone_id = drone_id

        if tracking_drone_id is None:
            tracking_drone_id = min(self.drone_states.keys())

        self.blackboard.active_target_drone_id = tracking_drone_id
        self.blackboard.active_target_position = target_position
        self.blackboard.mission_phase = "TRACKING"

        support_candidates = [drone_id for drone_id in self.drone_states.keys() if drone_id != tracking_drone_id]
        for index, drone_id in enumerate(support_candidates):
            self._set_drone_support_goal(drone_id, target_position, index)

        tracker_state = self.drone_states[tracking_drone_id]
        tracker_state.phase = "TRACKING"
        tracker_state.current_goal_id = f"{self.blackboard.active_target_id}_track"
        tracker_state.current_goal_kind = "track"
        tracker_state.current_goal_position = target_position
        tracker_state.current_goal_queue.clear()
        tracker_state.current_route_start_position = tracker_state.pose
        tracker_state.current_goal_yaw = math.atan2(target_position[1] - (tracker_state.pose[1] if tracker_state.pose else 0.0), target_position[0] - (tracker_state.pose[0] if tracker_state.pose else 0.0))
        tracker_state.waypoint_history.append(tracker_state.current_goal_id)
        self._set_speed_profile(tracking_drone_id, self.track_speed_mps)

        rospy.loginfo(
            "[clean_uav_core] mission manager promoted drone_%d to TRACKING target=%s",
            tracking_drone_id,
            self.blackboard.active_target_id,
        )

    def _set_drone_support_goal(self, drone_id: int, target_position: Tuple[float, float, float], support_index: int):
        state = self.drone_states[drone_id]
        target_position = self._normalize_search_position(target_position)
        angle = 2.0 * math.pi * (support_index % max(1, len(self.drone_states) - 1)) / max(1, len(self.drone_states) - 1)
        support_position = (
            target_position[0] + self.support_radius_m * math.cos(angle),
            target_position[1] + self.support_radius_m * math.sin(angle),
            self._clamp_search_goal_z(max(self.search_altitude_m, target_position[2])),
        )
        state.phase = "SUPPORT"
        state.current_goal_id = f"support_{self.blackboard.active_target_id}_{drone_id}"
        state.current_goal_kind = "support"
        state.current_goal_position = support_position
        state.current_goal_queue.clear()
        state.current_route_start_position = state.pose
        state.current_goal_yaw = math.atan2(target_position[1] - support_position[1], target_position[0] - support_position[0])
        state.support_anchor = target_position
        state.last_transition_ros_sec = _now_ros()
        state.waypoint_history.append(state.current_goal_id)
        self._set_speed_profile(drone_id, self.support_speed_mps)

    def _assign_return_goals(self):
        for drone_id, state in self.drone_states.items():
            home_waypoint = self._assign_home_waypoint(drone_id)
            if home_waypoint is None:
                continue
            state.phase = "FINISHED"
            state.current_goal_id = home_waypoint.waypoint_id
            state.current_goal_kind = home_waypoint.kind
            state.current_goal_position = home_waypoint.position
            state.current_goal_queue.clear()
            state.current_route_start_position = state.pose
            state.current_goal_yaw = 0.0
            state.last_transition_ros_sec = _now_ros()
            state.waypoint_history.append(home_waypoint.waypoint_id)
            self._set_speed_profile(drone_id, self.finish_speed_mps)

    def _all_targets_confirmed(self) -> bool:
        if not self.blackboard.target_registry:
            return False
        return all(record.confirmed for record in self.blackboard.target_registry.values())

    def _mark_waypoint_visited(self, waypoint: SearchWaypoint, drone_id: int, now_ros_sec: float):
        waypoint.visited_by = drone_id
        waypoint.visited_ros_sec = now_ros_sec
        waypoint.claimed_by = None
        self.blackboard.visited_nodes[waypoint.waypoint_id] = {
            "visited_by": drone_id,
            "visited_ros_sec": now_ros_sec,
            "kind": waypoint.kind,
        }

    def _maybe_replenish_task_queue(self):
        if any(wp.visited_by is None for wp in self.task_waypoints):
            return
        if self._all_targets_confirmed():
            return
        self.blackboard.reassign_count += 1
        for waypoint in self.task_waypoints:
            waypoint.claimed_by = None
            waypoint.visited_by = None
            waypoint.visited_ros_sec = None

    def _update_drone_goal(self, drone_id: int, state: DroneMissionState, now_ros_sec: float):
        if state.health not in ("OK", "DEGRADED"):
            state.phase = "EXPLORING"
            self._release_waypoint_if_needed(drone_id)
            return

        if self.blackboard.mission_phase == "WAIT_READY":
            state.phase = "IDLE"
            return

        if self.blackboard.mission_phase == "SEARCHING":
            if state.current_goal_position is not None and state.pose is not None:
                state.current_goal_distance_m = _distance(state.pose, state.current_goal_position)
                if state.current_goal_distance_m <= self.goal_reached_radius_m:
                    if state.current_goal_queue:
                        next_goal = self._normalize_search_position(state.current_goal_queue.pop(0))
                        state.current_goal_position = next_goal
                        state.current_goal_yaw = math.atan2(
                            next_goal[1] - state.pose[1],
                            next_goal[0] - state.pose[0],
                        )
                        state.current_goal_distance_m = _distance(state.pose, next_goal)
                        state.current_route_start_position = state.pose
                        state.last_transition_ros_sec = now_ros_sec
                        return
                    waypoint = self.blackboard.search_waypoints.get(state.current_goal_id)
                    if waypoint is not None and waypoint.visited_by is None:
                        self._mark_waypoint_visited(waypoint, drone_id, now_ros_sec)
                    state.current_goal_id = ""
                    state.current_goal_kind = ""
                    state.current_goal_position = None
                    state.current_goal_queue.clear()
                    state.current_route_start_position = None
                    state.current_goal_distance_m = float("inf")

            if state.current_goal_position is None:
                waypoint = self._assign_search_waypoint(drone_id)
                if waypoint is None:
                    state.phase = "EXPLORING"
                    self._maybe_replenish_task_queue()
                    waypoint = self._assign_search_waypoint(drone_id)
                    if waypoint is None:
                        return
                state.phase = "EXPLORING"
                state.current_goal_id = waypoint.waypoint_id
                state.current_goal_kind = waypoint.kind
                route_start = state.pose or self.home_positions.get(drone_id) or waypoint.position
                route = self._build_search_route(drone_id, state, route_start, waypoint.position)
                state.current_route_start_position = self._normalize_search_position(route_start)
                state.current_goal_position = self._normalize_search_position(route[0])
                state.current_goal_queue = [self._normalize_search_position(item) for item in route[1:]]
                current_reference = state.pose or route_start
                state.current_goal_yaw = math.atan2(
                    state.current_goal_position[1] - current_reference[1],
                    state.current_goal_position[0] - current_reference[0],
                )
                state.last_transition_ros_sec = now_ros_sec
                self._set_speed_profile(drone_id, self.explore_speed_mps)

        elif self.blackboard.mission_phase == "TRACKING":
            if self.blackboard.active_target_position is None:
                return
            if state.phase == "TRACKING" and state.pose is not None:
                state.current_goal_distance_m = _distance(state.pose, self.blackboard.active_target_position)
                if state.current_goal_distance_m <= self.target_confirmation_radius_m:
                    target_id = self.blackboard.active_target_id or "target_0"
                    record = self.blackboard.target_registry.get(target_id)
                    if record is not None:
                        record.confirmed = True
                    if self._all_targets_confirmed():
                        self.blackboard.mission_phase = "RETURNING"
                        self._assign_return_goals()
                        return
                    self.blackboard.mission_phase = "SEARCHING"
                    state.phase = "EXPLORING"
                    state.current_goal_id = ""
                    state.current_goal_kind = ""
                    state.current_goal_position = None
                    self._set_speed_profile(drone_id, self.explore_speed_mps)
                    self._maybe_replenish_task_queue()

        elif self.blackboard.mission_phase == "RETURNING":
            home_waypoint = self.home_waypoints.get(drone_id)
            if home_waypoint is None:
                return
            if state.current_goal_position is None:
                state.current_goal_id = home_waypoint.waypoint_id
                state.current_goal_kind = home_waypoint.kind
                state.current_goal_position = home_waypoint.position
                state.current_goal_queue.clear()
                state.current_route_start_position = state.pose
                state.current_goal_yaw = 0.0
                state.phase = "FINISHED"
                self._set_speed_profile(drone_id, self.finish_speed_mps)
            elif state.pose is not None:
                state.current_goal_distance_m = _distance(state.pose, home_waypoint.position)
                if state.current_goal_distance_m <= self.goal_reached_radius_m:
                    state.phase = "FINISHED"
                    state.current_goal_position = home_waypoint.position
                    self._mark_waypoint_visited(home_waypoint, drone_id, now_ros_sec)

    def _publish_goals(self):
        if self.mission_use_v2:
            publisher = self._goal_publishers["goalset"]
            for drone_id in self.drone_ids:
                state = self.drone_states[drone_id]
                if state.current_goal_position is None:
                    continue
                msg = GoalSet()
                msg.drone_id = drone_id
                msg.goal[0] = float(state.current_goal_position[0])
                msg.goal[1] = float(state.current_goal_position[1])
                msg.goal[2] = float(state.current_goal_position[2])
                publisher.publish(msg)
                state.last_command_ros_sec = _now_ros()
        else:
            for drone_id in self.drone_ids:
                state = self.drone_states[drone_id]
                if state.current_goal_position is None:
                    continue
                publisher = self._goal_publishers[drone_id]
                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "world"
                msg.pose.position.x = float(state.current_goal_position[0])
                msg.pose.position.y = float(state.current_goal_position[1])
                msg.pose.position.z = float(state.current_goal_position[2])
                quaternion = yaw_to_quaternion(state.current_goal_yaw)
                msg.pose.orientation.x = quaternion[0]
                msg.pose.orientation.y = quaternion[1]
                msg.pose.orientation.z = quaternion[2]
                msg.pose.orientation.w = quaternion[3]
                publisher.publish(msg)
                state.last_command_ros_sec = _now_ros()

    def _maybe_transition_to_return(self):
        if self.blackboard.mission_phase == "RETURNING":
            return
        if self.blackboard.mission_phase not in ("SEARCHING", "TRACKING"):
            return
        visited_count = len(self.blackboard.visited_nodes)
        total_count = len(self.task_waypoints)
        self.blackboard.task_coverage_pct = 100.0 * visited_count / float(total_count) if total_count else 0.0
        if self._all_targets_confirmed() and (visited_count >= total_count or total_count == 0):
            self.blackboard.mission_phase = "RETURNING"
            self._assign_return_goals()

    def _mission_finished(self) -> bool:
        if self.blackboard.mission_phase != "RETURNING":
            return False
        for drone_id in self.drone_ids:
            state = self.drone_states[drone_id]
            if state.pose is None:
                return False
            home = self.home_positions.get(drone_id)
            if home is None:
                return False
            if _distance(state.pose, home) > self.goal_reached_radius_m:
                return False
        return True

    def _request_benchmark_stop(self, reason: str):
        if self._benchmark_stop_requested or not self.mission_benchmark_track:
            return
        self._benchmark_stop_requested = True
        if self._stop_proxy is None:
            rospy.logwarn("[clean_uav_core] benchmark stop service unavailable, reason=%s", reason)
            return
        try:
            self._stop_proxy.wait_for_service(timeout=2.0)
        except Exception:
            rospy.logwarn("[clean_uav_core] benchmark stop service wait failed, reason=%s", reason)
            return
        try:
            response = self._stop_proxy()
            rospy.loginfo(
                "[clean_uav_core] benchmark stop request sent success=%s message=%s",
                getattr(response, "success", False),
                getattr(response, "message", ""),
            )
        except Exception as error:
            rospy.logwarn("[clean_uav_core] benchmark stop request failed: %s", error)

    def _format_goal(self, state: DroneMissionState) -> str:
        if state.current_goal_position is None:
            return "-"
        return f"({state.current_goal_position[0]:.1f}, {state.current_goal_position[1]:.1f}, {state.current_goal_position[2]:.1f})"

    def _render_table(self, now_ros_sec: float) -> str:
        lines = []
        phase_color = {
            "WAIT_READY": "yellow",
            "SEARCHING": "green",
            "TRACKING": "magenta",
            "RETURNING": "cyan",
            "DONE": "blue",
        }.get(self.blackboard.mission_phase, "white")
        header = (
            f"mission={self.blackboard.mission_phase} "
            f"mode={self.mission_manager_mode} "
            f"global_cov={self.blackboard.global_coverage_pct:.1f}% "
            f"task_cov={self.blackboard.task_coverage_pct:.1f}% "
            f"targets={sum(1 for record in self.blackboard.target_registry.values() if record.detected)}/{self._current_target_count()} "
            f"reassign={self.blackboard.reassign_count} "
            f"emergency={self.blackboard.emergency_recoveries} "
            f"stop={self.blackboard.stop_reason or '-'}"
        )
        lines.append(_color(f"[clean_uav_core] {header}", phase_color, self._use_color))
        lines.append(_color("drone | phase | health | goal | dist_m | speed_mps | ready | cmd_age_s", "dim", self._use_color))
        for drone_id in self.drone_ids:
            state = self.drone_states[drone_id]
            cmd_age = now_ros_sec - state.last_command_ros_sec if state.last_command_ros_sec is not None else float("nan")
            ready_flag = state.ready_since_ros_sec is not None and (now_ros_sec - state.ready_since_ros_sec) >= self.ready_stable_duration_sec
            phase_color_name = {
                "EXPLORING": "green",
                "TRACKING": "magenta",
                "SUPPORT": "yellow",
                "FINISHED": "cyan",
                "IDLE": "white",
            }.get(state.phase, "white")
            health_color = {
                "OK": "green",
                "DEGRADED": "yellow",
                "UNKNOWN": "white",
                "STALE": "red",
                "OFFLINE": "red",
            }.get(state.health, "white")
            line = (
                f"drone_{drone_id} | {state.phase} | {state.health} | {self._format_goal(state)} | "
                f"{state.current_goal_distance_m:5.2f} | {state.assigned_speed_mps:4.1f} | {_format_bool(ready_flag)} | "
                f"{cmd_age:5.2f}"
            )
            lines.append(
                f"{_color(f'drone_{drone_id}', 'cyan', self._use_color)} | "
                f"{_color(state.phase, phase_color_name, self._use_color)} | "
                f"{_color(state.health, health_color, self._use_color)} | "
                f"{self._format_goal(state)} | {state.current_goal_distance_m:5.2f} | {state.assigned_speed_mps:4.1f} | "
                f"{_format_bool(ready_flag)} | {cmd_age:5.2f}"
            )
        return "\n".join(lines)

    def _publish_feedback(self, now_wall_sec: float, now_ros_sec: float):
        payload = self._build_feedback_payload(now_wall_sec, now_ros_sec)
        payload_json = json.dumps(payload, sort_keys=True, ensure_ascii=False)
        if payload_json == self._last_feedback_hash:
            return
        self._last_feedback_hash = payload_json
        self._feedback_pub.publish(String(data=payload_json))

    def _build_feedback_payload(self, now_wall_sec: float, now_ros_sec: float) -> Dict[str, object]:
        return {
            "session_name": self.session_name,
            "mission_phase": self.blackboard.mission_phase,
            "mission_mode": self.mission_manager_mode,
            "wall_time_sec": now_wall_sec,
            "ros_time_sec": now_ros_sec,
            "global_coverage_pct": self.blackboard.global_coverage_pct,
            "task_coverage_pct": self.blackboard.task_coverage_pct,
            "reassign_count": self.blackboard.reassign_count,
            "emergency_recoveries": self.blackboard.emergency_recoveries,
            "stop_reason": self.blackboard.stop_reason,
            "target_count": self._current_target_count(),
            "target_detected_count": sum(1 for record in self.blackboard.target_registry.values() if record.detected),
            "active_target_id": self.blackboard.active_target_id,
            "active_target_drone_id": self.blackboard.active_target_drone_id,
            "route_memory": {
                "failed_segments": len(self._failed_segment_memory),
                "failed_segment_hits": sum(self._failed_segment_memory.values()),
            },
            "drone_states": {
                str(drone_id): {
                    "phase": state.phase,
                    "health": state.health,
                    "goal_id": state.current_goal_id,
                    "goal_kind": state.current_goal_kind,
                    "goal_distance_m": state.current_goal_distance_m,
                    "speed_mps": state.assigned_speed_mps,
                    "ready": state.ready_since_ros_sec is not None,
                }
                for drone_id, state in self.drone_states.items()
            },
        }

    def _write_csv_row(self, now_wall_sec: float, now_ros_sec: float):
        target_detected_count = sum(1 for record in self.blackboard.target_registry.values() if record.detected)
        visited_count = len(self.blackboard.visited_nodes)
        total_count = len(self.task_waypoints)
        ready_count = sum(
            1
            for state in self.drone_states.values()
            if state.ready_since_ros_sec is not None and (now_ros_sec - state.ready_since_ros_sec) >= self.ready_stable_duration_sec
        )
        for drone_id in self.drone_ids:
            state = self.drone_states[drone_id]
            row = {
                "wall_time_sec": now_wall_sec,
                "ros_time_sec": now_ros_sec,
                "mission_phase": self.blackboard.mission_phase,
                "mission_mode": self.mission_manager_mode,
                "drone_id": drone_id,
                "drone_phase": state.phase,
                "health": state.health,
                "goal_kind": state.current_goal_kind,
                "goal_id": state.current_goal_id,
                "goal_x": state.current_goal_position[0] if state.current_goal_position is not None else "",
                "goal_y": state.current_goal_position[1] if state.current_goal_position is not None else "",
                "goal_z": state.current_goal_position[2] if state.current_goal_position is not None else "",
                "goal_distance_m": state.current_goal_distance_m,
                "speed_mps": state.assigned_speed_mps,
                "coverage_global_pct": self.blackboard.global_coverage_pct,
                "coverage_task_pct": self.blackboard.task_coverage_pct,
                "target_count": self._current_target_count(),
                "target_detected_count": target_detected_count,
                "active_target_id": self.blackboard.active_target_id,
                "active_target_drone_id": self.blackboard.active_target_drone_id,
                "reassign_count": self.blackboard.reassign_count,
                "emergency_recoveries": self.blackboard.emergency_recoveries,
                "stop_reason": self.blackboard.stop_reason,
                "waypoint_visited_count": visited_count,
                "waypoint_total_count": total_count,
                "ready_count": ready_count,
            }
            self._csv_writer.writerow(row)
        self._csv_handle.flush()

    def _finalize(self, reason: str):
        if self._finalized:
            return
        self._finalized = True
        self.blackboard.stop_reason = reason
        self.blackboard.mission_finished_ros_sec = _now_ros()
        self.blackboard.mission_finished_wall_sec = _now_wall()
        summary = {
            "session_name": self.session_name,
            "config_file": str(self.config_file),
            "output_dir": str(self.output_dir),
            "mission_mode": self.mission_manager_mode,
            "mission_phase": self.blackboard.mission_phase,
            "stop_reason": reason,
            "global_coverage_pct": self.blackboard.global_coverage_pct,
            "task_coverage_pct": self.blackboard.task_coverage_pct,
            "reassign_count": self.blackboard.reassign_count,
            "emergency_recoveries": self.blackboard.emergency_recoveries,
            "target_count": self._current_target_count(),
            "target_detected_count": sum(1 for record in self.blackboard.target_registry.values() if record.detected),
            "route_memory": {
                "failed_segments": len(self._failed_segment_memory),
                "failed_segment_hits": sum(self._failed_segment_memory.values()),
                "top_failed_segments": sorted(
                    ((segment_key, count) for segment_key, count in self._failed_segment_memory.items()),
                    key=lambda item: item[1],
                    reverse=True,
                )[:20],
            },
            "drone_states": {
                str(drone_id): {
                    "phase": state.phase,
                    "health": state.health,
                    "goal_id": state.current_goal_id,
                    "goal_kind": state.current_goal_kind,
                    "goal_distance_m": state.current_goal_distance_m,
                    "speed_mps": state.assigned_speed_mps,
                    "waypoint_history": list(state.waypoint_history),
                }
                for drone_id, state in self.drone_states.items()
            },
            "target_registry": {
                target_id: {
                    "position": list(record.position),
                    "detected": record.detected,
                    "detected_by": record.detected_by,
                    "confirmed": record.confirmed,
                    "first_detection_ros_sec": record.first_detection_ros_sec,
                    "first_detection_wall_sec": record.first_detection_wall_sec,
                }
                for target_id, record in self.blackboard.target_registry.items()
            },
            "visited_nodes": self.blackboard.visited_nodes,
        }
        with self.summary_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)
        self._csv_handle.close()
        self._feedback_pub.publish(String(data=json.dumps(summary, sort_keys=True, ensure_ascii=False)))
        rospy.loginfo("[clean_uav_core] mission manager finalized reason=%s", reason)

    def _tick(self, _event):
        with self._lock:
            if rospy.is_shutdown():
                return
            now_wall_sec = _now_wall()
            now_ros_sec = _now_ros()

            self._start_mission_if_needed(now_ros_sec)
            for drone_id, state in self.drone_states.items():
                self._update_drone_goal(drone_id, state, now_ros_sec)

            self._maybe_transition_to_return()
            self._publish_goals()
            self._publish_feedback(now_wall_sec, now_ros_sec)
            self._write_csv_row(now_wall_sec, now_ros_sec)

            if now_wall_sec - self._last_render_wall_sec >= 2.0:
                self._last_render_wall_sec = now_wall_sec
                rospy.loginfo("\n%s", self._render_table(now_ros_sec))

            if self.blackboard.mission_phase == "RETURNING" and self._mission_finished():
                self.blackboard.mission_phase = "DONE"
                self._request_benchmark_stop("mission_complete")
                self._finalize("mission_complete")
                rospy.signal_shutdown("mission complete")

            if self.blackboard.mission_phase == "WAIT_READY" and self.ready_timeout_sec > 0.0 and self.blackboard.mission_started_ros_sec is not None:
                if (now_ros_sec - self.blackboard.mission_started_ros_sec) > self.ready_timeout_sec and self.wait_for_all_uavs:
                    rospy.logwarn("[clean_uav_core] readiness timeout, starting mission anyway")


def main():
    rospy.init_node("swarm_mission_manager", anonymous=False)
    manager = SwarmMissionManager()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        with getattr(manager, "_lock", threading.RLock()):
            if hasattr(manager, "_finalize") and not getattr(manager, "_finalized", True):
                manager._finalize("shutdown")


if __name__ == "__main__":
    main()