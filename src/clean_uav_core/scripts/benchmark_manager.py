#!/usr/bin/env python3

import csv
import json
import math
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

import rospy
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import AttitudeTarget, State as MavrosState
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PlannerBenchmarkEvent, PlannerReplanInfo, PositionCommand, Px4ctrlDebug
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

from clean_uav_core.srv import BenchmarkStartSession, BenchmarkStartSessionResponse


MAIN_CSV_HEADERS = [
    "wall_time_sec",
    "ros_time_sec",
    "session_id",
    "drone_id",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "speed_mps",
    "p_des_x",
    "p_des_y",
    "p_des_z",
    "v_des_x",
    "v_des_y",
    "v_des_z",
    "a_des_x",
    "a_des_y",
    "a_des_z",
    "jerk_des_x",
    "jerk_des_y",
    "jerk_des_z",
    "tracking_error_m",
    "jerk_norm",
    "velocity_efficiency",
    "goal_distance_m",
    "obstacle_clearance_m",
    "neighbor_distance_m",
    "safety_margin_m",
    "safety_violation",
    "control_lag_ms",
    "attitude_thrust",
    "actuator_ratio",
    "actuator_saturated",
    "planner_latency_ms",
    "planner_replan_interval_ms",
    "planner_iter_count",
    "planner_success",
    "planner_touch_goal",
    "planner_trigger_reason",
    "planner_failure_reason",
    "planner_astar_expanded_nodes",
    "planner_gradient_norm_final",
    "planner_cost_initial",
    "planner_cost_final",
    "trajectory_id",
    "mavros_connected",
    "mavros_mode",
    "last_event_type",
    "last_event_state",
]

REPLAN_CSV_HEADERS = [
    "wall_time_sec",
    "ros_time_sec",
    "session_id",
    "drone_id",
    "trajectory_id",
    "replan_count",
    "success",
    "touch_goal",
    "iter_count",
    "time_search_ms",
    "time_optimize_ms",
    "time_adjust_ms",
    "time_total_ms",
    "replan_interval_ms",
    "local_target_distance",
    "trigger_reason",
    "failure_reason",
    "a_star_expanded_nodes",
    "gradient_norm_final",
    "cost_initial",
    "cost_final",
]

EVENT_CSV_HEADERS = [
    "wall_time_sec",
    "ros_time_sec",
    "session_id",
    "drone_id",
    "trajectory_id",
    "previous_state",
    "current_state",
    "event_type",
    "trigger_reason",
    "success",
    "goal_distance",
    "caller",
]


def _safe_float(value) -> float:
    try:
        result = float(value)
    except Exception:
        return float("nan")
    return result


def _p95(values: Sequence[float]) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return float("nan")
    index = int(math.ceil(0.95 * len(finite))) - 1
    index = max(0, min(index, len(finite) - 1))
    return finite[index]


def _mean(values: Sequence[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return float("nan")
    return sum(finite) / float(len(finite))


def _correlation(xs: Sequence[float], ys: Sequence[float]) -> float:
    pairs = [(float(x), float(y)) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 2:
        return float("nan")
    mean_x = sum(item[0] for item in pairs) / float(len(pairs))
    mean_y = sum(item[1] for item in pairs) / float(len(pairs))
    cov = sum((item[0] - mean_x) * (item[1] - mean_y) for item in pairs)
    var_x = sum((item[0] - mean_x) ** 2 for item in pairs)
    var_y = sum((item[1] - mean_y) ** 2 for item in pairs)
    if var_x <= 1e-12 or var_y <= 1e-12:
        return float("nan")
    return cov / math.sqrt(var_x * var_y)


@dataclass
class DroneSessionState:
    drone_id: int
    goal: Optional[Tuple[float, float, float]] = None
    latest_odom: Optional[Odometry] = None
    latest_cmd: Optional[PositionCommand] = None
    latest_replan: Optional[PlannerReplanInfo] = None
    latest_event: Optional[PlannerBenchmarkEvent] = None
    latest_attitude: Optional[AttitudeTarget] = None
    latest_mavros_state: Optional[MavrosState] = None
    latest_px4_debug: Optional[Px4ctrlDebug] = None
    latest_model_states: Optional[ModelStates] = None
    latest_obstacle_clearance_m: Optional[float] = None
    latest_neighbor_distance_m: Optional[float] = None
    low_speed_since: Optional[float] = None
    terminal_reason: Optional[str] = None
    terminal_deadline: Optional[float] = None
    bag_process: Optional[subprocess.Popen] = None
    bag_topics: List[str] = field(default_factory=list)
    bag_path: str = ""
    bag_started_wall_sec: Optional[float] = None
    bag_stopped_wall_sec: Optional[float] = None
    main_csv_handle: Optional[object] = None
    main_csv_writer: Optional[csv.DictWriter] = None
    replan_csv_handle: Optional[object] = None
    replan_csv_writer: Optional[csv.DictWriter] = None
    event_csv_handle: Optional[object] = None
    event_csv_writer: Optional[csv.DictWriter] = None
    main_csv_path: str = ""
    replan_csv_path: str = ""
    event_csv_path: str = ""
    last_cmd_acc: Optional[Tuple[float, float, float]] = None
    last_cmd_acc_stamp: Optional[float] = None
    command_stamp: Optional[float] = None
    replan_stamp: Optional[float] = None
    event_stamp: Optional[float] = None
    attitude_stamp: Optional[float] = None
    odom_stamp: Optional[float] = None
    rows_written: int = 0
    replan_rows_written: int = 0
    event_rows_written: int = 0
    tracking_error_samples: List[float] = field(default_factory=list)
    speed_samples: List[float] = field(default_factory=list)
    control_lag_samples: List[float] = field(default_factory=list)
    safety_margin_samples: List[float] = field(default_factory=list)
    actuator_ratio_samples: List[float] = field(default_factory=list)
    planner_latency_samples: List[float] = field(default_factory=list)
    planner_interval_samples: List[float] = field(default_factory=list)
    planner_iter_samples: List[float] = field(default_factory=list)
    replan_success_count: int = 0
    replan_total_count: int = 0
    safety_violation_count: int = 0
    actuator_saturation_hits: int = 0
    actuator_saturation_samples: int = 0
    jerk_integral: float = 0.0
    last_row_ros_time: Optional[float] = None
    min_obstacle_clearance_m: float = float("inf")
    min_neighbor_distance_m: float = float("inf")
    min_safety_margin_m: float = float("inf")
    max_speed_mps: float = 0.0
    max_actuator_ratio: float = 0.0


class BenchmarkManager:
    def __init__(self):
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/swarm_benchmark/data"))
        self.planner_node_name = rospy.get_param("~planner_node_name", "ego_planner")
        self.drone_count = int(rospy.get_param("~drone_count", 1))
        self.default_vmax = float(rospy.get_param("~default_vmax", 10.0))
        self.default_record_rosbag = bool(rospy.get_param("~record_rosbag", True))
        self.default_low_speed_threshold = float(rospy.get_param("~low_speed_threshold", 0.1))
        self.default_low_speed_duration = float(rospy.get_param("~low_speed_duration", 2.0))
        self.default_goal_distance_threshold = float(rospy.get_param("~goal_distance_threshold", 0.5))
        self.stop_after_terminal_sec = float(rospy.get_param("~stop_after_terminal_sec", 2.0))
        self.point_stride = max(1, int(rospy.get_param("~point_stride", 10)))
        self.model_radius = float(rospy.get_param("~model_radius", 0.35))
        self.safety_margin_threshold = float(rospy.get_param("~safety_margin_threshold", 0.5))
        self.actuator_saturation_threshold = float(rospy.get_param("~actuator_saturation_threshold", 0.9))
        self.odom_topic_template = rospy.get_param("~odom_topic_template", "/drone_%d/odom")
        self.position_cmd_topic_template = rospy.get_param("~position_cmd_topic_template", "/drone_%d/position_cmd")
        self.replan_info_topic_template = rospy.get_param(
            "~replan_info_topic_template", f"/drone_%d/{self.planner_node_name}/planning/replan_info"
        )
        self.planner_event_topic_template = rospy.get_param(
            "~planner_event_topic_template", f"/drone_%d/{self.planner_node_name}/planning/benchmark_event"
        )
        self.safety_topic_template = rospy.get_param(
            "~safety_topic_template", f"/drone_%d/{self.planner_node_name}/grid_map/occupancy_inflate"
        )
        self.attitude_topic_template = rospy.get_param("~attitude_topic_template", "/iris_%d/mavros/setpoint_raw/attitude")
        self.mavros_state_topic_template = rospy.get_param("~mavros_state_topic_template", "/iris_%d/mavros/state")
        self.extrinsic_topic_template = rospy.get_param("~extrinsic_topic_template", "/drone_%d/vins_estimator/extrinsic")
        self.px4_debug_topic_template = rospy.get_param("~px4_debug_topic_template", "/drone_%d/px4ctrl/debugPx4ctrl")
        self.default_goal_xs = self._coerce_float_list(rospy.get_param("~default_goal_xs", []))
        self.default_goal_ys = self._coerce_float_list(rospy.get_param("~default_goal_ys", []))
        self.default_goal_zs = self._coerce_float_list(rospy.get_param("~default_goal_zs", []))
        self.default_drone_ids = self._coerce_int_list(rospy.get_param("~default_drone_ids", []))
        if not self.default_drone_ids:
            self.default_drone_ids = list(range(self.drone_count))

        self._lock = threading.RLock()
        self._subs = []
        self._states: Dict[int, DroneSessionState] = {
            drone_id: DroneSessionState(drone_id=drone_id) for drone_id in self.default_drone_ids
        }

        self._session_active = False
        self._session_id = ""
        self._session_name = ""
        self._session_mode = ""
        self._session_started_wall_sec: Optional[float] = None
        self._session_started_ros_sec: Optional[float] = None
        self._session_output_dir = ""
        self._session_drone_ids: List[int] = []
        self._session_vmax = self.default_vmax
        self._session_low_speed_threshold = self.default_low_speed_threshold
        self._session_low_speed_duration = self.default_low_speed_duration
        self._session_goal_distance_threshold = self.default_goal_distance_threshold
        self._session_record_rosbag = self.default_record_rosbag
        self._session_rosbag_topics_request: List[str] = []
        self._auto_session_started = False

        self._setup_subscribers()
        self._service = rospy.Service("start_session", BenchmarkStartSession, self._handle_start_session)
        self._monitor_timer = rospy.Timer(rospy.Duration(0.05), self._monitor_session)

        rospy.loginfo(
            "[clean_uav_core] benchmark_manager ready planner=%s drones=%d output_dir=%s",
            self.planner_node_name,
            self.drone_count,
            self.output_dir,
        )

    @staticmethod
    def _coerce_float_list(value) -> List[float]:
        if value is None:
            return []
        if isinstance(value, (list, tuple)):
            return [float(item) for item in value]
        text = str(value).strip()
        if not text:
            return []
        return [float(item) for item in text.replace(";", ",").split(",") if item.strip()]

    @staticmethod
    def _coerce_int_list(value) -> List[int]:
        if value is None:
            return []
        if isinstance(value, (list, tuple)):
            return [int(item) for item in value]
        text = str(value).strip()
        if not text:
            return []
        return [int(item) for item in text.replace(";", ",").split(",") if item.strip()]

    @staticmethod
    def _now_wall() -> float:
        return time.time()

    @staticmethod
    def _ros_stamp_to_sec(msg) -> float:
        try:
            stamp = msg.header.stamp
            value = stamp.to_sec()
            if value > 0.0:
                return value
        except Exception:
            pass
        return rospy.Time.now().to_sec()

    @staticmethod
    def _format_float(value: float) -> str:
        return f"{value:.6f}" if math.isfinite(value) else ""

    @staticmethod
    def _slugify_speed(value: float) -> str:
        text = f"{value:.1f}".rstrip("0").rstrip(".")
        return text.replace("-", "m").replace(".", "p")

    def _topic(self, template: str, drone_id: int) -> str:
        try:
            return template % drone_id
        except Exception:
            return str(template).format(drone_id=drone_id)

    def _ensure_state(self, drone_id: int) -> DroneSessionState:
        state = self._states.get(drone_id)
        if state is None:
            state = DroneSessionState(drone_id=drone_id)
            self._states[drone_id] = state
        return state

    def _setup_subscribers(self):
        for drone_id in self.default_drone_ids:
            self._subs.append(
                rospy.Subscriber(
                    self._topic(self.odom_topic_template, drone_id),
                    Odometry,
                    lambda msg, drone_id=drone_id: self._odom_callback(drone_id, msg),
                    queue_size=50,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    self._topic(self.position_cmd_topic_template, drone_id),
                    PositionCommand,
                    lambda msg, drone_id=drone_id: self._cmd_callback(drone_id, msg),
                    queue_size=50,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    self._topic(self.replan_info_topic_template, drone_id),
                    PlannerReplanInfo,
                    lambda msg, drone_id=drone_id: self._replan_callback(drone_id, msg),
                    queue_size=50,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    self._topic(self.planner_event_topic_template, drone_id),
                    PlannerBenchmarkEvent,
                    lambda msg, drone_id=drone_id: self._event_callback(drone_id, msg),
                    queue_size=50,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    self._topic(self.safety_topic_template, drone_id),
                    PointCloud2,
                    lambda msg, drone_id=drone_id: self._safety_callback(drone_id, msg),
                    queue_size=5,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    self._topic(self.attitude_topic_template, drone_id),
                    AttitudeTarget,
                    lambda msg, drone_id=drone_id: self._attitude_callback(drone_id, msg),
                    queue_size=50,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    self._topic(self.mavros_state_topic_template, drone_id),
                    MavrosState,
                    lambda msg, drone_id=drone_id: self._mavros_state_callback(drone_id, msg),
                    queue_size=20,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    self._topic(self.px4_debug_topic_template, drone_id),
                    Px4ctrlDebug,
                    lambda msg, drone_id=drone_id: self._px4_debug_callback(drone_id, msg),
                    queue_size=20,
                )
            )

        self._subs.append(rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=5))

    def _session_manifest_path(self) -> str:
        return os.path.join(self._session_output_dir, f"{self._session_id}_manifest.json")

    def _session_summary_path(self) -> str:
        return os.path.join(self._session_output_dir, f"{self._session_id}_summary.json")

    def _reset_state_for_session(self, state: DroneSessionState, goal: Optional[Tuple[float, float, float]]):
        state.goal = goal
        state.latest_odom = None
        state.latest_cmd = None
        state.latest_replan = None
        state.latest_event = None
        state.latest_attitude = None
        state.latest_mavros_state = None
        state.latest_px4_debug = None
        state.latest_obstacle_clearance_m = None
        state.latest_neighbor_distance_m = None
        state.low_speed_since = None
        state.terminal_reason = None
        state.terminal_deadline = None
        state.last_cmd_acc = None
        state.last_cmd_acc_stamp = None
        state.command_stamp = None
        state.replan_stamp = None
        state.event_stamp = None
        state.attitude_stamp = None
        state.odom_stamp = None
        state.rows_written = 0
        state.replan_rows_written = 0
        state.event_rows_written = 0
        state.tracking_error_samples.clear()
        state.speed_samples.clear()
        state.control_lag_samples.clear()
        state.safety_margin_samples.clear()
        state.actuator_ratio_samples.clear()
        state.planner_latency_samples.clear()
        state.planner_interval_samples.clear()
        state.planner_iter_samples.clear()
        state.replan_success_count = 0
        state.replan_total_count = 0
        state.safety_violation_count = 0
        state.actuator_saturation_hits = 0
        state.actuator_saturation_samples = 0
        state.jerk_integral = 0.0
        state.last_row_ros_time = None
        state.min_obstacle_clearance_m = float("inf")
        state.min_neighbor_distance_m = float("inf")
        state.min_safety_margin_m = float("inf")
        state.max_speed_mps = 0.0
        state.max_actuator_ratio = 0.0
        state.bag_topics = []
        state.bag_path = ""
        state.bag_started_wall_sec = None
        state.bag_stopped_wall_sec = None

    def _resolve_active_drone_ids(self, request_ids: Sequence[int]) -> List[int]:
        return [int(item) for item in request_ids] if request_ids else list(self.default_drone_ids)

    def _resolve_goals(self, goal_x, goal_y, goal_z, drone_ids: Sequence[int]) -> Dict[int, Tuple[float, float, float]]:
        goal_xs = self._coerce_float_list(goal_x)
        goal_ys = self._coerce_float_list(goal_y)
        goal_zs = self._coerce_float_list(goal_z)
        if not goal_xs:
            goal_xs = list(self.default_goal_xs)
        if not goal_ys:
            goal_ys = list(self.default_goal_ys)
        if not goal_zs:
            goal_zs = list(self.default_goal_zs)
        if len(goal_xs) != len(goal_ys) or len(goal_xs) != len(goal_zs) or len(goal_xs) != len(drone_ids):
            return {}
        return {
            int(drone_id): (float(goal_xs[index]), float(goal_ys[index]), float(goal_zs[index]))
            for index, drone_id in enumerate(drone_ids)
        }

    def _default_bag_topics_for_drone(self, drone_id: int) -> List[str]:
        topics = [
            self._topic(self.odom_topic_template, drone_id),
            self._topic(self.position_cmd_topic_template, drone_id),
            self._topic(self.extrinsic_topic_template, drone_id),
            self._topic(self.safety_topic_template, drone_id),
            self._topic(self.mavros_state_topic_template, drone_id),
            "/tf",
            "/tf_static",
        ]
        return sorted(set(topics))

    def _expand_bag_topics(self, topics: Sequence[str], drone_id: int) -> List[str]:
        expanded = []
        for topic in topics:
            if "%d" in topic or "{drone_id}" in topic:
                expanded.append(self._topic(topic, drone_id))
            else:
                expanded.append(topic)
        return sorted(set(expanded))

    def _open_writers(self, state: DroneSessionState):
        prefix = os.path.join(self._session_output_dir, f"drone_{state.drone_id}_{self._session_id}")

        state.main_csv_path = prefix + ".csv"
        state.main_csv_handle = open(state.main_csv_path, "w", newline="", encoding="utf-8")
        state.main_csv_writer = csv.DictWriter(state.main_csv_handle, fieldnames=MAIN_CSV_HEADERS)
        state.main_csv_writer.writeheader()
        state.main_csv_handle.flush()

        state.replan_csv_path = prefix + "_replan.csv"
        state.replan_csv_handle = open(state.replan_csv_path, "w", newline="", encoding="utf-8")
        state.replan_csv_writer = csv.DictWriter(state.replan_csv_handle, fieldnames=REPLAN_CSV_HEADERS)
        state.replan_csv_writer.writeheader()
        state.replan_csv_handle.flush()

        state.event_csv_path = prefix + "_event.csv"
        state.event_csv_handle = open(state.event_csv_path, "w", newline="", encoding="utf-8")
        state.event_csv_writer = csv.DictWriter(state.event_csv_handle, fieldnames=EVENT_CSV_HEADERS)
        state.event_csv_writer.writeheader()
        state.event_csv_handle.flush()

    @staticmethod
    def _close_writer(handle):
        if handle is None:
            return
        try:
            handle.flush()
            handle.close()
        except Exception:
            pass

    def _close_writers(self, state: DroneSessionState):
        self._close_writer(state.main_csv_handle)
        self._close_writer(state.replan_csv_handle)
        self._close_writer(state.event_csv_handle)
        state.main_csv_handle = None
        state.main_csv_writer = None
        state.replan_csv_handle = None
        state.replan_csv_writer = None
        state.event_csv_handle = None
        state.event_csv_writer = None

    def _start_rosbag_for_drone(self, state: DroneSessionState):
        if not self._session_record_rosbag or state.bag_process is not None:
            return
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        speed_slug = self._slugify_speed(self._session_vmax)
        state.bag_path = os.path.join(self._session_output_dir, f"drone_{state.drone_id}_vel{speed_slug}_{timestamp}.bag")
        state.bag_topics = (
            self._expand_bag_topics(self._session_rosbag_topics_request, state.drone_id)
            if self._session_rosbag_topics_request
            else self._default_bag_topics_for_drone(state.drone_id)
        )
        command = ["rosbag", "record", "-O", state.bag_path]
        command.extend(state.bag_topics)
        try:
            state.bag_process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            state.bag_started_wall_sec = self._now_wall()
            rospy.loginfo("[clean_uav_core] started rosbag for drone_%d: %s", state.drone_id, state.bag_path)
        except FileNotFoundError:
            state.bag_process = None
            rospy.logwarn("[clean_uav_core] rosbag not found; skip bag capture for drone_%d", state.drone_id)
        except Exception as exc:
            state.bag_process = None
            rospy.logwarn("[clean_uav_core] failed to start rosbag for drone_%d: %s", state.drone_id, exc)

    def _stop_rosbag_for_drone(self, state: DroneSessionState):
        if state.bag_process is None:
            return
        try:
            os.killpg(os.getpgid(state.bag_process.pid), signal.SIGINT)
            state.bag_process.wait(timeout=10.0)
        except Exception:
            try:
                os.killpg(os.getpgid(state.bag_process.pid), signal.SIGTERM)
            except Exception:
                pass
        finally:
            state.bag_process = None
            state.bag_stopped_wall_sec = self._now_wall()

    def _build_manifest(self) -> Dict[str, object]:
        return {
            "session_id": self._session_id,
            "session_name": self._session_name,
            "session_mode": self._session_mode,
            "planner_node_name": self.planner_node_name,
            "output_dir": self._session_output_dir,
            "drone_ids": list(self._session_drone_ids),
            "goals": {
                str(drone_id): list(self._states[drone_id].goal) if self._states[drone_id].goal is not None else None
                for drone_id in self._session_drone_ids
            },
            "started_wall_time_sec": self._session_started_wall_sec,
            "started_ros_time_sec": self._session_started_ros_sec,
            "v_max": self._session_vmax,
            "low_speed_threshold": self._session_low_speed_threshold,
            "low_speed_duration": self._session_low_speed_duration,
            "goal_distance_threshold": self._session_goal_distance_threshold,
            "record_rosbag": self._session_record_rosbag,
            "rosbag_topics_request": list(self._session_rosbag_topics_request),
        }

    def _write_manifest(self):
        with open(self._session_manifest_path(), "w", encoding="utf-8") as handle:
            json.dump(self._build_manifest(), handle, indent=2, ensure_ascii=False)

    def _handle_start_session(self, request):
        with self._lock:
            session_id, message = self._start_session_locked(request=request, source="manual")
        response = BenchmarkStartSessionResponse()
        response.accepted = bool(session_id)
        response.session_id = session_id or ""
        response.message = message
        return response

    def _start_session_locked(self, request=None, source: str = "manual"):
        if self._session_active:
            self._stop_session_locked("restart")

        request_drone_ids = list(request.drone_ids) if request is not None else []
        drone_ids = self._resolve_active_drone_ids(request_drone_ids)
        if not drone_ids:
            return None, "no active drones resolved"

        goal_x = request.goal_x if request is not None else []
        goal_y = request.goal_y if request is not None else []
        goal_z = request.goal_z if request is not None else []
        goals = self._resolve_goals(goal_x, goal_y, goal_z, drone_ids)
        if not goals:
            return None, "drone ids and goal arrays must have matching lengths"

        session_name = request.session_name.strip() if request is not None else ""
        if not session_name:
            session_name = f"{source}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self._session_id = session_name
        self._session_name = session_name
        self._session_mode = source
        self._session_started_wall_sec = self._now_wall()
        self._session_started_ros_sec = rospy.Time.now().to_sec()
        base_output_dir = request.output_dir.strip() if request is not None else ""
        base_output_dir = os.path.expanduser(base_output_dir or self.output_dir)
        self._session_output_dir = os.path.join(base_output_dir, self._session_id)
        os.makedirs(self._session_output_dir, exist_ok=True)
        self._session_drone_ids = list(drone_ids)

        requested_vmax = request.v_max if request is not None else 0.0
        self._session_vmax = requested_vmax if requested_vmax > 0.0 else self.default_vmax
        requested_low_speed = request.stop_speed_threshold if request is not None else 0.0
        requested_low_speed_duration = request.stop_duration_sec if request is not None else 0.0
        requested_goal_distance = request.goal_distance_threshold if request is not None else 0.0
        self._session_low_speed_threshold = requested_low_speed if requested_low_speed > 0.0 else self.default_low_speed_threshold
        self._session_low_speed_duration = requested_low_speed_duration if requested_low_speed_duration > 0.0 else self.default_low_speed_duration
        self._session_goal_distance_threshold = requested_goal_distance if requested_goal_distance > 0.0 else self.default_goal_distance_threshold
        if request is not None and request.rosbag_topics:
            self._session_rosbag_topics_request = list(request.rosbag_topics)
        else:
            self._session_rosbag_topics_request = []
        if request is None:
            self._session_record_rosbag = self.default_record_rosbag
        else:
            self._session_record_rosbag = request.record_rosbag or (not request.rosbag_topics and self.default_record_rosbag)

        for drone_id in self._session_drone_ids:
            state = self._ensure_state(drone_id)
            self._close_writers(state)
            self._stop_rosbag_for_drone(state)
            self._reset_state_for_session(state, goals.get(drone_id))
            self._open_writers(state)
            self._start_rosbag_for_drone(state)

        self._session_active = True
        if source == "auto":
            self._auto_session_started = True
        self._write_manifest()
        message = f"session {self._session_id} started in {self._session_output_dir}"
        rospy.loginfo("[clean_uav_core] %s", message)
        return self._session_id, message

    def _auto_start_if_needed(self, msg: PlannerBenchmarkEvent):
        if self._session_active or self._auto_session_started:
            return
        if msg.event_type != PlannerBenchmarkEvent.EVENT_STATE_TRANSITION:
            return
        if msg.current_state not in (
            PlannerBenchmarkEvent.STATE_GEN_NEW_TRAJ,
            PlannerBenchmarkEvent.STATE_REPLAN_TRAJ,
            PlannerBenchmarkEvent.STATE_EXEC_TRAJ,
        ):
            return
        session_id, message = self._start_session_locked(request=None, source="auto")
        if session_id:
            rospy.loginfo("[clean_uav_core] auto-started benchmark session: %s", message)

    def _arm_terminal(self, state: DroneSessionState, reason: str):
        now_sec = self._now_wall()
        if state.terminal_reason == "emergency_stop":
            return
        if state.terminal_reason is None or reason == "emergency_stop":
            state.terminal_reason = reason
            state.terminal_deadline = now_sec + self.stop_after_terminal_sec

    def _update_neighbor_distance_from_model_states(self, state: DroneSessionState) -> Optional[float]:
        if state.latest_odom is None or state.latest_model_states is None:
            return None
        drone_name = f"iris_{state.drone_id}"
        if drone_name not in state.latest_model_states.name:
            return None
        own_index = state.latest_model_states.name.index(drone_name)
        own_position = state.latest_odom.pose.pose.position
        min_distance = None
        for index, pose in enumerate(state.latest_model_states.pose):
            if index == own_index:
                continue
            other_name = state.latest_model_states.name[index]
            if not other_name.startswith("iris_"):
                continue
            dx = own_position.x - pose.position.x
            dy = own_position.y - pose.position.y
            dz = own_position.z - pose.position.z
            distance = max(0.0, math.sqrt(dx * dx + dy * dy + dz * dz) - 2.0 * self.model_radius)
            if min_distance is None or distance < min_distance:
                min_distance = distance
        state.latest_neighbor_distance_m = min_distance
        if min_distance is not None:
            state.min_neighbor_distance_m = min(state.min_neighbor_distance_m, min_distance)
        return min_distance

    def _compute_point_cloud_clearance(self, state: DroneSessionState, msg: PointCloud2) -> Optional[float]:
        if state.latest_odom is None:
            return None
        position = state.latest_odom.pose.pose.position
        min_distance = None
        sampled = 0
        for index, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % self.point_stride != 0:
                continue
            dx = position.x - point[0]
            dy = position.y - point[1]
            dz = position.z - point[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if min_distance is None or distance < min_distance:
                min_distance = distance
            sampled += 1
            if sampled >= 5000:
                break
        return min_distance

    def _event_callback(self, drone_id: int, msg: PlannerBenchmarkEvent):
        with self._lock:
            state = self._ensure_state(drone_id)
            state.latest_event = msg
            state.event_stamp = self._ros_stamp_to_sec(msg)
            self._auto_start_if_needed(msg)
            if not self._session_active or drone_id not in self._session_drone_ids:
                return
            self._write_event_row(state, msg)
            if msg.event_type == PlannerBenchmarkEvent.EVENT_GOAL_REACHED:
                self._arm_terminal(state, "goal_reached")
            elif msg.current_state == PlannerBenchmarkEvent.STATE_EMERGENCY_STOP:
                self._arm_terminal(state, "emergency_stop")

    def _replan_callback(self, drone_id: int, msg: PlannerReplanInfo):
        with self._lock:
            state = self._ensure_state(drone_id)
            state.latest_replan = msg
            state.replan_stamp = self._ros_stamp_to_sec(msg)
            if not self._session_active or drone_id not in self._session_drone_ids:
                return
            state.replan_total_count += 1
            if msg.success:
                state.replan_success_count += 1
            state.planner_latency_samples.append(float(msg.time_total_ms))
            state.planner_interval_samples.append(float(msg.replan_interval_ms))
            state.planner_iter_samples.append(float(msg.iter_count))
            self._write_replan_row(state, msg)

    def _odom_callback(self, drone_id: int, msg: Odometry):
        with self._lock:
            state = self._ensure_state(drone_id)
            state.latest_odom = msg
            state.odom_stamp = self._ros_stamp_to_sec(msg)
            self._update_neighbor_distance_from_model_states(state)

    def _cmd_callback(self, drone_id: int, msg: PositionCommand):
        with self._lock:
            state = self._ensure_state(drone_id)
            state.latest_cmd = msg
            state.command_stamp = self._ros_stamp_to_sec(msg)
            if state.last_cmd_acc is None:
                state.last_cmd_acc = (msg.acceleration.x, msg.acceleration.y, msg.acceleration.z)
                state.last_cmd_acc_stamp = state.command_stamp

    def _attitude_callback(self, drone_id: int, msg: AttitudeTarget):
        with self._lock:
            state = self._ensure_state(drone_id)
            state.latest_attitude = msg
            state.attitude_stamp = self._ros_stamp_to_sec(msg)

    def _mavros_state_callback(self, drone_id: int, msg: MavrosState):
        with self._lock:
            state = self._ensure_state(drone_id)
            state.latest_mavros_state = msg

    def _px4_debug_callback(self, drone_id: int, msg: Px4ctrlDebug):
        with self._lock:
            state = self._ensure_state(drone_id)
            state.latest_px4_debug = msg

    def _safety_callback(self, drone_id: int, msg: PointCloud2):
        with self._lock:
            state = self._ensure_state(drone_id)
            clearance = self._compute_point_cloud_clearance(state, msg)
            state.latest_obstacle_clearance_m = clearance
            if clearance is not None:
                state.min_obstacle_clearance_m = min(state.min_obstacle_clearance_m, clearance)

    def _model_states_callback(self, msg: ModelStates):
        with self._lock:
            for state in self._states.values():
                state.latest_model_states = msg
                self._update_neighbor_distance_from_model_states(state)

    def _compute_jerk_norm(self, state: DroneSessionState) -> float:
        if state.latest_cmd is None:
            return float("nan")
        jerk = state.latest_cmd.jerk
        jerk_norm = math.sqrt(jerk.x * jerk.x + jerk.y * jerk.y + jerk.z * jerk.z)
        if jerk_norm > 1e-6:
            return jerk_norm
        current_acc = (
            state.latest_cmd.acceleration.x,
            state.latest_cmd.acceleration.y,
            state.latest_cmd.acceleration.z,
        )
        if state.last_cmd_acc is None or state.last_cmd_acc_stamp is None or state.command_stamp is None:
            state.last_cmd_acc = current_acc
            state.last_cmd_acc_stamp = state.command_stamp
            return float("nan")
        delta_t = max(state.command_stamp - state.last_cmd_acc_stamp, 1e-6)
        jerk_x = (current_acc[0] - state.last_cmd_acc[0]) / delta_t
        jerk_y = (current_acc[1] - state.last_cmd_acc[1]) / delta_t
        jerk_z = (current_acc[2] - state.last_cmd_acc[2]) / delta_t
        state.last_cmd_acc = current_acc
        state.last_cmd_acc_stamp = state.command_stamp
        return math.sqrt(jerk_x * jerk_x + jerk_y * jerk_y + jerk_z * jerk_z)

    def _goal_distance(self, state: DroneSessionState) -> float:
        if state.goal is None or state.latest_odom is None:
            return float("nan")
        position = state.latest_odom.pose.pose.position
        dx = position.x - state.goal[0]
        dy = position.y - state.goal[1]
        dz = position.z - state.goal[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _tracking_error(self, state: DroneSessionState) -> float:
        if state.latest_odom is None or state.latest_cmd is None:
            return float("nan")
        position = state.latest_odom.pose.pose.position
        desired = state.latest_cmd.position
        dx = position.x - desired.x
        dy = position.y - desired.y
        dz = position.z - desired.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _speed(self, state: DroneSessionState) -> float:
        if state.latest_odom is None:
            return float("nan")
        velocity = state.latest_odom.twist.twist.linear
        return math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)

    def _safety_margin(self, state: DroneSessionState) -> float:
        candidates = []
        if state.latest_obstacle_clearance_m is not None:
            candidates.append(state.latest_obstacle_clearance_m)
        if state.latest_neighbor_distance_m is not None:
            candidates.append(state.latest_neighbor_distance_m)
        if not candidates:
            return float("nan")
        return min(candidates)

    def _actuator_ratio(self, state: DroneSessionState) -> float:
        candidates = []
        if state.latest_attitude is not None:
            candidates.append(float(state.latest_attitude.thrust))
        if state.latest_px4_debug is not None:
            candidates.append(float(state.latest_px4_debug.des_thr))
        finite = [max(0.0, min(1.0, value)) for value in candidates if math.isfinite(value)]
        if not finite:
            return float("nan")
        return max(finite)

    def _control_lag_ms(self, state: DroneSessionState) -> float:
        if state.odom_stamp is None or state.command_stamp is None:
            return float("nan")
        return abs(state.odom_stamp - state.command_stamp) * 1000.0

    def _write_replan_row(self, state: DroneSessionState, msg: PlannerReplanInfo):
        if state.replan_csv_writer is None:
            return
        row = {
            "wall_time_sec": f"{self._now_wall():.6f}",
            "ros_time_sec": f"{self._ros_stamp_to_sec(msg):.6f}",
            "session_id": self._session_id,
            "drone_id": state.drone_id,
            "trajectory_id": int(msg.trajectory_id),
            "replan_count": int(msg.replan_count),
            "success": int(bool(msg.success)),
            "touch_goal": int(bool(msg.touch_goal)),
            "iter_count": int(msg.iter_count),
            "time_search_ms": self._format_float(float(msg.time_search_ms)),
            "time_optimize_ms": self._format_float(float(msg.time_optimize_ms)),
            "time_adjust_ms": self._format_float(float(msg.time_adjust_ms)),
            "time_total_ms": self._format_float(float(msg.time_total_ms)),
            "replan_interval_ms": self._format_float(float(msg.replan_interval_ms)),
            "local_target_distance": self._format_float(float(msg.local_target_distance)),
            "trigger_reason": int(msg.trigger_reason),
            "failure_reason": int(msg.failure_reason),
            "a_star_expanded_nodes": int(msg.a_star_expanded_nodes),
            "gradient_norm_final": self._format_float(float(msg.gradient_norm_final)),
            "cost_initial": self._format_float(float(msg.cost_initial)),
            "cost_final": self._format_float(float(msg.cost_final)),
        }
        state.replan_csv_writer.writerow(row)
        state.replan_csv_handle.flush()
        state.replan_rows_written += 1

    def _write_event_row(self, state: DroneSessionState, msg: PlannerBenchmarkEvent):
        if state.event_csv_writer is None:
            return
        row = {
            "wall_time_sec": f"{self._now_wall():.6f}",
            "ros_time_sec": f"{self._ros_stamp_to_sec(msg):.6f}",
            "session_id": self._session_id,
            "drone_id": state.drone_id,
            "trajectory_id": int(msg.trajectory_id),
            "previous_state": int(msg.previous_state),
            "current_state": int(msg.current_state),
            "event_type": int(msg.event_type),
            "trigger_reason": int(msg.trigger_reason),
            "success": int(bool(msg.success)),
            "goal_distance": self._format_float(float(msg.goal_distance)),
            "caller": msg.caller,
        }
        state.event_csv_writer.writerow(row)
        state.event_csv_handle.flush()
        state.event_rows_written += 1

    def _write_main_row(self, state: DroneSessionState):
        if state.main_csv_writer is None or state.latest_odom is None:
            return

        odom = state.latest_odom
        position = odom.pose.pose.position
        velocity = odom.twist.twist.linear
        speed = self._speed(state)
        goal_distance = self._goal_distance(state)
        tracking_error = self._tracking_error(state)
        jerk_norm = self._compute_jerk_norm(state)
        safety_margin = self._safety_margin(state)
        control_lag_ms = self._control_lag_ms(state)
        actuator_ratio = self._actuator_ratio(state)
        actuator_saturated = int(math.isfinite(actuator_ratio) and actuator_ratio >= self.actuator_saturation_threshold)
        velocity_efficiency = speed / self._session_vmax if self._session_vmax > 1e-6 and math.isfinite(speed) else float("nan")

        desired_position = (float("nan"), float("nan"), float("nan"))
        desired_velocity = (float("nan"), float("nan"), float("nan"))
        desired_acc = (float("nan"), float("nan"), float("nan"))
        desired_jerk = (float("nan"), float("nan"), float("nan"))
        if state.latest_cmd is not None:
            desired_position = (state.latest_cmd.position.x, state.latest_cmd.position.y, state.latest_cmd.position.z)
            desired_velocity = (state.latest_cmd.velocity.x, state.latest_cmd.velocity.y, state.latest_cmd.velocity.z)
            desired_acc = (state.latest_cmd.acceleration.x, state.latest_cmd.acceleration.y, state.latest_cmd.acceleration.z)
            desired_jerk = (state.latest_cmd.jerk.x, state.latest_cmd.jerk.y, state.latest_cmd.jerk.z)

        replan = state.latest_replan
        event = state.latest_event
        row = {
            "wall_time_sec": f"{self._now_wall():.6f}",
            "ros_time_sec": f"{self._ros_stamp_to_sec(odom):.6f}",
            "session_id": self._session_id,
            "drone_id": state.drone_id,
            "x": f"{position.x:.6f}",
            "y": f"{position.y:.6f}",
            "z": f"{position.z:.6f}",
            "vx": f"{velocity.x:.6f}",
            "vy": f"{velocity.y:.6f}",
            "vz": f"{velocity.z:.6f}",
            "speed_mps": self._format_float(speed),
            "p_des_x": self._format_float(desired_position[0]),
            "p_des_y": self._format_float(desired_position[1]),
            "p_des_z": self._format_float(desired_position[2]),
            "v_des_x": self._format_float(desired_velocity[0]),
            "v_des_y": self._format_float(desired_velocity[1]),
            "v_des_z": self._format_float(desired_velocity[2]),
            "a_des_x": self._format_float(desired_acc[0]),
            "a_des_y": self._format_float(desired_acc[1]),
            "a_des_z": self._format_float(desired_acc[2]),
            "jerk_des_x": self._format_float(desired_jerk[0]),
            "jerk_des_y": self._format_float(desired_jerk[1]),
            "jerk_des_z": self._format_float(desired_jerk[2]),
            "tracking_error_m": self._format_float(tracking_error),
            "jerk_norm": self._format_float(jerk_norm),
            "velocity_efficiency": self._format_float(velocity_efficiency),
            "goal_distance_m": self._format_float(goal_distance),
            "obstacle_clearance_m": self._format_float(_safe_float(state.latest_obstacle_clearance_m)),
            "neighbor_distance_m": self._format_float(_safe_float(state.latest_neighbor_distance_m)),
            "safety_margin_m": self._format_float(safety_margin),
            "safety_violation": int(math.isfinite(safety_margin) and safety_margin < self.safety_margin_threshold),
            "control_lag_ms": self._format_float(control_lag_ms),
            "attitude_thrust": self._format_float(actuator_ratio),
            "actuator_ratio": self._format_float(actuator_ratio),
            "actuator_saturated": actuator_saturated,
            "planner_latency_ms": self._format_float(float(replan.time_total_ms) if replan is not None else float("nan")),
            "planner_replan_interval_ms": self._format_float(float(replan.replan_interval_ms) if replan is not None else float("nan")),
            "planner_iter_count": int(replan.iter_count) if replan is not None else "",
            "planner_success": int(bool(replan.success)) if replan is not None else "",
            "planner_touch_goal": int(bool(replan.touch_goal)) if replan is not None else "",
            "planner_trigger_reason": int(replan.trigger_reason) if replan is not None else "",
            "planner_failure_reason": int(replan.failure_reason) if replan is not None else "",
            "planner_astar_expanded_nodes": int(replan.a_star_expanded_nodes) if replan is not None else "",
            "planner_gradient_norm_final": self._format_float(float(replan.gradient_norm_final) if replan is not None else float("nan")),
            "planner_cost_initial": self._format_float(float(replan.cost_initial) if replan is not None else float("nan")),
            "planner_cost_final": self._format_float(float(replan.cost_final) if replan is not None else float("nan")),
            "trajectory_id": int(replan.trajectory_id) if replan is not None else "",
            "mavros_connected": int(bool(state.latest_mavros_state.connected)) if state.latest_mavros_state is not None else "",
            "mavros_mode": state.latest_mavros_state.mode if state.latest_mavros_state is not None else "",
            "last_event_type": int(event.event_type) if event is not None else "",
            "last_event_state": int(event.current_state) if event is not None else "",
        }
        state.main_csv_writer.writerow(row)
        state.main_csv_handle.flush()
        state.rows_written += 1

        if math.isfinite(tracking_error):
            state.tracking_error_samples.append(tracking_error)
        if math.isfinite(speed):
            state.speed_samples.append(speed)
            state.max_speed_mps = max(state.max_speed_mps, speed)
        if math.isfinite(control_lag_ms):
            state.control_lag_samples.append(control_lag_ms)
        if math.isfinite(safety_margin):
            state.safety_margin_samples.append(safety_margin)
            state.min_safety_margin_m = min(state.min_safety_margin_m, safety_margin)
            if safety_margin < self.safety_margin_threshold:
                state.safety_violation_count += 1
        if math.isfinite(actuator_ratio):
            state.actuator_ratio_samples.append(actuator_ratio)
            state.actuator_saturation_samples += 1
            state.max_actuator_ratio = max(state.max_actuator_ratio, actuator_ratio)
            if actuator_saturated:
                state.actuator_saturation_hits += 1
        if math.isfinite(jerk_norm):
            if state.last_row_ros_time is not None:
                dt = max(self._ros_stamp_to_sec(odom) - state.last_row_ros_time, 0.0)
                state.jerk_integral += jerk_norm * dt
            state.last_row_ros_time = self._ros_stamp_to_sec(odom)

    def _check_low_speed_goal_condition(self, state: DroneSessionState):
        if state.latest_odom is None or state.goal is None or state.terminal_reason is not None:
            return
        speed = self._speed(state)
        goal_distance = self._goal_distance(state)
        now_sec = self._now_wall()
        if math.isfinite(speed) and math.isfinite(goal_distance) and speed <= self._session_low_speed_threshold and goal_distance <= self._session_goal_distance_threshold:
            if state.low_speed_since is None:
                state.low_speed_since = now_sec
            elif now_sec - state.low_speed_since >= self._session_low_speed_duration:
                self._arm_terminal(state, "goal_reached")
        else:
            state.low_speed_since = None

    def _failure_prediction(self, state: DroneSessionState) -> str:
        tracking_error_p95 = _p95(state.tracking_error_samples)
        control_lag_p95 = _p95(state.control_lag_samples)
        planner_latency_p95 = _p95(state.planner_latency_samples)
        actuator_ratio_mean = _mean(state.actuator_ratio_samples)
        stress_score = self._computational_stress_score(state)

        predictions = []
        if (math.isfinite(tracking_error_p95) and tracking_error_p95 > max(1.0, 0.15 * self._session_vmax)) or (math.isfinite(control_lag_p95) and control_lag_p95 > 200.0):
            predictions.append("定位漂移")
        if (math.isfinite(planner_latency_p95) and planner_latency_p95 > 50.0) or stress_score > 0.75:
            predictions.append("计算超时")
        if (math.isfinite(actuator_ratio_mean) and actuator_ratio_mean > 0.7) or state.max_actuator_ratio >= self.actuator_saturation_threshold or state.max_speed_mps > 1.15 * self._session_vmax:
            predictions.append("动力学过载")
        return "正常" if not predictions else "/".join(predictions)

    def _computational_stress_score(self, state: DroneSessionState) -> float:
        latency_p95 = _p95(state.planner_latency_samples)
        interval_mean = _mean(state.planner_interval_samples)
        corr = _correlation(state.planner_iter_samples, state.planner_latency_samples)
        failure_rate = 1.0 - (float(state.replan_success_count) / float(state.replan_total_count)) if state.replan_total_count > 0 else 0.0
        components = []
        if math.isfinite(latency_p95):
            components.append(min(1.0, latency_p95 / 80.0))
        if math.isfinite(interval_mean) and interval_mean > 1e-6:
            components.append(min(1.0, 600.0 / max(interval_mean, 1.0)))
        if math.isfinite(corr):
            components.append(min(1.0, abs(corr)))
        components.append(min(1.0, failure_rate * 2.0))
        return min(1.0, _mean(components) if components else 0.0)

    def _build_summary(self, reason: str) -> Dict[str, object]:
        summary = self._build_manifest()
        summary["stop_reason"] = reason
        summary["stopped_wall_time_sec"] = self._now_wall()
        summary["drone_metrics"] = {}
        for drone_id in self._session_drone_ids:
            state = self._states[drone_id]
            summary["drone_metrics"][str(drone_id)] = {
                "goal": list(state.goal) if state.goal is not None else None,
                "rows_written": state.rows_written,
                "replan_rows_written": state.replan_rows_written,
                "event_rows_written": state.event_rows_written,
                "main_csv": os.path.basename(state.main_csv_path),
                "replan_csv": os.path.basename(state.replan_csv_path),
                "event_csv": os.path.basename(state.event_csv_path),
                "bag_path": os.path.basename(state.bag_path) if state.bag_path else "",
                "terminal_reason": state.terminal_reason or "",
                "tracking_error_mean": _mean(state.tracking_error_samples),
                "tracking_error_p95": _p95(state.tracking_error_samples),
                "speed_mean": _mean(state.speed_samples),
                "speed_max": state.max_speed_mps,
                "control_lag_mean_ms": _mean(state.control_lag_samples),
                "control_lag_p95_ms": _p95(state.control_lag_samples),
                "planner_latency_mean_ms": _mean(state.planner_latency_samples),
                "planner_latency_p95_ms": _p95(state.planner_latency_samples),
                "planner_replan_interval_mean_ms": _mean(state.planner_interval_samples),
                "planner_iter_latency_correlation": _correlation(state.planner_iter_samples, state.planner_latency_samples),
                "replan_total_count": state.replan_total_count,
                "replan_success_rate": (
                    float(state.replan_success_count) / float(state.replan_total_count)
                    if state.replan_total_count > 0
                    else float("nan")
                ),
                "computational_stress_score": self._computational_stress_score(state),
                "jerk_integral": state.jerk_integral,
                "min_obstacle_clearance_m": state.min_obstacle_clearance_m if state.min_obstacle_clearance_m < float("inf") else float("nan"),
                "min_neighbor_distance_m": state.min_neighbor_distance_m if state.min_neighbor_distance_m < float("inf") else float("nan"),
                "min_safety_margin_m": state.min_safety_margin_m if state.min_safety_margin_m < float("inf") else float("nan"),
                "safety_violation_count": state.safety_violation_count,
                "actuator_saturation_ratio": (
                    float(state.actuator_saturation_hits) / float(state.actuator_saturation_samples)
                    if state.actuator_saturation_samples > 0
                    else float("nan")
                ),
                "actuator_ratio_mean": _mean(state.actuator_ratio_samples),
                "actuator_ratio_max": state.max_actuator_ratio,
                "failure_prediction": self._failure_prediction(state),
            }
        return summary

    def _stop_session_locked(self, reason: str):
        if not self._session_active:
            return
        for drone_id in self._session_drone_ids:
            state = self._states[drone_id]
            self._stop_rosbag_for_drone(state)
            self._close_writers(state)
        summary = self._build_summary(reason)
        with open(self._session_summary_path(), "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)
        rospy.loginfo("[clean_uav_core] benchmark session %s stopped (%s)", self._session_id, reason)
        self._session_active = False

    def _monitor_session(self, _event):
        with self._lock:
            if not self._session_active:
                return

            now_sec = self._now_wall()
            emergency_due = False
            all_terminal = True
            for drone_id in self._session_drone_ids:
                state = self._states[drone_id]
                self._update_neighbor_distance_from_model_states(state)
                self._check_low_speed_goal_condition(state)
                self._write_main_row(state)
                if state.terminal_deadline is not None and now_sec >= state.terminal_deadline:
                    self._stop_rosbag_for_drone(state)
                    if state.terminal_reason == "emergency_stop":
                        emergency_due = True
                if state.terminal_reason is None or state.terminal_deadline is None or now_sec < state.terminal_deadline:
                    all_terminal = False

            if emergency_due:
                self._stop_session_locked("emergency_stop")
            elif all_terminal:
                self._stop_session_locked("all_goals_reached")

    def run(self):
        rospy.spin()


def main():
    rospy.init_node("benchmark_manager", anonymous=False)
    BenchmarkManager().run()


if __name__ == "__main__":
    main()
