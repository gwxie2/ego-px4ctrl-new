#!/usr/bin/env python3

"""Coverage memory, virtual target detection, and runtime metrics helper.

This module is intentionally standalone so the benchmark manager can import it
without changing the planner or controller C++ code paths.
"""

import json
import math
import os
import threading
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - workspace is expected to provide numpy
    raise RuntimeError("benchmark_coverage_engine requires numpy") from exc

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - plotting is optional in minimal environments
    plt = None

try:
    import rospy
    from std_msgs.msg import String
except Exception:  # pragma: no cover - helper can still be imported offline
    rospy = None
    String = None


def _now_wall() -> float:
    return datetime.now().timestamp()


def _ros_now() -> float:
    if rospy is None:
        return _now_wall()
    try:
        return rospy.Time.now().to_sec()
    except Exception:
        return _now_wall()


def _stamp_to_sec(msg) -> float:
    try:
        stamp = msg.header.stamp
        value = stamp.to_sec()
        if value > 0.0:
            return float(value)
    except Exception:
        pass
    return _ros_now()


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _coerce_float_triplets(raw_value) -> List[Tuple[float, float, float]]:
    if raw_value is None:
        return []
    if isinstance(raw_value, str):
        text = raw_value.strip()
        if not text:
            return []
        groups = []
        for chunk in text.replace("|", ";").split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            parts = [item.strip() for item in chunk.replace(" ", ",").split(",") if item.strip()]
            if len(parts) >= 3:
                groups.append((float(parts[0]), float(parts[1]), float(parts[2])))
        return groups
    if isinstance(raw_value, (list, tuple)):
        if raw_value and isinstance(raw_value[0], dict):
            groups = []
            for item in raw_value:
                groups.append((float(item["x"]), float(item["y"]), float(item["z"])))
            return groups
        if raw_value and isinstance(raw_value[0], (list, tuple)) and len(raw_value[0]) >= 3:
            return [(float(item[0]), float(item[1]), float(item[2])) for item in raw_value]
        flat = [float(item) for item in raw_value]
        groups = []
        for index in range(0, len(flat) - 2, 3):
            groups.append((flat[index], flat[index + 1], flat[index + 2]))
        return groups
    return []


def _clamp_search_target_z(value: float) -> float:
    return max(1.0, min(4.0, float(value)))


def _normalize_search_target(position: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return float(position[0]), float(position[1]), _clamp_search_target_z(position[2])


def _bitcount_u64(mask: np.ndarray) -> np.ndarray:
    byte_view = mask.view(np.uint8).reshape(mask.shape + (8,))
    return np.unpackbits(byte_view, axis=-1).sum(axis=-1)


@dataclass
class _DroneRuntimeState:
    drone_id: int
    position: Optional[Tuple[float, float, float]] = None
    velocity: Optional[Tuple[float, float, float]] = None
    yaw: float = 0.0
    odom_ros_sec: Optional[float] = None
    last_velocity: Optional[Tuple[float, float, float]] = None
    last_acceleration: Optional[Tuple[float, float, float]] = None
    last_jerk_ros_sec: Optional[float] = None
    jerk_integral: float = 0.0
    replan_count: int = 0
    replan_latency_samples: List[float] = field(default_factory=list)
    replan_interval_samples: List[float] = field(default_factory=list)
    last_replan_ros_sec: Optional[float] = None
    last_event_type: int = -1
    last_event_state: int = -1


@dataclass
class _TargetState:
    index: int
    position: Tuple[float, float, float]
    detected: bool = False
    detected_by: int = -1
    first_detection_ros_sec: Optional[float] = None
    first_detection_wall_sec: Optional[float] = None


class CoverageMemoryEngine:
    def __init__(
        self,
        map_size_m: float = 80.0,
        grid_resolution_m: float = 1.0,
        fov_deg: float = 80.0,
        target_distance_threshold_m: float = 2.0,
        coverage_publish_hz: float = 2.0,
        projection_range_scale: float = 2.0,
        min_projection_range_m: float = 2.0,
        stop_on_target_detected: bool = True,
        output_topic: str = "/benchmark/coverage_metrics",
        target_topic: str = "/benchmark/target_detected",
    ):
        self.map_size_m = float(map_size_m)
        self.grid_resolution_m = max(0.1, float(grid_resolution_m))
        self.fov_deg = float(fov_deg)
        self.target_distance_threshold_m = float(target_distance_threshold_m)
        self.coverage_publish_hz = max(0.1, float(coverage_publish_hz))
        self.projection_range_scale = float(projection_range_scale)
        self.min_projection_range_m = max(0.1, float(min_projection_range_m))
        self.stop_on_target_detected = bool(stop_on_target_detected)
        self.output_topic = str(output_topic)
        self.target_topic = str(target_topic)

        self._lock = threading.RLock()
        self._session_active = False
        self._session_id = ""
        self._output_dir = ""
        self._session_start_ros_sec = None
        self._session_start_wall_sec = None
        self._session_drone_ids: List[int] = []
        self._session_drone_bits: Dict[int, np.uint64] = {}
        self._session_goals: Dict[int, Tuple[float, float, float]] = {}
        self._drone_states: Dict[int, _DroneRuntimeState] = {}
        self._targets: List[_TargetState] = []
        self._coverage_history: List[Dict[str, float]] = []
        self._target_detection_history: List[Dict[str, object]] = []
        self._last_publish_ros_sec: Optional[float] = None
        self._last_coverage_area_m2: float = 0.0
        self._target_detected_once: bool = False
        self._stop_requested: bool = False
        self._stop_reason: str = ""
        self._metrics_pub = None
        self._target_pub = None

        self._grid_origin_x = -self.map_size_m / 2.0
        self._grid_origin_y = -self.map_size_m / 2.0
        self._grid_width = int(math.ceil(self.map_size_m / self.grid_resolution_m))
        self._grid_height = int(math.ceil(self.map_size_m / self.grid_resolution_m))
        xs = self._grid_origin_x + (np.arange(self._grid_width, dtype=np.float64) + 0.5) * self.grid_resolution_m
        ys = self._grid_origin_y + (np.arange(self._grid_height, dtype=np.float64) + 0.5) * self.grid_resolution_m
        self._cell_xs, self._cell_ys = np.meshgrid(xs, ys)
        self._coverage_mask = np.zeros((self._grid_height, self._grid_width), dtype=np.uint64)
        self._unique_drone_hits = np.zeros((self._grid_height, self._grid_width), dtype=np.uint8)

    @staticmethod
    def _ros_publish_ready() -> bool:
        if rospy is None or String is None:
            return False
        core = getattr(rospy, "core", None)
        is_initialized = getattr(core, "is_initialized", None)
        if not callable(is_initialized):
            return False
        try:
            return bool(is_initialized())
        except Exception:
            return False

    def _ensure_publishers(self):
        if not self._ros_publish_ready():
            return
        if self._metrics_pub is None:
            self._metrics_pub = rospy.Publisher(self.output_topic, String, queue_size=10, latch=True)
        if self._target_pub is None:
            self._target_pub = rospy.Publisher(self.target_topic, String, queue_size=10, latch=True)

    def _default_targets_from_goals(self, goals: Dict[int, Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        if not goals:
            return []
        ordered_goal_ids = sorted(goals.keys())
        return [goals[goal_id] for goal_id in ordered_goal_ids]

    def start_session(
        self,
        session_id: str,
        output_dir: str,
        drone_ids: Sequence[int],
        goals: Optional[Dict[int, Tuple[float, float, float]]] = None,
        target_positions=None,
        session_start_ros_sec: Optional[float] = None,
        session_start_wall_sec: Optional[float] = None,
    ):
        with self._lock:
            self._session_active = True
            self._session_id = str(session_id)
            self._output_dir = str(output_dir)
            self._session_start_ros_sec = float(session_start_ros_sec) if session_start_ros_sec is not None else _ros_now()
            self._session_start_wall_sec = float(session_start_wall_sec) if session_start_wall_sec is not None else _now_wall()
            self._session_drone_ids = [int(item) for item in drone_ids]
            self._session_drone_bits = {drone_id: np.uint64(1) << np.uint64(index) for index, drone_id in enumerate(self._session_drone_ids)}
            self._session_goals = dict(goals or {})
            parsed_targets = _coerce_float_triplets(target_positions)
            if not parsed_targets:
                parsed_targets = self._default_targets_from_goals(self._session_goals)
            parsed_targets = [_normalize_search_target(position) for position in parsed_targets]
            self._targets = [_TargetState(index=index, position=position) for index, position in enumerate(parsed_targets)]
            self._drone_states = {drone_id: _DroneRuntimeState(drone_id=drone_id) for drone_id in self._session_drone_ids}
            self._coverage_mask.fill(0)
            self._unique_drone_hits.fill(0)
            self._coverage_history.clear()
            self._target_detection_history.clear()
            self._last_publish_ros_sec = None
            self._last_coverage_area_m2 = 0.0
            self._target_detected_once = False
            self._stop_requested = False
            self._stop_reason = ""
            self._ensure_publishers()

    def stop_requested(self) -> bool:
        with self._lock:
            return bool(self._stop_requested)

    def stop_reason(self) -> str:
        with self._lock:
            return self._stop_reason

    def _mark_stop_requested(self, reason: str):
        if not self._stop_requested:
            self._stop_requested = True
            self._stop_reason = str(reason)

    def _cell_distance_and_angle(self, x: float, y: float, yaw: float):
        dx = self._cell_xs - x
        dy = self._cell_ys - y
        distance = np.hypot(dx, dy)
        angle = np.arctan2(dy, dx)
        angle_diff = np.arctan2(np.sin(angle - yaw), np.cos(angle - yaw))
        return distance, angle_diff

    def _project_visible_mask(self, position: Tuple[float, float, float], yaw: float) -> np.ndarray:
        x, y, z = position
        effective_range = max(self.min_projection_range_m, abs(z) * math.tan(math.radians(self.fov_deg / 2.0)) * self.projection_range_scale)
        half_fov = math.radians(self.fov_deg / 2.0)
        distance, angle_diff = self._cell_distance_and_angle(x, y, yaw)
        return (distance <= effective_range) & (np.abs(angle_diff) <= half_fov)

    def _record_coverage_update(self, drone_id: int, visible_mask: np.ndarray):
        drone_bit = self._session_drone_bits.get(int(drone_id))
        if drone_bit is None:
            return
        indices = np.where(visible_mask)
        if indices[0].size == 0:
            return
        prev_mask = self._coverage_mask[indices]
        new_cells = (prev_mask & drone_bit) == 0
        if not np.any(new_cells):
            return
        selected_rows = indices[0][new_cells]
        selected_cols = indices[1][new_cells]
        self._coverage_mask[selected_rows, selected_cols] = self._coverage_mask[selected_rows, selected_cols] | drone_bit
        self._unique_drone_hits[selected_rows, selected_cols] = np.minimum(
            self._unique_drone_hits[selected_rows, selected_cols] + 1,
            np.iinfo(np.uint8).max,
        )

    def _evaluate_targets(self, drone_id: int, position: Tuple[float, float, float], yaw: float, ros_sec: float):
        if not self._targets:
            return
        visible_mask = self._project_visible_mask(position, yaw)
        x, y, z = position
        for target in self._targets:
            if target.detected:
                continue
            tx, ty, tz = target.position
            distance = math.sqrt((x - tx) ** 2 + (y - ty) ** 2 + (z - tz) ** 2)
            if distance > self.target_distance_threshold_m:
                continue
            row = int((ty - self._grid_origin_y) / self.grid_resolution_m)
            col = int((tx - self._grid_origin_x) / self.grid_resolution_m)
            if row < 0 or col < 0 or row >= self._grid_height or col >= self._grid_width:
                continue
            if not bool(visible_mask[row, col]):
                continue
            target.detected = True
            target.detected_by = int(drone_id)
            target.first_detection_ros_sec = float(ros_sec)
            target.first_detection_wall_sec = _now_wall()
            self._target_detected_once = True
            if self.stop_on_target_detected:
                self._mark_stop_requested("target_detected")
            self._target_detection_history.append(
                {
                    "target_index": target.index,
                    "drone_id": int(drone_id),
                    "ros_time_sec": float(ros_sec),
                    "wall_time_sec": target.first_detection_wall_sec,
                    "distance_m": float(distance),
                }
            )
            self._publish_target_detected(target)
            break

    def _publish_target_detected(self, target: _TargetState):
        if self._target_pub is None or String is None or not self._ros_publish_ready():
            return
        payload = {
            "session_id": self._session_id,
            "target_index": target.index,
            "drone_id": target.detected_by,
            "ros_time_sec": target.first_detection_ros_sec,
            "wall_time_sec": target.first_detection_wall_sec,
            "position": list(target.position),
        }
        self._target_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _coverage_stats(self):
        explored_mask = self._coverage_mask != 0
        explored_cells = int(np.count_nonzero(explored_mask))
        redundant_cells = int(np.count_nonzero(_bitcount_u64(self._coverage_mask) >= 2))
        total_cells = int(self._coverage_mask.size)
        cell_area = self.grid_resolution_m * self.grid_resolution_m
        explored_area = explored_cells * cell_area
        total_area = total_cells * cell_area
        redundant_area = redundant_cells * cell_area
        coverage_pct = (explored_area / total_area * 100.0) if total_area > 0.0 else 0.0
        redundancy_factor = (redundant_area / explored_area) if explored_area > 0.0 else 0.0
        return {
            "explored_cells": explored_cells,
            "redundant_cells": redundant_cells,
            "total_cells": total_cells,
            "explored_area_m2": explored_area,
            "total_area_m2": total_area,
            "redundant_area_m2": redundant_area,
            "coverage_pct": coverage_pct,
            "redundancy_factor": redundancy_factor,
        }

    @staticmethod
    def _mean(values: Sequence[float]) -> float:
        finite = [float(value) for value in values if math.isfinite(float(value))]
        return sum(finite) / float(len(finite)) if finite else float("nan")

    @staticmethod
    def _p95(values: Sequence[float]) -> float:
        finite = sorted(float(value) for value in values if math.isfinite(float(value)))
        if not finite:
            return float("nan")
        index = int(math.ceil(0.95 * len(finite))) - 1
        index = max(0, min(index, len(finite) - 1))
        return float(finite[index])

    def _maybe_append_history(self, ros_sec: float):
        if self._last_publish_ros_sec is None:
            should_publish = True
        else:
            should_publish = (ros_sec - self._last_publish_ros_sec) >= (1.0 / self.coverage_publish_hz)
        if not should_publish:
            return
        stats = self._coverage_stats()
        delta_t = 0.0 if self._last_publish_ros_sec is None else max(0.0, ros_sec - self._last_publish_ros_sec)
        if delta_t > 0.0:
            exploration_rate = (stats["explored_area_m2"] - self._last_coverage_area_m2) / delta_t
        else:
            exploration_rate = 0.0
        self._last_publish_ros_sec = float(ros_sec)
        self._last_coverage_area_m2 = float(stats["explored_area_m2"])
        record = {
            "ros_time_sec": float(ros_sec),
            "coverage_pct": float(stats["coverage_pct"]),
            "exploration_rate_m2_s": float(exploration_rate),
            "redundancy_factor": float(stats["redundancy_factor"]),
            "explored_area_m2": float(stats["explored_area_m2"]),
            "redundant_area_m2": float(stats["redundant_area_m2"]),
        }
        self._coverage_history.append(record)
        self._publish_metrics(record)

    def _publish_metrics(self, sample_record: Optional[Dict[str, float]] = None):
        if self._metrics_pub is None or String is None or not self._ros_publish_ready():
            return
        payload = self.build_metrics_payload(sample_record=sample_record)
        self._metrics_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _jerk_stats(self):
        jerk_values = [state.jerk_integral for state in self._drone_states.values()]
        return {
            "jerk_integral_mean": self._mean(jerk_values),
            "jerk_integral_max": max(jerk_values) if jerk_values else float("nan"),
        }

    def _replan_stats(self):
        total_count = sum(state.replan_count for state in self._drone_states.values())
        latencies = [value for state in self._drone_states.values() for value in state.replan_latency_samples]
        intervals = [value for state in self._drone_states.values() for value in state.replan_interval_samples]
        elapsed = 0.0 if self._session_start_ros_sec is None else max(0.0, _ros_now() - self._session_start_ros_sec)
        frequency = (float(total_count) / elapsed) if elapsed > 1e-6 else 0.0
        return {
            "replan_count_total": int(total_count),
            "replan_frequency_hz": float(frequency),
            "optimization_latency_ms_mean": self._mean(latencies),
            "optimization_latency_ms_p95": self._p95(latencies),
            "replan_interval_ms_mean": self._mean(intervals),
        }

    def build_metrics_payload(self, sample_record: Optional[Dict[str, float]] = None) -> Dict[str, object]:
        with self._lock:
            stats = self._coverage_stats()
            jerk_stats = self._jerk_stats()
            replan_stats = self._replan_stats()
            detected_targets = [target for target in self._targets if target.detected]
            ttd_values = [
                target.first_detection_ros_sec - self._session_start_ros_sec
                for target in detected_targets
                if target.first_detection_ros_sec is not None and self._session_start_ros_sec is not None
            ]
            payload = {
                "session_id": self._session_id,
                "output_dir": self._output_dir,
                "map_size_m": self.map_size_m,
                "grid_resolution_m": self.grid_resolution_m,
                "fov_deg": self.fov_deg,
                "target_distance_threshold_m": self.target_distance_threshold_m,
                "coverage": {
                    "cumulative_coverage_pct": float(stats["coverage_pct"]),
                    "explored_area_m2": float(stats["explored_area_m2"]),
                    "total_area_m2": float(stats["total_area_m2"]),
                    "exploration_rate_m2_s": float(sample_record["exploration_rate_m2_s"] if sample_record else 0.0),
                    "redundancy_factor": float(stats["redundancy_factor"]),
                },
                "targets": {
                    "total_targets": len(self._targets),
                    "detected_targets": len(detected_targets),
                    "target_detected": self._target_detected_once,
                    "time_to_first_detection_sec": min(ttd_values) if ttd_values else None,
                    "detections": [
                        {
                            "target_index": target.index,
                            "drone_id": target.detected_by,
                            "ros_time_sec": target.first_detection_ros_sec,
                            "wall_time_sec": target.first_detection_wall_sec,
                            "position": list(target.position),
                        }
                        for target in detected_targets
                    ],
                },
                "trajectory": {
                    "jerk_integral_mean": float(jerk_stats["jerk_integral_mean"]),
                    "jerk_integral_max": float(jerk_stats["jerk_integral_max"]),
                },
                "replanning": replan_stats,
                "drone_ids": list(self._session_drone_ids),
                "history_samples": len(self._coverage_history),
                "stop_requested": self._stop_requested,
                "stop_reason": self._stop_reason,
            }
            if sample_record is not None:
                payload["sample"] = sample_record
            return payload

    def update_odom(self, drone_id: int, msg):
        with self._lock:
            if not self._session_active:
                return
            state = self._drone_states.get(int(drone_id))
            if state is None:
                return
            ros_sec = _stamp_to_sec(msg)
            position = msg.pose.pose.position
            velocity = msg.twist.twist.linear
            current_velocity = (float(velocity.x), float(velocity.y), float(velocity.z))
            current_position = (float(position.x), float(position.y), float(position.z))
            current_yaw = 0.0
            try:
                orientation = msg.pose.pose.orientation
                current_yaw = _quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
            except Exception:
                current_yaw = state.yaw

            if state.odom_ros_sec is not None and ros_sec > state.odom_ros_sec:
                dt = max(1e-6, ros_sec - state.odom_ros_sec)
                if state.last_velocity is not None:
                    current_acc = tuple((current_velocity[index] - state.last_velocity[index]) / dt for index in range(3))
                    if state.last_acceleration is not None and state.last_jerk_ros_sec is not None and ros_sec > state.last_jerk_ros_sec:
                        jerk_dt = max(1e-6, ros_sec - state.last_jerk_ros_sec)
                        jerk_norm = math.sqrt(
                            sum(((current_acc[index] - state.last_acceleration[index]) / jerk_dt) ** 2 for index in range(3))
                        )
                        state.jerk_integral += jerk_norm * jerk_dt
                    state.last_acceleration = current_acc
                    state.last_jerk_ros_sec = ros_sec
            state.position = current_position
            state.velocity = current_velocity
            state.yaw = current_yaw
            state.odom_ros_sec = ros_sec
            state.last_velocity = current_velocity

            visible_mask = self._project_visible_mask(current_position, current_yaw)
            self._record_coverage_update(int(drone_id), visible_mask)
            self._evaluate_targets(int(drone_id), current_position, current_yaw, ros_sec)
            self._maybe_append_history(ros_sec)

    def update_replan(self, drone_id: int, msg):
        with self._lock:
            if not self._session_active:
                return
            state = self._drone_states.get(int(drone_id))
            if state is None:
                return
            ros_sec = _stamp_to_sec(msg)
            if state.last_replan_ros_sec is not None and ros_sec > state.last_replan_ros_sec:
                state.replan_interval_samples.append((ros_sec - state.last_replan_ros_sec) * 1000.0)
            state.last_replan_ros_sec = ros_sec
            state.replan_count += 1
            try:
                state.replan_latency_samples.append(float(msg.time_total_ms))
            except Exception:
                pass

    def update_event(self, drone_id: int, msg):
        with self._lock:
            if not self._session_active:
                return
            state = self._drone_states.get(int(drone_id))
            if state is None:
                return
            state.last_event_type = int(getattr(msg, "event_type", -1))
            state.last_event_state = int(getattr(msg, "current_state", -1))

    def finalize_session(self, reason: str):
        with self._lock:
            if not self._session_active:
                return
            self._stop_reason = str(reason)
            self._session_active = False
            self._write_artifacts(reason)

    def _write_artifacts(self, reason: str):
        output_dir = Path(self._output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        metrics_path = output_dir / "coverage_metrics.json"
        target_path = output_dir / "target_detection.json"
        summary_path = output_dir / "coverage_memory_summary.json"
        history_path = output_dir / "coverage_history.json"
        payload = self.build_metrics_payload()
        payload["stop_reason"] = str(reason)
        payload["finished_wall_time_sec"] = _now_wall()
        payload["finished_ros_time_sec"] = _ros_now()
        with metrics_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, ensure_ascii=False)
        with target_path.open("w", encoding="utf-8") as handle:
            json.dump(self._target_detection_history, handle, indent=2, ensure_ascii=False)
        with history_path.open("w", encoding="utf-8") as handle:
            json.dump(self._coverage_history, handle, indent=2, ensure_ascii=False)
        with summary_path.open("w", encoding="utf-8") as handle:
            json.dump(
                {
                    "session_id": self._session_id,
                    "metrics_path": str(metrics_path),
                    "target_path": str(target_path),
                    "history_path": str(history_path),
                    "stop_reason": str(reason),
                    "history_samples": len(self._coverage_history),
                },
                handle,
                indent=2,
                ensure_ascii=False,
            )
        self._write_plot(output_dir, payload)

    def _write_plot(self, output_dir: Path, payload: Dict[str, object]):
        if plt is None or not self._coverage_history:
            return
        times = [float(item["ros_time_sec"]) - float(self._session_start_ros_sec or 0.0) for item in self._coverage_history]
        coverage = [float(item["coverage_pct"]) for item in self._coverage_history]
        exploration = [float(item["exploration_rate_m2_s"]) for item in self._coverage_history]
        redundancy = [float(item["redundancy_factor"]) for item in self._coverage_history]
        fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
        axes[0].plot(times, coverage, color="#145da0", linewidth=2)
        axes[0].set_ylabel("Coverage %")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(times, exploration, color="#0b6e4f", linewidth=2)
        axes[1].set_ylabel("Exploration m^2/s")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(times, redundancy, color="#b23a48", linewidth=2)
        axes[2].set_ylabel("Redundancy")
        axes[2].set_xlabel("Time (s)")
        axes[2].grid(True, alpha=0.3)
        fig.suptitle(f"Coverage vs. Time - {self._session_id}")
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        fig.savefig(output_dir / "coverage_vs_time.png", dpi=200, bbox_inches="tight")
        plt.close(fig)


__all__ = ["CoverageMemoryEngine"]
