#!/usr/bin/env python3

import argparse
import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry


@dataclass
class VehicleStats:
    name: str
    x_target: float
    x_reach_tol: float
    min_altitude: float
    max_position_jump: float

    first_time: Optional[float] = None
    last_time: Optional[float] = None
    sample_count: int = 0

    x_max: float = -1e9
    z_min: float = 1e9
    z_max: float = -1e9

    reached_x_target: bool = False
    reached_time: Optional[float] = None

    last_pos: Optional[Tuple[float, float, float]] = None
    max_jump_observed: float = 0.0
    jump_violation_count: int = 0

    mavros_connected_seen: bool = False
    mavros_offboard_seen: bool = False
    mavros_armed_seen: bool = False

    x_series: List[float] = field(default_factory=list)

    def update_odom(self, t_sec: float, x: float, y: float, z: float):
        if self.first_time is None:
            self.first_time = t_sec
        self.last_time = t_sec
        self.sample_count += 1

        self.x_max = max(self.x_max, x)
        self.z_min = min(self.z_min, z)
        self.z_max = max(self.z_max, z)
        self.x_series.append(x)

        if not self.reached_x_target and x >= self.x_target - self.x_reach_tol:
            self.reached_x_target = True
            self.reached_time = t_sec

        if self.last_pos is not None:
            jump = math.sqrt((x - self.last_pos[0]) ** 2 + (y - self.last_pos[1]) ** 2 + (z - self.last_pos[2]) ** 2)
            self.max_jump_observed = max(self.max_jump_observed, jump)
            if jump > self.max_position_jump:
                self.jump_violation_count += 1
        self.last_pos = (x, y, z)

    def update_state(self, state_msg: State):
        self.mavros_connected_seen = self.mavros_connected_seen or bool(state_msg.connected)
        self.mavros_offboard_seen = self.mavros_offboard_seen or (state_msg.mode == "OFFBOARD")
        self.mavros_armed_seen = self.mavros_armed_seen or bool(state_msg.armed)

    def x_progress_ratio(self, x_start_assumed: float = 0.0) -> float:
        denom = max(1e-6, self.x_target - x_start_assumed)
        return max(0.0, min(1.0, (self.x_max - x_start_assumed) / denom))


class Phase4AcceptanceQuant:
    def __init__(self, args):
        self.duration = args.duration
        self.output = args.output
        self.md_output = args.md_output

        self.min_separation = args.min_separation
        self.separation_violation_limit = args.separation_violation_limit
        self.x_target = args.x_target
        self.x_reach_tol = args.x_reach_tol
        self.min_altitude = args.min_altitude
        self.max_position_jump = args.max_position_jump

        self.start_wall = time.time()
        self.latest_pos: Dict[str, Optional[Tuple[float, float, float]]] = {"iris_0": None, "iris_1": None}
        self.min_separation_observed = 1e9
        self.separation_violation_count = 0

        self.vehicles = {
            "iris_0": VehicleStats("iris_0", self.x_target, self.x_reach_tol, self.min_altitude, self.max_position_jump),
            "iris_1": VehicleStats("iris_1", self.x_target, self.x_reach_tol, self.min_altitude, self.max_position_jump),
        }

        rospy.Subscriber(args.uav0_odom_topic, Odometry, lambda m: self._cb_odom("iris_0", m), queue_size=200)
        rospy.Subscriber(args.uav1_odom_topic, Odometry, lambda m: self._cb_odom("iris_1", m), queue_size=200)
        rospy.Subscriber(args.uav0_state_topic, State, lambda m: self._cb_state("iris_0", m), queue_size=20)
        rospy.Subscriber(args.uav1_state_topic, State, lambda m: self._cb_state("iris_1", m), queue_size=20)

    @staticmethod
    def _stamp_to_sec(msg_stamp) -> float:
        try:
            sec = msg_stamp.to_sec()
            if sec > 0:
                return sec
        except Exception:
            pass
        return rospy.Time.now().to_sec()

    def _cb_state(self, name: str, msg: State):
        self.vehicles[name].update_state(msg)

    def _cb_odom(self, name: str, msg: Odometry):
        stamp = self._stamp_to_sec(msg.header.stamp)
        p = msg.pose.pose.position
        self.vehicles[name].update_odom(stamp, p.x, p.y, p.z)
        self.latest_pos[name] = (p.x, p.y, p.z)

        p0 = self.latest_pos["iris_0"]
        p1 = self.latest_pos["iris_1"]
        if p0 is not None and p1 is not None:
            sep = math.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2 + (p0[2] - p1[2]) ** 2)
            self.min_separation_observed = min(self.min_separation_observed, sep)
            if sep < self.min_separation:
                self.separation_violation_count += 1

    def _vehicle_summary(self, vs: VehicleStats):
        return {
            "sample_count": vs.sample_count,
            "x_max": round(vs.x_max, 4) if vs.sample_count > 0 else None,
            "x_progress_ratio": round(vs.x_progress_ratio(), 4),
            "z_min": round(vs.z_min, 4) if vs.sample_count > 0 else None,
            "z_max": round(vs.z_max, 4) if vs.sample_count > 0 else None,
            "reached_x_target": vs.reached_x_target,
            "reached_time": vs.reached_time,
            "max_position_jump": round(vs.max_jump_observed, 4),
            "jump_violation_count": vs.jump_violation_count,
            "mavros_connected_seen": vs.mavros_connected_seen,
            "mavros_offboard_seen": vs.mavros_offboard_seen,
            "mavros_armed_seen": vs.mavros_armed_seen,
        }

    def _decision(self):
        v0 = self.vehicles["iris_0"]
        v1 = self.vehicles["iris_1"]

        reasons = []

        all_reached = v0.reached_x_target and v1.reached_x_target
        if not all_reached:
            reasons.append("至少一架无人机未达到X目标")

        no_collision_proxy = self.separation_violation_count <= self.separation_violation_limit
        if not no_collision_proxy:
            reasons.append("机间距低于安全阈值次数过多")

        no_crash_proxy = (v0.z_min >= self.min_altitude) and (v1.z_min >= self.min_altitude)
        if not no_crash_proxy:
            reasons.append("检测到高度低于最小安全高度")

        continuity_ok = (v0.jump_violation_count == 0) and (v1.jump_violation_count == 0)
        if not continuity_ok:
            reasons.append("检测到轨迹跳变，连续性不满足")

        mavros_ok = (
            v0.mavros_connected_seen and v0.mavros_offboard_seen and v0.mavros_armed_seen and
            v1.mavros_connected_seen and v1.mavros_offboard_seen and v1.mavros_armed_seen
        )
        if not mavros_ok:
            reasons.append("MAVROS状态未同时满足Connected+OFFBOARD+ARMED")

        passed = all_reached and no_collision_proxy and no_crash_proxy and continuity_ok and mavros_ok

        score = 100.0
        if not all_reached:
            score -= 45.0
        if not no_collision_proxy:
            score -= 25.0
        if not no_crash_proxy:
            score -= 20.0
        if not continuity_ok:
            score -= 10.0
        if not mavros_ok:
            score -= 10.0
        score = max(0.0, score)

        return {
            "pass": passed,
            "score": round(score, 1),
            "criteria": {
                "all_reached_x_target": all_reached,
                "no_collision_proxy": no_collision_proxy,
                "no_crash_proxy": no_crash_proxy,
                "trajectory_continuity": continuity_ok,
                "mavros_state_ok": mavros_ok,
            },
            "fail_reasons": reasons,
        }

    def run(self):
        deadline = time.time() + self.duration
        while (time.time() < deadline) and (not rospy.is_shutdown()):
            time.sleep(0.05)

        decision = self._decision()
        report = {
            "meta": {
                "duration_sec": self.duration,
                "generated_at_unix": time.time(),
                "x_target": self.x_target,
                "x_reach_tol": self.x_reach_tol,
                "min_separation": self.min_separation,
                "min_altitude": self.min_altitude,
                "max_position_jump": self.max_position_jump,
            },
            "vehicles": {
                "iris_0": self._vehicle_summary(self.vehicles["iris_0"]),
                "iris_1": self._vehicle_summary(self.vehicles["iris_1"]),
            },
            "safety": {
                "min_separation_observed": round(self.min_separation_observed, 4) if self.min_separation_observed < 1e9 else None,
                "separation_violation_count": self.separation_violation_count,
                "separation_violation_limit": self.separation_violation_limit,
            },
            "decision": decision,
        }

        with open(self.output, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

        if self.md_output:
            self._write_markdown(report)

        print(json.dumps(report, indent=2, ensure_ascii=False))
        print("\n[phase4_acceptance_quant] report written to {}".format(self.output))
        if self.md_output:
            print("[phase4_acceptance_quant] markdown written to {}".format(self.md_output))

    def _write_markdown(self, report: dict):
        decision = report["decision"]
        v0 = report["vehicles"]["iris_0"]
        v1 = report["vehicles"]["iris_1"]
        safety = report["safety"]

        lines = []
        lines.append("# Phase4 验收报告")
        lines.append("")
        lines.append("- 结果：**{}**".format("PASS ✅" if decision["pass"] else "FAIL ❌"))
        lines.append("- 分数：**{} / 100**".format(decision["score"]))
        lines.append("- 关键判据：")
        for k, v in decision["criteria"].items():
            lines.append("  - {}: {}".format(k, "true" if v else "false"))

        lines.append("")
        lines.append("## 无人机进度")
        lines.append("- iris_0: x_max={}, reached_x_target={}, z_min={}, jump_violations={}".format(
            v0["x_max"], v0["reached_x_target"], v0["z_min"], v0["jump_violation_count"]))
        lines.append("- iris_1: x_max={}, reached_x_target={}, z_min={}, jump_violations={}".format(
            v1["x_max"], v1["reached_x_target"], v1["z_min"], v1["jump_violation_count"]))

        lines.append("")
        lines.append("## 安全代理")
        lines.append("- min_separation_observed={}".format(safety["min_separation_observed"]))
        lines.append("- separation_violation_count={} (limit={})".format(
            safety["separation_violation_count"], safety["separation_violation_limit"]))

        if decision["fail_reasons"]:
            lines.append("")
            lines.append("## 失败原因")
            for r in decision["fail_reasons"]:
                lines.append("- {}".format(r))

        with open(self.md_output, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Phase4 acceptance quant tool")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--output", type=str, default="/tmp/phase4_acceptance_report.json")
    parser.add_argument("--md_output", type=str, default="")

    parser.add_argument("--uav0_odom_topic", type=str, default="/iris_0/odometry")
    parser.add_argument("--uav1_odom_topic", type=str, default="/iris_1/odometry")
    parser.add_argument("--uav0_state_topic", type=str, default="/iris_0/mavros/state")
    parser.add_argument("--uav1_state_topic", type=str, default="/iris_1/mavros/state")

    parser.add_argument("--x_target", type=float, default=18.0)
    parser.add_argument("--x_reach_tol", type=float, default=0.8)

    parser.add_argument("--min_separation", type=float, default=0.6)
    parser.add_argument("--separation_violation_limit", type=int, default=0)

    parser.add_argument("--min_altitude", type=float, default=0.2)
    parser.add_argument("--max_position_jump", type=float, default=2.5)

    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("phase4_acceptance_quant", anonymous=True)
    Phase4AcceptanceQuant(args).run()


if __name__ == "__main__":
    main()
