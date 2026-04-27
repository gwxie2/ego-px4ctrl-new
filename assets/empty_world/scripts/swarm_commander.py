#!/usr/bin/env python3
"""Swarm commander for multi-UAV goal publishing in ROS 1.

Behavior:
- Auto-creates namespaces /drone_0 ... /drone_N-1
- Publishes geometry_msgs/PoseStamped goals at a fixed rate
- Uses an X-shaped crossing pattern so paths intersect near x ~= 20
- Leaves an explicit LLM/JSON adapter hook for future integration

Default topic compatibility:
- /drone_0_planning/goal
- /drone_1_planning/goal
- ...
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from math import radians, sin
from typing import Dict, List, Tuple

import rospy
from geometry_msgs.msg import PoseStamped


Point = Tuple[float, float, float]


@dataclass
class PathProfile:
    name: str
    start: Point
    end: Point
    duration: float


class SwarmCommander:
    def __init__(self, num_drones: int, rate_hz: float, frame_id: str):
        self.num_drones = max(1, int(num_drones))
        self.rate_hz = float(rate_hz)
        self.frame_id = frame_id

        self.z_goal = 0.1
        self.cycle_seconds = float(rospy.get_param("~cycle_seconds", 20.0))
        self.enable_loop = bool(rospy.get_param("~enable_loop", True))
        self.max_lane_offset = float(rospy.get_param("~max_lane_offset", 12.0))
        self.angle_step_deg = float(rospy.get_param("~angle_step_deg", 30.0))
        self.max_angle_span_deg = float(rospy.get_param("~max_angle_span_deg", 160.0))

        self.namespaces = [f"/drone_{i}" for i in range(self.num_drones)]
        self.publishers: Dict[str, rospy.Publisher] = {
            ns: rospy.Publisher(f"{ns}_planning/goal", PoseStamped, queue_size=10)
            for ns in self.namespaces
        }

        self.profiles = self._build_profiles()
        self.start_time = rospy.Time.now().to_sec()

    def _build_angle_sequence(self, count: int) -> List[float]:
        if count <= 1:
            return [0.0]

        span_deg = min(self.max_angle_span_deg, self.angle_step_deg * (count - 1))
        step_deg = span_deg / (count - 1)
        start_deg = -span_deg / 2.0
        return [start_deg + i * step_deg for i in range(count)]

    def _build_profiles(self) -> Dict[str, PathProfile]:
        profiles: Dict[str, PathProfile] = {}
        angles_deg = self._build_angle_sequence(self.num_drones)

        for i, ns in enumerate(self.namespaces):
            angle_deg = angles_deg[i]
            angle_rad = radians(angle_deg)
            lane_offset = self.max_lane_offset * sin(angle_rad)
            start = (6.0, lane_offset, self.z_goal)
            end = (34.0, -lane_offset, self.z_goal)
            profiles[ns] = PathProfile(
                name=f"lane_{i}",
                start=start,
                end=end,
                duration=self.cycle_seconds,
            )

        return profiles

    def llm_logic_adapter(self):
        """Reserved hook for future LLM/VLM integration.

        Future JSON payload example:
        {
                    "drone_0": {"start": [6, -6, 0.1], "end": [34, 6, 0.1]},
                    "drone_1": {"start": [6, 0, 0.1], "end": [34, 0, 0.1]},
                    "drone_2": {"start": [6, 6, 0.1], "end": [34, -6, 0.1]}
        }

        Return value should be a dict mapping namespace -> PathProfile-like config.
        """
        return self.profiles

    def _smooth_progress(self, elapsed: float, duration: float) -> float:
        if duration <= 0.0:
            return 1.0
        if self.enable_loop:
            phase = (elapsed % duration) / duration
        else:
            phase = min(max(elapsed / duration, 0.0), 1.0)
        # Smoothstep: 3t^2 - 2t^3
        return phase * phase * (3.0 - 2.0 * phase)

    def _interpolate(self, start: Point, end: Point, t: float) -> Point:
        return (
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
            start[2] + (end[2] - start[2]) * t,
        )

    def _build_goal(self, point: Point, stamp: rospy.Time) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = point
        msg.pose.orientation.w = 1.0
        return msg

    def _goal_for_namespace(self, ns: str, now_sec: float) -> Point:
        profile = self.profiles[ns]
        progress = self._smooth_progress(now_sec - self.start_time, profile.duration)
        goal = self._interpolate(profile.start, profile.end, progress)
        return goal

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo(
            "swarm_commander started: drones=%d rate=%.1fHz frame_id=%s cycle=%.1fs"
            % (self.num_drones, self.rate_hz, self.frame_id, self.cycle_seconds)
        )
        rospy.loginfo("Namespaces: %s" % ", ".join(self.namespaces))

        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            now_sec = stamp.to_sec()
            cfg = self.llm_logic_adapter()

            for ns, pub in self.publishers.items():
                profile = cfg[ns]
                goal = self._interpolate(
                    profile.start,
                    profile.end,
                    self._smooth_progress(now_sec - self.start_time, profile.duration),
                )
                pub.publish(self._build_goal(goal, stamp))

            # ==================== LLM/VLM Interface (reserved) ====================
            # Current mode: hard-coded cross-path profiles.
            # Future mode:
            # 1) Subscribe to a JSON topic or service carrying swarm mission tasks.
            # 2) Replace `llm_logic_adapter()` with a parser that turns JSON into
            #    per-drone path profiles.
            # 3) Optionally add obstacle-aware timing or task re-planning here.
            # ======================================================================

            rate.sleep()


def parse_args():
    parser = argparse.ArgumentParser(description="Swarm goal commander for ROS 1")
    parser.add_argument("--num-drones", type=int, default=3, help="Number of drones to command")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Publish rate in Hz")
    parser.add_argument("--frame-id", type=str, default="world", help="PoseStamped frame_id")
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("swarm_commander")
    commander = SwarmCommander(
        num_drones=args.num_drones,
        rate_hz=args.rate_hz,
        frame_id=args.frame_id,
    )
    commander.spin()


if __name__ == "__main__":
    main()
