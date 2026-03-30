#!/usr/bin/env python3
"""Publish dynamic goals for every drone described in the Phase 1 position rules."""

import math
import re
from pathlib import Path

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped


def get_clean_uav_core_path():
    return Path(rospkg.RosPack().get_path("clean_uav_core"))


def resolve_config_path(config_path):
    workspace_root = get_clean_uav_core_path().parent.parent
    if not config_path:
        return workspace_root / "docs" / "uav_position_goal.md"

    candidate = Path(config_path)
    if candidate.is_absolute():
        return candidate

    workspace_candidate = workspace_root / candidate
    if workspace_candidate.exists():
        return workspace_candidate

    package_candidate = get_clean_uav_core_path() / candidate
    if package_candidate.exists():
        return package_candidate

    return workspace_candidate


def parse_uav_ids(config_file):
    drone_ids = []
    pattern = re.compile(r"`?drone_(\d+)`?", re.IGNORECASE)

    try:
        with Path(config_file).open("r", encoding="utf-8") as handle:
            for line in handle:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue

                match = pattern.search(stripped)
                if not match:
                    continue

                drone_ids.append(int(match.group(1)))

        drone_ids = sorted(set(drone_ids))
        if not drone_ids:
            raise ValueError(f"no drone configuration found in {config_file}")

        expected_ids = list(range(len(drone_ids)))
        if drone_ids != expected_ids:
            raise ValueError(f"drone ids must be contiguous starting from 0, got {drone_ids}")

        return drone_ids

    except FileNotFoundError:
        rospy.logerr(f"[swarm_dynamic_commander] Config file not found: {config_file}")
    except Exception as e:
        rospy.logerr(f"[swarm_dynamic_commander] Error parsing config: {e}")
    return []


def build_phase1_uav_configs(config_file, radius=15.0, start_z=0.10, goal_z=1.50):
    drone_ids = parse_uav_ids(config_file)
    if not drone_ids:
        return {}

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

    rospy.loginfo(f"[swarm_dynamic_commander] Built {len(uav_configs)} UAV configs from Phase 1 rules in {config_file}")
    return uav_configs


def yaw_to_quaternion(yaw):
    """将偏航角转换为四元数"""
    import tf.transformations as transformations
    quaternion = transformations.quaternion_from_euler(0, 0, yaw)
    return quaternion


class SwarmDynamicCommander:
    """Publish moving goals for every configured drone."""

    def __init__(self):
        config_file = resolve_config_path(rospy.get_param("~config_file", ""))

        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))
        self.start_delay = max(0.0, float(rospy.get_param("~start_delay", 20.0)))

        self.enable_oscillation = rospy.get_param("~enable_oscillation", True)
        self.period_sec = max(0.1, float(rospy.get_param("~period_sec", 30.0)))
        self.y_amplitude = abs(float(rospy.get_param("~y_amplitude", 0.5)))
        self.phase_offset_step = float(rospy.get_param("~phase_offset_step", 0.5))

        self.uav_configs = build_phase1_uav_configs(config_file)
        if not self.uav_configs:
            rospy.logerr("[swarm_dynamic_commander] No UAV configs loaded, exiting...")
            rospy.signal_shutdown("No UAV configs loaded")
            return

        self.publishers = {}
        self.goal_configs = {}
        self.connection_states = {}

        for drone_id, config in self.uav_configs.items():
            topic_name = f"/drone_{drone_id}/goal"
            self.publishers[drone_id] = rospy.Publisher(
                topic_name,
                PoseStamped,
                queue_size=10
            )
            self.connection_states[drone_id] = None

            self.goal_configs[drone_id] = config["goal"].copy()

        rospy.loginfo(f"[swarm_dynamic_commander] Created {len(self.publishers)} goal publishers:")
        for drone_id in sorted(self.publishers.keys()):
            cfg = self.goal_configs[drone_id]
            rospy.loginfo(f"  /drone_{drone_id}/goal -> ({cfg['x']:.2f}, {cfg['y']:.2f}, {cfg['z']:.2f})")

        self.start_time = None

    def _make_goal_msg(self, drone_id, y_offset=0.0):
        cfg = self.goal_configs[drone_id]

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = cfg["x"]
        msg.pose.position.y = cfg["y"] + y_offset
        msg.pose.position.z = cfg["z"]

        quaternion = yaw_to_quaternion(cfg["yaw"])
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        return msg

    def run(self):
        if self.start_delay > 0.0:
            rospy.loginfo(f"[swarm_dynamic_commander] Waiting {self.start_delay}s before publishing goals")
            rospy.sleep(self.start_delay)

        self.start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(self.publish_rate)
        omega = 2.0 * math.pi / self.period_sec if self.enable_oscillation else 0.0

        last_log_time = rospy.Time(0)

        rospy.loginfo("[swarm_dynamic_commander] Starting goal publishing loop")

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            t = now.to_sec() - self.start_time if self.enable_oscillation else 0.0

            for drone_id, pub in self.publishers.items():
                y_offset = 0.0
                if self.enable_oscillation:
                    phase = drone_id * self.phase_offset_step
                    y_offset = self.y_amplitude * math.sin(omega * t + phase)

                msg = self._make_goal_msg(drone_id, y_offset)
                pub.publish(msg)

                has_subscribers = pub.get_num_connections() > 0
                if self.connection_states[drone_id] is None or self.connection_states[drone_id] != has_subscribers:
                    topic_name = f"/drone_{drone_id}/goal"
                    if has_subscribers:
                        rospy.loginfo(f"[swarm_dynamic_commander] Goal topic connected: {topic_name}")
                    else:
                        rospy.logwarn(f"[swarm_dynamic_commander] Goal topic has no subscribers yet: {topic_name}")
                    self.connection_states[drone_id] = has_subscribers

            if (now - last_log_time).to_sec() > 2.0:
                rospy.loginfo(f"[swarm_dynamic_commander] Publishing goals for {len(self.publishers)} UAVs")
                if self.enable_oscillation:
                    for drone_id in sorted(self.publishers.keys())[:3]:
                        y_offset = self.y_amplitude * math.sin(omega * t + drone_id * self.phase_offset_step)
                        cfg = self.goal_configs[drone_id]
                        rospy.loginfo(f"  drone_{drone_id}: ({cfg['x']:.2f}, {cfg['y']+y_offset:.2f}, {cfg['z']:.2f})")
                disconnected = [drone_id for drone_id, connected in self.connection_states.items() if connected is False]
                if disconnected:
                    rospy.logwarn(f"[swarm_dynamic_commander] Goal subscribers missing for drones: {disconnected}")
                last_log_time = now

            rate.sleep()


def main():
    rospy.init_node("swarm_dynamic_commander", anonymous=False)

    try:
        SwarmDynamicCommander().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[swarm_dynamic_commander] Shutting down")
    except Exception as e:
        rospy.logerr(f"[swarm_dynamic_commander] Error: {e}")
        raise


if __name__ == "__main__":
    main()
