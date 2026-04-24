#!/usr/bin/env python3
"""为 V2 规划沙盒持续发布 GoalSet 目标，并在启动初期等待编队与订阅关系稳定。"""

import math
import re
from pathlib import Path

import rospkg
import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import GoalSet


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
        rospy.logerr(f"[swarm_dynamic_commander_v2] Config file not found: {config_file}")
    except Exception as error:
        rospy.logerr(f"[swarm_dynamic_commander_v2] Error parsing config: {error}")
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

        uav_configs[drone_id] = {
            "start": {
                "x": start_x,
                "y": start_y,
                "z": start_z,
            },
            "goal": {
                "x": -start_x,
                "y": -start_y,
                "z": goal_z,
            },
        }

    rospy.loginfo(
        f"[swarm_dynamic_commander_v2] Built {len(uav_configs)} UAV configs from Phase 1 rules in {config_file}"
    )
    return uav_configs


def _coerce_float_list(value):
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    text = str(value).strip()
    if not text:
        return []
    return [float(item) for item in text.replace(";", ",").split(",") if item.strip()]


def _coerce_int_list(value):
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [int(item) for item in value]
    text = str(value).strip()
    if not text:
        return []
    return [int(item) for item in text.replace(";", ",").split(",") if item.strip()]


def build_goal_configs(config_file, default_drone_ids, goal_xs, goal_ys, goal_zs):
    if goal_xs or goal_ys or goal_zs:
        if not (goal_xs and goal_ys and goal_zs):
            raise ValueError("default_goal_xs/default_goal_ys/default_goal_zs must be provided together")
        if not (len(goal_xs) == len(goal_ys) == len(goal_zs)):
            raise ValueError("default_goal_xs/default_goal_ys/default_goal_zs must have the same length")

        drone_ids = list(default_drone_ids) if default_drone_ids else list(range(len(goal_xs)))
        if len(drone_ids) != len(goal_xs):
            raise ValueError("default_drone_ids length must match the goal arrays")

        return {
            drone_id: {
                "x": goal_xs[index],
                "y": goal_ys[index],
                "z": goal_zs[index],
            }
            for index, drone_id in enumerate(drone_ids)
        }

    uav_configs = build_phase1_uav_configs(config_file)
    return {drone_id: config["goal"].copy() for drone_id, config in uav_configs.items()}


class SwarmDynamicCommanderV2:
    """面向多机目标发布的轻量调度器。

    设计原则：
    1. 先等无人机起飞到稳定高度，再发目标。
    2. 先等至少一个 planner 订阅 GoalSet，再进入正式发布循环。
    3. 采用持续发布而非一次性触发，降低启动时序抖动带来的丢目标风险。
    """

    def __init__(self):
        config_file = resolve_config_path(rospy.get_param("~config_file", ""))

        self.goal_topic = rospy.get_param("~goal_topic", "/goal_with_id")
        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))
        self.start_delay = max(0.0, float(rospy.get_param("~start_delay", 20.0)))
        self.wait_for_all_uavs = bool(rospy.get_param("~wait_for_all_uavs", False))
        self.odom_topic_template = rospy.get_param("~odom_topic_template", "/drone_%d/odom")
        self.ready_z_threshold = float(rospy.get_param("~ready_z_threshold", 0.8))
        self.ready_stable_duration = float(rospy.get_param("~ready_stable_duration", 1.5))
        self.ready_timeout = float(rospy.get_param("~ready_timeout", 60.0))
        self.ready_post_delay = max(0.0, float(rospy.get_param("~ready_post_delay", 0.0)))
        self.goal_subscriber_timeout = max(0.0, float(rospy.get_param("~goal_subscriber_timeout", 5.0)))

        self.enable_oscillation = rospy.get_param("~enable_oscillation", True)
        self.period_sec = max(0.1, float(rospy.get_param("~period_sec", 30.0)))
        self.y_amplitude = abs(float(rospy.get_param("~y_amplitude", 0.5)))
        self.phase_offset_step = float(rospy.get_param("~phase_offset_step", 0.5))

        self.default_drone_ids = _coerce_int_list(rospy.get_param("~default_drone_ids", []))
        self.default_goal_xs = _coerce_float_list(rospy.get_param("~default_goal_xs", []))
        self.default_goal_ys = _coerce_float_list(rospy.get_param("~default_goal_ys", []))
        self.default_goal_zs = _coerce_float_list(rospy.get_param("~default_goal_zs", []))

        self.goal_configs = build_goal_configs(
            config_file,
            self.default_drone_ids,
            self.default_goal_xs,
            self.default_goal_ys,
            self.default_goal_zs,
        )
        if not self.goal_configs:
            rospy.logerr("[swarm_dynamic_commander_v2] No UAV configs loaded, exiting")
            rospy.signal_shutdown("No UAV configs loaded")
            return

        self.publisher = rospy.Publisher(self.goal_topic, GoalSet, queue_size=20)
        self.start_time = None
        self.last_connection_state = None
        self.ready_since = {}

        if self.wait_for_all_uavs:
            for drone_id in sorted(self.goal_configs.keys()):
                odom_topic = self.odom_topic_template % drone_id
                self.ready_since[drone_id] = None
                rospy.Subscriber(odom_topic, Odometry, self._make_odom_callback(drone_id), queue_size=1)

        rospy.loginfo(
            f"[swarm_dynamic_commander_v2] Publishing GoalSet for {len(self.goal_configs)} UAVs to {self.goal_topic}"
        )

    def _make_odom_callback(self, drone_id):
        def _callback(msg):
            z_value = float(msg.pose.pose.position.z)
            now = rospy.Time.now()
            if z_value >= self.ready_z_threshold:
                if self.ready_since[drone_id] is None:
                    self.ready_since[drone_id] = now
            else:
                self.ready_since[drone_id] = None

        return _callback

    def _wait_for_all_ready(self):
        if not self.wait_for_all_uavs:
            return True

        start_time = rospy.Time.now()
        rate = rospy.Rate(5.0)
        rospy.loginfo(
            "[swarm_dynamic_commander_v2] Waiting for all UAVs to reach Z >= %.2f for %.1fs",
            self.ready_z_threshold,
            self.ready_stable_duration,
        )

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            all_ready = True
            not_ready = []
            for drone_id in sorted(self.ready_since.keys()):
                ready_since = self.ready_since[drone_id]
                stable_time = 0.0 if ready_since is None else (now - ready_since).to_sec()
                if stable_time < self.ready_stable_duration:
                    all_ready = False
                    not_ready.append(f"drone_{drone_id}:{stable_time:.1f}/{self.ready_stable_duration:.1f}s")

            if all_ready:
                if self.ready_post_delay > 0.0:
                    rospy.loginfo(
                        "[swarm_dynamic_commander_v2] Swarm ready, extra sleep %.1fs before goal publish",
                        self.ready_post_delay,
                    )
                    rospy.sleep(self.ready_post_delay)
                return True

            rospy.loginfo_throttle(
                1.0,
                "[swarm_dynamic_commander_v2] Waiting swarm ready: %s",
                ", ".join(not_ready),
            )

            elapsed = (now - start_time).to_sec()
            if self.ready_timeout > 0.0 and elapsed >= self.ready_timeout:
                rospy.logwarn(
                    "[swarm_dynamic_commander_v2] Timed out after %.1fs waiting for swarm readiness",
                    self.ready_timeout,
                )
                return False

            rate.sleep()

        return False

    def _wait_for_goal_subscriber(self):
        """在首个 GoalSet 发布前，给 planner 一小段连接窗口，减少启动竞态。"""
        if self.goal_subscriber_timeout <= 0.0:
            return

        start_time = rospy.Time.now()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.publisher.get_num_connections() > 0:
                rospy.loginfo(
                    "[swarm_dynamic_commander_v2] GoalSet subscriber connected before first publish"
                )
                return

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= self.goal_subscriber_timeout:
                rospy.logwarn(
                    "[swarm_dynamic_commander_v2] No GoalSet subscriber after %.1fs, will publish anyway",
                    self.goal_subscriber_timeout,
                )
                return

            rospy.loginfo_throttle(
                1.0,
                "[swarm_dynamic_commander_v2] Waiting GoalSet subscriber: %.1f/%.1fs",
                elapsed,
                self.goal_subscriber_timeout,
            )
            rate.sleep()

    def _make_goal_msg(self, drone_id, y_offset=0.0):
        cfg = self.goal_configs[drone_id]

        msg = GoalSet()
        msg.drone_id = drone_id
        msg.goal[0] = cfg["x"]
        msg.goal[1] = cfg["y"] + y_offset
        msg.goal[2] = cfg["z"]
        return msg

    def run(self):
        if not self._wait_for_all_ready():
            return

        if self.start_delay > 0.0:
            rospy.loginfo(f"[swarm_dynamic_commander_v2] Waiting {self.start_delay}s before publishing goals")
            rospy.sleep(self.start_delay)

        # 在第一批目标发出前，先等待 planner 真正连上 GoalSet 话题，避免“刚发完就没人接”的竞态。
        self._wait_for_goal_subscriber()

        self.start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(self.publish_rate)
        omega = 2.0 * math.pi / self.period_sec if self.enable_oscillation else 0.0
        last_log_time = rospy.Time(0)

        rospy.loginfo("[swarm_dynamic_commander_v2] Starting GoalSet publishing loop")

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = now.to_sec() - self.start_time if self.enable_oscillation else 0.0

            for drone_id in sorted(self.goal_configs.keys()):
                # 持续发布而不是一次性触发：planner 即使晚到，也会在后续循环里重新收到最新目标。
                y_offset = 0.0
                if self.enable_oscillation:
                    phase = drone_id * self.phase_offset_step
                    y_offset = self.y_amplitude * math.sin(omega * elapsed + phase)

                self.publisher.publish(self._make_goal_msg(drone_id, y_offset))

            has_subscribers = self.publisher.get_num_connections() > 0
            if self.last_connection_state is None or self.last_connection_state != has_subscribers:
                if has_subscribers:
                    rospy.loginfo(f"[swarm_dynamic_commander_v2] GoalSet topic connected: {self.goal_topic}")
                else:
                    rospy.logwarn(f"[swarm_dynamic_commander_v2] GoalSet topic has no subscribers yet: {self.goal_topic}")
                self.last_connection_state = has_subscribers

            if (now - last_log_time).to_sec() > 2.0:
                rospy.loginfo(f"[swarm_dynamic_commander_v2] Publishing GoalSet for {len(self.goal_configs)} UAVs")
                last_log_time = now

            rate.sleep()


def main():
    rospy.init_node("swarm_dynamic_commander_v2", anonymous=False)

    try:
        SwarmDynamicCommanderV2().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[swarm_dynamic_commander_v2] Shutting down")
    except Exception as error:
        rospy.logerr(f"[swarm_dynamic_commander_v2] Error: {error}")
        raise


if __name__ == "__main__":
    main()