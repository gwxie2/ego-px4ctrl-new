#!/usr/bin/env python3

import argparse
import json
import math
import statistics
import time
from collections import defaultdict

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates


class TopicStats:
    def __init__(self):
        self.arrivals = []
        self.header_stamps = []

    def add(self, now_sec: float, stamp_sec: float):
        self.arrivals.append(now_sec)
        self.header_stamps.append(stamp_sec)

    def summary(self):
        if len(self.arrivals) < 2:
            return {
                "count": len(self.arrivals),
                "hz": 0.0,
                "max_gap_sec": None,
                "mean_gap_sec": None,
                "backward_stamp_count": 0,
            }
        gaps = [self.arrivals[i] - self.arrivals[i - 1] for i in range(1, len(self.arrivals))]
        hz = (len(self.arrivals) - 1) / max(self.arrivals[-1] - self.arrivals[0], 1e-6)
        backward_stamp_count = 0
        for i in range(1, len(self.header_stamps)):
            if self.header_stamps[i] + 1e-9 < self.header_stamps[i - 1]:
                backward_stamp_count += 1
        return {
            "count": len(self.arrivals),
            "hz": round(hz, 3),
            "max_gap_sec": round(max(gaps), 4),
            "mean_gap_sec": round(sum(gaps) / len(gaps), 4),
            "backward_stamp_count": backward_stamp_count,
        }


class Phase3HealthQuant:
    def __init__(self, duration_sec: float):
        self.duration_sec = duration_sec
        self.start_wall = time.time()

        self.topic_stats = defaultdict(TopicStats)

        self.last_truth = {
            "iris_0": None,
            "iris_1": None,
        }
        self.odom_truth_err = {
            "iris_0": [],
            "iris_1": [],
        }
        self.odom_pose_offset = {
            "iris_0": [],
            "iris_1": [],
        }

        self.last_pose = {
            "iris_0": None,
            "iris_1": None,
        }

        self.mavros_state = {
            "iris_0": {"armed": None, "mode": None, "connected": None},
            "iris_1": {"armed": None, "mode": None, "connected": None},
        }

        self.model_name_to_idx = {}

        self._subs = []
        self._setup_subs()

    def _now(self):
        return rospy.Time.now().to_sec()

    def _record_topic(self, topic: str, stamp_sec: float):
        self.topic_stats[topic].add(self._now(), stamp_sec)

    def _setup_subs(self):
        self._subs.extend([
            rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb_model_states, queue_size=20),
            rospy.Subscriber("/iris_0/odometry", Odometry, lambda m: self._cb_odom("iris_0", m), queue_size=50),
            rospy.Subscriber("/iris_1/odometry", Odometry, lambda m: self._cb_odom("iris_1", m), queue_size=50),
            rospy.Subscriber("/iris_0/vins_estimator/imu_propagate", Odometry, lambda m: self._cb_topic("/iris_0/vins_estimator/imu_propagate", m), queue_size=50),
            rospy.Subscriber("/iris_1/vins_estimator/imu_propagate", Odometry, lambda m: self._cb_topic("/iris_1/vins_estimator/imu_propagate", m), queue_size=50),
            rospy.Subscriber("/iris_0/vins_estimator/odometry", Odometry, lambda m: self._cb_topic("/iris_0/vins_estimator/odometry", m), queue_size=50),
            rospy.Subscriber("/iris_1/vins_estimator/odometry", Odometry, lambda m: self._cb_topic("/iris_1/vins_estimator/odometry", m), queue_size=50),
            rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped, lambda m: self._cb_pose("iris_0", m), queue_size=50),
            rospy.Subscriber("/iris_1/mavros/local_position/pose", PoseStamped, lambda m: self._cb_pose("iris_1", m), queue_size=50),
            rospy.Subscriber("/iris_0/realsense/depth_camera/depth/image_raw", Image, lambda m: self._cb_topic("/iris_0/realsense/depth", m), queue_size=50),
            rospy.Subscriber("/iris_1/realsense/depth_camera/depth/image_raw", Image, lambda m: self._cb_topic("/iris_1/realsense/depth", m), queue_size=50),
            rospy.Subscriber("/iris_0/mavros/state", State, lambda m: self._cb_state("iris_0", m), queue_size=20),
            rospy.Subscriber("/iris_1/mavros/state", State, lambda m: self._cb_state("iris_1", m), queue_size=20),
        ])

    def _cb_topic(self, key: str, msg):
        stamp_sec = msg.header.stamp.to_sec() if hasattr(msg, "header") else self._now()
        self._record_topic(key, stamp_sec)

    def _cb_state(self, name: str, msg: State):
        self.mavros_state[name] = {
            "armed": bool(msg.armed),
            "mode": msg.mode,
            "connected": bool(msg.connected),
        }

    def _cb_pose(self, name: str, msg: PoseStamped):
        self._record_topic(f"/{name}/mavros/local_position/pose", msg.header.stamp.to_sec())
        self.last_pose[name] = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _cb_model_states(self, msg: ModelStates):
        self._record_topic("/gazebo/model_states", self._now())
        if not self.model_name_to_idx:
            for i, n in enumerate(msg.name):
                self.model_name_to_idx[n] = i

        for name in ["iris_0", "iris_1"]:
            if name in self.model_name_to_idx:
                idx = self.model_name_to_idx[name]
                p = msg.pose[idx].position
                self.last_truth[name] = (p.x, p.y, p.z)

    def _cb_odom(self, name: str, msg: Odometry):
        topic = f"/{name}/odometry"
        self._record_topic(topic, msg.header.stamp.to_sec())
        odom_p = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

        truth = self.last_truth.get(name)
        if truth is not None:
            err = math.sqrt((odom_p[0] - truth[0]) ** 2 + (odom_p[1] - truth[1]) ** 2 + (odom_p[2] - truth[2]) ** 2)
            self.odom_truth_err[name].append(err)

        pose = self.last_pose.get(name)
        if pose is not None:
            err2 = math.sqrt((odom_p[0] - pose[0]) ** 2 + (odom_p[1] - pose[1]) ** 2 + (odom_p[2] - pose[2]) ** 2)
            self.odom_pose_offset[name].append(err2)

    @staticmethod
    def _vec_summary(values):
        values = list(values)
        if not values:
            return {"count": 0, "mean": None, "p95": None, "max": None}
        sorted_vals = sorted(values)
        p95_idx = min(len(sorted_vals) - 1, int(0.95 * (len(sorted_vals) - 1)))
        return {
            "count": len(values),
            "mean": round(statistics.mean(values), 4),
            "p95": round(sorted_vals[p95_idx], 4),
            "max": round(max(values), 4),
        }

    def _health_score(self):
        score = 100.0

        req_hz = {
            "/iris_0/odometry": 20.0,
            "/iris_1/odometry": 20.0,
            "/iris_0/realsense/depth": 10.0,
            "/iris_1/realsense/depth": 10.0,
        }

        for topic, min_hz in req_hz.items():
            hz = self.topic_stats[topic].summary()["hz"]
            if hz < min_hz:
                score -= min(20.0, (min_hz - hz) * 1.5)

        for name in ["iris_0", "iris_1"]:
            mean_err = self._vec_summary(self.odom_truth_err[name])["mean"]
            if mean_err is not None:
                if mean_err > 2.0:
                    score -= min(25.0, (mean_err - 2.0) * 5.0)
                elif mean_err > 0.8:
                    score -= min(10.0, (mean_err - 0.8) * 3.0)

            off = self._vec_summary(self.odom_pose_offset[name])["mean"]
            if off is not None and off > 1.0:
                score -= min(15.0, (off - 1.0) * 3.0)

        for name in ["iris_0", "iris_1"]:
            st = self.mavros_state[name]
            if not st["connected"]:
                score -= 10.0

        return round(max(0.0, min(100.0, score)), 1)

    def run(self):
        deadline = time.time() + self.duration_sec
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)

        report = {
            "meta": {
                "duration_sec": self.duration_sec,
                "generated_at_unix": time.time(),
            },
            "topic_health": {k: self.topic_stats[k].summary() for k in sorted(self.topic_stats.keys())},
            "odom_vs_truth_error_m": {name: self._vec_summary(self.odom_truth_err[name]) for name in ["iris_0", "iris_1"]},
            "odom_vs_pose_offset_m": {name: self._vec_summary(self.odom_pose_offset[name]) for name in ["iris_0", "iris_1"]},
            "mavros_state": self.mavros_state,
            "health_score": self._health_score(),
        }
        return report


def main():
    parser = argparse.ArgumentParser(description="Phase-3 VINS health quantification tool")
    parser.add_argument("--duration", type=float, default=60.0, help="Sampling duration in seconds")
    parser.add_argument("--output", type=str, default="/tmp/phase3_health_report.json", help="Output json path")
    args = parser.parse_args()

    rospy.init_node("phase3_health_quant", anonymous=True)
    tool = Phase3HealthQuant(duration_sec=args.duration)
    report = tool.run()

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\n[phase3_health_quant] report written to {args.output}")


if __name__ == "__main__":
    main()
