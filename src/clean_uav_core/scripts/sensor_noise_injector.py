#!/usr/bin/env python3

from __future__ import annotations

import copy
import heapq
import random
import threading
from dataclasses import dataclass

import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from scipy.ndimage import median_filter as scipy_median_filter
from sensor_msgs.msg import Image


@dataclass(frozen=True)
class OdomPreset:
    latency_s: float
    jitter_s: float
    rate_hz: float
    drift_step_m: float


@dataclass(frozen=True)
class DepthPreset:
    noise_sigma_mm: float
    noise_slope_mm: float
    hole_base: float
    hole_edge_gain: float
    hole_depth_gain: float
    quantization_mm: float
    edge_sensitivity: float


PRESETS = {
    "clean": {
        "odom": OdomPreset(latency_s=0.0, jitter_s=0.0, rate_hz=0.0, drift_step_m=0.0),
        "depth": DepthPreset(
            noise_sigma_mm=0.0,
            noise_slope_mm=0.0,
            hole_base=0.0,
            hole_edge_gain=0.0,
            hole_depth_gain=0.0,
            quantization_mm=1.0,
            edge_sensitivity=1.0,
        ),
    },
    "nominal": {
        "odom": OdomPreset(latency_s=0.005, jitter_s=0.002, rate_hz=100.0, drift_step_m=0.0001),
        "depth": DepthPreset(
            noise_sigma_mm=3.0,
            noise_slope_mm=0.5,
            hole_base=0.002,
            hole_edge_gain=0.005,
            hole_depth_gain=0.002,
            quantization_mm=5.0,
            edge_sensitivity=1.2,
        ),
    },
    "mild_no_latency": {
        "odom": OdomPreset(latency_s=0.0, jitter_s=0.0, rate_hz=0.0, drift_step_m=0.0),
        "depth": DepthPreset(
            noise_sigma_mm=8.0,
            noise_slope_mm=1.8,
            hole_base=0.01,
            hole_edge_gain=0.02,
            hole_depth_gain=0.008,
            quantization_mm=20.0,
            edge_sensitivity=2.0,
        ),
    },
    "mild": {
        "odom": OdomPreset(latency_s=0.015, jitter_s=0.005, rate_hz=50.0, drift_step_m=0.0005),
        "depth": DepthPreset(
            noise_sigma_mm=8.0,
            noise_slope_mm=1.8,
            hole_base=0.01,
            hole_edge_gain=0.02,
            hole_depth_gain=0.008,
            quantization_mm=20.0,
            edge_sensitivity=2.0,
        ),
    },
    "extreme": {
        "odom": OdomPreset(latency_s=0.08, jitter_s=0.03, rate_hz=15.0, drift_step_m=0.0035),
        "depth": DepthPreset(
            noise_sigma_mm=20.0,
            noise_slope_mm=4.5,
            hole_base=0.04,
            hole_edge_gain=0.07,
            hole_depth_gain=0.025,
            quantization_mm=50.0,
            edge_sensitivity=3.5,
        ),
    },
}


class SensorNoiseInjector:
    def __init__(self):
        self.enabled = rospy.get_param("~sensor_degradation_enabled", False)
        self.mode = str(rospy.get_param("~sensor_degradation_mode", "clean")).strip().lower()
        if self.mode not in PRESETS:
            rospy.logwarn(
                "[clean_uav_core] sensor_noise_injector got unsupported mode '%s', falling back to clean",
                self.mode,
            )
            self.mode = "clean"

        self.odom_input_topic = rospy.get_param("~odom_input_topic", "truth_odom")
        self.odom_output_topic = rospy.get_param("~odom_output_topic", "sensor_odom")
        self.depth_input_topic = rospy.get_param("~depth_input_topic", "depth_preprocessed")
        self.depth_output_topic = rospy.get_param("~depth_output_topic", "depth_output")
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.1))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 25.0))
        self.depth_median_filter_size = int(rospy.get_param("~depth_median_filter_size", 0))
        self.queue_capacity = int(rospy.get_param("~odom_queue_capacity", 200))
        self.odom_timer_hz = max(float(rospy.get_param("~odom_timer_hz", 100.0)), 1.0)

        self.odom_preset = PRESETS[self.mode]["odom"]
        self.depth_preset = PRESETS[self.mode]["depth"]
        self.depth_quantization_mm = max(1.0, float(self.depth_preset.quantization_mm))
        self.odom_min_interval_s = 0.0
        if self.odom_preset.rate_hz > 0.0:
            self.odom_min_interval_s = 1.0 / self.odom_preset.rate_hz

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.odom_queue = []
        self.odom_seq = 0
        self.last_odom_publish_time = rospy.Time(0)
        self.odom_drift = np.zeros(3, dtype=np.float64)
        self.rng = np.random.default_rng()

        self.odom_pub = rospy.Publisher(self.odom_output_topic, Odometry, queue_size=10, latch=True)
        self.depth_pub = rospy.Publisher(self.depth_output_topic, Image, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_input_topic, Odometry, self._odom_callback, queue_size=10)
        self.depth_sub = rospy.Subscriber(self.depth_input_topic, Image, self._depth_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.odom_timer_hz), self._flush_odom_queue)

        rospy.loginfo(
            "[clean_uav_core] sensor_noise_injector enabled=%s mode=%s median_filter=%d odom: %s -> %s depth: %s -> %s",
            self.enabled,
            self.mode,
            self.depth_median_filter_size,
            self.odom_input_topic,
            self.odom_output_topic,
            self.depth_input_topic,
            self.depth_output_topic,
        )

    @staticmethod
    def _safe_publish(publisher, message) -> bool:
        if rospy.is_shutdown():
            return False
        try:
            publisher.publish(message)
            return True
        except (rospy.ROSException, rospy.ROSInterruptException):
            return False
        except Exception:
            return False

    def _apply_odom_preset(self, msg: Odometry, publish_time: rospy.Time) -> Odometry:
        out_msg = copy.deepcopy(msg)
        out_msg.header.stamp = publish_time

        if not self.enabled or self.mode == "clean":
            return out_msg

        if self.odom_preset.drift_step_m > 0.0:
            drift_step = self.rng.normal(0.0, self.odom_preset.drift_step_m, size=3)
            self.odom_drift += drift_step

        out_msg.pose.pose.position.x += float(self.odom_drift[0])
        out_msg.pose.pose.position.y += float(self.odom_drift[1])
        out_msg.pose.pose.position.z += float(self.odom_drift[2])
        return out_msg

    def _odom_callback(self, msg: Odometry):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        if not self.enabled or self.mode == "clean":
            direct_msg = copy.deepcopy(msg)
            direct_msg.header.stamp = now
            self._safe_publish(self.odom_pub, direct_msg)
            return

        publish_delay = 0.0
        publish_delay = self.odom_preset.latency_s
        if self.odom_preset.jitter_s > 0.0:
            publish_delay += random.uniform(-self.odom_preset.jitter_s, self.odom_preset.jitter_s)
        publish_delay = max(0.0, publish_delay)
        publish_time = now + rospy.Duration.from_sec(publish_delay)

        degraded_msg = self._apply_odom_preset(msg, publish_time)

        with self.lock:
            heapq.heappush(self.odom_queue, (publish_time.to_sec(), self.odom_seq, degraded_msg))
            self.odom_seq += 1
            if len(self.odom_queue) > self.queue_capacity:
                heapq.heappop(self.odom_queue)

    def _flush_odom_queue(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        with self.lock:
            if not self.odom_queue:
                return

            if self.odom_min_interval_s > 0.0 and self.last_odom_publish_time != rospy.Time(0):
                if (now - self.last_odom_publish_time).to_sec() < self.odom_min_interval_s:
                    return

            if self.odom_queue[0][0] > now.to_sec():
                return

            _, _, msg = heapq.heappop(self.odom_queue)
            if not self._safe_publish(self.odom_pub, msg):
                return
            self.last_odom_publish_time = now

    def _convert_depth_to_meters(self, msg: Image):
        if msg.encoding == "16UC1":
            depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            return np.asarray(depth_mm, dtype=np.float32) / 1000.0

        if msg.encoding == "32FC1":
            depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            return np.asarray(depth_m, dtype=np.float32)

        return None

    def _depth_callback(self, msg: Image):
        if rospy.is_shutdown():
            return
        depth_m = self._convert_depth_to_meters(msg)
        if depth_m is None:
            rospy.logwarn_throttle(10.0, "[clean_uav_core] sensor_noise_injector ignored unsupported depth encoding: %s", msg.encoding)
            return

        depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
        valid = (depth_m >= self.min_depth_m) & (depth_m <= self.max_depth_m)

        if self.enabled and self.mode != "clean":
            noisy_depth_m = depth_m.copy()
            valid_depth = noisy_depth_m[valid]
            if valid_depth.size:
                sigma_mm = self.depth_preset.noise_sigma_mm + self.depth_preset.noise_slope_mm * valid_depth
                sigma_m = sigma_mm / 1000.0
                noisy_depth_m[valid] = valid_depth + self.rng.normal(0.0, 1.0, size=valid_depth.shape) * sigma_m

            gradient_y, gradient_x = np.gradient(noisy_depth_m)
            gradient = np.sqrt(gradient_x * gradient_x + gradient_y * gradient_y)
            depth_norm = np.clip(noisy_depth_m / max(self.max_depth_m, 1e-6), 0.0, 1.0)
            edge_norm = np.clip(gradient / self.depth_preset.edge_sensitivity, 0.0, 1.0)

            hole_probability = (
                self.depth_preset.hole_base
                + self.depth_preset.hole_edge_gain * edge_norm
                + self.depth_preset.hole_depth_gain * depth_norm
            )
            hole_probability = np.clip(hole_probability, 0.0, 0.95)
            hole_mask = self.rng.random(noisy_depth_m.shape) < hole_probability
            noisy_depth_m[hole_mask] = 0.0
            depth_m = noisy_depth_m

        # P2: optional median filter on depth image (applies after noise injection)
        if self.depth_median_filter_size > 1:
            filt_size = self.depth_median_filter_size
            # only filter the valid-depth region to avoid smearing depth discontinuities at boundaries
            masked = np.where(valid, depth_m, 0.0)
            filtered = scipy_median_filter(masked, size=filt_size)
            # re-apply the mask so that originally-invalid pixels stay zero
            depth_m = np.where(valid, filtered, 0.0)

        quantized_mm = np.zeros_like(depth_m, dtype=np.uint16)
        valid = (depth_m >= self.min_depth_m) & (depth_m <= self.max_depth_m)
        if np.any(valid):
            scaled_mm = np.round(depth_m[valid] * 1000.0 / self.depth_quantization_mm) * self.depth_quantization_mm
            quantized_mm[valid] = np.clip(scaled_mm, 0, 65535).astype(np.uint16)

        out_msg = self.bridge.cv2_to_imgmsg(quantized_mm, encoding="16UC1")
        out_msg.header = copy.deepcopy(msg.header)
        self._safe_publish(self.depth_pub, out_msg)


if __name__ == "__main__":
    rospy.init_node("sensor_noise_injector")
    SensorNoiseInjector()
    rospy.spin()