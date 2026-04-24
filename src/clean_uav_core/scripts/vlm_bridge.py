#!/usr/bin/env python3

import math
import sys
import threading
from dataclasses import dataclass

import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import GoalSet
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


DEFAULT_FX = 554.254691191187
DEFAULT_FY = 554.254691191187
DEFAULT_CX = 320.5
DEFAULT_CY = 240.5
DEFAULT_CAM2BODY_ROT = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
)


def quaternion_to_rotation_matrix(quaternion_msg):
    x = float(quaternion_msg.x)
    y = float(quaternion_msg.y)
    z = float(quaternion_msg.z)
    w = float(quaternion_msg.w)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def multiply_matrix_vector(matrix, vector):
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def add_vectors(lhs, rhs):
    return (lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2])


class ObjectDetector:
    """Placeholder detector for future VLM integrations.

    当前类只回传手动输入的像素坐标，便于先打通 2D -> 3D -> planner 的链路。
    未来可在 detect 中接入 Grounding DINO，取检测框中心点作为目标像素；
    也可在这里接入 GPT-4o 或其他 VLM API，把文本指令解析成像素或候选区域。
    """

    def detect(self, image_msg, manual_pixel=None, prompt=""):
        del image_msg
        del prompt
        return manual_pixel


@dataclass
class DetectionResult:
    pixel: tuple
    depth_override: float = None
    prompt: str = ""
    backend: str = "manual"


class ManualObjectDetector(ObjectDetector):
    def detect(self, image_msg, manual_pixel=None, prompt=""):
        del image_msg
        if manual_pixel is None:
            return None
        return DetectionResult(pixel=manual_pixel, prompt=prompt, backend="manual")


class TopicObjectDetector(ObjectDetector):
    def __init__(self, topic_name, allow_manual_override=True, prefer_topic=True):
        self.allow_manual_override = allow_manual_override
        self.prefer_topic = prefer_topic
        self.lock = threading.RLock()
        self.latest_detection = None
        rospy.Subscriber(topic_name, Point, self.detected_pixel_callback, queue_size=10)

    def detected_pixel_callback(self, msg):
        depth_override = float(msg.z)
        if not math.isfinite(depth_override) or depth_override <= 0.0:
            depth_override = None

        detection = DetectionResult(
            pixel=(float(msg.x), float(msg.y)),
            depth_override=depth_override,
            backend="topic",
        )
        with self.lock:
            self.latest_detection = detection

    def detect(self, image_msg, manual_pixel=None, prompt=""):
        del image_msg
        with self.lock:
            latest_detection = self.latest_detection

        if latest_detection is not None and self.prefer_topic:
            return DetectionResult(
                pixel=latest_detection.pixel,
                depth_override=latest_detection.depth_override,
                prompt=prompt,
                backend="topic",
            )

        if manual_pixel is not None and self.allow_manual_override:
            return DetectionResult(pixel=manual_pixel, prompt=prompt, backend="manual_override")

        if latest_detection is not None:
            return DetectionResult(
                pixel=latest_detection.pixel,
                depth_override=latest_detection.depth_override,
                prompt=prompt,
                backend="topic",
            )

        return None


class VlmBridge:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.RLock()

        self.drone_id = int(rospy.get_param("~drone_id", 0))
        self.output_mode = str(rospy.get_param("~output_mode", "auto")).strip().lower()
        self.detector_backend = str(rospy.get_param("~detector_backend", "manual")).strip().lower()
        self.allow_manual_pixel_override = bool(
            rospy.get_param("~allow_manual_pixel_override", True)
        )
        self.default_text_query = str(rospy.get_param("~default_text_query", "")).strip()
        self.frame_id = str(rospy.get_param("~frame_id", "world")).strip()
        self.goal_topic_v1 = str(
            rospy.get_param("~goal_topic_v1", f"/drone_{self.drone_id}/goal")
        ).strip()
        self.goal_topic_v2 = str(rospy.get_param("~goal_topic_v2", "/goal_with_id")).strip()

        self.use_camera_info = bool(rospy.get_param("~use_camera_info", False))
        self.enable_cli = bool(rospy.get_param("~enable_cli", True))
        self.require_rgb_for_manual_input = bool(
            rospy.get_param("~require_rgb_for_manual_input", False)
        )

        self.map_size_z = float(rospy.get_param("~map_size_z", 5.0))
        self.virtual_ceil_height = float(rospy.get_param("~virtual_ceil_height", 3.0))
        self.camera_offset = (
            float(rospy.get_param("~camera_offset_x", 0.0)),
            float(rospy.get_param("~camera_offset_y", 0.0)),
            float(rospy.get_param("~camera_offset_z", 0.0)),
        )

        self.fx = float(rospy.get_param("~fx", DEFAULT_FX))
        self.fy = float(rospy.get_param("~fy", DEFAULT_FY))
        self.cx = float(rospy.get_param("~cx", DEFAULT_CX))
        self.cy = float(rospy.get_param("~cy", DEFAULT_CY))
        self.intrinsics_ready = self._intrinsics_valid(self.fx, self.fy)

        self.cam2body_rot = self._load_cam2body_rotation()

        self.latest_image_msg = None
        self.latest_depth_msg = None
        self.latest_depth_image = None
        self.latest_depth_encoding = ""
        self.latest_odom_msg = None
        self.camera_info_msg = None
        self.latest_text_query = self.default_text_query

        self.mode_cache = None
        self.last_mode_log = rospy.Time(0)

        self.detector = self._build_detector()

        self.goal_pub_v1 = rospy.Publisher(self.goal_topic_v1, PoseStamped, queue_size=1)
        self.goal_pub_v2 = rospy.Publisher(self.goal_topic_v2, GoalSet, queue_size=10)
        self.debug_goal_pub = rospy.Publisher("~projected_goal", PoseStamped, queue_size=1)

        rospy.Subscriber("~image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("~depth_raw", Image, self.depth_callback, queue_size=1)
        rospy.Subscriber("~odom", Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber("~pixel_input", Point, self.pixel_input_callback, queue_size=10)
        rospy.Subscriber("~text_query", String, self.text_query_callback, queue_size=10)
        rospy.Subscriber("~detect_trigger", String, self.detect_trigger_callback, queue_size=10)

        if self.use_camera_info:
            rospy.Subscriber("~camera_info", CameraInfo, self.camera_info_callback, queue_size=1)

        rospy.loginfo(
            "[clean_uav_core] vlm_bridge started: drone_id=%d output_mode=%s detector_backend=%s goal_v1=%s goal_v2=%s",
            self.drone_id,
            self.output_mode,
            self.detector_backend,
            self.goal_topic_v1,
            self.goal_topic_v2,
        )
        rospy.loginfo(
            "[clean_uav_core] vlm_bridge intrinsics source=%s fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
            "camera_info" if self.use_camera_info else "manual_param",
            self.fx,
            self.fy,
            self.cx,
            self.cy,
        )

        if self.enable_cli:
            thread = threading.Thread(target=self.cli_loop, name="vlm_bridge_cli", daemon=True)
            thread.start()

    @staticmethod
    def _intrinsics_valid(fx, fy):
        return math.isfinite(fx) and math.isfinite(fy) and fx > 0.0 and fy > 0.0

    def _build_detector(self):
        if self.detector_backend == "manual":
            return ManualObjectDetector()
        if self.detector_backend in ("topic", "external_topic"):
            return TopicObjectDetector(
                "~detected_pixel",
                allow_manual_override=self.allow_manual_pixel_override,
                prefer_topic=False,
            )
        if self.detector_backend == "prefer_topic":
            return TopicObjectDetector(
                "~detected_pixel",
                allow_manual_override=self.allow_manual_pixel_override,
                prefer_topic=True,
            )

        rospy.logwarn(
            "[clean_uav_core] vlm_bridge unknown detector_backend=%s, fallback to manual",
            self.detector_backend,
        )
        self.detector_backend = "manual"
        return ManualObjectDetector()

    def _load_cam2body_rotation(self):
        raw_value = rospy.get_param(
            "~cam2body_rotation",
            [
                0.0,
                0.0,
                1.0,
                -1.0,
                0.0,
                0.0,
                0.0,
                -1.0,
                0.0,
            ],
        )
        if isinstance(raw_value, (list, tuple)) and len(raw_value) == 9:
            flat = [float(item) for item in raw_value]
            return (
                (flat[0], flat[1], flat[2]),
                (flat[3], flat[4], flat[5]),
                (flat[6], flat[7], flat[8]),
            )
        return DEFAULT_CAM2BODY_ROT

    def image_callback(self, msg):
        with self.lock:
            self.latest_image_msg = msg

    def depth_callback(self, msg):
        try:
            if msg.encoding == "16UC1":
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            elif msg.encoding == "32FC1":
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            else:
                rospy.logwarn_throttle(
                    5.0,
                    "[clean_uav_core] vlm_bridge unsupported depth encoding: %s",
                    msg.encoding,
                )
                return
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "[clean_uav_core] vlm_bridge depth conversion failed: %s", error)
            return

        with self.lock:
            self.latest_depth_msg = msg
            self.latest_depth_image = depth_image
            self.latest_depth_encoding = msg.encoding

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom_msg = msg

    def camera_info_callback(self, msg):
        if len(msg.K) < 9:
            return
        fx = float(msg.K[0])
        fy = float(msg.K[4])
        cx = float(msg.K[2])
        cy = float(msg.K[5])
        if not self._intrinsics_valid(fx, fy):
            rospy.logwarn_throttle(5.0, "[clean_uav_core] vlm_bridge camera_info intrinsics invalid")
            return

        with self.lock:
            self.fx = fx
            self.fy = fy
            self.cx = cx
            self.cy = cy
            self.camera_info_msg = msg
            self.intrinsics_ready = True

    def pixel_input_callback(self, msg):
        depth_override = float(msg.z)
        if not math.isfinite(depth_override) or depth_override <= 0.0:
            depth_override = None
        self.process_request(
            manual_pixel=(float(msg.x), float(msg.y)),
            depth_override=depth_override,
            source="topic_manual",
        )

    def text_query_callback(self, msg):
        text = str(msg.data).strip()
        with self.lock:
            self.latest_text_query = text
        rospy.loginfo("[clean_uav_core] vlm_bridge updated text query: %s", text)

    def detect_trigger_callback(self, msg):
        prompt = str(msg.data).strip()
        self.process_request(manual_pixel=None, source="detect_trigger", prompt=prompt)

    def cli_loop(self):
        if sys.stdin.closed:
            rospy.logwarn("[clean_uav_core] vlm_bridge CLI disabled because stdin is closed")
            return

        rospy.loginfo(
            "[clean_uav_core] vlm_bridge CLI ready. Input: 'u v', 'u v depth_m', or 'detect <text query>'. Type 'quit' to exit CLI thread."
        )

        while not rospy.is_shutdown():
            try:
                raw_line = sys.stdin.readline()
            except Exception as error:
                rospy.logwarn("[clean_uav_core] vlm_bridge CLI read failed: %s", error)
                return

            if raw_line == "":
                rospy.logwarn("[clean_uav_core] vlm_bridge CLI reached EOF; stop interactive input")
                return

            text = raw_line.strip()
            if not text:
                continue
            if text.lower() in ("quit", "exit"):
                rospy.loginfo("[clean_uav_core] vlm_bridge CLI thread exit requested")
                return
            if text.lower() == "help":
                rospy.loginfo(
                    "[clean_uav_core] vlm_bridge CLI usage: 'u v', 'u v depth_m', or 'detect <text query>'"
                )
                continue

            if text.lower().startswith("detect"):
                prompt = text[6:].strip() if len(text) > 6 else ""
                self.process_request(manual_pixel=None, source="cli_detect", prompt=prompt)
                continue

            parts = text.split()
            if len(parts) not in (2, 3):
                rospy.logwarn("[clean_uav_core] vlm_bridge invalid input: %s", text)
                continue

            try:
                u = float(parts[0])
                v = float(parts[1])
                depth_override = float(parts[2]) if len(parts) == 3 else None
            except ValueError:
                rospy.logwarn("[clean_uav_core] vlm_bridge failed to parse CLI input: %s", text)
                continue

            if depth_override is not None and (not math.isfinite(depth_override) or depth_override <= 0.0):
                depth_override = None

            self.process_request(
                manual_pixel=(u, v),
                depth_override=depth_override,
                source="cli_manual",
            )

    def resolve_output_mode(self):
        if self.output_mode in ("v1", "v2"):
            return self.output_mode

        candidates = [
            (f"/drone_{self.drone_id}/ego_planner_v2/fsm/flight_type", "v2"),
            (f"/drone_{self.drone_id}/ego_planner/fsm/flight_type", "v1"),
            ("~planner_flight_type", None),
        ]

        detected = None
        for param_name, mode_hint in candidates:
            if not rospy.has_param(param_name):
                continue
            value = rospy.get_param(param_name)
            try:
                flight_type = int(value)
            except (TypeError, ValueError):
                continue
            if flight_type == 2:
                detected = "v2"
                break
            if flight_type == 1:
                detected = "v1"
                break
            if mode_hint is not None:
                detected = mode_hint
                break

        if detected is None:
            detected = "v1"

        now = rospy.Time.now()
        if self.mode_cache != detected or (now - self.last_mode_log).to_sec() > 10.0:
            rospy.loginfo("[clean_uav_core] vlm_bridge auto-selected output mode: %s", detected)
            self.mode_cache = detected
            self.last_mode_log = now
        return detected

    def sample_depth(self, depth_image, depth_encoding, u, v):
        row = int(round(v))
        col = int(round(u))

        if depth_image is None:
            return None, "depth_not_ready"
        if row < 0 or col < 0 or row >= depth_image.shape[0] or col >= depth_image.shape[1]:
            return None, "pixel_out_of_range"

        raw_value = depth_image[row, col]
        try:
            depth_value = float(raw_value)
        except (TypeError, ValueError):
            return None, "depth_not_numeric"

        if not math.isfinite(depth_value) or depth_value <= 0.0:
            return None, "depth_invalid"

        if depth_encoding == "16UC1":
            depth_m = depth_value / 1000.0
        elif depth_encoding == "32FC1":
            depth_m = depth_value
        else:
            return None, "depth_encoding_unsupported"

        if not math.isfinite(depth_m) or depth_m <= 0.0:
            return None, "depth_invalid"

        return depth_m, None

    def project_pixel_to_3d(self, u, v, depth_m, odom_msg):
        if not self.intrinsics_ready:
            raise ValueError("camera_intrinsics_not_ready")
        if depth_m is None or not math.isfinite(depth_m) or depth_m <= 0.0:
            raise ValueError("depth_invalid")
        if odom_msg is None:
            raise ValueError("odom_not_ready")

        point_cam = (
            (float(u) - self.cx) * depth_m / self.fx,
            (float(v) - self.cy) * depth_m / self.fy,
            depth_m,
        )
        point_body = add_vectors(
            multiply_matrix_vector(self.cam2body_rot, point_cam),
            self.camera_offset,
        )

        body_rotation = quaternion_to_rotation_matrix(odom_msg.pose.pose.orientation)
        body_translation = (
            float(odom_msg.pose.pose.position.x),
            float(odom_msg.pose.pose.position.y),
            float(odom_msg.pose.pose.position.z),
        )
        point_world = add_vectors(
            multiply_matrix_vector(body_rotation, point_body),
            body_translation,
        )
        return point_world

    def process_request(self, manual_pixel=None, depth_override=None, source="manual", prompt=""):
        with self.lock:
            latest_image_msg = self.latest_image_msg
            latest_depth_image = self.latest_depth_image
            latest_depth_encoding = self.latest_depth_encoding
            latest_odom_msg = self.latest_odom_msg
            latest_text_query = self.latest_text_query

        if latest_depth_image is None:
            rospy.logwarn("[clean_uav_core] vlm_bridge reject request (%s): depth not ready", source)
            return False
        if latest_odom_msg is None:
            rospy.logwarn("[clean_uav_core] vlm_bridge reject request (%s): odom not ready", source)
            return False
        if self.require_rgb_for_manual_input and latest_image_msg is None:
            rospy.logwarn("[clean_uav_core] vlm_bridge reject request (%s): RGB image not ready", source)
            return False
        if not self.intrinsics_ready:
            rospy.logwarn("[clean_uav_core] vlm_bridge reject request (%s): intrinsics not ready", source)
            return False

        resolved_prompt = prompt if str(prompt).strip() else latest_text_query
        detection = self.detector.detect(
            latest_image_msg,
            manual_pixel=manual_pixel,
            prompt=resolved_prompt,
        )
        if detection is None:
            rospy.logwarn(
                "[clean_uav_core] vlm_bridge reject request (%s): detector backend=%s returned no pixel",
                source,
                self.detector_backend,
            )
            return False

        pixel_u = float(detection.pixel[0])
        pixel_v = float(detection.pixel[1])

        depth_m = depth_override if depth_override is not None else detection.depth_override
        if depth_m is None:
            depth_m, error_reason = self.sample_depth(
                latest_depth_image,
                latest_depth_encoding,
                pixel_u,
                pixel_v,
            )
            if depth_m is None:
                rospy.logwarn(
                    "[clean_uav_core] vlm_bridge reject request (%s): %s at pixel (%.1f, %.1f)",
                    source,
                    error_reason,
                    pixel_u,
                    pixel_v,
                )
                return False

        try:
            point_world = self.project_pixel_to_3d(pixel_u, pixel_v, depth_m, latest_odom_msg)
        except ValueError as error:
            rospy.logwarn("[clean_uav_core] vlm_bridge reject request (%s): %s", source, error)
            return False

        if point_world[2] > self.map_size_z:
            rospy.logwarn(
                "[clean_uav_core] vlm_bridge reject request (%s): z=%.3f exceeds map_size_z=%.3f",
                source,
                point_world[2],
                self.map_size_z,
            )
            return False
        if self.virtual_ceil_height > 0.0 and point_world[2] > self.virtual_ceil_height:
            rospy.logwarn(
                "[clean_uav_core] vlm_bridge reject request (%s): z=%.3f exceeds virtual_ceil_height=%.3f",
                source,
                point_world[2],
                self.virtual_ceil_height,
            )
            return False

        self.publish_goal(point_world)
        rospy.loginfo(
            "[clean_uav_core] vlm_bridge source=%s backend=%s prompt='%s' pixel=(%.1f, %.1f) depth=%.3f -> world=(%.3f, %.3f, %.3f)",
            source,
            detection.backend,
            resolved_prompt,
            pixel_u,
            pixel_v,
            depth_m,
            point_world[0],
            point_world[1],
            point_world[2],
        )
        return True

    def publish_goal(self, point_world):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = point_world[0]
        pose_msg.pose.position.y = point_world[1]
        pose_msg.pose.position.z = point_world[2]
        pose_msg.pose.orientation.w = 1.0
        self.debug_goal_pub.publish(pose_msg)

        mode = self.resolve_output_mode()
        if mode == "v2":
            goal_msg = GoalSet()
            goal_msg.drone_id = self.drone_id
            goal_msg.goal[0] = point_world[0]
            goal_msg.goal[1] = point_world[1]
            goal_msg.goal[2] = point_world[2]
            self.goal_pub_v2.publish(goal_msg)
        else:
            self.goal_pub_v1.publish(pose_msg)


def main():
    rospy.init_node("vlm_bridge", anonymous=False)
    VlmBridge()
    rospy.spin()


if __name__ == "__main__":
    main()