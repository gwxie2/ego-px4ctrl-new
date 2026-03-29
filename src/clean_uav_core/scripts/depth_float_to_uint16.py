#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthFloatToUint16:
    def __init__(self):
        self.input_depth_topic = rospy.get_param("~input_depth_topic")
        self.output_depth_topic = rospy.get_param("~output_depth_topic")
        self.min_m = float(rospy.get_param("~min_m", 0.3))
        self.max_m = float(rospy.get_param("~max_m", 25.0))

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.output_depth_topic, Image, queue_size=1)
        self.sub = rospy.Subscriber(self.input_depth_topic, Image, self.depth_callback, queue_size=1)

        rospy.loginfo(
            "[clean_uav_core] depth_float_to_uint16 %s -> %s (min=%.2fm, max=%.2fm)",
            self.input_depth_topic,
            self.output_depth_topic,
            self.min_m,
            self.max_m,
        )

    def depth_callback(self, msg: Image):
        if msg.encoding == "16UC1":
            self.pub.publish(msg)
            return

        if msg.encoding != "32FC1":
            return

        depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)

        valid = (depth_m >= self.min_m) & (depth_m <= self.max_m)
        depth_mm = np.zeros_like(depth_m, dtype=np.uint16)
        depth_mm[valid] = np.clip(depth_m[valid] * 1000.0, 0, 65535).astype(np.uint16)

        out = self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
        out.header = msg.header
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("depth_float_to_uint16")
    DepthFloatToUint16()
    rospy.spin()
