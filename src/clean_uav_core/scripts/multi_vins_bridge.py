#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class MultiVinsBridge:
    def __init__(self):
        self.vehicle_type = rospy.get_param("~vehicle_type", "iris")
        self.vehicle_num = int(rospy.get_param("~vehicle_num", 2))
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.force_world_frame = bool(rospy.get_param("~force_world_frame", True))
        self.odom_mode = rospy.get_param("~odom_mode", "vins_only")
        self.primary_odom_topic_suffix = rospy.get_param("~primary_odom_topic_suffix", "vins_estimator/imu_propagate").strip("/")
        self.enable_mavros_vision_pose = bool(rospy.get_param("~enable_mavros_vision_pose", True))
        self.enable_fallback_local_odom = bool(rospy.get_param("~enable_fallback_local_odom", True))
        self.primary_timeout_sec = float(rospy.get_param("~primary_timeout_sec", 0.6))

        self.odom_pubs = {}
        self.pose_pubs = {}
        self.last_primary_stamp = {}

        for vehicle_id in range(self.vehicle_num):
            input_topic = f"/{self.vehicle_type}_{vehicle_id}/{self.primary_odom_topic_suffix}"
            fallback_topic = f"/{self.vehicle_type}_{vehicle_id}/mavros/local_position/odom"
            odom_topic = f"/{self.vehicle_type}_{vehicle_id}/odometry"
            pose_topic = f"/{self.vehicle_type}_{vehicle_id}/mavros/vision_pose/pose"

            self.odom_pubs[vehicle_id] = rospy.Publisher(odom_topic, Odometry, queue_size=20)
            self.last_primary_stamp[vehicle_id] = rospy.Time(0)
            if self.enable_mavros_vision_pose:
                self.pose_pubs[vehicle_id] = rospy.Publisher(pose_topic, PoseStamped, queue_size=20)

            rospy.Subscriber(input_topic, Odometry, self._primary_odom_callback, callback_args=vehicle_id, queue_size=20)
            if self.enable_fallback_local_odom:
                rospy.Subscriber(fallback_topic, Odometry, self._fallback_odom_callback, callback_args=vehicle_id, queue_size=20)

            rospy.loginfo(
                "[clean_uav_core] multi_vins_bridge mode=%s uav%d: primary=%s, fallback=%s -> %s%s",
                self.odom_mode,
                vehicle_id,
                input_topic,
                fallback_topic if self.enable_fallback_local_odom else "disabled",
                odom_topic,
                f", {pose_topic}" if self.enable_mavros_vision_pose else "",
            )

    def _primary_odom_callback(self, msg: Odometry, vehicle_id: int):
        self.last_primary_stamp[vehicle_id] = rospy.Time.now()
        self._publish(vehicle_id, msg)

    def _fallback_odom_callback(self, msg: Odometry, vehicle_id: int):
        now = rospy.Time.now()
        dt = (now - self.last_primary_stamp[vehicle_id]).to_sec()
        if dt <= self.primary_timeout_sec:
            return
        self._publish(vehicle_id, msg)

    def _publish(self, vehicle_id: int, msg: Odometry):
        out_odom = Odometry()
        out_odom.header = msg.header
        if self.force_world_frame:
            out_odom.header.frame_id = self.world_frame
        elif not out_odom.header.frame_id:
            out_odom.header.frame_id = self.world_frame
        out_odom.child_frame_id = msg.child_frame_id if msg.child_frame_id else "base_link"
        out_odom.pose = msg.pose
        out_odom.twist = msg.twist

        self.odom_pubs[vehicle_id].publish(out_odom)

        if self.enable_mavros_vision_pose and vehicle_id in self.pose_pubs:
            out_pose = PoseStamped()
            out_pose.header.stamp = out_odom.header.stamp if out_odom.header.stamp != rospy.Time() else rospy.Time.now()
            out_pose.header.frame_id = out_odom.header.frame_id
            out_pose.pose = out_odom.pose.pose
            self.pose_pubs[vehicle_id].publish(out_pose)


if __name__ == "__main__":
    rospy.init_node("multi_vins_bridge")
    MultiVinsBridge()
    rospy.spin()