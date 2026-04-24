#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomPoseAdapter:
    def __init__(self):
        self.input_odom_topic = rospy.get_param("~input_odom_topic", "/odometry")
        self.output_pose_topic = rospy.get_param("~output_pose_topic", "/pose")
        self.output_odom_topic = rospy.get_param("~output_odom_topic", "")
        self.frame_id_override = rospy.get_param("~frame_id_override", "")
        self.offset_x = float(rospy.get_param("~offset_x", 0.0))
        self.offset_y = float(rospy.get_param("~offset_y", 0.0))
        self.offset_z = float(rospy.get_param("~offset_z", 0.0))

        self.pose_pub = rospy.Publisher(self.output_pose_topic, PoseStamped, queue_size=10, latch=True)
        self.odom_pub = None
        if self.output_odom_topic:
            self.odom_pub = rospy.Publisher(self.output_odom_topic, Odometry, queue_size=10, latch=True)
        
        self.last_log_time = rospy.Time(0)

        rospy.Subscriber(self.input_odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.loginfo(
            "[clean_uav_core] odom_pose_adapter listening %s -> pose: %s, odom: %s (offset=[%.3f, %.3f, %.3f])",
            self.input_odom_topic,
            self.output_pose_topic,
            self.output_odom_topic,
            self.offset_x,
            self.offset_y,
            self.offset_z,
        )

    def odom_callback(self, msg: Odometry):
        import copy
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id_override if self.frame_id_override else msg.header.frame_id
        pose_msg.pose = copy.deepcopy(msg.pose.pose)
        pose_msg.pose.position.x += self.offset_x
        pose_msg.pose.position.y += self.offset_y
        pose_msg.pose.position.z += self.offset_z

        self.pose_pub.publish(pose_msg)

        if self.odom_pub is not None:
            odom_msg = copy.deepcopy(msg)
            odom_msg.header = pose_msg.header
            odom_msg.pose.pose = pose_msg.pose
            self.odom_pub.publish(odom_msg)

        now = rospy.Time.now()
        if (now - self.last_log_time).to_sec() > 5.0:
            rospy.loginfo(
                "[clean_uav_core] odom_pose_adapter relaying pose, frame_id=%s, xyz=(%.2f, %.2f, %.2f)",
                pose_msg.header.frame_id,
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            )
            self.last_log_time = now


if __name__ == "__main__":
    rospy.init_node("odom_pose_adapter")
    OdomPoseAdapter()
    rospy.spin()