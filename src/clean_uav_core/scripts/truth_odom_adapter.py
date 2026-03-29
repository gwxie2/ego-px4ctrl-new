#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class TruthOdomAdapter:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "iris")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.child_frame = rospy.get_param("~child_frame", "base_link")
        self.odom_topic = rospy.get_param("~odom_topic", "/cleanroom/truth_odom")
        self.pose_topic = rospy.get_param("~pose_topic", "/cleanroom/truth_pose")

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)

        self.model_index = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)

        rospy.loginfo("[clean_uav_core] truth_odom_adapter waiting for model '%s'", self.model_name)

    def model_states_callback(self, msg: ModelStates):
        if self.model_index is None:
            try:
                self.model_index = msg.name.index(self.model_name)
                rospy.loginfo("[clean_uav_core] truth_odom_adapter locked model '%s' at index %d", self.model_name, self.model_index)
            except ValueError:
                rospy.logwarn_throttle(5.0, "[clean_uav_core] model '%s' not found in /gazebo/model_states", self.model_name)
                return

        if self.model_index >= len(msg.pose) or self.model_index >= len(msg.twist):
            rospy.logwarn_throttle(5.0, "[clean_uav_core] model index out of range for '%s'", self.model_name)
            return

        stamp = rospy.Time.now()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.world_frame
        odom_msg.child_frame_id = self.child_frame
        odom_msg.pose.pose = msg.pose[self.model_index]
        odom_msg.twist.twist = msg.twist[self.model_index]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose = msg.pose[self.model_index]

        try:
            self.odom_pub.publish(odom_msg)
        except rospy.ROSException:
            rospy.logwarn("[clean_uav_core] truth_odom_adapter odom publish failed (topic closed)")
        try:
            self.pose_pub.publish(pose_msg)
        except rospy.ROSException:
            rospy.logwarn("[clean_uav_core] truth_odom_adapter pose publish failed (topic closed)")


if __name__ == "__main__":
    rospy.init_node("truth_odom_adapter")
    TruthOdomAdapter()
    rospy.spin()