#!/usr/bin/env python3
# 本文件用于在ROS系统中发布轨迹开始的触发消息。通过订阅无人机的位置信息（里程计），在启动后等待一段时间，然后从当前位姿发布指定数量的PoseStamped消息到约定的主题上。这些消息可以被轨迹服务器或其他组件订阅，用于触发轨迹规划或执行。该脚本确保在发布触发消息之前已经接收到位置信息，并且提供了一些参数配置选项，例如发布的频率和次数。

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class TrajStartTrigger:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/truth_odom")
        self.trigger_topic = rospy.get_param("~trigger_topic", "/traj_start_trigger")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.delay = float(rospy.get_param("~delay", 12.0))
        self.repeat = int(rospy.get_param("~repeat", 5))
        self.rate = float(rospy.get_param("~rate", 2.0))

        self.latest_pose = None
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.trigger_pub = rospy.Publisher(self.trigger_topic, PoseStamped, queue_size=10)

    def odom_callback(self, msg):
        self.latest_pose = msg.pose.pose

    def run(self):
        rospy.loginfo(
            "[clean_uav_core] traj_start_trigger waiting %.2fs before publishing to %s",
            self.delay,
            self.trigger_topic,
        )
        rospy.sleep(self.delay)

        wait_rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and self.latest_pose is None:
            rospy.logwarn_throttle(2.0, "[clean_uav_core] traj_start_trigger waiting for odometry on %s", self.odom_topic)
            wait_rate.sleep()

        if rospy.is_shutdown():
            return

        pub_rate = rospy.Rate(self.rate if self.rate > 0.0 else 2.0)
        for index in range(self.repeat):
            if rospy.is_shutdown():
                return

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.pose = self.latest_pose
            self.trigger_pub.publish(msg)
            rospy.loginfo(
                "[clean_uav_core] traj_start_trigger published trigger %d/%d from current pose",
                index + 1,
                self.repeat,
            )
            pub_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("traj_start_trigger")
    TrajStartTrigger().run()
