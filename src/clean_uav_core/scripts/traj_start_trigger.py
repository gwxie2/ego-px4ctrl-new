#!/usr/bin/env python3
"""在里程计稳定后，从当前位姿发布轨迹开始触发消息。

这个节点的角色很明确：它不是规划器，也不是控制器，而是把“无人机已经
站稳了，可以开始规划/执行”这件事转换成一个标准的 PoseStamped 触发事件。
"""

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
        # 只缓存最新姿态，不在回调里做复杂判断，避免把订阅回调变成重逻辑入口。
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
            # 等待首个有效里程计，避免把空位姿发给后续节点。
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
