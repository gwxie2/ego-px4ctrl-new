#!/usr/bin/env python3
# 本文件是 clean_uav_core 包中的 dual_traj_start_trigger.py 脚本，用于在多旋翼无人机系统中发布同步的轨迹开始触发消息。

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class DualTrajStartTrigger:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.delay = float(rospy.get_param("~delay", 12.0))
        self.repeat = int(rospy.get_param("~repeat", 8))
        self.rate = float(rospy.get_param("~rate", 3.0))
        self.wait_odom_timeout = float(rospy.get_param("~wait_odom_timeout", 0.0))

        self.uav_configs = [
            {
                "name": "uav0",
                "odom_topic": rospy.get_param("~uav0_odom_topic", "/uav0/truth_odom"),
                "trigger_topic": rospy.get_param("~uav0_trigger_topic", "/uav0/traj_start_trigger"),
                "latest_pose": None,
            },
            {
                "name": "uav1",
                "odom_topic": rospy.get_param("~uav1_odom_topic", "/uav1/truth_odom"),
                "trigger_topic": rospy.get_param("~uav1_trigger_topic", "/uav1/traj_start_trigger"),
                "latest_pose": None,
            },
        ]

        self.publishers = []
        self.subscribers = []
        for index, config in enumerate(self.uav_configs):
            self.publishers.append(rospy.Publisher(config["trigger_topic"], PoseStamped, queue_size=10))
            self.subscribers.append(
                rospy.Subscriber(
                    config["odom_topic"],
                    Odometry,
                    self._make_odom_callback(index),
                    queue_size=1,
                )
            )

    def _make_odom_callback(self, index):
        def _callback(msg):
            self.uav_configs[index]["latest_pose"] = msg.pose.pose

        return _callback

    def _all_ready(self):
        return all(config["latest_pose"] is not None for config in self.uav_configs)

    def run(self):
        rospy.loginfo(
            "[clean_uav_core] dual_traj_start_trigger waiting %.2fs before synchronized trigger",
            self.delay,
        )
        rospy.sleep(self.delay)

        start_wait = rospy.Time.now()
        wait_rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self._all_ready():
            waited = (rospy.Time.now() - start_wait).to_sec()
            if self.wait_odom_timeout > 0.0 and waited > self.wait_odom_timeout:
                rospy.logwarn(
                    "[clean_uav_core] dual_traj_start_trigger timeout waiting odom (%.2fs), keep waiting until odom is ready",
                    waited,
                )
                start_wait = rospy.Time.now()
            rospy.logwarn_throttle(2.0, "[clean_uav_core] dual_traj_start_trigger waiting for all UAV odometry")
            wait_rate.sleep()

        if rospy.is_shutdown():
            return

        pub_rate = rospy.Rate(self.rate if self.rate > 0.0 else 2.0)
        for index in range(self.repeat):
            if rospy.is_shutdown():
                return

            now = rospy.Time.now()
            for i, config in enumerate(self.uav_configs):
                if config["latest_pose"] is None:
                    continue
                msg = PoseStamped()
                msg.header.stamp = now
                msg.header.frame_id = self.frame_id
                msg.pose = config["latest_pose"]
                self.publishers[i].publish(msg) # 发布同步的轨迹开始触发消息，包含当前位姿信息

            rospy.loginfo(
                "[clean_uav_core] dual_traj_start_trigger published synchronized trigger %d/%d",
                index + 1,
                self.repeat,
            )
            pub_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("dual_traj_start_trigger")
    DualTrajStartTrigger().run()
