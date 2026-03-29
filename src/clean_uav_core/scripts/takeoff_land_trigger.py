#!/usr/bin/env python3
# 本文件用于在ROS系统中发布起飞或降落命令。通过参数配置，可以指定发布的命令类型（起飞或降落）、发布的主题名称、发布的频率以及发布的次数。该脚本会在启动后等待一段时间，然后按照配置发布指定数量的命令消息。这对于自动化测试或演示无人机的起飞和降落功能非常有用。

import rospy
from quadrotor_msgs.msg import TakeoffLand


class TakeoffLandTrigger:
    def __init__(self):
        self.command = rospy.get_param("~command", "takeoff").strip().lower()
        self.topic = rospy.get_param("~topic", "/px4ctrl/takeoff_land")
        self.delay = float(rospy.get_param("~delay", 5.0))
        self.repeat = int(rospy.get_param("~repeat", 5))
        self.rate_hz = float(rospy.get_param("~rate", 2.0))

        self.publisher = rospy.Publisher(self.topic, TakeoffLand, queue_size=10, latch=True)

        if self.command == "takeoff":
            self.command_value = TakeoffLand.TAKEOFF
        elif self.command == "land":
            self.command_value = TakeoffLand.LAND
        else:
            raise ValueError("Unsupported command: {}".format(self.command))

    def run(self):
        rospy.loginfo(
            "[clean_uav_core] takeoff_land_trigger waiting %.2fs before publishing '%s' to %s",
            self.delay,
            self.command,
            self.topic,
        )
        rospy.sleep(self.delay)

        msg = TakeoffLand()
        msg.header.stamp = rospy.Time.now()
        msg.takeoff_land_cmd = self.command_value

        rate = rospy.Rate(self.rate_hz)
        for index in range(self.repeat):
            if rospy.is_shutdown():
                return
            msg.header.stamp = rospy.Time.now()
            self.publisher.publish(msg)
            rospy.loginfo(
                "[clean_uav_core] takeoff_land_trigger published %s (%d/%d)",
                self.command,
                index + 1,
                self.repeat,
            )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("takeoff_land_trigger")
    node = TakeoffLandTrigger()
    node.run()