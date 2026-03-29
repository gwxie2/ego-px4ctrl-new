#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped


class DynamicDualGoalPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))
        self.period_sec = max(0.1, float(rospy.get_param("~period_sec", 6.0)))
        self.goal_x = float(rospy.get_param("~goal_x", 18.0))
        self.goal_z = float(rospy.get_param("~goal_z", 1.0))
        self.start_delay = max(0.0, float(rospy.get_param("~start_delay", 0.0)))

        self.uav0_topic = rospy.get_param("~uav0_topic", "/uav0/goal")
        self.uav1_topic = rospy.get_param("~uav1_topic", "/uav1/goal")

        self.uav0_y_center = float(rospy.get_param("~uav0_y_center", 3.0))
        self.uav0_y_amp = abs(float(rospy.get_param("~uav0_y_amp", 0.5)))
        self.uav1_y_center = float(rospy.get_param("~uav1_y_center", -3.0))
        self.uav1_y_amp = abs(float(rospy.get_param("~uav1_y_amp", 0.5)))

        self.phase_offset_uav0 = float(rospy.get_param("~phase_offset_uav0", 0.0))
        self.phase_offset_uav1 = float(rospy.get_param("~phase_offset_uav1", 0.0))

        self.uav0_pub = rospy.Publisher(self.uav0_topic, PoseStamped, queue_size=10)
        self.uav1_pub = rospy.Publisher(self.uav1_topic, PoseStamped, queue_size=10)
        self.start_t = rospy.Time.now().to_sec()

        rospy.loginfo(
            "[clean_uav_core] dynamic_dual_goal_publisher start: x=%.2f, z=%.2f, T=%.2fs, start_delay=%.2fs, y0=[%.2f, %.2f], y1=[%.2f, %.2f]",
            self.goal_x,
            self.goal_z,
            self.period_sec,
            self.start_delay,
            self.uav0_y_center - self.uav0_y_amp,
            self.uav0_y_center + self.uav0_y_amp,
            self.uav1_y_center - self.uav1_y_amp,
            self.uav1_y_center + self.uav1_y_amp,
        )

    def _make_goal(self, y_value: float, stamp: rospy.Time) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = y_value
        msg.pose.position.z = self.goal_z
        msg.pose.orientation.w = 1.0
        return msg

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        omega = 2.0 * math.pi / self.period_sec
        last_log = rospy.Time(0)

        if self.start_delay > 0.0:
            rospy.loginfo(
                "[clean_uav_core] dynamic_dual_goal_publisher waiting %.2fs before publishing goals",
                self.start_delay,
            )
            rospy.sleep(self.start_delay)
            self.start_t = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            t = now.to_sec() - self.start_t

            y0 = self.uav0_y_center + self.uav0_y_amp * math.sin(omega * t + self.phase_offset_uav0)
            y1 = self.uav1_y_center + self.uav1_y_amp * math.sin(omega * t + self.phase_offset_uav1)

            self.uav0_pub.publish(self._make_goal(y0, now))
            self.uav1_pub.publish(self._make_goal(y1, now))

            if (now - last_log).to_sec() > 2.0:
                rospy.loginfo(
                    "[clean_uav_core] dynamic goals: uav0=(%.2f, %.2f, %.2f), uav1=(%.2f, %.2f, %.2f)",
                    self.goal_x,
                    y0,
                    self.goal_z,
                    self.goal_x,
                    y1,
                    self.goal_z,
                )
                last_log = now

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("dynamic_dual_goal_publisher")
    DynamicDualGoalPublisher().run()
