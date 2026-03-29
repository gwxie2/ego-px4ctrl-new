#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped


class GoalPointPublisher:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/move_base_simple/goal")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.x = float(rospy.get_param("~x", 5.0))
        self.y = float(rospy.get_param("~y", 0.0))
        self.z = float(rospy.get_param("~z", 1.0))
        self.delay = float(rospy.get_param("~delay", 2.0))
        self.repeat = int(rospy.get_param("~repeat", 5))
        self.rate_hz = float(rospy.get_param("~rate", 2.0))

        self.publisher = rospy.Publisher(self.topic, PoseStamped, queue_size=1, latch=True)

    def run(self):
        if self.delay > 0.0:
            rospy.loginfo("[clean_uav_core] goal_point_publisher waiting %.2f s", self.delay)
            rospy.sleep(self.delay)

        rate = rospy.Rate(self.rate_hz)

        for attempt in range(1, self.repeat + 1):
            if rospy.is_shutdown():
                return

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
            msg.pose.orientation.w = 1.0

            self.publisher.publish(msg)
            rospy.loginfo(
                "[clean_uav_core] goal_point_publisher sent goal to %s: (%.2f, %.2f, %.2f), attempt %d/%d",
                self.topic,
                self.x,
                self.y,
                self.z,
                attempt,
                self.repeat,
            )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("goal_point_publisher")
    GoalPointPublisher().run()