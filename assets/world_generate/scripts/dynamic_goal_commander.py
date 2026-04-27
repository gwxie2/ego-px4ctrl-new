#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped


class DynamicGoalCommander:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.speed_x = float(rospy.get_param("~speed_x", 0.5))
        self.x_start = float(rospy.get_param("~x_start", 4.0))
        self.x_end = float(rospy.get_param("~x_end", 18.0))
        self.z_goal = float(rospy.get_param("~z_goal", 0.1))
        self.frame_id = rospy.get_param("~frame_id", "world")

        self.y_drone0 = float(rospy.get_param("~drone0_y", 1.5))
        self.y_drone1 = float(rospy.get_param("~drone1_y", -1.5))

        self.pub_drone0 = rospy.Publisher(
            "/drone_0_planning/goal", PoseStamped, queue_size=10
        )
        self.pub_drone1 = rospy.Publisher(
            "/drone_1_planning/goal", PoseStamped, queue_size=10
        )

        self.start_time = rospy.Time.now().to_sec()

    def _compute_x(self, now_sec: float) -> float:
        x = self.x_start + self.speed_x * (now_sec - self.start_time)
        if x > self.x_end:
            x = self.x_end
        return x

    def _build_goal(self, x: float, y: float, stamp: rospy.Time) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = self.z_goal
        msg.pose.orientation.w = 1.0
        return msg

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo(
            "dynamic_goal_commander started: rate=%.1fHz speed_x=%.2f x:[%.1f -> %.1f]"
            % (self.rate_hz, self.speed_x, self.x_start, self.x_end)
        )

        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            x_goal = self._compute_x(stamp.to_sec())

            goal0 = self._build_goal(x_goal, self.y_drone0, stamp)
            goal1 = self._build_goal(x_goal, self.y_drone1, stamp)

            self.pub_drone0.publish(goal0)
            self.pub_drone1.publish(goal1)

            # ==================== LLM/VLM Interface (reserved) ====================
            # Future extension idea:
            # 1) Subscribe to a high-level JSON command topic, e.g. /vlm/high_level_goal
            # 2) Parse JSON schema such as:
            #    {
            #      "drone_0": {"x": 12.0, "y": 1.5, "z": 0.1, "mode": "track"},
            #      "drone_1": {"x": 12.0, "y": -1.5, "z": 0.1, "mode": "track"},
            #      "speed_x": 0.6
            #    }
            # 3) Replace x/y/z hard-coded trajectory logic with JSON-driven targets.
            # ======================================================================

            rate.sleep()


def main():
    rospy.init_node("dynamic_goal_commander")
    commander = DynamicGoalCommander()
    commander.spin()


if __name__ == "__main__":
    main()
