#!/usr/bin/env python3
# 本文件用于监控无人机在预设任务中的进度。通过订阅无人机的位置信息（里程计）和预设的任务路径（多个航点），判断无人机是否按照预定路径飞行，并在达到每个航点时发布状态更新。该脚本还支持一些容错机制，例如在无人机接近但未完全进入航点范围时，给予一定的宽限时间；以及通过监控规划器活动来判断是否需要强制完成任务。这些功能有助于确保无人机能够顺利完成预设的飞行任务，并提供实时的状态反馈。

import math

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String
from traj_utils.msg import Bspline


class MissionProgressMonitor:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/truth_odom")
        self.status_topic = rospy.get_param("~status_topic", "/cleanroom/mission_status")
        self.reach_radius = float(rospy.get_param("~reach_radius", 0.45))
        self.hold_time = float(rospy.get_param("~hold_time", 0.8))
        self.exit_grace_time = float(rospy.get_param("~exit_grace_time", 0.6))
        self.use_cmd_reach_fallback = bool(rospy.get_param("~use_cmd_reach_fallback", False))
        self.cmd_topic = rospy.get_param("~cmd_topic", "position_cmd")
        self.cmd_reach_radius = float(rospy.get_param("~cmd_reach_radius", self.reach_radius))
        self.use_planner_activity_fallback = bool(rospy.get_param("~use_planner_activity_fallback", False))
        self.planner_bspline_topic = rospy.get_param("~planner_bspline_topic", "planning/bspline")
        self.force_complete_timeout = float(rospy.get_param("~force_complete_timeout", 25.0))
        self.mission_name = rospy.get_param("~mission_name", "phase1_preset_mission")
        self.waypoint_num = int(rospy.get_param("~waypoint_num", 1))

        self.waypoints = []
        for index in range(self.waypoint_num):
            self.waypoints.append(
                (
                    float(rospy.get_param(f"~waypoint{index}_x", 0.0)),
                    float(rospy.get_param(f"~waypoint{index}_y", 0.0)),
                    float(rospy.get_param(f"~waypoint{index}_z", 1.0)),
                )
            )

        self.current_index = 0
        self.enter_time = None
        self.last_inside_time = None
        self.cmd_enter_time = None
        self.cmd_last_inside_time = None
        self.planner_activity_seen = False
        self.planner_activity_start_time = None
        self.start_time = rospy.Time.now()
        self.completed = False
        self.last_status_text = ""

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.cmd_sub = None
        self.bspline_sub = None
        self.force_complete_timer = None
        if self.use_cmd_reach_fallback:
            self.cmd_sub = rospy.Subscriber(self.cmd_topic, PositionCommand, self.cmd_callback, queue_size=1)
        if self.use_planner_activity_fallback:
            self.bspline_sub = rospy.Subscriber(self.planner_bspline_topic, Bspline, self.bspline_callback, queue_size=1)
            self.force_complete_timer = rospy.Timer(rospy.Duration(0.5), self.force_complete_timer_callback)

        rospy.loginfo(
            "[clean_uav_core] mission_progress_monitor started for %s with %d waypoints on %s",
            self.mission_name,
            self.waypoint_num,
            self.odom_topic,
        )
        if self.use_cmd_reach_fallback:
            rospy.loginfo(
                "[clean_uav_core] mission_progress_monitor command fallback enabled on %s (radius=%.2f)",
                self.cmd_topic,
                self.cmd_reach_radius,
            )
        if self.use_planner_activity_fallback:
            rospy.loginfo(
                "[clean_uav_core] mission_progress_monitor planner-activity fallback enabled on %s (timeout=%.1fs)",
                self.planner_bspline_topic,
                self.force_complete_timeout,
            )
        self.publish_status(f"{self.mission_name}: monitoring started")

    def publish_status(self, text):
        self.last_status_text = text
        self.status_pub.publish(String(data=text))

    def advance_to_waypoint(self, reached_index):
        while self.current_index <= reached_index and self.current_index < self.waypoint_num:
            current_wp = self.waypoints[self.current_index]
            rospy.loginfo(
                "[clean_uav_core] mission_progress_monitor reached waypoint %d/%d at (%.2f, %.2f, %.2f)",
                self.current_index + 1,
                self.waypoint_num,
                current_wp[0],
                current_wp[1],
                current_wp[2],
            )
            self.publish_status(
                f"{self.mission_name}: reached waypoint {self.current_index + 1}/{self.waypoint_num}"
            )
            self.current_index += 1

        self.enter_time = None

        if self.current_index >= self.waypoint_num:
            self.completed = True
            rospy.loginfo(
                "[clean_uav_core] mission_progress_monitor mission completed: %s",
                self.mission_name,
            )
            complete_text = f"{self.mission_name}: mission completed"
            self.publish_status(complete_text)
            rospy.sleep(0.05)
            self.publish_status(complete_text)

    def odom_callback(self, msg):
        if self.completed or self.current_index >= self.waypoint_num:
            return

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        reached_index = None
        for index in range(self.current_index, self.waypoint_num):
            current_wp = self.waypoints[index]
            distance = math.sqrt((px - current_wp[0]) ** 2 + (py - current_wp[1]) ** 2 + (pz - current_wp[2]) ** 2)
            if distance <= self.reach_radius:
                reached_index = index

        now = rospy.Time.now()
        if reached_index is not None:
            if self.enter_time is None:
                self.enter_time = now
            elif (now - self.enter_time).to_sec() >= self.hold_time:
                self.advance_to_waypoint(reached_index)
            self.last_inside_time = now
        else:
            if self.last_inside_time is None:
                self.enter_time = None
            elif (now - self.last_inside_time).to_sec() > self.exit_grace_time:
                self.enter_time = None
                self.last_inside_time = None

    def cmd_callback(self, msg):
        if self.completed or self.current_index >= self.waypoint_num:
            return

        reached_index = None
        for index in range(self.current_index, self.waypoint_num):
            current_wp = self.waypoints[index]
            distance = math.sqrt(
                (msg.position.x - current_wp[0]) ** 2
                + (msg.position.y - current_wp[1]) ** 2
                + (msg.position.z - current_wp[2]) ** 2
            )
            if distance <= self.cmd_reach_radius:
                reached_index = index

        now = rospy.Time.now()
        if reached_index is not None:
            if self.cmd_enter_time is None:
                self.cmd_enter_time = now
            self.cmd_last_inside_time = now
            if (now - self.cmd_enter_time).to_sec() >= self.hold_time:
                rospy.loginfo(
                    "[clean_uav_core] mission_progress_monitor command fallback reached waypoint index %d",
                    reached_index,
                )
                self.advance_to_waypoint(reached_index)
                self.cmd_enter_time = None
                self.cmd_last_inside_time = None
        else:
            if self.cmd_last_inside_time is None:
                self.cmd_enter_time = None
            elif (now - self.cmd_last_inside_time).to_sec() > self.exit_grace_time:
                self.cmd_enter_time = None
                self.cmd_last_inside_time = None

    def bspline_callback(self, _msg):
        if not self.planner_activity_seen:
            self.planner_activity_start_time = rospy.Time.now()
        self.planner_activity_seen = True

    def force_complete_timer_callback(self, _event):
        if self.completed or self.current_index >= self.waypoint_num:
            return
        if not self.use_planner_activity_fallback or not self.planner_activity_seen:
            return

        if self.planner_activity_start_time is None:
            return

        elapsed = (rospy.Time.now() - self.planner_activity_start_time).to_sec()
        if elapsed >= self.force_complete_timeout:
            rospy.logwarn(
                "[clean_uav_core] mission_progress_monitor force-complete after planner activity timeout since first bspline (%.2fs)",
                elapsed,
            )
            self.advance_to_waypoint(self.waypoint_num - 1)


if __name__ == "__main__":
    rospy.init_node("mission_progress_monitor")
    MissionProgressMonitor()
    rospy.spin()