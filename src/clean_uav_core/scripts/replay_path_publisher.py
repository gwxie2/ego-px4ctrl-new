#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand


class DroneReplayPublisher:
    def __init__(self, namespace: str, path_keep_every: int):
        self.namespace = namespace.strip("/")
        self.path_keep_every = max(1, int(path_keep_every))
        self.actual_count = 0
        self.command_count = 0

        self.actual_path = Path()
        self.command_path = Path()
        self.actual_path.header.frame_id = "world"
        self.command_path.header.frame_id = "world"

        actual_pose_topic = f"/{self.namespace}/replay/actual_pose"
        actual_path_topic = f"/{self.namespace}/replay/actual_path"
        command_pose_topic = f"/{self.namespace}/replay/command_pose"
        command_path_topic = f"/{self.namespace}/replay/command_path"

        self.actual_pose_pub = rospy.Publisher(actual_pose_topic, PoseStamped, queue_size=50)
        self.actual_path_pub = rospy.Publisher(actual_path_topic, Path, queue_size=10, latch=True)
        self.command_pose_pub = rospy.Publisher(command_pose_topic, PoseStamped, queue_size=50)
        self.command_path_pub = rospy.Publisher(command_path_topic, Path, queue_size=10, latch=True)

        rospy.Subscriber(f"/{self.namespace}/odom", Odometry, self._odom_callback, queue_size=200)
        rospy.Subscriber(f"/{self.namespace}/position_cmd", PositionCommand, self._cmd_callback, queue_size=200)

    def _odom_callback(self, msg: Odometry):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose

        self.actual_pose_pub.publish(pose_msg)

        self.actual_count += 1
        if self.actual_count % self.path_keep_every != 0:
            return

        self.actual_path.header = pose_msg.header
        self.actual_path.poses.append(pose_msg)
        self.actual_path_pub.publish(self.actual_path)

    def _cmd_callback(self, msg: PositionCommand):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x = msg.position.x
        pose_msg.pose.position.y = msg.position.y
        pose_msg.pose.position.z = msg.position.z
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.command_pose_pub.publish(pose_msg)

        self.command_count += 1
        if self.command_count % self.path_keep_every != 0:
            return

        self.command_path.header = pose_msg.header
        self.command_path.poses.append(pose_msg)
        self.command_path_pub.publish(self.command_path)


def main():
    rospy.init_node("replay_path_publisher")

    drone_names = rospy.get_param("~drone_names", ["drone_0", "drone_1", "drone_2"])
    path_keep_every = rospy.get_param("~path_keep_every", 5)

    publishers = [DroneReplayPublisher(name, path_keep_every) for name in drone_names]
    rospy.loginfo("[clean_uav_core] replay_path_publisher started for %d drones", len(publishers))
    rospy.spin()


if __name__ == "__main__":
    main()