#!/usr/bin/env python3
"""
Swarm Trajectory Trigger

支持任意数量无人机的同步轨迹触发器。
等待所有 UAV 里程计就绪后，同步发布轨迹开始触发消息。
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class SwarmTrajTrigger:
    """同步轨迹触发器 - 支持任意数量 UAV"""

    def __init__(self):
        # ===== 参数读取 =====
        self.num_uavs = int(rospy.get_param("~num_uavs", 2))
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.delay = float(rospy.get_param("~delay", 12.0))
        self.repeat = int(rospy.get_param("~repeat", 8))
        self.rate = float(rospy.get_param("~rate", 3.0))
        self.wait_odom_timeout = float(rospy.get_param("~wait_odom_timeout", 30.0))

        # 话题命名空间前缀
        self.namespace_prefix = rospy.get_param("~namespace_prefix", "drone")

        rospy.loginfo(f"[swarm_traj_trigger] Initializing for {self.num_uavs} UAVs")

        # ===== 动态创建 UAV 配置 =====
        self.uav_configs = []
        self.publishers = []
        self.odom_subs = []
        self.latest_poses = [None] * self.num_uavs

        for drone_id in range(self.num_uavs):
            odom_topic = f"/{self.namespace_prefix}_{drone_id}/odom"
            trigger_topic = f"/{self.namespace_prefix}_{drone_id}/traj_start_trigger"

            self.uav_configs.append({
                "drone_id": drone_id,
                "name": f"{self.namespace_prefix}_{drone_id}",
                "odom_topic": odom_topic,
                "trigger_topic": trigger_topic,
                "latest_pose": None,
            })

            # 创建 Publisher 到触发话题
            self.publishers.append(rospy.Publisher(trigger_topic, PoseStamped, queue_size=10))

            # 创建 Subscriber 到里程计话题
            self.odom_subs.append(
                rospy.Subscriber(
                    odom_topic,
                    Odometry,
                    self._make_odom_callback(drone_id),
                    queue_size=1,
                )
            )

            rospy.loginfo(f"[swarm_traj_trigger] drone_{drone_id}: odom={odom_topic}, trigger={trigger_topic}")

    def _make_odom_callback(self, drone_id):
        """创建里程计回调函数（闭包）"""
        def callback(msg):
            self.latest_poses[drone_id] = msg.pose.pose
        return callback

    def _all_ready(self):
        """检查所有 UAV 里程计是否就绪"""
        ready_count = sum(1 for pose in self.latest_poses if pose is not None)
        if ready_count < self.num_uavs:
            rospy.loginfo_throttle(
                5.0,
                f"[swarm_traj_trigger] Odometry ready: {ready_count}/{self.num_uavs}"
            )
        return ready_count == self.num_uavs

    def run(self):
        """主循环"""
        rospy.loginfo(f"[swarm_traj_trigger] Waiting {self.delay}s before synchronized trigger")
        rospy.sleep(self.delay)

        # 等待所有 UAV 里程计就绪
        start_wait = rospy.Time.now()
        wait_rate = rospy.Rate(20.0)

        while not rospy.is_shutdown() and not self._all_ready():
            waited = (rospy.Time.now() - start_wait).to_sec()

            # 超时检查
            if self.wait_odom_timeout > 0.0 and waited > self.wait_odom_timeout:
                ready_count = sum(1 for pose in self.latest_poses if pose is not None)
                rospy.logwarn(
                    f"[swarm_traj_trigger] Timeout waiting odom ({waited:.1f}s), "
                    f"only {ready_count}/{self.num_uavs} UAVs ready. Continuing anyway..."
                )
                break

            rospy.logwarn_throttle(
                2.0,
                f"[swarm_traj_trigger] Waiting for all UAV odometry ({waited:.1f}s elapsed)"
            )
            wait_rate.sleep()

        if rospy.is_shutdown():
            return

        rospy.loginfo(f"[swarm_traj_trigger] All UAVs ready! Starting synchronized trigger...")

        # 同步发布触发信号
        pub_rate = rospy.Rate(self.rate if self.rate > 0.0 else 2.0)

        for i in range(self.repeat):
            if rospy.is_shutdown():
                return

            now = rospy.Time.now()

            # 为每个 UAV 发布触发消息
            for drone_id in range(self.num_uavs):
                if self.latest_poses[drone_id] is None:
                    rospy.logwarn(
                        f"[swarm_traj_trigger] drone_{drone_id} odometry not available, skipping..."
                    )
                    continue

                msg = PoseStamped()
                msg.header.stamp = now
                msg.header.frame_id = self.frame_id
                msg.pose = self.latest_poses[drone_id]

                self.publishers[drone_id].publish(msg)

            rospy.loginfo(
                f"[swarm_traj_trigger] Published synchronized trigger {i+1}/{self.repeat}"
            )
            pub_rate.sleep()

        rospy.loginfo(f"[swarm_traj_trigger] Completed {self.repeat} trigger(s)")


def main():
    """主函数"""
    rospy.init_node("swarm_traj_trigger", anonymous=False)

    try:
        trigger = SwarmTrajTrigger()
        trigger.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[swarm_traj_trigger] Shutting down")
    except Exception as e:
        rospy.logerr(f"[swarm_traj_trigger] Error: {e}")
        raise


if __name__ == "__main__":
    main()
