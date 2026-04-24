#!/usr/bin/env python3
"""在 MAVROS 状态稳定后，发布起飞或降落命令。

它的职责是把“能不能发起飞/降落”这件事单独抽出来：先确认飞控已连接、
未处于 armed 状态，再按固定频率重复发命令，降低单次消息丢失带来的风险。
"""

import rospy
from mavros_msgs.msg import State
from quadrotor_msgs.msg import TakeoffLand


class TakeoffLandTrigger:
    def __init__(self):
        self.command = rospy.get_param("~command", "takeoff").strip().lower()
        self.topic = rospy.get_param("~topic", "/px4ctrl/takeoff_land")
        self.delay = float(rospy.get_param("~delay", 5.0))
        self.repeat = int(rospy.get_param("~repeat", 5))
        self.rate_hz = float(rospy.get_param("~rate", 2.0))
        self.state_topic = rospy.get_param("~state_topic", "mavros/state")
        self.connection_timeout = float(rospy.get_param("~connection_timeout", 30.0))
        self.stable_duration = float(rospy.get_param("~stable_duration", 2.0))

        self.publisher = rospy.Publisher(self.topic, TakeoffLand, queue_size=10, latch=True)
        self.latest_state = None
        self.connected_since = None

        self.state_subscriber = rospy.Subscriber(
            self.state_topic,
            State,
            self._state_callback,
            queue_size=10,
        )

        if self.command == "takeoff":
            self.command_value = TakeoffLand.TAKEOFF
        elif self.command == "land":
            self.command_value = TakeoffLand.LAND
        else:
            raise ValueError("Unsupported command: {}".format(self.command))

    def _state_callback(self, msg):
        # 只记录最新飞控状态，并维持“连接且未 armed”的稳定计时。
        self.latest_state = msg

        if msg.connected and not msg.armed:
            if self.connected_since is None:
                self.connected_since = rospy.Time.now()
        else:
            self.connected_since = None

    def _wait_until_ready(self):
        rospy.loginfo(
            "[clean_uav_core] takeoff_land_trigger waiting for MAVROS state on %s",
            self.state_topic,
        )

        start_time = rospy.Time.now()
        wait_rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if self.latest_state is not None and self.latest_state.connected and not self.latest_state.armed:
                # 只有连接态和非 armed 状态持续稳定一段时间后，才开始发起飞/降落命令。
                stable_time = 0.0
                if self.connected_since is not None:
                    stable_time = (rospy.Time.now() - self.connected_since).to_sec()

                if stable_time >= self.stable_duration:
                    rospy.loginfo(
                        "[clean_uav_core] takeoff_land_trigger readiness gate passed after %.1fs",
                        elapsed,
                    )
                    return True

                rospy.loginfo_throttle(
                    1.0,
                    "[clean_uav_core] takeoff_land_trigger connected and disarmed, waiting %.1fs/%.1fs stability",
                    stable_time,
                    self.stable_duration,
                )
            else:
                rospy.logwarn_throttle(
                    2.0,
                    "[clean_uav_core] takeoff_land_trigger waiting for connected=true and armed=false on %s",
                    self.state_topic,
                )

            if self.connection_timeout > 0.0 and elapsed >= self.connection_timeout:
                rospy.logerr(
                    "[clean_uav_core] takeoff_land_trigger timed out waiting %.1fs for MAVROS readiness on %s",
                    self.connection_timeout,
                    self.state_topic,
                )
                return False

            wait_rate.sleep()

    def run(self):
        if not self._wait_until_ready():
            return

        rospy.loginfo(
            "[clean_uav_core] takeoff_land_trigger readiness satisfied, waiting %.2fs before publishing '%s' to %s",
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