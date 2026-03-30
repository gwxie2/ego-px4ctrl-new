#!/usr/bin/env python3
"""Wait until UAV odometry indicates stable takeoff, then exec a command."""

import argparse
import os
import sys

import rospy
from nav_msgs.msg import Odometry

class OdomReadinessGate:
    def __init__(self, odom_topic, z_threshold, timeout, stable_duration, post_ready_delay):
        self.odom_topic = odom_topic
        self.z_threshold = z_threshold
        self.timeout = timeout
        self.stable_duration = stable_duration
        self.post_ready_delay = post_ready_delay
        self.above_threshold_since = None
        
        self.subscriber = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self._odom_callback,
            queue_size=1,
        )

    def _odom_callback(self, msg):
        now = rospy.Time.now()
        z = msg.pose.pose.position.z
        if z >= self.z_threshold:
            if self.above_threshold_since is None:
                self.above_threshold_since = now
        else:
            self.above_threshold_since = None

    def wait(self):
        start_time = rospy.Time.now()
        rate = rospy.Rate(5.0)

        rospy.loginfo(
            "[clean_uav_core] wait_for_takeoff_and_exec waiting for %s to reach Z >= %.2f",
            self.odom_topic,
            self.z_threshold
        )

        while not rospy.is_shutdown():
            if self.above_threshold_since is not None:
                stable_time = (rospy.Time.now() - self.above_threshold_since).to_sec()
                if stable_time >= self.stable_duration:
                    if self.post_ready_delay > 0.0:
                        rospy.loginfo(
                            "[clean_uav_core] takeoff confirmed on %s, extra sleep %.1fs before exec",
                            self.odom_topic,
                            self.post_ready_delay,
                        )
                        rospy.sleep(self.post_ready_delay)
                    rospy.loginfo(
                        "[clean_uav_core] UAV achieved stable takeoff Z >= %.2f for %.1fs. Triggering exec!",
                        self.z_threshold,
                        self.stable_duration,
                    )
                    return True

                rospy.loginfo_throttle(
                    1.0,
                    "[clean_uav_core] wait_for_takeoff_and_exec stabilizing %s above %.2fm: %.1f/%.1fs",
                    self.odom_topic,
                    self.z_threshold,
                    stable_time,
                    self.stable_duration,
                )

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if self.timeout > 0.0 and elapsed >= self.timeout:
                rospy.logwarn(
                    "[clean_uav_core] wait_for_takeoff_and_exec timed out after %.1fs on %s",
                    self.timeout,
                    self.odom_topic,
                )
                return False

            rate.sleep()
        return False

def parse_args(argv):
    if "--" not in argv:
        raise ValueError("missing '--' separator before command")

    separator_index = argv.index("--")
    gate_args = argv[:separator_index]
    command = argv[separator_index + 1 :]
    if not command:
        raise ValueError("missing command after '--' separator")

    parser = argparse.ArgumentParser(description="Wait for UAV takeoff before exec")
    parser.add_argument("--odom-topic", required=True)
    parser.add_argument("--z-threshold", type=float, default=0.7)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--stable-duration", type=float, default=1.5)
    parser.add_argument("--post-ready-delay", type=float, default=1.0)

    return parser.parse_args(gate_args), command

def main():
    try:
        args, command = parse_args(sys.argv[1:])
    except ValueError as error:
        print(f"[clean_uav_core] wait_for_takeoff_and_exec argument error: {error}", file=sys.stderr)
        return 2

    rospy.init_node("wait_for_takeoff_and_exec", anonymous=True, disable_signals=True)
    
    gate = OdomReadinessGate(
        odom_topic=args.odom_topic,
        z_threshold=args.z_threshold,
        timeout=args.timeout,
        stable_duration=args.stable_duration,
        post_ready_delay=args.post_ready_delay,
    )

    if not gate.wait():
        return 1

    rospy.loginfo(
        "[clean_uav_core] wait_for_takeoff_and_exec launching command: %s",
        " ".join(command),
    )
    os.execvp(command[0], command)
    return 0

if __name__ == "__main__":
    main()
