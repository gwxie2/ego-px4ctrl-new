#!/usr/bin/env python3
"""Wait until a Gazebo model is stable in model_states, then exec a command."""

import argparse
import os
import sys

import rospy
from gazebo_msgs.msg import ModelStates


class ModelReadinessGate:
    def __init__(self, model_name, topic_name, timeout, stable_duration, post_ready_delay):
        self.model_name = model_name
        self.topic_name = topic_name
        self.timeout = timeout
        self.stable_duration = stable_duration
        self.post_ready_delay = post_ready_delay
        self.model_visible = False
        self.visible_since = None

        self.subscriber = rospy.Subscriber(
            self.topic_name,
            ModelStates,
            self._model_states_callback,
            queue_size=1,
        )

    def _model_states_callback(self, msg):
        now = rospy.Time.now()
        if self.model_name in msg.name:
            if not self.model_visible:
                self.model_visible = True
                self.visible_since = now
        else:
            self.model_visible = False
            self.visible_since = None

    def wait(self):
        start_time = rospy.Time.now()
        rate = rospy.Rate(10.0)

        rospy.loginfo(
            "[clean_uav_core] wait_for_model_and_exec waiting for model '%s' on %s",
            self.model_name,
            self.topic_name,
        )

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if self.model_visible and self.visible_since is not None:
                stable_time = (rospy.Time.now() - self.visible_since).to_sec()
                if stable_time < self.stable_duration:
                    rospy.loginfo_throttle(
                        1.0,
                        "[clean_uav_core] wait_for_model_and_exec saw model '%s', stabilizing %.1f/%.1fs",
                        self.model_name,
                        stable_time,
                        self.stable_duration,
                    )
                else:
                    if self.post_ready_delay > 0.0:
                        rospy.loginfo(
                            "[clean_uav_core] wait_for_model_and_exec model '%s' ready, extra sleep %.1fs",
                            self.model_name,
                            self.post_ready_delay,
                        )
                        rospy.sleep(self.post_ready_delay)
                    return True

            if self.timeout > 0.0 and elapsed >= self.timeout:
                rospy.logerr(
                    "[clean_uav_core] wait_for_model_and_exec timed out after %.1fs waiting for '%s'",
                    self.timeout,
                    self.model_name,
                )
                return False

            rospy.logwarn_throttle(
                2.0,
                "[clean_uav_core] wait_for_model_and_exec still waiting for '%s' (%.1fs elapsed)",
                self.model_name,
                elapsed,
            )
            rate.sleep()


def parse_args(argv):
    if "--" not in argv:
        raise ValueError("missing '--' separator before command")

    separator_index = argv.index("--")
    gate_args = argv[:separator_index]
    command = argv[separator_index + 1 :]
    if not command:
        raise ValueError("missing command after '--' separator")

    parser = argparse.ArgumentParser(description="Wait for Gazebo model readiness before exec")
    parser.add_argument("--model-name", required=True)
    parser.add_argument("--topic-name", default="/gazebo/model_states")
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--stable-duration", type=float, default=2.0)
    parser.add_argument("--post-ready-delay", type=float, default=2.0)

    return parser.parse_args(gate_args), command


def main():
    try:
        args, command = parse_args(sys.argv[1:])
    except ValueError as error:
        print(f"[clean_uav_core] wait_for_model_and_exec argument error: {error}", file=sys.stderr)
        return 2

    rospy.init_node("wait_for_model_and_exec", anonymous=True, disable_signals=True)
    gate = ModelReadinessGate(
        model_name=args.model_name,
        topic_name=args.topic_name,
        timeout=args.timeout,
        stable_duration=args.stable_duration,
        post_ready_delay=args.post_ready_delay,
    )

    if not gate.wait():
        return 1

    rospy.loginfo(
        "[clean_uav_core] wait_for_model_and_exec launching command: %s",
        " ".join(command),
    )
    os.execvp(command[0], command)
    return 0


if __name__ == "__main__":
    sys.exit(main())