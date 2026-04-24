#!/usr/bin/env python3
"""等待无人机达到稳定起飞状态后，再执行后续命令。

这个包装器把“起飞稳定性门控”和“真正的业务命令”拆开，避免 launch 直接拉起
下游程序时因为 odom 尚未稳定而提前进入错误状态。
"""

import argparse
import os
import signal
import subprocess
import sys

import rospy
from nav_msgs.msg import Odometry


WORKSPACE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")
)


def _resolve_workspace_path(path_text):
    path_text = str(path_text).strip()
    if not path_text:
        return ""
    expanded = os.path.expanduser(path_text)
    if os.path.isabs(expanded):
        return os.path.normpath(expanded)
    return os.path.normpath(os.path.join(WORKSPACE_ROOT, expanded))


def _bool_arg(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class OdomReadinessTracker:
    def __init__(self, odom_topic, z_threshold):
        self.odom_topic = rospy.resolve_name(odom_topic)
        self.z_threshold = z_threshold
        self.above_threshold_since = None
        self.subscriber = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self._odom_callback,
            queue_size=1,
        )

    def _odom_callback(self, msg):
        # 只有连续稳定高于阈值，才认为起飞状态已经真正稳定下来。
        now = rospy.Time.now()
        z = msg.pose.pose.position.z
        if z >= self.z_threshold:
            if self.above_threshold_since is None:
                self.above_threshold_since = now
        else:
            self.above_threshold_since = None


class OdomReadinessGate:
    def __init__(
        self,
        odom_topic,
        z_threshold,
        timeout,
        stable_duration,
        post_ready_delay,
        wait_all_ready=False,
        swarm_size=1,
        swarm_odom_template="/drone_%d/odom",
    ):
        self.odom_topic = rospy.resolve_name(odom_topic)
        self.timeout = timeout
        self.stable_duration = stable_duration
        self.post_ready_delay = post_ready_delay
        self.wait_all_ready = wait_all_ready
        self.swarm_size = max(1, int(swarm_size))
        self.trackers = {self.odom_topic: OdomReadinessTracker(self.odom_topic, z_threshold)}

        if self.wait_all_ready:
            for drone_id in range(self.swarm_size):
                topic_name = rospy.resolve_name(swarm_odom_template % drone_id)
                if topic_name in self.trackers:
                    continue
                self.trackers[topic_name] = OdomReadinessTracker(topic_name, z_threshold)

    def _stable_time(self, tracker, now):
        if tracker.above_threshold_since is None:
            return 0.0
        return (now - tracker.above_threshold_since).to_sec()

    def _all_ready(self, now):
        for tracker in self.trackers.values():
            if self._stable_time(tracker, now) < self.stable_duration:
                return False
        return True

    def wait(self):
        start_time = rospy.Time.now()
        rate = rospy.Rate(5.0)

        if self.wait_all_ready:
            rospy.loginfo(
                "[clean_uav_core] wait_for_takeoff_and_exec waiting for %d UAVs to reach ready state, trigger topic=%s",
                len(self.trackers),
                self.odom_topic,
            )
        else:
            rospy.loginfo(
                "[clean_uav_core] wait_for_takeoff_and_exec waiting for %s to reach Z >= %.2f",
                self.odom_topic,
                self.trackers[self.odom_topic].z_threshold,
            )

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            own_tracker = self.trackers[self.odom_topic]
            stable_time = self._stable_time(own_tracker, now)

            # 单机/多机都先看“主触发无人机”是否已经稳定；多机模式下再附加检查所有伙伴。
            if stable_time >= self.stable_duration:
                if self.wait_all_ready and not self._all_ready(now):
                    not_ready_topics = []
                    for topic_name, tracker in self.trackers.items():
                        ready_time = self._stable_time(tracker, now)
                        if ready_time < self.stable_duration:
                            not_ready_topics.append(f"{topic_name}:{ready_time:.1f}/{self.stable_duration:.1f}s")
                    rospy.loginfo_throttle(
                        1.0,
                        "[clean_uav_core] wait_for_takeoff_and_exec own UAV ready, waiting for swarm peers: %s",
                        ", ".join(not_ready_topics),
                    )
                else:
                    if self.post_ready_delay > 0.0:
                        rospy.loginfo(
                            "[clean_uav_core] readiness confirmed on %s, extra sleep %.1fs before exec",
                            self.odom_topic,
                            self.post_ready_delay,
                        )
                        rospy.sleep(self.post_ready_delay)
                    rospy.loginfo(
                        "[clean_uav_core] UAV readiness satisfied for %s. Triggering command!",
                        self.odom_topic,
                    )
                    return True
            else:
                rospy.loginfo_throttle(
                    1.0,
                    "[clean_uav_core] wait_for_takeoff_and_exec stabilizing %s above %.2fm: %.1f/%.1fs",
                    self.odom_topic,
                    own_tracker.z_threshold,
                    stable_time,
                    self.stable_duration,
                )

            elapsed = (now - start_time).to_sec()
            if self.timeout > 0.0 and elapsed >= self.timeout:
                rospy.logwarn(
                    "[clean_uav_core] wait_for_takeoff_and_exec timed out after %.1fs on %s",
                    self.timeout,
                    self.odom_topic,
                )
                return False

            rate.sleep()
        return False


def run_command(command, log_file):
    log_file = _resolve_workspace_path(log_file)
    if not log_file:
        os.execvp(command[0], command)
        return 0

    log_dir = os.path.dirname(log_file)
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
    rospy.loginfo("[clean_uav_core] wait_for_takeoff_and_exec logging child output to %s", log_file)

    child = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    def _forward_signal(signum, _frame):
        if child.poll() is None:
            child.send_signal(signum)

    signal.signal(signal.SIGINT, _forward_signal)
    signal.signal(signal.SIGTERM, _forward_signal)

    with open(log_file, "a", encoding="utf-8") as handle:
        handle.write(f"\n===== launch {rospy.Time.now().to_sec():.3f} =====\n")
        for line in child.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            handle.write(line)
        return child.wait()

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
    parser.add_argument("--wait-all-ready", default="false")
    parser.add_argument("--swarm-size", type=int, default=1)
    parser.add_argument("--swarm-odom-template", default="/drone_%d/odom")
    parser.add_argument("--log-file", nargs="?", const="", default="")

    return parser.parse_args(gate_args), command

def main():
    try:
        args, command = parse_args(sys.argv[1:])
    except ValueError as error:
        print(f"[clean_uav_core] wait_for_takeoff_and_exec argument error: {error}", file=sys.stderr)
        return 2

    rospy.init_node(
        "wait_for_takeoff_and_exec",
        anonymous=True,
        disable_signals=True,
        argv=[sys.argv[0]],
    )

    # 显式保留 argv[0]，避免 launch-prefix 包裹后把 wrapper 节点名和真正子进程混在一起。
    gate = OdomReadinessGate(
        odom_topic=args.odom_topic,
        z_threshold=args.z_threshold,
        timeout=args.timeout,
        stable_duration=args.stable_duration,
        post_ready_delay=args.post_ready_delay,
        wait_all_ready=_bool_arg(args.wait_all_ready),
        swarm_size=args.swarm_size,
        swarm_odom_template=args.swarm_odom_template,
    )

    if not gate.wait():
        return 1

    rospy.loginfo(
        "[clean_uav_core] wait_for_takeoff_and_exec launching command: %s",
        " ".join(command),
    )
    return run_command(command, args.log_file.strip())

if __name__ == "__main__":
    main()
