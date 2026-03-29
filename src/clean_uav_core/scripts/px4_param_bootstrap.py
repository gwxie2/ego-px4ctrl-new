#!/usr/bin/env python3
# 本文件用于在PX4参数系统中设置COM_RCL_EXCEPT参数。本参数是PX4中一个重要的参数，控制着飞控在接收到异常RC信号时的行为。通过设置COM_RCL_EXCEPT=4，我们可以让飞控在接收到异常RC信号时进入降落模式，从而提高飞行安全性。这个脚本会尝试多次设置参数，并在每次尝试前拉取最新的参数缓存，以确保参数能够成功设置。

import rospy
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamGet, ParamGetRequest, ParamPull, ParamPullRequest, ParamSet, ParamSetRequest


class Px4ParamBootstrap:
    def __init__(self):
        self.service_name = rospy.get_param("~service_name", "/mavros/param/set") # mavros设置参数
        self.get_service_name = rospy.get_param("~get_service_name", "/mavros/param/get")
        self.pull_service_name = rospy.get_param("~pull_service_name", "/mavros/param/pull") # mavros拉取参数缓存
        self.param_name = rospy.get_param("~param_name", "COM_RCL_EXCEPT")
        self.integer_value = int(rospy.get_param("~integer_value", 4))
        self.real_value = float(rospy.get_param("~real_value", 0.0))
        self.retry_count = int(rospy.get_param("~retry_count", 20))
        self.retry_interval = float(rospy.get_param("~retry_interval", 1.0))

    def _get_param(self, proxy):
        request = ParamGetRequest(param_id=self.param_name)
        return proxy(request)

    def _pull_params(self, proxy):
        try:
            response = proxy(ParamPullRequest(force_pull=True))
            rospy.loginfo(
                "[clean_uav_core] px4_param_bootstrap pulled parameter cache: success=%s count=%s",
                response.success,
                response.param_received,
            )
            return response.success
        except rospy.ServiceException as exc:
            rospy.logwarn(
                "[clean_uav_core] px4_param_bootstrap pull error: %s",
                exc,
            )
            return False

    def run(self):
        rospy.loginfo(
            "[clean_uav_core] px4_param_bootstrap waiting for %s / %s / %s for %s=%d",
            self.service_name,
            self.get_service_name,
            self.pull_service_name,
            self.param_name,
            self.integer_value,
        )
        rospy.wait_for_service(self.service_name)
        rospy.wait_for_service(self.get_service_name)
        rospy.wait_for_service(self.pull_service_name)

        set_proxy = rospy.ServiceProxy(self.service_name, ParamSet)
        get_proxy = rospy.ServiceProxy(self.get_service_name, ParamGet)
        pull_proxy = rospy.ServiceProxy(self.pull_service_name, ParamPull)

        self._pull_params(pull_proxy)

        request = ParamSetRequest()
        request.param_id = self.param_name
        request.value = ParamValue(integer=self.integer_value, real=self.real_value)

        for attempt in range(1, self.retry_count + 1):
            if rospy.is_shutdown():
                return
            try:
                current = self._get_param(get_proxy)
                if current.success:
                    if current.value.integer == self.integer_value and current.value.real == self.real_value: # 如果参数已经设置为目标值，则直接返回成功
                        rospy.loginfo(
                            "[clean_uav_core] px4_param_bootstrap found %s already set to integer=%d real=%.3f on attempt %d",
                            self.param_name,
                            current.value.integer,
                            current.value.real,
                            attempt,
                        )
                        return
                else:
                    rospy.logwarn(
                        "[clean_uav_core] px4_param_bootstrap could not read %s on attempt %d",
                        self.param_name,
                        attempt,
                    )
                    self._pull_params(pull_proxy)
                    rospy.sleep(self.retry_interval)
                    continue

                response = set_proxy(request)
                if response.success:
                    rospy.loginfo(
                        "[clean_uav_core] px4_param_bootstrap set %s=%d on attempt %d",
                        self.param_name,
                        self.integer_value,
                        attempt,
                    )
                    return
                rospy.logwarn(
                    "[clean_uav_core] px4_param_bootstrap attempt %d failed for %s",
                    attempt,
                    self.param_name,
                )
            except rospy.ServiceException as exc:
                rospy.logwarn(
                    "[clean_uav_core] px4_param_bootstrap service error on attempt %d: %s",
                    attempt,
                    exc,
                )

            rospy.sleep(self.retry_interval)

        rospy.logerr(
            "[clean_uav_core] px4_param_bootstrap exhausted retries for %s",
            self.param_name,
        )


if __name__ == "__main__":
    rospy.init_node("px4_param_bootstrap")
    Px4ParamBootstrap().run()