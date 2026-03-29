#!/usr/bin/env bash

# Phase-1 runtime environment bootstrap for the clean-room workspace.
# Usage:
#   source /home/guanwen/XTDrone/cleanroom_ws/tools/source_phase1_env.sh

_phase1_this_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_phase1_ws_root="$(cd "${_phase1_this_dir}/.." && pwd)"
_xtdrone_root="$(cd "${_phase1_ws_root}/.." && pwd)"
_user_catkin_ws="/home/guanwen/catkin_ws"
_user_gazebo_src="${_user_catkin_ws}/src/gazebo_ros_pkgs"

if [ -n "${PHASE1_PX4_ROOT}" ] && [ -d "${PHASE1_PX4_ROOT}" ]; then
  _phase1_px4_root="${PHASE1_PX4_ROOT}"
elif [ -d "/home/guanwen/PX4_Firmware" ]; then
  _phase1_px4_root="/home/guanwen/PX4_Firmware"
else
  _phase1_px4_root="${_xtdrone_root}/PX4_Firmware"
fi

if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

if [ -f "${_user_catkin_ws}/devel/setup.bash" ]; then
  source "${_user_catkin_ws}/devel/setup.bash"
fi

if [ -f "${_phase1_ws_root}/devel/setup.bash" ]; then
  source "${_phase1_ws_root}/devel/setup.bash"
fi

if [ -d "${_user_catkin_ws}/devel" ]; then
  export CMAKE_PREFIX_PATH="${_phase1_ws_root}/devel:${_user_catkin_ws}/devel:${CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${_user_catkin_ws}/devel/lib:${LD_LIBRARY_PATH}"
  export PATH="${_user_catkin_ws}/devel/bin:${PATH}"
  export PYTHONPATH="${_user_catkin_ws}/devel/lib/python3/dist-packages:${PYTHONPATH}"
fi

if [ -d "${_user_gazebo_src}" ]; then
  export ROS_PACKAGE_PATH="${_user_gazebo_src}/gazebo_dev:${_user_gazebo_src}/gazebo_msgs:${_user_gazebo_src}/gazebo_ros:${_user_gazebo_src}/gazebo_plugins:${_user_gazebo_src}/gazebo_ros_control:${_user_gazebo_src}/gazebo_ros_pkgs:${ROS_PACKAGE_PATH}"
fi

export ROS_PACKAGE_PATH="${_phase1_px4_root}:${_phase1_px4_root}/Tools/sitl_gazebo:${ROS_PACKAGE_PATH}"

if [ -d "${_phase1_px4_root}/Tools/sitl_gazebo/models" ]; then
  export GAZEBO_MODEL_PATH="${_phase1_px4_root}/Tools/sitl_gazebo/models:${GAZEBO_MODEL_PATH}"
fi

if [ -d "${_phase1_px4_root}/build/px4_sitl_default/build_gazebo" ]; then
  export GAZEBO_PLUGIN_PATH="${_phase1_px4_root}/build/px4_sitl_default/build_gazebo:${GAZEBO_PLUGIN_PATH}"
  export LD_LIBRARY_PATH="${_phase1_px4_root}/build/px4_sitl_default/build_gazebo:${LD_LIBRARY_PATH}"
fi

echo "[phase1_env] cleanroom_ws=${_phase1_ws_root}"
if [ -f "${_user_catkin_ws}/devel/setup.bash" ]; then
  echo "[phase1_env] overlay catkin_ws=${_user_catkin_ws}"
fi
echo "[phase1_env] XTDrone root=${_xtdrone_root}"
echo "[phase1_env] PX4 root=${_phase1_px4_root}"
echo "[phase1_env] ROS_PACKAGE_PATH updated for px4 and mavlink_sitl_gazebo"
echo "[phase1_env] GAZEBO_MODEL_PATH prepended with ${_phase1_px4_root}/Tools/sitl_gazebo/models"