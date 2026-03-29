#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/guanwen/XTDrone/cleanroom_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
timeout --foreground 65s roslaunch clean_uav_core phase4_dual_uav_dynamic_goal_stack.launch start_sim:=true enable_vins:=false gui:=false use_rviz:=false > /home/guanwen/XTDrone/cleanroom_ws/tools/ab_logs/phase4_mavros_bridgeodom60_3.log 2>&1 || true
