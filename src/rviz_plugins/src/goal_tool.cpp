/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 */

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/string_property.h"

#include <quadrotor_msgs/GoalSet.h>

#include "goal_tool.h"

namespace rviz
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new StringProperty("Pose Topic", "goal",
                                       "The topic on which to publish PoseStamped goals.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
  goalset_topic_property_ = new StringProperty("GoalSet Topic", "/goal_with_id",
                                               "The topic on which to publish GoalSet goals.",
                                               getPropertyContainer(), SLOT(updateTopic()), this);
  goalset_drone_id_property_ = new IntProperty("GoalSet Drone ID", 0,
                                               "Drone ID used when publishing GoalSet goals.",
                                               getPropertyContainer(), SLOT(updateTopic()), this);
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
  pub_droneID_goal_ = nh_.advertise<quadrotor_msgs::GoalSet>(goalset_topic_property_->getStdString(), 1);
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> pose = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(pose, goal);
  pub_goal_.publish(goal);

  quadrotor_msgs::GoalSet goal_with_id;
  goal_with_id.drone_id = goalset_drone_id_property_->getInt();
  goal_with_id.goal[0] = x;
  goal_with_id.goal[1] = y;
  goal_with_id.goal[2] = z;
  pub_droneID_goal_.publish(goal_with_id);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::Goal3DTool, rviz::Tool)