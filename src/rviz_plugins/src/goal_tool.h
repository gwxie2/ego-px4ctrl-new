/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 */

#ifndef RVIZ_GOAL_TOOL_H
#define RVIZ_GOAL_TOOL_H

#ifndef Q_MOC_RUN
# include <QObject>
# include <ros/ros.h>
# include "pose_tool.h"
#endif

namespace rviz
{
class StringProperty;
class IntProperty;

class Goal3DTool : public Pose3DTool
{
Q_OBJECT
public:
  Goal3DTool();
  virtual ~Goal3DTool() {}
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double z, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_goal_, pub_droneID_goal_;

  StringProperty* topic_property_;
  StringProperty* goalset_topic_property_;
  IntProperty* goalset_drone_id_property_;
};
}

#endif