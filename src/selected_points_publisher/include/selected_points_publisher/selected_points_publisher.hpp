/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 */

#ifndef SELECTED_POINTS_PUBLISHER_HPP
#define SELECTED_POINTS_PUBLISHER_HPP

#ifndef Q_MOC_RUN
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include "rviz/tool.h"
#include <QCursor>
#include <QObject>
#endif

#include <geometry_msgs/PoseStamped.h>
#include "rviz/default_plugin/tools/selection_tool.h"

namespace rviz_plugin_selected_points_publisher
{
class SelectedPointsPublisher : public rviz::SelectionTool
{
  Q_OBJECT
public:
  SelectedPointsPublisher();
  virtual ~SelectedPointsPublisher();
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);

public Q_SLOTS:
  void updateTopic();

protected:
  int processSelectedArea();
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher rviz_selected_publisher_, goal_publisher_;

  std::string tf_frame_;
  std::string selected_drones_topic_, goal_topic_;
  std::vector<geometry_msgs::PoseStamped> selected_drones_;

  bool selecting_;
  int num_selected_points_;
};
}

#endif