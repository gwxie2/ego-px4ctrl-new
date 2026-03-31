/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 */

#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/geometry.h"

#include "selected_points_publisher/selected_points_publisher.hpp"

#include <QVariant>
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>

namespace rviz_plugin_selected_points_publisher
{
SelectedPointsPublisher::SelectedPointsPublisher()
  : private_node_handle_("~")
{
  updateTopic();
}

SelectedPointsPublisher::~SelectedPointsPublisher()
{
}

void SelectedPointsPublisher::updateTopic()
{
  private_node_handle_.param("frame_id", tf_frame_, std::string("/base_link"));
  private_node_handle_.param("selected_drones_topic", selected_drones_topic_, std::string("/rviz_selected_drones"));
  private_node_handle_.param("goal_topic", goal_topic_, std::string("/goal"));

  rviz_selected_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(selected_drones_topic_.c_str(), 100);
  goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(goal_topic_.c_str(), 100);
  num_selected_points_ = 0;
}

int SelectedPointsPublisher::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  int flags = rviz::SelectionTool::processKeyEvent(event, panel);
  if (event->type() == QKeyEvent::KeyPress)
  {
    if (event->key() == 'c' || event->key() == 'C')
    {
      rviz::SelectionManager* selection_manager = context_->getSelectionManager();
      rviz::M_Picked selection = selection_manager->getSelection();
      selection_manager->removeSelection(selection);
      num_selected_points_ = 0;
    }
    else if (event->key() == 'p' || event->key() == 'P')
    {
      ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic",
                            "Publishing " << num_selected_points_ << " selected points to topic "
                                           << node_handle_.resolveName(selected_drones_topic_));
      for (int index = 0; index < static_cast<int>(selected_drones_.size()); ++index)
      {
        rviz_selected_publisher_.publish(selected_drones_[index]);
      }
    }
  }

  return flags;
}

int SelectedPointsPublisher::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = rviz::SelectionTool::processMouseEvent(event);
  if (event.alt())
  {
    selecting_ = false;
  }
  else
  {
    if (event.leftDown())
    {
      selecting_ = true;
    }
    if (event.rightUp())
    {
      Ogre::Vector3 intersection;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
      {
        this->processSelectedArea();
        for (int index = 0; index < static_cast<int>(selected_drones_.size()); ++index)
        {
          rviz_selected_publisher_.publish(selected_drones_[index]);
        }

        ros::Duration(0.1).sleep();

        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.frame_id = std::string("world");
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.pose.position.x = intersection.x;
        goal_msg.pose.position.y = intersection.y;
        goal_msg.pose.position.z = 1.0;
        goal_publisher_.publish(goal_msg);
      }
    }
  }

  if (selecting_ && event.leftUp())
  {
    this->processSelectedArea();
  }
  return flags;
}

int SelectedPointsPublisher::processSelectedArea()
{
  rviz::SelectionManager* selection_manager = context_->getSelectionManager();
  rviz::PropertyTreeModel* model = selection_manager->getPropertyModel();

  selected_drones_.clear();
  int index = 0;
  while (model->hasIndex(index, 0))
  {
    QModelIndex child_index = model->index(index, 0);
    rviz::Property* child = model->getProp(child_index);
    rviz::VectorProperty* subchild = static_cast<rviz::VectorProperty*>(child->childAt(0));
    Ogre::Vector3 point_data = subchild->getVector();
    std::string name = child->getNameStd();

    if (name.substr(0, 12) == std::string("Marker drone"))
    {
      geometry_msgs::PoseStamped drone_msg;
      drone_msg.header.frame_id = std::string("drone_") + name.substr(13, 20);
      drone_msg.header.stamp = ros::Time::now();
      drone_msg.pose.position.x = point_data.x;
      drone_msg.pose.position.y = point_data.y;
      drone_msg.pose.position.z = point_data.z;
      selected_drones_.push_back(drone_msg);
    }

    index++;
  }

  num_selected_points_ = index;
  ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                        "Number of points in the selected area: " << num_selected_points_);

  return 0;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_selected_points_publisher::SelectedPointsPublisher, rviz::Tool)