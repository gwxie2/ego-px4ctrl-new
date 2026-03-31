#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <ros/ros.h>
#include <uav_utils/geometry_utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

ros::Publisher goals_pub_, new_goals_arrow_pub_;
ros::Time last_publish_time_;
bool need_clear_{false};

struct Selected_t
{
  int drone_id;
  Eigen::Vector3d p;
};

vector<Selected_t> drones_;

void displayArrowList(const vector<Eigen::Vector3d>& start, const vector<Eigen::Vector3d>& end, const double scale,
                      const int id, const int32_t action)
{
  if (start.size() != end.size())
  {
    ROS_ERROR("start.size() != end.size(), return");
    return;
  }

  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "world";
  arrow.header.stamp = ros::Time::now();
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = action;
  arrow.color.r = 0;
  arrow.color.g = 0;
  arrow.color.b = 0;
  arrow.color.a = 1.0;
  arrow.scale.x = scale;
  arrow.scale.y = 4 * scale;
  arrow.scale.z = 4 * scale;

  for (int index = 0; index < static_cast<int>(start.size()); index++)
  {
    geometry_msgs::Point st, ed;
    st.x = start[index](0);
    st.y = start[index](1);
    st.z = start[index](2);
    ed.x = end[index](0);
    ed.y = end[index](1);
    ed.z = end[index](2);
    arrow.points.clear();
    arrow.points.push_back(st);
    arrow.points.push_back(ed);
    arrow.id = index + id;
    array.markers.push_back(arrow);
  }

  new_goals_arrow_pub_.publish(array);
}

void selected_drones_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static ros::Time last_select_time = ros::Time(0);
  ros::Time now = ros::Time::now();
  if ((now - last_select_time).toSec() > 2)
  {
    drones_.clear();
  }

  Selected_t drone;
  drone.drone_id = atoi(msg->header.frame_id.substr(6, 10).c_str());
  drone.p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  drones_.push_back(drone);

  cout.precision(3);
  cout << "received drone " << drone.drone_id << " at " << drone.p.transpose() << ", total:" << drones_.size() << endl;

  last_select_time = now;
}

void user_goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (drones_.empty())
  {
    ROS_WARN("[assign_goals_node] No selected drones, ignore goal");
    return;
  }

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (size_t index = 0; index < drones_.size(); ++index)
  {
    center += drones_[index].p;
  }
  center /= drones_.size();

  Eigen::Vector3d user_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Vector3d movement = user_goal - center;

  vector<Eigen::Vector3d> each_one_starts(drones_.size()), each_one_goals(drones_.size());
  for (size_t index = 0; index < drones_.size(); ++index)
  {
    each_one_starts[index] = drones_[index].p;
    each_one_goals[index] = drones_[index].p + movement;
    cout.precision(3);
    cout << "drone " << drones_[index].drone_id << ", start=" << drones_[index].p.transpose() << ", end="
         << each_one_goals[index].transpose() << endl;
  }

  displayArrowList(each_one_starts, each_one_goals, 0.05, 0, visualization_msgs::Marker::ADD);
  last_publish_time_ = ros::Time::now();
  need_clear_ = true;

  for (size_t index = 0; index < drones_.size(); ++index)
  {
    quadrotor_msgs::GoalSet goal_msg;
    goal_msg.drone_id = drones_[index].drone_id;
    goal_msg.goal[0] = each_one_goals[index](0);
    goal_msg.goal[1] = each_one_goals[index](1);
    goal_msg.goal[2] = each_one_goals[index](2);
    goals_pub_.publish(goal_msg);
    ros::Duration(0.01).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "assign_goals");
  ros::NodeHandle nh("~");

  srand(floor(ros::Time::now().toSec() * 10));

  string selected_drones_topic("/rviz_selected_drones");
  string user_goal_topic("/goal");
  string goal_output_topic("/goal_user2brig");
  string arrow_topic("/new_goals_arrow");
  nh.param("selected_drones_topic", selected_drones_topic, selected_drones_topic);
  nh.param("user_goal_topic", user_goal_topic, user_goal_topic);
  nh.param("goal_output_topic", goal_output_topic, goal_output_topic);
  nh.param("arrow_topic", arrow_topic, arrow_topic);

  ros::Subscriber selected_drones_sub = nh.subscribe<geometry_msgs::PoseStamped>(selected_drones_topic, 100, selected_drones_cb);
  ros::Subscriber user_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(user_goal_topic, 10, user_goal_cb);

  goals_pub_ = nh.advertise<quadrotor_msgs::GoalSet>(goal_output_topic, 10);
  new_goals_arrow_pub_ = nh.advertise<visualization_msgs::MarkerArray>(arrow_topic, 10);

  ROS_INFO("[assign_goals_node] Start running with selected_drones_topic=%s, user_goal_topic=%s, goal_output_topic=%s",
           selected_drones_topic.c_str(), user_goal_topic.c_str(), goal_output_topic.c_str());

  while (ros::ok())
  {
    if (need_clear_ && (ros::Time::now() - last_publish_time_).toSec() > 2)
    {
      need_clear_ = false;
      std::vector<Eigen::Vector3d> blank(1);
      blank[0] = Eigen::Vector3d::Zero();
      displayArrowList(blank, blank, 0.05, 0, visualization_msgs::Marker::DELETEALL);
      cout << "DELETEALL Arrows." << endl;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  return 0;
}