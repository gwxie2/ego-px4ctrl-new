#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/GoalSet.h>
#include <ros/ros.h>
#include <uav_utils/geometry_utils.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

ros::Publisher goals_pub_;

struct Drone_Info_t
{
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  int goal_id{-1};
  Eigen::Vector3d cur_p{Eigen::Vector3d::Zero()};
  double cur_yaw{0.0};
  Eigen::Vector3d last_p{Eigen::Vector3d::Zero()};
  ros::Time arrived_time;
  bool arrived_for_a_while{true};
  bool odom_received{false};
};

struct Goal_t
{
  Eigen::Vector3d p;
  bool occupied{false};
};

std::vector<Drone_Info_t> drones_;

string expandDroneTemplate(const string& topic_template, int drone_id)
{
  string topic = topic_template;
  size_t token_pos = topic.find("%d");
  if (token_pos != string::npos)
  {
    topic.replace(token_pos, 2, to_string(drone_id));
  }
  return topic;
}

void set_odom_data(const nav_msgs::Odometry::ConstPtr& msg, const int& drone_id)
{
  drones_[drone_id].odom_received = true;
  drones_[drone_id].last_p = drones_[drone_id].cur_p;
  drones_[drone_id].cur_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  drones_[drone_id].cur_yaw = uav_utils::get_yaw_from_quaternion(Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z));
}

void odoms_sim_sub_cb(const nav_msgs::Odometry::ConstPtr& msg, const int& drone_id)
{
  set_odom_data(msg, drone_id);
}

void combined_odoms_sim_sub_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  int id = atoi(msg->child_frame_id.substr(6, 10).c_str());
  if (msg->child_frame_id.substr(0, 6) != string("drone_") || id >= static_cast<int>(drones_.size()))
  {
    ROS_ERROR("[random_goals_node] Wrong child_frame_id: %s, or wrong drone_id: %d", msg->child_frame_id.substr(0, 6).c_str(), id);
    return;
  }
  set_odom_data(msg, id);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "random_goals");
  ros::NodeHandle nh("~");

  srand(floor(ros::Time::now().toSec() * 10));

  int drone_num, goal_num;
  nh.param("drone_num", drone_num, -1);
  nh.param("goal_num", goal_num, -1);

  string goal_output_topic("/goal_user2brig");
  string odom_topic_template("/drone_%d_visual_slam/odom");
  string combined_odom_topic("/others_odom");
  nh.param("goal_output_topic", goal_output_topic, goal_output_topic);
  nh.param("odom_topic_template", odom_topic_template, odom_topic_template);
  nh.param("combined_odom_topic", combined_odom_topic, combined_odom_topic);

  vector<Goal_t> goals(goal_num);
  for (int index = 0; index < goal_num; ++index)
  {
    vector<double> point;
    nh.getParam("goal" + to_string(index), point);
    goals[index].p << point[0], point[1], point[2];
    goals[index].occupied = false;
  }

  drones_.resize(drone_num);
  std::vector<ros::Subscriber> odoms_sim_sub(drone_num);
  for (int index = 0; index < drone_num; ++index)
  {
    odoms_sim_sub[index] = nh.subscribe<nav_msgs::Odometry>(expandDroneTemplate(odom_topic_template, index), 1000,
                                                            boost::bind(odoms_sim_sub_cb, _1, index));
  }

  ros::Subscriber combined_odoms_sim_sub = nh.subscribe<nav_msgs::Odometry>(combined_odom_topic, 1000, combined_odoms_sim_sub_cb);
  goals_pub_ = nh.advertise<quadrotor_msgs::GoalSet>(goal_output_topic, 10);

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();

    for (int index = 0; index < drone_num; ++index)
    {
      double distance_to_goal = (drones_[index].cur_p - drones_[index].goal).norm();
      double last_distance_to_goal = (drones_[index].last_p - drones_[index].goal).norm();
      if (distance_to_goal > 0.1 || last_distance_to_goal > 0.1)
      {
        drones_[index].arrived_time = now;
      }
      drones_[index].arrived_for_a_while |= ((now - drones_[index].arrived_time).toSec() > 2);
    }

    int drone_trials = 0;
    while (drone_trials < drone_num)
    {
      int drone_id = floor((static_cast<double>(rand()) / RAND_MAX) * drone_num);
      if (drones_[drone_id].odom_received && drones_[drone_id].arrived_for_a_while)
      {
        int goal_trials = 0;
        while (goal_trials < goal_num)
        {
          int goal_id = floor((static_cast<double>(rand()) / RAND_MAX) * goal_num);
          double ang = acos(((goals[goal_id].p - drones_[drone_id].cur_p).normalized())
                                .dot((Eigen::Vector3d(cos(drones_[drone_id].cur_yaw), sin(drones_[drone_id].cur_yaw), 0)).normalized()));
          if (!goals[goal_id].occupied && ang > 0 && ang < M_PI / 6)
          {
            if (drones_[drone_id].goal_id >= 0)
            {
              goals[drones_[drone_id].goal_id].occupied = false;
            }
            goals[goal_id].occupied = true;
            drones_[drone_id].arrived_for_a_while = false;
            drones_[drone_id].goal = goals[goal_id].p;
            drones_[drone_id].goal_id = goal_id;

            quadrotor_msgs::GoalSet msg;
            msg.drone_id = drone_id;
            msg.goal[0] = drones_[drone_id].goal(0);
            msg.goal[1] = drones_[drone_id].goal(1);
            msg.goal[2] = drones_[drone_id].goal(2);
            goals_pub_.publish(msg);
            cout << "drone_id=" << drone_id << " goal=" << drones_[drone_id].goal.transpose() << endl;
            goto multi_loop;
          }
          goal_trials++;
        }
      }
      drone_trials++;
    }

  multi_loop:
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  return 0;
}