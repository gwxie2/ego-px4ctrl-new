#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/Joy.h>


using namespace std;
bool flag_mandatory_stoped_ = false;

ros::Publisher joy_pub_;

void joy_sub_cb(const sensor_msgs::Joy::ConstPtr &msg)
{

  if ( msg->buttons[0] || msg->buttons[1] || msg->buttons[2] || msg->buttons[3] )
  {
    flag_mandatory_stoped_ = true;
  }

  if ( flag_mandatory_stoped_ )
  {
    joy_pub_.publish(*msg);
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_take_over_ground_station");
  ros::NodeHandle nh("~");

  string joy_input_topic("/joy");
  string joy_output_topic("/joystick_from_users");
  nh.param("joy_input_topic", joy_input_topic, joy_input_topic);
  nh.param("joy_output_topic", joy_output_topic, joy_output_topic);

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>(joy_input_topic, 10, joy_sub_cb);
  joy_pub_ = nh.advertise<sensor_msgs::Joy>(joy_output_topic, 10);

  ros::spin();

  return 0;
}
