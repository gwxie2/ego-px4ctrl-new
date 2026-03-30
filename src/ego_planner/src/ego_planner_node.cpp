#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <iostream>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  std::cerr << "[ego_planner_node] entering main" << std::endl;
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");
  std::cerr << "[ego_planner_node] ros node initialized" << std::endl;

  EGOReplanFSM rebo_replan;

  try
  {
    rebo_replan.init(nh);
  }
  catch (const std::exception &exception)
  {
    std::cerr << "[ego_planner_node] init exception: " << exception.what() << std::endl;
    throw;
  }
  catch (...)
  {
    std::cerr << "[ego_planner_node] init exception: unknown" << std::endl;
    throw;
  }

  std::cerr << "[ego_planner_node] init completed" << std::endl;

  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

// #include <ros/ros.h>
// #include <csignal>
// #include <visualization_msgs/Marker.h>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

// void SignalHandler(int signal) {
//   if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
//     ros::shutdown();
//   }
// }

// int main(int argc, char **argv) {

//   signal(SIGINT, SignalHandler);
//   signal(SIGTERM,SignalHandler);

//   ros::init(argc, argv, "ego_planner_node", ros::init_options::NoSigintHandler);
//   ros::NodeHandle nh("~");

//   EGOReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ros::Duration(1.0).sleep();
//   ros::AsyncSpinner async_spinner(4);
//   async_spinner.start();
//   ros::waitForShutdown();

//   return 0;
// }