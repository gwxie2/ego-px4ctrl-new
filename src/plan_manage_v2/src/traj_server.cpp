#include <algorithm>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <traj_utils_v2/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace Eigen;

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;
Eigen::Vector3d odom_pos_;
Eigen::Vector3d traj_start_pos_;
bool has_odom_ = false;
bool cmd_time_initialized_ = false;
bool startup_yaw_hold_enabled_ = true;
bool yaw_guidance_armed_ = false;
bool enable_stale_traj_extrapolation_ = false;
bool retime_position_cmd_now_ = true;
double odom_yaw_ = 0.0;
double traj_start_yaw_ = 0.0;
double startup_yaw_hold_time_ = 0.8;
double startup_yaw_release_speed_ = 0.6;
double startup_yaw_release_distance_ = 0.35;
double stale_traj_extrapolation_timeout_ = 0.35;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;

namespace
{
inline bool is_finite_double(double value)
{
  return std::isfinite(value);
}

inline bool is_finite_vector3(const Eigen::Vector3d &value)
{
  return is_finite_double(value.x()) && is_finite_double(value.y()) && is_finite_double(value.z());
}

inline bool is_valid_trajectory_msg(const traj_utils_v2::PolyTrajPtr &msg)
{
  if (!msg)
  {
    return false;
  }

  if (msg->order != 5)
  {
    return false;
  }

  if (msg->duration.empty() || msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    return false;
  }

  auto all_finite = [](const std::vector<double> &values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
  };

  return all_finite(msg->duration) && all_finite(msg->coef_x) && all_finite(msg->coef_y) && all_finite(msg->coef_z);
}
} // namespace

double getYawFromQuaternion(const geometry_msgs::Quaternion &q_msg)
{
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  return atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
               q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (!msg)
  {
    ROS_ERROR_THROTTLE(1.0, "[traj_server] dropped null odometry message");
    return;
  }
  // 里程计是命令发布的根基，任何 NaN/Inf 都会被下游控制器放大。
  if (!is_finite_double(msg->pose.pose.position.x) || !is_finite_double(msg->pose.pose.position.y) || !is_finite_double(msg->pose.pose.position.z) ||
      !is_finite_double(msg->pose.pose.orientation.x) || !is_finite_double(msg->pose.pose.orientation.y) || !is_finite_double(msg->pose.pose.orientation.z) || !is_finite_double(msg->pose.pose.orientation.w))
  {
    ROS_ERROR_THROTTLE(1.0, "[traj_server] dropped odometry with invalid pose values");
    return;
  }

  has_odom_ = true;
  odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  last_pos_ = odom_pos_;
  odom_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);

  if (!receive_traj_ || !cmd_time_initialized_)
  {
    last_yaw_ = odom_yaw_;
    last_yawdot_ = 0.0;
  }
}

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(traj_utils_v2::PolyTrajPtr msg)
{
  if (!is_valid_trajectory_msg(msg))
  {
    ROS_ERROR_THROTTLE(1.0, "[traj_server] dropped invalid trajectory message");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  // 新轨迹到来时，重置计时基准，避免 yaw 过渡和 heartbeat 逻辑继承旧轨迹状态。
  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;
  cmd_time_initialized_ = false;
  yaw_guidance_armed_ = !startup_yaw_hold_enabled_;
  traj_start_pos_ = odom_pos_;
  traj_start_yaw_ = odom_yaw_;

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  if (startup_yaw_hold_enabled_ && !yaw_guidance_armed_)
  {
    // 起步阶段锁住 yaw，等无人机离开地面并建立稳定水平速度后再释放朝向跟随。
    const double horizontal_speed = vel.head<2>().norm();
    const double horizontal_distance = (pos - traj_start_pos_).head<2>().norm();
    if (t_cur >= startup_yaw_hold_time_ ||
        (horizontal_speed >= startup_yaw_release_speed_ && horizontal_distance >= startup_yaw_release_distance_))
    {
      yaw_guidance_armed_ = true;
    }
  }

  double yaw_temp = traj_start_yaw_;
  if (yaw_guidance_armed_)
  {
    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                              ? traj_->getPos(t_cur + time_forward_) - pos
                              : traj_->getPos(traj_duration_) - pos;
    yaw_temp = dir.norm() > 0.1
                  ? atan2(dir(1), dir(0))
                  : last_yaw_;
  }

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

void publish_cmd(const ros::Time &command_stamp, Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{
  if (!is_finite_vector3(p) || !is_finite_vector3(v) || !is_finite_vector3(a) || !is_finite_vector3(j) || !is_finite_double(y) || !is_finite_double(yd))
  {
    ROS_ERROR_THROTTLE(1.0, "[traj_server] refused to publish invalid PositionCommand");
    return;
  }

  cmd.header.stamp = command_stamp;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

bool allow_stale_traj_extrapolation(const double t_cur)
{
  return enable_stale_traj_extrapolation_ && t_cur >= traj_duration_ && t_cur <= traj_duration_ + stale_traj_extrapolation_timeout_;
}

void evaluate_stale_traj_extrapolation(const double t_cur, Eigen::Vector3d &pos, Eigen::Vector3d &vel,
                                       Eigen::Vector3d &acc, Eigen::Vector3d &jer)
{
  const double delta_t = std::max(0.0, t_cur - traj_duration_);
  const Eigen::Vector3d end_pos = traj_->getPos(traj_duration_);
  const Eigen::Vector3d end_vel = traj_->getVel(traj_duration_);
  const Eigen::Vector3d end_acc = traj_->getAcc(traj_duration_);

  pos = end_pos + end_vel * delta_t + 0.5 * end_acc * delta_t * delta_t;
  vel = end_vel + end_acc * delta_t;
  acc = end_acc;
  jer.setZero();
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_ || !traj_)
    return;
  if (!has_odom_)
    return;

  if (!is_finite_double(traj_duration_) || traj_duration_ <= 0.0)
  {
    ROS_ERROR_THROTTLE(1.0, "[traj_server] invalid trajectory duration: %.6f", traj_duration_);
    return;
  }

  ros::Time time_now = ros::Time::now();
  const double t_cur = (time_now - start_time_).toSec();
  const bool allow_stale_extrapolation = allow_stale_traj_extrapolation(t_cur);
  const ros::Time command_stamp = retime_position_cmd_now_
                                      ? time_now
                                      : (start_time_ + ros::Duration(std::max(0.0, t_cur)));

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    if (!allow_stale_extrapolation)
    {
      ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

      receive_traj_ = false;
      publish_cmd(command_stamp, last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
      return;
    }
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last;
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  static bool finished = false;
#endif
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    if (!cmd_time_initialized_)
    {
      time_last = time_now;
      cmd_time_initialized_ = true;
    }

    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    if (!is_finite_vector3(pos) || !is_finite_vector3(vel) || !is_finite_vector3(acc) || !is_finite_vector3(jer))
    {
      ROS_ERROR_THROTTLE(1.0, "[traj_server] trajectory evaluation produced invalid finite values");
      receive_traj_ = false;
      return;
    }

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, vel, std::max((time_now - time_last).toSec(), 0.01));
    /*** calculate yaw ***/

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    // publish
    publish_cmd(command_stamp, pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }
  else if (allow_stale_extrapolation)
  {
    if (!cmd_time_initialized_)
    {
      time_last = time_now;
      cmd_time_initialized_ = true;
    }

    evaluate_stale_traj_extrapolation(t_cur, pos, vel, acc, jer);
    if (!is_finite_vector3(pos) || !is_finite_vector3(vel) || !is_finite_vector3(acc) || !is_finite_vector3(jer))
    {
      ROS_ERROR_THROTTLE(1.0, "[traj_server] stale trajectory extrapolation produced invalid finite values");
      receive_traj_ = false;
      return;
    }
    yaw_yawdot = calculate_yaw(t_cur, pos, vel, std::max((time_now - time_last).toSec(), 0.01));

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    publish_cmd(command_stamp, pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }

#if FLIP_YAW_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(command_stamp, pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI)
        last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI)
        last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_)
        finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_)
        finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(command_stamp, pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/enable_stale_traj_extrapolation", enable_stale_traj_extrapolation_, false);
  nh.param("traj_server/stale_traj_extrapolation_timeout", stale_traj_extrapolation_timeout_, 0.35);
  nh.param("traj_server/retime_position_cmd_now", retime_position_cmd_now_, true);
  nh.param("traj_server/startup_yaw_hold_enabled", startup_yaw_hold_enabled_, true);
  nh.param("traj_server/startup_yaw_hold_time", startup_yaw_hold_time_, 0.8);
  nh.param("traj_server/startup_yaw_release_speed", startup_yaw_release_speed_, 0.6);
  nh.param("traj_server/startup_yaw_release_distance", startup_yaw_release_distance_, 0.35);
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;
  last_pos_.setZero();
  odom_pos_.setZero();
  traj_start_pos_.setZero();

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}