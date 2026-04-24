#include "input.h"

#include <cmath>

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

inline bool is_finite_quaternion(const Eigen::Quaterniond &value)
{
    return is_finite_double(value.w()) && is_finite_double(value.x()) && is_finite_double(value.y()) && is_finite_double(value.z());
}
} // namespace

RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;
    mode = 0.0;
    gear = 0.0;
    reboot_cmd = 0.0;
    last_reboot_cmd = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    if (!pMsg)
    {
        ROS_ERROR_THROTTLE(1.0, "RC data callback received a null message pointer.");
        return;
    }
    if (pMsg->channels.size() < 8)
    {
        ROS_ERROR_THROTTLE(1.0, "RC data validity check fail. expected at least 8 channels, got %zu", pMsg->channels.size());
        return;
    }

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;

    if (!(mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1))
    {
        ROS_ERROR_THROTTLE(1.0, "RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
        return;
    }

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    if (!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    // 1
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else
        enter_hover_mode = false;

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else
        is_hover_mode = false;

    // 2
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;

        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

    // 3
    if (!is_hover_mode && !is_command_mode)
    {
        if (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;

    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = std::abs(ch[0]) < 1e-5 && std::abs(ch[1]) < 1e-5 && std::abs(ch[2]) < 1e-5 && std::abs(ch[3]) < 1e-5;
    return centered;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    p.setZero();
    v.setZero();
    q.setIdentity();
    w.setZero();
    recv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    if (!pMsg)
    {
        ROS_ERROR_THROTTLE(1.0, "Odom callback received a null message pointer.");
        return;
    }

    if (!is_finite_double(pMsg->pose.pose.position.x) || !is_finite_double(pMsg->pose.pose.position.y) || !is_finite_double(pMsg->pose.pose.position.z) ||
        !is_finite_double(pMsg->pose.pose.orientation.x) || !is_finite_double(pMsg->pose.pose.orientation.y) || !is_finite_double(pMsg->pose.pose.orientation.z) || !is_finite_double(pMsg->pose.pose.orientation.w) ||
        !is_finite_double(pMsg->twist.twist.linear.x) || !is_finite_double(pMsg->twist.twist.linear.y) || !is_finite_double(pMsg->twist.twist.linear.z) ||
        !is_finite_double(pMsg->twist.twist.angular.x) || !is_finite_double(pMsg->twist.twist.angular.y) || !is_finite_double(pMsg->twist.twist.angular.z))
    {
        ROS_ERROR_THROTTLE(1.0, "Odom callback dropped invalid finite values.");
        return;
    }

    ros::Time now = ros::Time::now();

    msg = *pMsg;

    uav_utils::extract_odometry(pMsg, p, v, q, w);

    if (!is_finite_vector3(p) || !is_finite_vector3(v) || !is_finite_quaternion(q) || !is_finite_vector3(w))
    {
        ROS_ERROR_THROTTLE(1.0, "Odom callback produced invalid finite values after extraction.");
        return;
    }

    rcv_stamp = now;
    recv_new_msg = true;

// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 30 )
        {
            ROS_WARN("ODOM frequency seems lower than 30Hz (current %.1fHz), which may be too low!", (double)one_min_count);
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    w.setZero();
    a.setZero();
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    if (!pMsg)
    {
        ROS_ERROR_THROTTLE(1.0, "IMU callback received a null message pointer.");
        return;
    }
    if (!is_finite_double(pMsg->orientation.x) || !is_finite_double(pMsg->orientation.y) || !is_finite_double(pMsg->orientation.z) || !is_finite_double(pMsg->orientation.w) ||
        !is_finite_double(pMsg->angular_velocity.x) || !is_finite_double(pMsg->angular_velocity.y) || !is_finite_double(pMsg->angular_velocity.z) ||
        !is_finite_double(pMsg->linear_acceleration.x) || !is_finite_double(pMsg->linear_acceleration.y) || !is_finite_double(pMsg->linear_acceleration.z))
    {
        ROS_ERROR_THROTTLE(1.0, "IMU callback dropped invalid finite values.");
        return;
    }

    ros::Time now = ros::Time::now();

    msg = *pMsg;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    if (!is_finite_vector3(w) || !is_finite_vector3(a) || !is_finite_quaternion(q))
    {
        ROS_ERROR_THROTTLE(1.0, "IMU callback produced invalid finite values after unpacking.");
        return;
    }

    rcv_stamp = now;

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 30 )
        {
            ROS_WARN("IMU frequency seems lower than 30Hz (current %.1fHz), which may be too low!", (double)one_min_count);
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

State_Data_t::State_Data_t()
{
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{
    if (!pMsg)
    {
        ROS_ERROR_THROTTLE(1.0, "State callback received a null message pointer.");
        return;
    }

    current_state = *pMsg;
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    if (!pMsg)
    {
        ROS_ERROR_THROTTLE(1.0, "ExtendedState callback received a null message pointer.");
        return;
    }
    current_extended_state = *pMsg;
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
    p.setZero();
    v.setZero();
    a.setZero();
    j.setZero();
    yaw = 0.0;
    yaw_rate = 0.0;
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
    if (!pMsg)
    {
        ROS_ERROR_THROTTLE(1.0, "PositionCommand callback received a null message pointer.");
        return;
    }
    if (!is_finite_double(pMsg->position.x) || !is_finite_double(pMsg->position.y) || !is_finite_double(pMsg->position.z) ||
        !is_finite_double(pMsg->velocity.x) || !is_finite_double(pMsg->velocity.y) || !is_finite_double(pMsg->velocity.z) ||
        !is_finite_double(pMsg->acceleration.x) || !is_finite_double(pMsg->acceleration.y) || !is_finite_double(pMsg->acceleration.z) ||
        !is_finite_double(pMsg->jerk.x) || !is_finite_double(pMsg->jerk.y) || !is_finite_double(pMsg->jerk.z) ||
        !is_finite_double(pMsg->yaw) || !is_finite_double(pMsg->yaw_dot))
    {
        ROS_ERROR_THROTTLE(1.0, "PositionCommand callback dropped invalid finite values.");
        return;
    }

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
    volt = 0.0;
    percentage = 0.0;
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{
    if (!pMsg)
    {
        ROS_ERROR_THROTTLE(1.0, "Battery callback received a null message pointer.");
        return;
    }

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    double voltage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        if (!is_finite_double(pMsg->cell_voltage[i]))
        {
            ROS_ERROR_THROTTLE(1.0, "Battery callback dropped invalid cell voltage values.");
            return;
        }
        voltage += pMsg->cell_voltage[i];
    }
    if (pMsg->cell_voltage.empty())
    {
        voltage = pMsg->voltage;
    }
    if (!is_finite_double(voltage) || !is_finite_double(pMsg->percentage))
    {
        ROS_ERROR_THROTTLE(1.0, "Battery callback dropped invalid finite values.");
        return;
    }
    volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static ros::Time last_print_t = ros::Time(0);
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).toSec() > 10)
        {
            ROS_INFO("[px4ctrl] Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    else
    {
        if ((rcv_stamp - last_print_t).toSec() > 1)
        {
            // ROS_ERROR("[px4ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0);
    triggered = false;
    takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{
    if (!pMsg)
    {
        ROS_ERROR_THROTTLE(1.0, "TakeoffLand callback received a null message pointer.");
        return;
    }
    if (pMsg->takeoff_land_cmd != quadrotor_msgs::TakeoffLand::TAKEOFF && pMsg->takeoff_land_cmd != quadrotor_msgs::TakeoffLand::LAND)
    {
        ROS_ERROR_THROTTLE(1.0, "TakeoffLand callback dropped unknown command value: %u", static_cast<unsigned>(pMsg->takeoff_land_cmd));
        return;
    }

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}
