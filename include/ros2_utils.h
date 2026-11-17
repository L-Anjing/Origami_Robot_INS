#pragma once
#include "common_lib.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace origami
{
using ImuMsg = sensor_msgs::msg::Imu;
using OdometryMsg = nav_msgs::msg::Odometry;
using PathMsg = nav_msgs::msg::Path;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
inline builtin_interfaces::msg::Time stamp_message(int64_t ns)
{
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return t;
}
inline PoseStampedMsg pose_message(const Json& state, int64_t ns, const std::string& frame)
{
    PoseStampedMsg m;
    m.header.stamp = stamp_message(ns);
    m.header.frame_id = frame;
    const auto p = vector3(state.at("position"));
    const auto& q = state.at("quaternion");
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = q.at(0);
    m.pose.orientation.x = q.at(1);
    m.pose.orientation.y = q.at(2);
    m.pose.orientation.z = q.at(3);
    return m;
}
} // namespace origami
