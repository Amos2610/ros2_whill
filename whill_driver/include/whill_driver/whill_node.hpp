// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    whill_node.hpp
 * @brief   Definitions of the unique node that works as WHILL
 */
#ifndef WHILL_DRIVER_WHILL_NODE_H_
#define WHILL_DRIVER_WHILL_NODE_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "whill_msgs/msg/model_cr2_state.hpp"
#include "whill_msgs/srv/set_battery_voltage_out.hpp"
#include "whill_msgs/srv/set_power.hpp"
#include "whill_msgs/srv/set_speed_profile.hpp"

#include "model_cr2/whill.hpp"

namespace whill_driver
{

class WhillNode : public rclcpp::Node
{
public:
  RCLCPP_PUBLIC
  WhillNode();

  explicit WhillNode(const rclcpp::NodeOptions & options);

  RCLCPP_PUBLIC
  ~WhillNode();

  // wheel radius [m]
  const double wheel_radius = 0.1325;
  // wheel tread [m]
  const double wheel_tread = 0.496;

private:
  void Initialize();
  std::shared_ptr<model_cr2::Whill> whill_;

  void OnStatesModelCr2Timer();
  rclcpp::TimerBase::SharedPtr states_model_cr2_timer_;
  rclcpp::Publisher<whill_msgs::msg::ModelCr2State>::SharedPtr states_model_cr2_pub_;

  void OnControllerJoy(const sensor_msgs::msg::Joy::SharedPtr joy);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_joy_sub_;

  void OnControllerCmdVel(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_cmd_vel_sub_;

  // rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joystick_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr states_joint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr states_odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // void OnJoy(const sensor_msgs::msg::Joy::SharedPtr joy);
  // void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr odom);
  void OnWhillCallbackData1(const whill_msgs::msg::ModelCr2State::SharedPtr msg);

  void OnSetPowerSrv(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<whill_msgs::srv::SetPower::Request> request,
    const std::shared_ptr<whill_msgs::srv::SetPower::Response> response);
  rclcpp::Service<whill_msgs::srv::SetPower>::SharedPtr set_power_srv_;

  void OnSetSpeedProfileSrv(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<whill_msgs::srv::SetSpeedProfile::Request> request,
    const std::shared_ptr<whill_msgs::srv::SetSpeedProfile::Response> response);
  rclcpp::Service<whill_msgs::srv::SetSpeedProfile>::SharedPtr set_speed_profile_srv_;

  void OnSetBatteryVoltageOutSrv(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<whill_msgs::srv::SetBatteryVoltageOut::Request> request,
    const std::shared_ptr<whill_msgs::srv::SetBatteryVoltageOut::Response> response);
  rclcpp::Service<whill_msgs::srv::SetBatteryVoltageOut>::SharedPtr set_battery_voltage_out_srv_;

  int ConvertToWhillJoy(float raw_joy);
  bool IsOutside(uint8_t target, uint8_t end1, uint8_t end2);
};

}  // namespace whill_driver

#endif  // WHILL_DRIVER_WHILL_NODE_H_
