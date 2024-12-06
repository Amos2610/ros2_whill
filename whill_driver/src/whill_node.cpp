// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    whill_node.cpp
 * @brief   Functions of the node that works as WHILL
 */
#include "whill_driver/whill_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "tf2_ros/transform_broadcaster.h"
#include "whill_driver/odom.h"
#include "utils/rotation_tools.h"
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

Odometry odom;

namespace whill_driver
{

/**
 * The default serial port name
 */
const std::string kDefaultPortName = "/dev/ttyUSB0";

/**
 * The default publish interval [ms]
 */
constexpr uint16_t kDefaulPpublishIntervalMs = 400;

// declare_parameter("publish_interval_ms", kDefaulPpublishIntervalMs);
// int publish_interval_ms = get_parameter("publish_interval_ms").as_int();
int publish_interval_ms = kDefaulPpublishIntervalMs;

void WhillNode::Initialize()
{
  // load parameters
  declare_parameter("port_name", kDefaultPortName);
  std::string port_name = get_parameter("port_name").as_string();
  whill_ = std::make_shared<model_cr2::Whill>(port_name);

  // declare_parameter("publish_interval_ms", kDefaulPpublishIntervalMs);
  // int publish_interval_ms = get_parameter("publish_interval_ms").as_int();
  RCLCPP_INFO(this->get_logger(), "publish_interval_ms: %d", publish_interval_ms);
  auto publish_duration = std::chrono::duration<double, std::milli>(publish_interval_ms);

  // publish
  states_model_cr2_pub_ = this->create_publisher<whill_msgs::msg::ModelCr2State>(
    "/whill/states/model_cr2", 10);
  states_model_cr2_timer_ =
    this->create_wall_timer(publish_duration, std::bind(&WhillNode::OnStatesModelCr2Timer, this));
  states_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/whill/joint_states", 10);
  states_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("whill/odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  // joystick_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/whill/joystick", 10);

  // subscription
  controller_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/whill/controller/joy", 10, std::bind(&WhillNode::OnControllerJoy, this, _1));
  controller_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/whill/controller/cmd_vel", 10, std::bind(&WhillNode::OnControllerCmdVel, this, _1));

  // service
  set_power_srv_ = this->create_service<whill_msgs::srv::SetPower>(
    "/whill/set_power_srv", std::bind(&WhillNode::OnSetPowerSrv, this, _1, _2, _3));
  set_speed_profile_srv_ = this->create_service<whill_msgs::srv::SetSpeedProfile>(
    "/whill/set_speed_profile_srv", std::bind(&WhillNode::OnSetSpeedProfileSrv, this, _1, _2, _3));
  set_battery_voltage_out_srv_ = this->create_service<whill_msgs::srv::SetBatteryVoltageOut>(
    "/whill/set_battery_voltage_out_srv",
    std::bind(&WhillNode::OnSetBatteryVoltageOutSrv, this, _1, _2, _3));

  // start sending WHILL State Dataset1
  whill_->SendStartSendingDataCommand(
    publish_interval_ms, model_cr2::kDatasetNumber1,
    model_cr2::kSpeedMode0);
}

WhillNode::WhillNode()
: Node("whill")
{
  this->Initialize();
}

WhillNode::WhillNode(const rclcpp::NodeOptions & options)
: Node("whill_node", options)
{
  this->Initialize();
}

WhillNode::~WhillNode()
{
}

void WhillNode::OnStatesModelCr2Timer()
{
  auto current_time = this->now();
  auto msg = std::make_shared<whill_msgs::msg::ModelCr2State>();
  if (whill_->ReceiveDataset1(msg) < 1) {return;}
  // RCLCPP_INFO(this->get_logger(), "WHILL state received msg: %d", msg->header.seq);
  RCLCPP_INFO(this->get_logger(), "WHILL state received left_motor_angle: %f", msg->left_motor_angle);
  RCLCPP_INFO(this->get_logger(), "WHILL state received right_motor_angle: %f", msg->right_motor_angle);
  // states_model_cr2_pub_->publish(*msg);

  // JointState
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = current_time;
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.velocity.resize(2);

  joint_state.name[0] = "left_wheel_joint";
  joint_state.position[0] = msg->left_motor_angle;

  joint_state.name[1] = "right_wheel_joint";
  joint_state.position[1] = msg->right_motor_angle;

  static double joint_past[2] = {0.0f, 0.0f};

  // debug
  RCLCPP_INFO(this->get_logger(), "publish_interval_ms: %d", publish_interval_ms);
  if (publish_interval_ms == -1) {
    joint_state.velocity[0] = rad_diff(joint_past[0], joint_state.position[0]) / double(publish_interval_ms) * 1000.0f; // Rad/sec
    joint_state.velocity[1] = rad_diff(joint_past[1], joint_state.position[1]) / double(publish_interval_ms) * 1000.0f; // Rad/sec
  }
  else if (publish_interval_ms == 0) {
    joint_state.velocity[0] = 0.0f;
    joint_state.velocity[1] = 0.0f;
  }
  else {
    joint_state.velocity[0] = rad_diff(joint_past[0], joint_state.position[0]) / double(publish_interval_ms) * 1000.0f; // Rad/sec
    joint_state.velocity[1] = rad_diff(joint_past[1], joint_state.position[1]) / double(publish_interval_ms) * 1000.0f; // Rad/sec
  }
  joint_past[0] = joint_state.position[0];
  joint_past[1] = joint_state.position[1];

  RCLCPP_INFO(this->get_logger(), "left: %f, right: %f", joint_state.velocity[0], joint_state.velocity[1]);
  states_model_cr2_pub_->publish(*msg);
  states_joint_pub_->publish(joint_state);

  // Odometry
  if (publish_interval_ms == -1) {
    odom.update(joint_state, publish_interval_ms / 1000.0f);
  }
  else if (publish_interval_ms >= 0) {
    if (publish_interval_ms == 0) {
      odom.zeroVelocity();
    }
    else {
      odom.update(joint_state, publish_interval_ms / 1000.0f);
    }
  }
  
  nav_msgs::msg::Odometry odom_msg = odom.getROSOdometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  states_odom_pub_->publish(odom_msg);

  //TF
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = current_time;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_link";
  odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
  odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
  tf_broadcaster_->sendTransform(odom_tf);
}

void WhillNode::OnControllerJoy(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  whill_->SendSetJoystickCommand(ConvertToWhillJoy(joy->axes[1]), ConvertToWhillJoy(-joy->axes[0]));
  RCLCPP_INFO(this->get_logger(), "[Joy] front:['%f'],  right:['%f']", joy->axes[1], joy->axes[0]);
}

void WhillNode::OnControllerCmdVel(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
  // [m/s] to [km/h]: *3.6
  // SetVelocityCommand takes command unit (0.004 km/h): *250
  int linear = cmd_vel->linear.x * 900;

  // wheel_tread: 0.496
  // [rad/s] to [km/h]: *wheel_tread*3.6
  // SetVelocityCommand takes command unit (0.004 km/h): *250
  // The direction of rotation is reversed in ROS and SetVelocityCommand
  int angular = cmd_vel->angular.z * -446.4;
  whill_->SendSetVelocityCommand(linear, angular);
  RCLCPP_INFO(
    this->get_logger(), "[CmdVel] linear:['%f'], angular:['%f']", cmd_vel->linear.x,
    cmd_vel->angular.z);
}

void WhillNode::OnSetPowerSrv(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<whill_msgs::srv::SetPower::Request> request,
  const std::shared_ptr<whill_msgs::srv::SetPower::Response> response)
{
  (void)request_header;
  switch (request->p0) {
    case 0:
      whill_->SetPowerOff();
      RCLCPP_INFO(this->get_logger(), "WHILL power off");
      response->result = 1;
      break;
    case 1:
      whill_->SetPowerOn();
      usleep(10000);
      whill_->SetPowerOn();
      usleep(2000);
      RCLCPP_INFO(this->get_logger(), "WHILL power on");
      response->result = 1;
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "p0 must be assinged 0 or 1");
      response->result = -1;
      break;
  }
}


void WhillNode::OnSetSpeedProfileSrv(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<whill_msgs::srv::SetSpeedProfile::Request> request,
  const std::shared_ptr<whill_msgs::srv::SetSpeedProfile::Response> response)
{
  (void)request_header;
  response->result = -1;

  uint8_t s1 = uint8_t(request->s1);
  uint8_t fm1 = uint8_t(request->fm1);
  uint8_t fa1 = uint8_t(request->fa1);
  uint8_t fd1 = uint8_t(request->fd1);
  uint8_t rm1 = uint8_t(request->rm1);
  uint8_t ra1 = uint8_t(request->ra1);
  uint8_t rd1 = uint8_t(request->rd1);
  uint8_t tm1 = uint8_t(request->tm1);
  uint8_t ta1 = uint8_t(request->ta1);
  uint8_t td1 = uint8_t(request->td1);
  if (IsOutside(s1, 0, 5)) {
    RCLCPP_WARN(this->get_logger(), "s1 must be assingned between 0 - 5");
    return;
  }
  if (IsOutside(fm1, 8, 60)) {
    RCLCPP_WARN(this->get_logger(), "fm1 must be assingned between 8 - 60");
    return;
  }
  if (IsOutside(fa1, 10, 90)) {
    RCLCPP_WARN(this->get_logger(), "fa1 must be assingned between 10 - 90");
    return;
  }
  if (IsOutside(fd1, 40, 160)) {
    RCLCPP_WARN(this->get_logger(), "fd1 must be assingned between 40 - 160");
    return;
  }
  if (IsOutside(rm1, 8, 30)) {
    RCLCPP_WARN(this->get_logger(), "rm1 must be assingned between 8 - 30");
    return;
  }
  if (IsOutside(ra1, 10, 50)) {
    RCLCPP_WARN(this->get_logger(), "ra1 must be assingned between 10 - 50");
    return;
  }
  if (IsOutside(rd1, 40, 90)) {
    RCLCPP_WARN(this->get_logger(), "rd1 must be assingned between 40 - 90");
    return;
  }
  if (IsOutside(tm1, 8, 35)) {
    RCLCPP_WARN(this->get_logger(), "tm1 must be assingned between 8 - 35");
    return;
  }
  if (IsOutside(ta1, 10, 60)) {
    RCLCPP_WARN(this->get_logger(), "ta1 must be assingned between 10 - 60");
    return;
  }
  if (IsOutside(td1, 40, 160)) {
    RCLCPP_WARN(this->get_logger(), "td1 must be assingned between 40 - 160");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Speed profile is set");
  whill_->SendSetSpeedProfileCommand(s1, fm1, fa1, fd1, rm1, ra1, rd1, tm1, ta1, td1);
  response->result = 1;
}

void WhillNode::OnSetBatteryVoltageOutSrv(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<whill_msgs::srv::SetBatteryVoltageOut::Request> request,
  const std::shared_ptr<whill_msgs::srv::SetBatteryVoltageOut::Response> response)
{
  (void)request_header;
  switch (request->v0) {
    case 0:
      whill_->SendSetBatteryVoltageOutCommand(0);
      RCLCPP_INFO(this->get_logger(), "battery voltage out: disable");
      response->result = 1;
      break;
    case 1:
      whill_->SendSetBatteryVoltageOutCommand(1);
      RCLCPP_INFO(this->get_logger(), "battery voltage out: enable");
      response->result = 1;
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "v0 must be assigned 0 or 1");
      response->result = -1;
      break;
  }
  RCLCPP_WARN(this->get_logger(), "BatteryVoltageOut command is not available on Model CR2!!");
}

/**
 * The function ConvertToWhillJoy converts a float value to an integer value between -100 and 100.
 *
 * @param raw_joy The `ConvertToWhillJoy` function takes a float value `raw_joy` as input, which
 * represents the raw joystick input. The function then converts this raw joystick input to an integer
 * value between -100 and 100, ensuring that the value does not exceed these limits.
 *
 * @return The function `ConvertToWhillJoy` takes a float value `raw_joy`, multiplies it by 100, and
 * then checks if the result is less than -100 or greater than 100. If the result is less than -100, it
 * returns -100. If the result is greater than 100, it returns 100. Otherwise, it returns the
 * calculated value `joy`.
 */
int WhillNode::ConvertToWhillJoy(float raw_joy)
{
  int joy = (int)(raw_joy * 100.0f);
  if (joy < -100) {return -100;}
  if (joy > 100) {return 100;}
  return joy;
}

/**
 * The function determines if a target value is outside the range defined by two other values.
 *
 * @param[in] target The value that we want to check if it is outside the range defined by end1 and end2.
 * @param[in] end1 The value of the first end point.
 * @param[in] end2 The "end2" parameter represents one end of a range.
 *
 * @return a boolean value. It returns true if the target value is outside the range defined by the
 * end1 and end2 values, and false otherwise.
 */
bool WhillNode::IsOutside(uint8_t target, uint8_t end1, uint8_t end2)
{
  uint8_t bottom = (end1 < end2) ? end1 : end2;
  uint8_t top = (end1 < end2) ? end2 : end1;
  if (target > top) {return true;}
  if (target < bottom) {return true;}
  return false;
}

}  // namespace whill_driver

RCLCPP_COMPONENTS_REGISTER_NODE(whill_driver::WhillNode)
