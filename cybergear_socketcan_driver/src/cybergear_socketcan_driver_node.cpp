// MIT License
//
// Copyright (c) 2024 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "cybergear_socketcan_driver_node.hpp"

#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <can_msgs/msg/frame.hpp>
#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <cybergear_socketcan_driver_node_params.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace cybergear_socketcan_driver
{
CybergearSocketCanDriverNode::CybergearSocketCanDriverNode(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, rclcpp::NodeOptions(node_options).use_intra_process_comms(true)),
  recived_can_msg_(false),
  no_response_can_msg_counter_(0),
  packet_(nullptr),
  last_subscribe_joint_state_(nullptr),
  last_subscribe_can_frame_(nullptr),
  last_subscribe_setpoint_(nullptr),
  can_frame_subscriber_(nullptr),
  joint_trajectory_subscriber_(nullptr),
  joint_setpoint_subscriber_(nullptr),
  can_frame_publisher_(nullptr),
  joint_state_publisher_(nullptr),
  joint_temperature_publisher_(nullptr),
  send_can_frame_timer_(nullptr),
  update_parameter_timer_(nullptr),
  enable_torque_service_(nullptr),
  zero_position_service_(nullptr),
  diagnostic_updater_(nullptr),
  param_listener_(nullptr),
  params_(nullptr),
  single_joint_trajectory_(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());
  RCLCPP_WARN(this->get_logger(), "THIS NODE IS UNDER DEVELOPMENT");

  param_listener_ = std::make_unique<cybergear_socketcan_driver_node::ParamListener>(
    this->get_node_parameters_interface());
  params_ =
    std::make_unique<cybergear_socketcan_driver_node::Params>(param_listener_->get_params());

  cybergear_driver_core::CybergearPacketParam packet_param;
  packet_param.device_id = static_cast<int>(params_->device_id);
  packet_param.primary_id = static_cast<int>(params_->primary_id);
  packet_param.max_position = static_cast<float>(params_->anguler_position.max);
  packet_param.min_position = static_cast<float>(params_->anguler_position.min);
  packet_param.max_velocity = static_cast<float>(params_->anguler_velocity.max);
  packet_param.min_velocity = static_cast<float>(params_->anguler_velocity.min);
  packet_param.max_effort = static_cast<float>(params_->anguler_effort.max);
  packet_param.min_effort = static_cast<float>(params_->anguler_effort.min);
  packet_param.max_gain_kp = static_cast<float>(params_->pid_gain_range.kp.max);
  packet_param.min_gain_kp = static_cast<float>(params_->pid_gain_range.kp.min);
  packet_param.max_gain_kd = static_cast<float>(params_->pid_gain_range.kd.max);
  packet_param.min_gain_kd = static_cast<float>(params_->pid_gain_range.kd.min);
  packet_param.temperature_scale = static_cast<float>(params_->temperature.scale);
  packet_ = std::make_unique<cybergear_driver_core::CybergearPacket>(packet_param);

  can_frame_publisher_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 3);
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_state", 3);
  joint_temperature_publisher_ =
    this->create_publisher<sensor_msgs::msg::Temperature>("~/temperature", 3);

  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface());
  diagnostic_updater_->setHardwareID("CyberGear-" + std::to_string(params_->device_id));
  diagnostic_updater_->add(
    "CyberGear Status",
    std::bind(
      &CybergearSocketCanDriverNode::canFrameDiagnosricsCallback, this, std::placeholders::_1));

  can_frame_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus",
    3,
    std::bind(
      &CybergearSocketCanDriverNode::subscribeCanFrameCallback, this, std::placeholders::_1));

  if (params_->command_topic_type == "trajectory") {
    joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory",
      3,
      std::bind(
        &CybergearSocketCanDriverNode::subscribeJointTrajectoryCallback,
        this,
        std::placeholders::_1));
  } else if (params_->command_topic_type == "setpoint") {
    joint_setpoint_subscriber_ =
      this->create_subscription<cybergear_driver_msgs::msg::SetpointStamped>(
      "~/joint_setpoint",
      2,
      std::bind(
        &CybergearSocketCanDriverNode::subscribeJointSetpointCallback,
        this,
        std::placeholders::_1));
  }

  const auto send_duration_milliseconds = static_cast<unsigned int>(1e3 / params_->send_frequency);
  const auto update_param_duration_milliseconds =
    static_cast<unsigned int>(1e3 / params_->update_param_frequency);

  send_can_frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_duration_milliseconds),
    std::bind(&CybergearSocketCanDriverNode::sendCanFrameTimerCallback, this));
  update_parameter_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(update_param_duration_milliseconds),
    std::bind(&CybergearSocketCanDriverNode::updateParameterTimerCallback, this));

  enable_torque_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/enable_torque",
    std::bind(
      &CybergearSocketCanDriverNode::enableTorqueServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  zero_position_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/zero_position",
    std::bind(
      &CybergearSocketCanDriverNode::zeroPositionServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

CybergearSocketCanDriverNode::CybergearSocketCanDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_socketcan_driver", node_options)
{}

CybergearSocketCanDriverNode::~CybergearSocketCanDriverNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

cybergear_driver_core::CybergearPacket & CybergearSocketCanDriverNode::packet()
{
  return *packet_;
}

cybergear_socketcan_driver_node::Params & CybergearSocketCanDriverNode::params()
{
  return *params_;
}

void CybergearSocketCanDriverNode::procFeedbackPacketCallback(const can_msgs::msg::Frame &) {}

void CybergearSocketCanDriverNode::procFeedbackJointStateCallback(
  const sensor_msgs::msg::JointState &)
{}

void CybergearSocketCanDriverNode::procFeedbackTemperatureCallabck(
  const sensor_msgs::msg::Temperature &)
{}

void CybergearSocketCanDriverNode::sendCanFrameFromTrajectoryCallback(
  can_msgs::msg::Frame &, const SingleJointTrajectoryPoints &)
{}

void CybergearSocketCanDriverNode::sendCanFrameFromSetpointCallback(
  can_msgs::msg::Frame &, const cybergear_driver_msgs::msg::SetpointStamped &)
{}

void CybergearSocketCanDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = packet_->createChangeToOperationModeCommand();
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}

// TODO(Naoki Takahashi) parse read ram parameter
void CybergearSocketCanDriverNode::subscribeCanFrameCallback(
  const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  if (!packet_->frameId().isDevice(msg->id)) {
    return;
  }
  // TODO(Naoki Takahashi) params_->wait_power_on

  last_subscribe_can_frame_ = msg;

  if (packet_->frameId().isFault(msg->id)) {
    RCLCPP_ERROR(this->get_logger(), "Detect fault state from cybergear");
    std::string can_frame_data;
    for (const auto & d : msg->data) {
      can_frame_data += std::to_string(d) + ", ";
    }
    RCLCPP_ERROR_STREAM(this->get_logger(), "data: " << can_frame_data);
    rclcpp::shutdown();
    return;
  }
  if (packet_->frameId().isFeedback(msg->id)) {
    procFeedbackPacket(*msg);
  }
  recived_can_msg_ = true;
}

void CybergearSocketCanDriverNode::subscribeJointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr & msg)
{
  bool has_this_joint_cmd = false;
  int cmd_index = 0;

  for (const auto & joint_name : msg->joint_names) {
    if (joint_name == this->params().joint_name) {
      has_this_joint_cmd = true;
      break;
    }
    cmd_index++;
  }
  if (!has_this_joint_cmd) {
    return;
  }
  if (msg->points.size() <= static_cast<unsigned int>(cmd_index)) {
    return;
  }
  single_joint_trajectory_.reset();
  single_joint_trajectory_ = std::make_shared<SingleJointTrajectoryPoints>();
  if (single_joint_trajectory_) {
    single_joint_trajectory_->load(this->params().joint_name, *msg);
    if (last_subscribe_joint_state_) {
      single_joint_trajectory_->initTrajectoryPoint(*last_subscribe_joint_state_);
    }
  }
}

void CybergearSocketCanDriverNode::subscribeJointSetpointCallback(
  const cybergear_driver_msgs::msg::SetpointStamped::ConstSharedPtr & msg)
{
  last_subscribe_setpoint_ = msg;
}

void CybergearSocketCanDriverNode::sendCanFrameTimerCallback()
{
  static int wait_state_print_cycle_counter = 0;

  if (!recived_can_msg_) {
    if (wait_state_print_cycle_counter > params_->send_frequency) {
      RCLCPP_INFO(this->get_logger(), "Waiting for feedback message from the CAN bus..");
      wait_state_print_cycle_counter = 0;
    }
    sendFeedbackRequst();
    wait_state_print_cycle_counter++;
    no_response_can_msg_counter_++;
    return;
  }
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);

  if (last_subscribe_setpoint_) {
    sendCanFrameFromSetpointCallback(*msg, *last_subscribe_setpoint_);
  } else if (single_joint_trajectory_) {
    sendCanFrameFromTrajectoryCallback(*msg, *single_joint_trajectory_);
  } else {
    msg->id = packet_->frameId().getFeedbackId();
  }
  can_frame_publisher_->publish(std::move(msg));
  recived_can_msg_ = false;
}

void CybergearSocketCanDriverNode::updateParameterTimerCallback()
{
  if (param_listener_->is_old(*params_)) {
    param_listener_->refresh_dynamic_parameters();
    *params_ = param_listener_->get_params();
    RCLCPP_INFO(this->get_logger(), "Changed dynamic parameters");
  }
}

// TODO(Naoki Takahashi) wait for result
void CybergearSocketCanDriverNode::enableTorqueServiceCallback(
  const std_srvs::srv::SetBool::Request::ConstSharedPtr & request,
  const std_srvs::srv::SetBool::Response::SharedPtr & response)
{
  RCLCPP_INFO(this->get_logger(), "Calling enableTorqueServiceCallback");

  // Prevent abrupt behavior during torque switching
  if (last_subscribe_setpoint_) {
    last_subscribe_setpoint_.reset();
  }
  if (single_joint_trajectory_) {
    single_joint_trajectory_.reset();
  }
  sendResetTorque();

  sendChangeRunMode();

  if (request->data) {
    sendEnableTorque();
    response->message = "Sent enable torque message";
  } else {
    sendResetTorque();
    response->message = "Sent reset torque message";
  }
  response->success = true;
  RCLCPP_INFO_STREAM(this->get_logger(), response->message);
}

void CybergearSocketCanDriverNode::zeroPositionServiceCallback(
  const std_srvs::srv::Trigger::Request::ConstSharedPtr &,
  const std_srvs::srv::Trigger::Response::ConstSharedPtr &)
{
  RCLCPP_INFO(this->get_logger(), "Calling zeroPositionServiceCallback");
  sendZeroPosition();
}

// TODO(Naoki Takahashi): more information
void CybergearSocketCanDriverNode::canFrameDiagnosricsCallback(
  diagnostic_updater::DiagnosticStatusWrapper & diag_status)
{
  const auto no_response_can_msg_counter = no_response_can_msg_counter_;
  no_response_can_msg_counter_ = 0;

  if (!last_subscribe_can_frame_) {
    diag_status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not recived CAN frame");
    return;
  }
  std::string control_mode;

  if (packet_->frameId().isResetMode(last_subscribe_can_frame_->id)) {
    control_mode = "reset";
  } else if (packet_->frameId().isCaliMode(last_subscribe_can_frame_->id)) {
    control_mode = "cali";
  } else if (packet_->frameId().isRunningMode(last_subscribe_can_frame_->id)) {
    control_mode = "running";
  } else {
    control_mode = "unknown";
  }
  diag_status.add("Control mode", control_mode);
  diag_status.add("Raw ID", last_subscribe_can_frame_->id);
  diag_status.add("No response", no_response_can_msg_counter);
  diag_status.add("Joint trajectory subscribed", (single_joint_trajectory_ != nullptr));

  if (packet_->frameId().hasError(last_subscribe_can_frame_->id)) {
    diag_status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Recived error from CyberGear");
  } else if (no_response_can_msg_counter > 0) {
    diag_status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No response CyberGear");
  } else {
    diag_status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No problem");
  }
  last_subscribe_can_frame_.reset();
}

void CybergearSocketCanDriverNode::procFeedbackPacket(const can_msgs::msg::Frame & msg)
{
  if (packet_->frameId().hasError(msg.id)) {
    RCLCPP_WARN(this->get_logger(), "Detect error state from cybergear");
  }
  std_msgs::msg::Header header_msg;
  header_msg.stamp = this->get_clock()->now();
  header_msg.frame_id = params_->joint_name;

  if (joint_state_publisher_) {
    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();

    joint_state_msg->header = header_msg;
    joint_state_msg->name.push_back(params_->joint_name);
    joint_state_msg->position.push_back(packet_->parsePosition(msg.data));
    joint_state_msg->velocity.push_back(packet_->parseVelocity(msg.data));
    joint_state_msg->effort.push_back(packet_->parseEffort(msg.data));

    if (!last_subscribe_joint_state_) {
      last_subscribe_joint_state_ = std::make_unique<sensor_msgs::msg::JointState>();
    }
    *last_subscribe_joint_state_ = *joint_state_msg;

    procFeedbackJointStateCallback(*joint_state_msg);

    joint_state_publisher_->publish(std::move(joint_state_msg));
  }
  if (joint_temperature_publisher_) {
    auto temperature_msg = std::make_unique<sensor_msgs::msg::Temperature>();

    temperature_msg->header = header_msg;
    temperature_msg->temperature = packet_->parseTemperature(msg.data);
    temperature_msg->variance = 0.0;  // unknown

    procFeedbackTemperatureCallabck(*temperature_msg);

    joint_temperature_publisher_->publish(std::move(temperature_msg));
  }
  procFeedbackPacketCallback(msg);
}

void CybergearSocketCanDriverNode::setDefaultCanFrame(can_msgs::msg::Frame::UniquePtr & msg)
{
  constexpr uint8_t kDlc = 8;
  // TODO(Naoki Takahashi) params_->wait_power_on
  if (!msg) {
    return;
  }
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = params_->joint_name;
  msg->is_rtr = false;
  msg->is_extended = true;
  msg->is_error = false;
  msg->dlc = kDlc;
}

void CybergearSocketCanDriverNode::sendChangeRunMode()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  sendChangeRunModeCallback(*msg);
  if (msg->id == 0) {
    return;
  }
  can_frame_publisher_->publish(std::move(msg));
}

void CybergearSocketCanDriverNode::sendEnableTorque()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  msg->id = packet_->frameId().getEnableTorqueId();
  can_frame_publisher_->publish(std::move(msg));
}

void CybergearSocketCanDriverNode::sendResetTorque()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  msg->id = packet_->frameId().getResetTorqueId();
  can_frame_publisher_->publish(std::move(msg));
}

void CybergearSocketCanDriverNode::sendFeedbackRequst()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  msg->id = packet_->frameId().getFeedbackId();
  can_frame_publisher_->publish(std::move(msg));
}

void CybergearSocketCanDriverNode::sendZeroPosition()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  const auto can_frame = packet_->createZeroPosition();
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg->data.begin());
  msg->id = can_frame.id;
  can_frame_publisher_->publish(std::move(msg));
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearSocketCanDriverNode)
