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

#include <cmath>
#include <memory>
#include <string>
#include <limits>
#include <array>
#include <vector>
#include <chrono>
#include <functional>
#include <algorithm>
#include <utility>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cybergear_socketcan_driver_node_params.hpp>
#include <cybergear_driver_core/cybergear_driver_core.hpp>

namespace cybergear_socketcan_driver
{
CybergearSocketCanDriverNode::CybergearSocketCanDriverNode(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, rclcpp::NodeOptions(node_options).use_intra_process_comms(true)),
  m_recived_can_msg(false),
  m_last_sense_anguler_position(0.0),
  m_dest_anguler_positions(),
  m_packet(nullptr),
  m_last_subscribe_can_frame(nullptr),
  m_can_frame_subscriber(nullptr),
  m_joint_trajectory_subscriber(nullptr),
  m_can_frame_publisher(nullptr),
  m_joint_state_publisher(nullptr),
  m_joint_temperature_publisher(nullptr),
  m_send_can_frame_timer(nullptr),
  m_update_parameter_timer(nullptr),
  m_enable_torque_service(nullptr),
  m_diagnostic_updater(nullptr),
  m_param_listener(nullptr),
  m_params(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());
  RCLCPP_WARN(this->get_logger(), "THIS NODE IS UNDER DEVELOPMENT");

  m_param_listener = std::make_unique<cybergear_socketcan_driver_node::ParamListener>(
    this->get_node_parameters_interface());
  m_params = std::make_unique<cybergear_socketcan_driver_node::Params>(
    m_param_listener->get_params());

  cybergear_driver_core::CybergearPacketParam packet_param;
  packet_param.device_id = m_params->device_id;
  packet_param.primary_id = m_params->primary_id;
  packet_param.max_position = m_params->anguler_position.max;
  packet_param.min_position = m_params->anguler_position.min;
  packet_param.max_velocity = m_params->anguler_velocity.max;
  packet_param.min_velocity = m_params->anguler_velocity.min;
  packet_param.max_effort = m_params->anguler_effort.max;
  packet_param.min_effort = m_params->anguler_effort.min;
  packet_param.max_gain_kp = m_params->pid_gain_range.kp.max;
  packet_param.min_gain_kp = m_params->pid_gain_range.kp.min;
  packet_param.max_gain_kd = m_params->pid_gain_range.kd.max;
  packet_param.min_gain_kd = m_params->pid_gain_range.kd.min;
  packet_param.temperature_scale = m_params->temperature.scale;
  m_packet = std::make_unique<cybergear_driver_core::CybergearPacket>(packet_param);

  m_can_frame_publisher = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 3);
  m_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>(
    "~/joint_state", 3);
  m_joint_temperature_publisher = this->create_publisher<sensor_msgs::msg::Temperature>(
    "~/temperature", 3);

  m_diagnostic_updater = std::make_unique<diagnostic_updater::Updater>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface());
  m_diagnostic_updater->setHardwareIDf("CyberGear-%i", m_params->device_id);
  m_diagnostic_updater->add(
    "CyberGear Status",
    std::bind(
      &CybergearSocketCanDriverNode::canFrameDiagnosricsCallback,
      this,
      std::placeholders::_1));

  m_can_frame_subscriber = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus",
    3,
    std::bind(
      &CybergearSocketCanDriverNode::subscribeCanFrameCallback,
      this,
      std::placeholders::_1
    )
  );
  m_joint_trajectory_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory",
    3,
    std::bind(
      &CybergearSocketCanDriverNode::subscribeJointTrajectoryCallback,
      this,
      std::placeholders::_1
    )
  );

  const unsigned int send_duration_milliseconds = 1e3 / m_params->send_frequency;
  const unsigned int update_param_duration_milliseconds = 1e3 / m_params->update_param_frequency;

  m_send_can_frame_timer = this->create_wall_timer(
    std::chrono::milliseconds(send_duration_milliseconds),
    std::bind(&CybergearSocketCanDriverNode::sendCanFrameTimerCallback, this));
  m_update_parameter_timer = this->create_wall_timer(
    std::chrono::milliseconds(update_param_duration_milliseconds),
    std::bind(&CybergearSocketCanDriverNode::updateParameterTimerCallback, this));

  m_enable_torque_service = this->create_service<std_srvs::srv::SetBool>(
    "~/enable_torque",
    std::bind(
      &CybergearSocketCanDriverNode::enableTorqueServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
}

CybergearSocketCanDriverNode::CybergearSocketCanDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_socketcan_driver", node_options) {}

CybergearSocketCanDriverNode::~CybergearSocketCanDriverNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void CybergearSocketCanDriverNode::subscribeCanFrameCallback(
  const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  if (!m_packet->frameId().isDevice(msg->id)) {
    return;
  }
  // TODO(Naoki Takahashi) m_params->wait_power_on

  m_last_subscribe_can_frame = msg;

  if (m_packet->frameId().isFault(msg->id)) {
    RCLCPP_ERROR(this->get_logger(), "Detect fault state from cybergear");
    std::string can_frame_data;
    for (const auto & d : msg->data) {
      can_frame_data += std::to_string(d) + ", ";
    }
    RCLCPP_ERROR_STREAM(this->get_logger(), "data: " << can_frame_data);
    rclcpp::shutdown();
    return;
  }
  if (m_packet->frameId().isFeedback(msg->id)) {
    procFeedbackPacket(*msg);
  }
  m_recived_can_msg = true;
}

void CybergearSocketCanDriverNode::subscribeJointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr & msg)
{
  bool has_this_joint_cmd = false;
  int cmd_index = 0;
  for (const auto & joint_name : msg->joint_names) {
    if (joint_name == m_params->joint_name) {
      has_this_joint_cmd = true;
      break;
    }
    cmd_index++;
  }
  if (!has_this_joint_cmd) {
    return;
  } else if (msg->points.size() <= static_cast<unsigned int>(cmd_index)) {
    return;
  } else if (msg->points[cmd_index].positions.size() < 1) {
    return;
  }
  m_dest_anguler_positions.resize(msg->points[cmd_index].positions.size());
  std::copy(
    msg->points[cmd_index].positions.cbegin(),
    msg->points[cmd_index].positions.cend(),
    m_dest_anguler_positions.begin());
}

void CybergearSocketCanDriverNode::sendCanFrameTimerCallback()
{
  static int wait_state_print_cycle_counter = 0;

  if (!m_recived_can_msg) {
    if (wait_state_print_cycle_counter > m_params->send_frequency) {
      RCLCPP_INFO(this->get_logger(), "Waiting for feedback message from the CAN bus..");
      wait_state_print_cycle_counter = 0;
    }
    sendFeedbackRequst();
    wait_state_print_cycle_counter++;
    return;
  }
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);

  const auto can_frame = m_packet->createPositionCommand(
    getDestAngulerPosition(),
    m_params->pid_gain.kp,
    m_params->pid_gain.kd
  );
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg->data.begin());
  msg->id = can_frame->id;
  m_can_frame_publisher->publish(std::move(msg));

  m_recived_can_msg = false;
}

void CybergearSocketCanDriverNode::updateParameterTimerCallback()
{
  if (m_param_listener->is_old(*m_params)) {
    m_param_listener->refresh_dynamic_parameters();
    *m_params = m_param_listener->get_params();
    RCLCPP_INFO(this->get_logger(), "Changed dynamic parameters");
  }
}

// TODO(Naoki Takahashi) wait for result
void CybergearSocketCanDriverNode::enableTorqueServiceCallback(
  const std_srvs::srv::SetBool::Request::ConstSharedPtr & request,
  const std_srvs::srv::SetBool::Response::SharedPtr & response)
{
  RCLCPP_INFO(this->get_logger(), "Calling enableTorqueServiceCallback");
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

// TODO(Naoki Takahashi): more information
void CybergearSocketCanDriverNode::canFrameDiagnosricsCallback(
  diagnostic_updater::DiagnosticStatusWrapper & diag_status)
{
  if (!m_last_subscribe_can_frame) {
    diag_status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Not recived CAN frame");
    return;
  }
  std::string control_mode;
  if (m_packet->frameId().isResetMode(m_last_subscribe_can_frame->id)) {
    control_mode = "reset";
  } else if (m_packet->frameId().isCaliMode(m_last_subscribe_can_frame->id)) {
    control_mode = "cali";
  } else if (m_packet->frameId().isRunningMode(m_last_subscribe_can_frame->id)) {
    control_mode = "running";
  }
  diag_status.add("Control mode", control_mode);
  diag_status.add("Raw ID", m_last_subscribe_can_frame->id);

  if (m_packet->frameId().hasError(m_last_subscribe_can_frame->id)) {
    diag_status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Recived error");
  } else {
    diag_status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Recived CAN frame");
  }
  m_last_subscribe_can_frame.reset();
}

void CybergearSocketCanDriverNode::procFeedbackPacket(const can_msgs::msg::Frame & msg)
{
  if (m_packet->frameId().hasError(msg.id)) {
    RCLCPP_WARN(this->get_logger(), "Detect error state from cybergear");
  }
  if (m_packet->frameId().isResetMode(msg.id)) {
    RCLCPP_DEBUG(this->get_logger(), "Reset mode now");
  }
  if (m_packet->frameId().isRunningMode(msg.id)) {
    RCLCPP_DEBUG(this->get_logger(), "Running mode now");
  }
  std_msgs::msg::Header header_msg;
  header_msg.stamp = this->get_clock()->now();
  header_msg.frame_id = m_params->joint_name;

  if (m_joint_state_publisher) {
    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();

    joint_state_msg->header = header_msg;
    joint_state_msg->name.push_back(m_params->joint_name);
    joint_state_msg->position.push_back(m_packet->persePosition(msg.data));
    joint_state_msg->velocity.push_back(m_packet->perseVelocity(msg.data));
    joint_state_msg->effort.push_back(m_packet->perseEffort(msg.data));

    m_last_sense_anguler_position = joint_state_msg->position[0];

    m_joint_state_publisher->publish(std::move(joint_state_msg));
  }
  if (m_joint_temperature_publisher) {
    auto temperature_msg = std::make_unique<sensor_msgs::msg::Temperature>();

    temperature_msg->header = header_msg;
    temperature_msg->temperature = m_packet->perseTemperature(msg.data);
    temperature_msg->variance = 0.0;  // unknown

    m_joint_temperature_publisher->publish(std::move(temperature_msg));
  }
}

void CybergearSocketCanDriverNode::setDefaultCanFrame(can_msgs::msg::Frame::UniquePtr & msg)
{
  // TODO(Naoki Takahashi) m_params->wait_power_on
  if (!msg) {
    return;
  }
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = m_params->joint_name;
  msg->is_rtr = false;
  msg->is_extended = true;
  msg->is_error = false;
  msg->dlc = 8;
}

void CybergearSocketCanDriverNode::sendEnableTorque()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  msg->id = m_packet->frameId().getEnableTorqueId();
  m_can_frame_publisher->publish(std::move(msg));
}

void CybergearSocketCanDriverNode::sendResetTorque()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  msg->id = m_packet->frameId().getResetTorqueId();
  m_can_frame_publisher->publish(std::move(msg));
}

void CybergearSocketCanDriverNode::sendFeedbackRequst()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  msg->id = m_packet->frameId().getFeedbackId();
  m_can_frame_publisher->publish(std::move(msg));
}

float CybergearSocketCanDriverNode::getDestAngulerPosition()
{
  if (0 < m_dest_anguler_positions.size()) {
    return m_dest_anguler_positions[0];
  }
  return m_last_sense_anguler_position;
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearSocketCanDriverNode)
