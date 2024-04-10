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

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cybergear_socketcan_driver_node_params.hpp>
#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <cybergear_socketcan_driver/single_joint_trajectory_points.hpp>

namespace cybergear_socketcan_driver
{

class CybergearSocketCanDriverNode : public rclcpp::Node
{
public:
  explicit CybergearSocketCanDriverNode(const std::string & node_name, const rclcpp::NodeOptions &);
  explicit CybergearSocketCanDriverNode(const rclcpp::NodeOptions &);
  virtual ~CybergearSocketCanDriverNode();

protected:
  cybergear_driver_core::CybergearPacket & packet();
  cybergear_socketcan_driver_node::Params & params();

  virtual void procFeedbackPacketCallback(const can_msgs::msg::Frame &);
  virtual void procFeedbackJointStateCallback(const sensor_msgs::msg::JointState &);
  virtual void procFeedbackTemperatureCallabck(const sensor_msgs::msg::Temperature &);

  virtual void sendCanFrameCallback(can_msgs::msg::Frame &);
  virtual void sendChangeRunModeCallback(can_msgs::msg::Frame &);

  virtual void subscribeJointTrajectoryPointCallback(
    const SingleJointTrajectoryPoints::SharedPtr &);

private:
  bool m_recived_can_msg;

  std::unique_ptr<cybergear_driver_core::CybergearPacket> m_packet;

  sensor_msgs::msg::JointState::UniquePtr m_last_subscribe_joint_state;
  can_msgs::msg::Frame::ConstSharedPtr m_last_subscribe_can_frame;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_can_frame_subscriber;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    m_joint_trajectory_subscriber;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_can_frame_publisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_joint_temperature_publisher;

  rclcpp::TimerBase::SharedPtr m_send_can_frame_timer;
  rclcpp::TimerBase::SharedPtr m_update_parameter_timer;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enable_torque_service;

  std::unique_ptr<diagnostic_updater::Updater> m_diagnostic_updater;

  std::unique_ptr<cybergear_socketcan_driver_node::ParamListener> m_param_listener;
  std::unique_ptr<cybergear_socketcan_driver_node::Params> m_params;

  SingleJointTrajectoryPoints::SharedPtr m_single_joint_trajectory;

  void subscribeCanFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &);
  void subscribeJointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr &);
  void sendCanFrameTimerCallback();
  void updateParameterTimerCallback();
  void enableTorqueServiceCallback(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr & request,
    const std_srvs::srv::SetBool::Response::SharedPtr & response);

  void canFrameDiagnosricsCallback(diagnostic_updater::DiagnosticStatusWrapper &);

  void procFeedbackPacket(const can_msgs::msg::Frame &);

  void setDefaultCanFrame(can_msgs::msg::Frame::UniquePtr &);

  void sendChangeRunMode();
  void sendEnableTorque();
  void sendResetTorque();
  void sendFeedbackRequst();
};
}  // namespace cybergear_socketcan_driver
