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
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cybergear_socketcan_driver_node_params.hpp>
#include <cybergear_driver_core/cybergear_driver_core.hpp>

namespace cybergear_socketcan_driver
{

class CybergearSocketCanDriverNode : public rclcpp::Node
{
public:
  explicit CybergearSocketCanDriverNode(const rclcpp::NodeOptions &);
  ~CybergearSocketCanDriverNode();

private:
  bool m_recived_can_msg;

  float m_last_sense_anguler_position;
  std::vector<double> m_dest_anguler_positions;

  std::unique_ptr<cybergear_driver_core::CybergearPacket> m_packet;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_can_frame_subscriber;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    m_joint_trajectory_subscriber;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_can_frame_publisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_joint_temperature_publisher;

  rclcpp::TimerBase::SharedPtr m_send_can_frame_timer;
  rclcpp::TimerBase::SharedPtr m_update_parameter_timer;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enable_torque_service;

  std::unique_ptr<cybergear_socketcan_driver_node::ParamListener> m_param_listener;
  std::unique_ptr<cybergear_socketcan_driver_node::Params> m_params;

  void subscribeCanFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &);
  void subscribeJointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr &);
  void sendCanFrameTimerCallback();
  void updateParameterTimerCallback();
  void enableTorqueServiceCallback(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr & request,
    const std_srvs::srv::SetBool::Response::SharedPtr & response);

  void procFeedbackPacket(const can_msgs::msg::Frame &);

  void setDefaultCanFrame(can_msgs::msg::Frame::UniquePtr &);

  void sendEnableTorque();
  void sendResetTorque();
  void sendFeedbackRequst();

  float getDestAngulerPosition();
};
}  // namespace cybergear_socketcan_driver
