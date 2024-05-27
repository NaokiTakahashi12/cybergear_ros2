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

#include <can_msgs/msg/frame.hpp>
#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <cybergear_driver_msgs/msg/setpoint_stamped.hpp>
#include <cybergear_socketcan_driver/single_joint_trajectory_points.hpp>
#include <cybergear_socketcan_driver_node_params.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace cybergear_socketcan_driver
{

class CybergearSocketCanDriverNode : public rclcpp::Node
{
public:
  CybergearSocketCanDriverNode() = delete;
  explicit CybergearSocketCanDriverNode(const std::string & node_name, const rclcpp::NodeOptions &);
  explicit CybergearSocketCanDriverNode(const rclcpp::NodeOptions &);
  CybergearSocketCanDriverNode(const CybergearSocketCanDriverNode &) = delete;
  CybergearSocketCanDriverNode(CybergearSocketCanDriverNode &&) = delete;
  ~CybergearSocketCanDriverNode() override;

  CybergearSocketCanDriverNode & operator=(const CybergearSocketCanDriverNode &) = delete;
  CybergearSocketCanDriverNode & operator=(CybergearSocketCanDriverNode &&) = delete;

protected:
  cybergear_driver_core::CybergearPacket & packet();
  cybergear_socketcan_driver_node::Params & params();

  virtual void procFeedbackPacketCallback(const can_msgs::msg::Frame &);
  virtual void procFeedbackJointStateCallback(const sensor_msgs::msg::JointState &);
  virtual void procFeedbackTemperatureCallabck(const sensor_msgs::msg::Temperature &);

  virtual void sendCanFrameFromTrajectoryCallback(
    can_msgs::msg::Frame &, const SingleJointTrajectoryPoints &);
  virtual void sendCanFrameFromSetpointCallback(
    can_msgs::msg::Frame &, const cybergear_driver_msgs::msg::SetpointStamped &);
  virtual void sendChangeRunModeCallback(can_msgs::msg::Frame &);

private:
  bool recived_can_msg_;
  unsigned int no_response_can_msg_counter_;

  std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

  sensor_msgs::msg::JointState::UniquePtr last_subscribe_joint_state_;
  can_msgs::msg::Frame::ConstSharedPtr last_subscribe_can_frame_;
  cybergear_driver_msgs::msg::SetpointStamped::ConstSharedPtr last_subscribe_setpoint_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_trajectory_subscriber_;
  rclcpp::Subscription<cybergear_driver_msgs::msg::SetpointStamped>::SharedPtr
    joint_setpoint_subscriber_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr joint_temperature_publisher_;

  rclcpp::TimerBase::SharedPtr send_can_frame_timer_;
  rclcpp::TimerBase::SharedPtr update_parameter_timer_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_torque_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_position_service_;

  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  std::unique_ptr<cybergear_socketcan_driver_node::ParamListener> param_listener_;
  std::unique_ptr<cybergear_socketcan_driver_node::Params> params_;

  SingleJointTrajectoryPoints::SharedPtr single_joint_trajectory_;

  void subscribeCanFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &);
  void subscribeJointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr &);
  void subscribeJointSetpointCallback(
    const cybergear_driver_msgs::msg::SetpointStamped::ConstSharedPtr &);
  void sendCanFrameTimerCallback();
  void updateParameterTimerCallback();
  void enableTorqueServiceCallback(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr & request,
    const std_srvs::srv::SetBool::Response::SharedPtr & response);
  void zeroPositionServiceCallback(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr & request,
    const std_srvs::srv::Trigger::Response::ConstSharedPtr & response);

  void canFrameDiagnosricsCallback(diagnostic_updater::DiagnosticStatusWrapper &);

  void procFeedbackPacket(const can_msgs::msg::Frame &);

  void setDefaultCanFrame(can_msgs::msg::Frame::UniquePtr &);

  void sendChangeRunMode();
  void sendEnableTorque();
  void sendResetTorque();
  void sendFeedbackRequst();
  void sendZeroPosition();
};
}  // namespace cybergear_socketcan_driver
