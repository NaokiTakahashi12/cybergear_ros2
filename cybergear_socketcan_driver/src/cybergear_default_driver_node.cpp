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

#include <can_msgs/msg/frame.hpp>
#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "cybergear_socketcan_driver_node.hpp"

namespace cybergear_socketcan_driver
{
class CybergearDefaultDriverNode : public CybergearSocketCanDriverNode
{
public:
  CybergearDefaultDriverNode() = delete;
  explicit CybergearDefaultDriverNode(const rclcpp::NodeOptions &);
  CybergearDefaultDriverNode(const CybergearDefaultDriverNode &) = delete;
  CybergearDefaultDriverNode(CybergearDefaultDriverNode &&) = delete;
  ~CybergearDefaultDriverNode() override;

  CybergearDefaultDriverNode & operator=(const CybergearDefaultDriverNode &) = delete;
  CybergearDefaultDriverNode & operator=(CybergearDefaultDriverNode &&) = delete;

protected:
  void procFeedbackJointStateCallback(const sensor_msgs::msg::JointState &) final;
  void sendCanFrameFromTrajectoryCallback(
    can_msgs::msg::Frame &, const SingleJointTrajectoryPoints &) final;
  void sendCanFrameFromSetpointCallback(
    can_msgs::msg::Frame &, const cybergear_driver_msgs::msg::SetpointStamped &) final;
  void sendChangeRunModeCallback(can_msgs::msg::Frame &) final;

private:
  float last_sense_anguler_position_;
};

CybergearDefaultDriverNode::CybergearDefaultDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_default_driver", node_options),
  last_sense_anguler_position_(0.0F)
{}

CybergearDefaultDriverNode::~CybergearDefaultDriverNode() {}

void CybergearDefaultDriverNode::procFeedbackJointStateCallback(
  const sensor_msgs::msg::JointState & msg)
{
  if (msg.position.empty()) {
    return;
  }
  last_sense_anguler_position_ = static_cast<float>(msg.position[0]);
}

void CybergearDefaultDriverNode::sendCanFrameFromTrajectoryCallback(
  can_msgs::msg::Frame & msg, const SingleJointTrajectoryPoints & single_joint_trajectory)
{
  cybergear_driver_core::MoveParam move_param;

  if (!single_joint_trajectory.points().empty()) {
    move_param.position = single_joint_trajectory.getLerpPosition(this->get_clock()->now());
    move_param.velocity = single_joint_trajectory.getLerpVelocity(this->get_clock()->now());
    move_param.effort = single_joint_trajectory.getLerpEffort(this->get_clock()->now());
  } else {
    move_param.position = last_sense_anguler_position_;
    move_param.velocity = 0.0F;
    move_param.effort = 0.0F;
  }
  move_param.kp = static_cast<float>(this->params().pid_gain.kp);
  move_param.kd = static_cast<float>(this->params().pid_gain.kd);

  const auto can_frame = this->packet().createMoveCommand(move_param);
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}

void CybergearDefaultDriverNode::sendCanFrameFromSetpointCallback(
  can_msgs::msg::Frame & msg, const cybergear_driver_msgs::msg::SetpointStamped & setpoint_msg)
{
  cybergear_driver_core::MoveParam move_param;

  move_param.position = setpoint_msg.point.position;
  move_param.velocity = setpoint_msg.point.velocity;
  move_param.effort = setpoint_msg.point.effort;
  move_param.kp = static_cast<float>(this->params().pid_gain.kp);
  move_param.kd = static_cast<float>(this->params().pid_gain.kd);

  const auto can_frame = this->packet().createMoveCommand(move_param);
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}

void CybergearDefaultDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createChangeToOperationModeCommand();
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearDefaultDriverNode)
