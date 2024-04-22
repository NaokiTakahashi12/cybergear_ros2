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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <can_msgs/msg/frame.hpp>

namespace cybergear_socketcan_driver
{
class CybergearTorqueDriverNode : public CybergearSocketCanDriverNode
{
public:
  explicit CybergearTorqueDriverNode(const rclcpp::NodeOptions &);
  virtual ~CybergearTorqueDriverNode();

protected:
  void sendCanFrameFromTrajectoryCallback(
    can_msgs::msg::Frame &,
    const SingleJointTrajectoryPoints &) final;
  void sendCanFrameFromSetpointCallback(
    can_msgs::msg::Frame &,
    const cybergear_driver_msgs::msg::SetpointStamped &) final;
  void sendChangeRunModeCallback(can_msgs::msg::Frame &) final;

private:
  float getDestCurrent(const float dest_torque);
};

CybergearTorqueDriverNode::CybergearTorqueDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_torque_driver", node_options) {}

CybergearTorqueDriverNode::~CybergearTorqueDriverNode() {}

void CybergearTorqueDriverNode::sendCanFrameFromTrajectoryCallback(
  can_msgs::msg::Frame & msg,
  const SingleJointTrajectoryPoints & single_joint_trajectory)
{
  float effort = 0.0f;

  if (0 < single_joint_trajectory.points().size()) {
    effort = single_joint_trajectory.getLerpEffort(this->get_clock()->now());
  }
  const auto can_frame = this->packet().createCurrentCommand(
    getDestCurrent(effort));
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearTorqueDriverNode::sendCanFrameFromSetpointCallback(
  can_msgs::msg::Frame & msg,
  const cybergear_driver_msgs::msg::SetpointStamped & setpoint_msg)
{
  const auto can_frame = this->packet().createCurrentCommand(
    getDestCurrent(setpoint_msg.point.effort));
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearTorqueDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createChangeToCurrentModeCommand();
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

float CybergearTorqueDriverNode::getDestCurrent(const float dest_torque)
{
  const float torque_constant = this->params().torque_constant;

  if (0 == dest_torque) {
    return 0.0f;
  } else if (0 == torque_constant) {
    return 0.0f;
  }
  return dest_torque / torque_constant;
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearTorqueDriverNode)
