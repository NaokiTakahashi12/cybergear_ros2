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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "cybergear_socketcan_driver_node.hpp"

namespace cybergear_socketcan_driver
{
class CybergearVelocityDriverNode : public CybergearSocketCanDriverNode
{
public:
  CybergearVelocityDriverNode() = delete;
  explicit CybergearVelocityDriverNode(const rclcpp::NodeOptions &);
  CybergearVelocityDriverNode(const CybergearVelocityDriverNode &) = delete;
  CybergearVelocityDriverNode(CybergearVelocityDriverNode &&) = delete;
  ~CybergearVelocityDriverNode() override;

  CybergearVelocityDriverNode & operator=(const CybergearVelocityDriverNode &) = delete;
  CybergearVelocityDriverNode & operator=(CybergearVelocityDriverNode &&) = delete;

protected:
  void sendCanFrameFromTrajectoryCallback(
    can_msgs::msg::Frame &, const SingleJointTrajectoryPoints &) final;
  void sendCanFrameFromSetpointCallback(
    can_msgs::msg::Frame &, const cybergear_driver_msgs::msg::SetpointStamped &) final;
  void sendChangeRunModeCallback(can_msgs::msg::Frame &) final;
};

CybergearVelocityDriverNode::CybergearVelocityDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_velocity_driver", node_options)
{}

CybergearVelocityDriverNode::~CybergearVelocityDriverNode() {}

void CybergearVelocityDriverNode::sendCanFrameFromTrajectoryCallback(
  can_msgs::msg::Frame & msg, const SingleJointTrajectoryPoints & single_joint_trajectory)
{
  float velocity = 0.0F;

  if (!single_joint_trajectory.points().empty()) {
    velocity = single_joint_trajectory.getLerpVelocity(this->get_clock()->now());
  }
  const auto can_frame = this->packet().createVelocityCommand(velocity);
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}

void CybergearVelocityDriverNode::sendCanFrameFromSetpointCallback(
  can_msgs::msg::Frame & msg, const cybergear_driver_msgs::msg::SetpointStamped & setpoint_msg)
{
  const auto can_frame = this->packet().createVelocityCommand(setpoint_msg.point.velocity);
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}

void CybergearVelocityDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createChangeToVelocityModeCommand();
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearVelocityDriverNode)
