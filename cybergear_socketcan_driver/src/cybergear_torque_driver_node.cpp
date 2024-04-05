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
  void sendCanFrameCallback(can_msgs::msg::Frame &) final;
  void sendChangeRunModeCallback(can_msgs::msg::Frame &) final;
  void subscribeJointTrajectoryPointCallback(
    const trajectory_msgs::msg::JointTrajectoryPoint &) final;

private:
  float getDestTorque();
  float getDestCurrent();

  std::vector<float> m_dest_effort;
};

CybergearTorqueDriverNode::CybergearTorqueDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_torque_driver", node_options),
  m_dest_effort() {}

CybergearTorqueDriverNode::~CybergearTorqueDriverNode() {}

void CybergearTorqueDriverNode::sendCanFrameCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createCurrentCommand(getDestCurrent());
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearTorqueDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createChangeToCurrentModeCommand();
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearTorqueDriverNode::subscribeJointTrajectoryPointCallback(
  const trajectory_msgs::msg::JointTrajectoryPoint & msg)
{
  if (msg.effort.size() < 1) {
    return;
  }
  const int dest_effort_count = msg.effort.size();
  m_dest_effort.resize(dest_effort_count);

  for (int i = 0; i < dest_effort_count; ++i) {
    m_dest_effort[i] = static_cast<float>(msg.effort[i]);
  }
}

float CybergearTorqueDriverNode::getDestTorque()
{
  if (0 < m_dest_effort.size()) {
    return m_dest_effort[0];
  }
  return 0.0f;
}

float CybergearTorqueDriverNode::getDestCurrent()
{
  const float dest_torque = getDestTorque();
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
