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
#include <cybergear_driver_core/cybergear_driver_core.hpp>

namespace cybergear_socketcan_driver
{
class CybergearDefaultDriverNode : public CybergearSocketCanDriverNode
{
public:
  explicit CybergearDefaultDriverNode(const rclcpp::NodeOptions &);
  virtual ~CybergearDefaultDriverNode();

protected:
  void procFeedbackJointStateCallback(const sensor_msgs::msg::JointState &) final;
  void sendCanFrameCallback(can_msgs::msg::Frame &) final;
  void sendChangeRunModeCallback(can_msgs::msg::Frame &) final;
  void subscribeJointTrajectoryPointCallback(
    const SingleJointTrajectoryPoints::SharedPtr &) final;

private:
  float m_last_sense_anguler_position;

  SingleJointTrajectoryPoints::SharedPtr m_dest_joint_trajectory;

  float getDestAngulerPosition();
  float getDestAngulerVelocity();
  float getDestTorque();
};

CybergearDefaultDriverNode::CybergearDefaultDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_default_driver", node_options),
  m_last_sense_anguler_position(0.0f),
  m_dest_joint_trajectory(nullptr) {}

CybergearDefaultDriverNode::~CybergearDefaultDriverNode() {}

void CybergearDefaultDriverNode::procFeedbackJointStateCallback(
  const sensor_msgs::msg::JointState & msg)
{
  if (msg.position.size() < 1) {
    return;
  }
  m_last_sense_anguler_position = msg.position[0];
}

void CybergearDefaultDriverNode::sendCanFrameCallback(can_msgs::msg::Frame & msg)
{
  cybergear_driver_core::MoveParam move_param;

  move_param.position = getDestAngulerPosition();
  move_param.velocity = getDestAngulerVelocity();
  move_param.effort = getDestTorque();
  move_param.kp = this->params().pid_gain.kp;
  move_param.kd = this->params().pid_gain.kd;

  const auto can_frame = this->packet().createMoveCommand(move_param);
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearDefaultDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createChangeToOperationModeCommand();
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearDefaultDriverNode::subscribeJointTrajectoryPointCallback(
  const SingleJointTrajectoryPoints::SharedPtr & joint_trajectory)
{
  if (joint_trajectory->points().size() < 1) {
    return;
  }
  m_dest_joint_trajectory = joint_trajectory;
}

float CybergearDefaultDriverNode::getDestAngulerPosition()
{
  if (!m_dest_joint_trajectory) {
    return m_last_sense_anguler_position;
  } else if (0 < m_dest_joint_trajectory->points().size()) {
    return m_dest_joint_trajectory->getLerpPosition(this->get_clock()->now());
  }
  return m_last_sense_anguler_position;
}

float CybergearDefaultDriverNode::getDestAngulerVelocity()
{
  if (!m_dest_joint_trajectory) {
    return 0.0f;
  } else if (0 < m_dest_joint_trajectory->points().size()) {
    return m_dest_joint_trajectory->getLerpVelocity(this->get_clock()->now());
  }
  return 0.0f;
}

float CybergearDefaultDriverNode::getDestTorque()
{
  if (!m_dest_joint_trajectory) {
    return 0.0f;
  } else if (0 < m_dest_joint_trajectory->points().size()) {
    return m_dest_joint_trajectory->getLerpEffort(this->get_clock()->now());
  }
  return 0.0f;
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearDefaultDriverNode)
