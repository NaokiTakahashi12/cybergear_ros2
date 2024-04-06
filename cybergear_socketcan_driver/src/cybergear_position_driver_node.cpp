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

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <can_msgs/msg/frame.hpp>
#include <cybergear_socketcan_driver/single_joint_trajectory_points.hpp>

namespace cybergear_socketcan_driver
{
class CybergearPositionDriverNode : public CybergearSocketCanDriverNode
{
public:
  explicit CybergearPositionDriverNode(const rclcpp::NodeOptions &);
  virtual ~CybergearPositionDriverNode();

protected:
  void procFeedbackJointStateCallback(const sensor_msgs::msg::JointState &) final;
  void sendCanFrameCallback(can_msgs::msg::Frame &) final;
  void sendChangeRunModeCallback(can_msgs::msg::Frame &) final;
  void subscribeJointTrajectoryPointCallback(
    const SingleJointTrajectoryPoints::SharedPtr &) final;

private:
  float m_last_sense_anguler_position;

  std::vector<float> m_dest_anguler_positions;

  float getDestAngulerPosition();
};

CybergearPositionDriverNode::CybergearPositionDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_position_driver", node_options),
  m_last_sense_anguler_position(0),
  m_dest_anguler_positions() {}

CybergearPositionDriverNode::~CybergearPositionDriverNode() {}

void CybergearPositionDriverNode::procFeedbackJointStateCallback(
  const sensor_msgs::msg::JointState & msg)
{
  if (msg.position.size() < 1) {
    return;
  }
  m_last_sense_anguler_position = msg.position[0];
}

void CybergearPositionDriverNode::sendCanFrameCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createPositionCommand(getDestAngulerPosition());
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearPositionDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createChangeToPositionModeCommand();
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearPositionDriverNode::subscribeJointTrajectoryPointCallback(
  const SingleJointTrajectoryPoints::SharedPtr & joint_trajectory)
{
  if (!joint_trajectory) {
    return;
  } else if (joint_trajectory->points().size() < 1) {
    return;
  }
  m_dest_anguler_positions.resize(joint_trajectory->points().size());
  int point_index = 0;

  for (const auto & point : joint_trajectory->points()) {
    m_dest_anguler_positions[point_index] = point.position;
    point_index++;
  }
}

float CybergearPositionDriverNode::getDestAngulerPosition()
{
  if (0 < m_dest_anguler_positions.size()) {
    return m_dest_anguler_positions[0];
  }
  return m_last_sense_anguler_position;
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearPositionDriverNode)
