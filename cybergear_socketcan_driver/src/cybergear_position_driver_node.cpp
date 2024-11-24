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

#include <cmath>

#include <can_msgs/msg/frame.hpp>
#include <cybergear_socketcan_driver/single_joint_trajectory_points.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "cybergear_socketcan_driver_node.hpp"

namespace cybergear_socketcan_driver
{
class CybergearPositionDriverNode : public CybergearSocketCanDriverNode
{
public:
  CybergearPositionDriverNode() = delete;
  explicit CybergearPositionDriverNode(const rclcpp::NodeOptions &);
  CybergearPositionDriverNode(const CybergearPositionDriverNode &) = delete;
  CybergearPositionDriverNode(CybergearPositionDriverNode &&) = delete;
  ~CybergearPositionDriverNode() override;

  CybergearPositionDriverNode & operator=(const CybergearPositionDriverNode &) = delete;
  CybergearPositionDriverNode & operator=(CybergearPositionDriverNode &&) = delete;

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

CybergearPositionDriverNode::CybergearPositionDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_position_driver", node_options),
  last_sense_anguler_position_(0)
{}

CybergearPositionDriverNode::~CybergearPositionDriverNode() {}

void CybergearPositionDriverNode::procFeedbackJointStateCallback(
  const sensor_msgs::msg::JointState & msg)
{
  if (msg.position.empty()) {
    return;
  }
  last_sense_anguler_position_ = static_cast<float>(msg.position[0]);
}

void CybergearPositionDriverNode::sendCanFrameFromTrajectoryCallback(
  can_msgs::msg::Frame & msg, const SingleJointTrajectoryPoints & single_joint_trajectory)
{
  float position = NAN;

  if (!single_joint_trajectory.points().empty()) {
    position = single_joint_trajectory.getLerpPosition(this->get_clock()->now());
  } else {
    position = last_sense_anguler_position_;
  }
  const auto can_frame = this->packet().createPositionCommand(position);
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}

void CybergearPositionDriverNode::sendCanFrameFromSetpointCallback(
  can_msgs::msg::Frame & msg, const cybergear_driver_msgs::msg::SetpointStamped & setpoint_msg)
{
  const auto can_frame = this->packet().createPositionCommand(setpoint_msg.point.position);
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}

void CybergearPositionDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createChangeToPositionModeCommand();
  std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
  msg.id = can_frame.id;
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearPositionDriverNode)
