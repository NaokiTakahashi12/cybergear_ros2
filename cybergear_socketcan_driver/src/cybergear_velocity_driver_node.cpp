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
class CybergearVeclocityDriverNode : public CybergearSocketCanDriverNode
{
public:
  explicit CybergearVeclocityDriverNode(const rclcpp::NodeOptions &);
  virtual ~CybergearVeclocityDriverNode();

protected:
  void sendCanFrameCallback(can_msgs::msg::Frame &) final;
  void sendChangeRunModeCallback(can_msgs::msg::Frame &) final;
  void subscribeJointTrajectoryPointCallback(
    const SingleJointTrajectoryPoints::SharedPtr &) final;

private:
  std::vector<float> m_dest_anguler_velocities;

  float getDestAngulerVelocity();
};

CybergearVeclocityDriverNode::CybergearVeclocityDriverNode(const rclcpp::NodeOptions & node_options)
: CybergearSocketCanDriverNode("cybergear_velocity_driver", node_options),
  m_dest_anguler_velocities() {}

CybergearVeclocityDriverNode::~CybergearVeclocityDriverNode() {}

void CybergearVeclocityDriverNode::sendCanFrameCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createVelocityCommand(getDestAngulerVelocity());
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearVeclocityDriverNode::sendChangeRunModeCallback(can_msgs::msg::Frame & msg)
{
  const auto can_frame = this->packet().createChangeToVelocityModeCommand();
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;
}

void CybergearVeclocityDriverNode::subscribeJointTrajectoryPointCallback(
  const SingleJointTrajectoryPoints::SharedPtr & joint_trajectory)
{
  if (!joint_trajectory) {
    return;
  } else if (joint_trajectory->points().size() < 1) {
    return;
  }
  m_dest_anguler_velocities.resize(joint_trajectory->points().size());
  int point_index = 0;

  for (const auto & point : joint_trajectory->points()) {
    m_dest_anguler_velocities[point_index] = point.velocity;
    point_index++;
  }
}

float CybergearVeclocityDriverNode::getDestAngulerVelocity()
{
  if (0 < m_dest_anguler_velocities.size()) {
    return m_dest_anguler_velocities[0];
  }
  return 0.0;
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearVeclocityDriverNode)
