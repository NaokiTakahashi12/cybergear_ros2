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

#include <memory>
#include <atomic>
#include <vector>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <can_msgs/msg/frame.hpp>
#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <change_cybergear_id_node_parameters.hpp>

namespace cybergear_maintenance_tools
{
class ChangeCybergearIdNode : public rclcpp::Node
{
public:
  explicit ChangeCybergearIdNode(const rclcpp::NodeOptions & options);
  ~ChangeCybergearIdNode();

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_can_frame_publisher;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_can_frame_subscriber;

  rclcpp::TimerBase::SharedPtr m_send_can_frame_timer;

  std::unique_ptr<change_cybergear_id_node::ParamListener> m_param_listener;
  std::unique_ptr<change_cybergear_id_node::Params> m_params;

  std::unique_ptr<cybergear_driver_core::CybergearPacket> m_packet;

  void subscribeCanFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &);
  void sendCanFrameTimerCallback();
};

ChangeCybergearIdNode::ChangeCybergearIdNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("change_cybergear_id", options),
  m_can_frame_publisher(nullptr),
  m_can_frame_subscriber(nullptr),
  m_send_can_frame_timer(nullptr),
  m_param_listener(nullptr),
  m_params(nullptr),
  m_packet(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

  m_param_listener = std::make_unique<change_cybergear_id_node::ParamListener>(
    this->get_node_parameters_interface());
  m_params = std::make_unique<change_cybergear_id_node::Params>(
    m_param_listener->get_params());

  cybergear_driver_core::CybergearPacketParam packet_param;
  packet_param.primary_id = m_params->primary_id;
  packet_param.device_id = m_params->device_id;
  m_packet = std::make_unique<cybergear_driver_core::CybergearPacket>(packet_param);

  m_can_frame_publisher = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 3);

  m_can_frame_subscriber = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus",
    3,
    std::bind(
      &ChangeCybergearIdNode::subscribeCanFrameCallback,
      this,
      std::placeholders::_1));

  const unsigned int send_duration_milliseconds = 1e3 / m_params->send_frequency;

  m_send_can_frame_timer = this->create_wall_timer(
    std::chrono::milliseconds(send_duration_milliseconds),
    std::bind(
      &ChangeCybergearIdNode::sendCanFrameTimerCallback,
      this));
}

ChangeCybergearIdNode::~ChangeCybergearIdNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void ChangeCybergearIdNode::subscribeCanFrameCallback(
  const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  const unsigned int device_id = m_packet->frameId().getFrameId(msg->id);
  if (device_id == m_params->primary_id) {
    return;
  } else if (!m_packet->frameId().isInfo(msg->id)) {
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Found new CyberGear device: " << device_id);
}

void ChangeCybergearIdNode::sendCanFrameTimerCallback()
{
  static unsigned int callback_counter = 0;

  if (callback_counter == 0) {
    auto msg = std::make_unique<can_msgs::msg::Frame>();
    msg->header.stamp = this->get_clock()->now();
    msg->is_rtr = false;
    msg->is_extended = true;
    msg->is_error = false;
    msg->dlc = 8;
    msg->id = m_packet->frameId().getChangeDeviceId(m_params->target_id);
    m_can_frame_publisher->publish(std::move(msg));
  } else {
    const double send_duration_seconds = 1 / m_params->send_frequency;
    const unsigned int wait_count = callback_counter;
    const double wait_seconds = wait_count * send_duration_seconds;
    if (wait_seconds > m_params->wait_recive_can_frame) {
      rclcpp::shutdown();
    }
  }
  callback_counter++;
}
}  // namespace cybergear_maintenance_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_maintenance_tools::ChangeCybergearIdNode)
