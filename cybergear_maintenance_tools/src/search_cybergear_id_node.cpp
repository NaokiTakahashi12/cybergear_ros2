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
#include <search_cybergear_id_node_parameters.hpp>

namespace cybergear_maintenance_tools
{
class SearchCybergearIdNode : public rclcpp::Node
{
public:
  explicit SearchCybergearIdNode(const rclcpp::NodeOptions & options);
  ~SearchCybergearIdNode();

private:
  const unsigned int min_id_;
  const unsigned int max_id_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_subscriber_;

  rclcpp::TimerBase::SharedPtr send_can_frame_timer_;

  std::unique_ptr<search_cybergear_id_node::ParamListener> param_listener_;
  std::unique_ptr<search_cybergear_id_node::Params> params_;

  std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

  void subscribeCanFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &);
  void sendCanFrameTimerCallback();
};

SearchCybergearIdNode::SearchCybergearIdNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("search_cybergear_id", options),
  min_id_(0),
  max_id_(255),
  can_frame_publisher_(nullptr),
  can_frame_subscriber_(nullptr),
  send_can_frame_timer_(nullptr),
  param_listener_(nullptr),
  params_(nullptr),
  packet_(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

  param_listener_ = std::make_unique<search_cybergear_id_node::ParamListener>(
    this->get_node_parameters_interface());
  params_ = std::make_unique<search_cybergear_id_node::Params>(
    param_listener_->get_params());

  cybergear_driver_core::CybergearPacketParam packet_param;
  packet_param.primary_id = params_->primary_id;
  packet_ = std::make_unique<cybergear_driver_core::CybergearPacket>(packet_param);

  can_frame_publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 3);

  can_frame_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus",
    3,
    std::bind(
      &SearchCybergearIdNode::subscribeCanFrameCallback,
      this,
      std::placeholders::_1));

  const unsigned int send_duration_milliseconds = 1e3 / params_->send_frequency;

  send_can_frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_duration_milliseconds),
    std::bind(
      &SearchCybergearIdNode::sendCanFrameTimerCallback,
      this));
}

SearchCybergearIdNode::~SearchCybergearIdNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void SearchCybergearIdNode::subscribeCanFrameCallback(
  const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  const unsigned int device_id = packet_->frameId().getFrameId(msg->id);
  if (device_id == params_->primary_id) {
    return;
  } else if (!packet_->frameId().isInfo(msg->id)) {
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Found CyberGear device: " << device_id);
}

void SearchCybergearIdNode::sendCanFrameTimerCallback()
{
  static unsigned int callback_counter = 0;

  if (callback_counter < max_id_) {
    auto msg = std::make_unique<can_msgs::msg::Frame>();
    msg->header.stamp = this->get_clock()->now();
    msg->is_rtr = false;
    msg->is_extended = true;
    msg->is_error = false;
    msg->dlc = 8;
    msg->id = packet_->frameId().getInfoId(callback_counter);
    can_frame_publisher_->publish(std::move(msg));
  } else {
    const double send_duration_seconds = 1 / params_->send_frequency;
    const unsigned int wait_count = callback_counter - max_id_;
    const double wait_seconds = wait_count * send_duration_seconds;
    if (wait_seconds > params_->wait_recive_can_frame) {
      rclcpp::shutdown();
    }
  }
  callback_counter++;
}
}  // namespace cybergear_maintenance_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_maintenance_tools::SearchCybergearIdNode)
