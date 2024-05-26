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

#include <atomic>
#include <memory>
#include <utility>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <change_cybergear_id_node_parameters.hpp>
#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace cybergear_maintenance_tools
{
class ChangeCybergearIdNode : public rclcpp::Node
{
public:
  ChangeCybergearIdNode() = delete;
  explicit ChangeCybergearIdNode(const rclcpp::NodeOptions & options);
  ChangeCybergearIdNode(const ChangeCybergearIdNode &) = delete;
  ChangeCybergearIdNode(ChangeCybergearIdNode &&) = delete;
  ~ChangeCybergearIdNode() override;

  ChangeCybergearIdNode & operator=(const ChangeCybergearIdNode &) = delete;
  ChangeCybergearIdNode & operator=(ChangeCybergearIdNode &&) = delete;

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_subscriber_;

  rclcpp::TimerBase::SharedPtr send_can_frame_timer_;

  std::unique_ptr<change_cybergear_id_node::ParamListener> param_listener_;
  std::unique_ptr<change_cybergear_id_node::Params> params_;

  std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

  void subscribeCanFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &);
  void sendCanFrameTimerCallback();
};

ChangeCybergearIdNode::ChangeCybergearIdNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("change_cybergear_id", options),
  can_frame_publisher_(nullptr),
  can_frame_subscriber_(nullptr),
  send_can_frame_timer_(nullptr),
  param_listener_(nullptr),
  params_(nullptr),
  packet_(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

  param_listener_ = std::make_unique<change_cybergear_id_node::ParamListener>(
    this->get_node_parameters_interface());
  params_ = std::make_unique<change_cybergear_id_node::Params>(param_listener_->get_params());

  cybergear_driver_core::CybergearPacketParam packet_param;
  packet_param.primary_id = static_cast<int>(params_->primary_id);
  packet_param.device_id = static_cast<int>(params_->device_id);
  packet_ = std::make_unique<cybergear_driver_core::CybergearPacket>(packet_param);

  can_frame_publisher_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 3);

  can_frame_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus",
    3,
    std::bind(&ChangeCybergearIdNode::subscribeCanFrameCallback, this, std::placeholders::_1));

  const auto send_duration_milliseconds = static_cast<unsigned int>(1e3 / params_->send_frequency);

  send_can_frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_duration_milliseconds),
    std::bind(&ChangeCybergearIdNode::sendCanFrameTimerCallback, this));
}

ChangeCybergearIdNode::~ChangeCybergearIdNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void ChangeCybergearIdNode::subscribeCanFrameCallback(
  const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  const unsigned int device_id = packet_->frameId().getFrameId(msg->id);
  if (device_id == params_->primary_id) {
    return;
  }
  if (!packet_->frameId().isInfo(msg->id)) {
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Found new CyberGear device: " << device_id);
}

void ChangeCybergearIdNode::sendCanFrameTimerCallback()
{
  constexpr uint8_t kDlc = 8;
  static unsigned int callback_counter = 0;

  if (callback_counter == 0) {
    auto msg = std::make_unique<can_msgs::msg::Frame>();
    msg->header.stamp = this->get_clock()->now();
    msg->is_rtr = false;
    msg->is_extended = true;
    msg->is_error = false;
    msg->dlc = kDlc;
    msg->id = packet_->frameId().getChangeDeviceId(params_->target_id);
    can_frame_publisher_->publish(std::move(msg));
  } else {
    const double send_duration_seconds = 1 / params_->send_frequency;
    const unsigned int wait_count = callback_counter;
    const double wait_seconds = wait_count * send_duration_seconds;
    if (wait_seconds > params_->wait_recive_can_frame) {
      rclcpp::shutdown();
    }
  }
  callback_counter++;
}
}  // namespace cybergear_maintenance_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_maintenance_tools::ChangeCybergearIdNode)
