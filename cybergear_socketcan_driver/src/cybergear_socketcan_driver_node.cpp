#include <cmath>
#include <memory>
#include <string>
#include <limits>
#include <array>
#include <chrono>
#include <functional>
#include <utility>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cybergear_socketcan_driver_node_params.hpp>
#include <cybergear_socketcan_driver/cybergear_socketcan_driver.hpp>

namespace cybergear_socketcan_driver
{

class CybergearSocketCanDriverNode : public rclcpp::Node
{
public:
  explicit CybergearSocketCanDriverNode(const rclcpp::NodeOptions &);
  ~CybergearSocketCanDriverNode();

private:
  // TODO debug
  float m_last_anguler_position;
  float m_last_anguler_velocity;

  std::unique_ptr<CybergearFrameId> m_cg_frame_id;
  std::unique_ptr<BoundedFloatByteConverter> m_anguler_position_converter;
  std::unique_ptr<BoundedFloatByteConverter> m_anguler_velocity_converter;
  std::unique_ptr<BoundedFloatByteConverter> m_anguler_effort_converter;
  std::unique_ptr<ScaledFloatByteConverter> m_temperature_converter;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_can_frame_subscriber;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_can_frame_publisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_joint_temperature_publisher;
  rclcpp::TimerBase::SharedPtr m_send_can_frame_timer;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enable_torque_service;

  std::unique_ptr<cybergear_socketcan_driver_node::ParamListener> m_param_listener;
  std::unique_ptr<cybergear_socketcan_driver_node::Params> m_params;

  void subscribeCanFrameCallback(const can_msgs::msg::Frame::ConstSharedPtr &);
  void sendCanFrameTimerCallback();
  void enableTorqueServiceCallback(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr & request,
    const std_srvs::srv::SetBool::Response::SharedPtr & response);

  void setDefaultCanFrame(can_msgs::msg::Frame::UniquePtr &);

  void sendEnableTorque();
  void sendResetTorque();
};

CybergearSocketCanDriverNode::CybergearSocketCanDriverNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("cybergear_socketcan_driver",
    rclcpp::NodeOptions(node_options).use_intra_process_comms(true)),
  m_cg_frame_id(nullptr),
  m_anguler_position_converter(nullptr),
  m_anguler_velocity_converter(nullptr),
  m_anguler_effort_converter(nullptr),
  m_temperature_converter(nullptr),
  m_can_frame_subscriber(nullptr),
  m_can_frame_publisher(nullptr),
  m_joint_state_publisher(nullptr),
  m_joint_temperature_publisher(nullptr),
  m_send_can_frame_timer(nullptr),
  m_enable_torque_service(nullptr),
  m_param_listener(nullptr),
  m_params(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());
  RCLCPP_WARN(this->get_logger(), "THIS NODE IS UNDER DEVELOPMENT");

  m_param_listener = std::make_unique<cybergear_socketcan_driver_node::ParamListener>(
    this->get_node_parameters_interface());
  m_params = std::make_unique<cybergear_socketcan_driver_node::Params>(
    m_param_listener->get_params());

  m_cg_frame_id = std::make_unique<CybergearFrameId>(
    m_params->device_id,
    m_params->primary_id);
  m_anguler_position_converter = std::make_unique<BoundedFloatByteConverter>(
    m_params->anguler_position.max,
    m_params->anguler_position.min);
  m_anguler_velocity_converter = std::make_unique<BoundedFloatByteConverter>(
    m_params->anguler_velocity.max,
    m_params->anguler_velocity.min);
  m_anguler_effort_converter = std::make_unique<BoundedFloatByteConverter>(
    m_params->anguler_effort.max,
    m_params->anguler_effort.min);
  m_temperature_converter = std::make_unique<ScaledFloatByteConverter>(
    m_params->temperature.scale);

  m_can_frame_publisher = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 3);
  m_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>(
    "~/joint_state", 3);
  m_joint_temperature_publisher = this->create_publisher<sensor_msgs::msg::Temperature>(
    "~/temperature", 3);
  m_can_frame_subscriber = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus",
    3,
    std::bind(
      &CybergearSocketCanDriverNode::subscribeCanFrameCallback,
      this,
      std::placeholders::_1
    )
  );

  const unsigned int send_duration_milliseconds = 1e3 / m_params->send_frequency;

  m_send_can_frame_timer = this->create_wall_timer(
    std::chrono::milliseconds(send_duration_milliseconds),
    std::bind(&CybergearSocketCanDriverNode::sendCanFrameTimerCallback, this));

  m_enable_torque_service = this->create_service<std_srvs::srv::SetBool>(
    "~/enable_torque",
    std::bind(
      &CybergearSocketCanDriverNode::enableTorqueServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
}

CybergearSocketCanDriverNode::~CybergearSocketCanDriverNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void CybergearSocketCanDriverNode::subscribeCanFrameCallback(
  const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  if (!m_cg_frame_id->isDevice(msg->id)) {
    return;
  }
  // TODO m_params->wait_power_on
  // TODO temperature publish with timestamp
  // TODO always feedback subscribe

  if (m_cg_frame_id->isFault(msg->id)) {
    RCLCPP_ERROR(this->get_logger(), "Detect fault state from cybergear");
    std::string can_frame_data;
    for (const auto & d : msg->data) {
      can_frame_data += std::to_string(d) + ", ";
    }
    RCLCPP_ERROR_STREAM(this->get_logger(), "data: " << can_frame_data);
    rclcpp::shutdown();
  }
  if (!m_cg_frame_id->isFeedback(msg->id)) {
    return;
  }
  if (m_cg_frame_id->hasError(msg->id)) {
    RCLCPP_ERROR(this->get_logger(), "Detect error state from cybergear");
  }
  if (m_cg_frame_id->isResetMode(msg->id)) {
    RCLCPP_INFO(this->get_logger(), "Reset mode now");
  }
  if (m_cg_frame_id->isRunningMode(msg->id)) {
    RCLCPP_INFO(this->get_logger(), "Running mode now");
  }
  std_msgs::msg::Header header_msg;
  header_msg.stamp = this->get_clock()->now();
  header_msg.frame_id = m_params->joint_name;

  if (m_joint_state_publisher) {
    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state_msg->header = header_msg;
    joint_state_msg->name.push_back(m_params->joint_name);
    joint_state_msg->position.push_back(m_anguler_position_converter->toFloat<8>(msg->data, 0));
    joint_state_msg->velocity.push_back(m_anguler_velocity_converter->toFloat<8>(msg->data, 2));
    joint_state_msg->effort.push_back(m_anguler_effort_converter->toFloat<8>(msg->data, 4));
    m_last_anguler_position = joint_state_msg->position.at(0);
    m_last_anguler_velocity = joint_state_msg->velocity.at(0);
    m_joint_state_publisher->publish(std::move(joint_state_msg));
  }
  // TODO imple
  if (m_joint_temperature_publisher) {
    auto temperature_msg = std::make_unique<sensor_msgs::msg::Temperature>();
    temperature_msg->header = header_msg;
    temperature_msg->temperature = m_temperature_converter->toFloat<8>(msg->data, 6);
    temperature_msg->variance = 0.0;  // unknown
    m_joint_temperature_publisher->publish(std::move(temperature_msg));
  }
}

void CybergearSocketCanDriverNode::sendCanFrameTimerCallback()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);

  decltype(auto) cmd_pos = m_anguler_position_converter->toByte(m_last_anguler_position);
  std::copy(cmd_pos.begin(), cmd_pos.end(), msg->data.begin());

  decltype(auto) cmd_vel = m_anguler_velocity_converter->toByte(m_last_anguler_velocity);
  std::copy(cmd_vel.begin(), cmd_vel.end(), msg->data.begin() + 2);

  // TODO imple
  msg->data[5] = 0x0f;
  msg->data[7] = 0x01;

  msg->id = m_cg_frame_id->getCommandId(
    m_anguler_effort_converter->toDoubleByte(0.0));
  m_can_frame_publisher->publish(std::move(msg));
}

// TODO wait for result
void CybergearSocketCanDriverNode::enableTorqueServiceCallback(
  const std_srvs::srv::SetBool::Request::ConstSharedPtr & request,
  const std_srvs::srv::SetBool::Response::SharedPtr & response)
{
  RCLCPP_INFO(this->get_logger(), "Calling enableTorqueServiceCallback");
  if (request->data) {
    sendEnableTorque();
    response->message = "Sent enable torque message";
  } else {
    sendResetTorque();
    response->message = "Sent reset torque message";
  }
  response->success = true;
  RCLCPP_INFO_STREAM(this->get_logger(), response->message);
}

void CybergearSocketCanDriverNode::setDefaultCanFrame(can_msgs::msg::Frame::UniquePtr & msg)
{
  // TODO m_params->wait_power_on
  if (!msg) {
    return;
  }
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = m_params->joint_name;
  msg->is_rtr = false;
  msg->is_extended = true;
  msg->is_error = false;
  msg->dlc = 8;
}

void CybergearSocketCanDriverNode::sendEnableTorque()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  msg->id = m_cg_frame_id->getEnableTorqueId();
  m_can_frame_publisher->publish(std::move(msg));
}

void CybergearSocketCanDriverNode::sendResetTorque()
{
  auto msg = std::make_unique<can_msgs::msg::Frame>();
  setDefaultCanFrame(msg);
  msg->id = m_cg_frame_id->getResetTorqueId();
  m_can_frame_publisher->publish(std::move(msg));
}
}  // namespace cybergear_socketcan_driver

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_socketcan_driver::CybergearSocketCanDriverNode)
