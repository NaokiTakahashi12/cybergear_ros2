#include "cybergear_control/cybergear_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <string>

#include "cybergear_driver_core/cybergear_packet.hpp"
#include "cybergear_driver_core/cybergear_packet_param.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "ros2_socketcan/socket_can_id.hpp"

using namespace std::chrono_literals;

namespace cybergear_control {
bool stob(std::string s) {
  auto result = false;  // failure to assert is false

  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  std::istringstream is(s);
  // first try simple integer conversion
  is >> result;

  if (is.fail()) {
    // simple integer failed; try boolean
    is.clear();
    is >> std::boolalpha >> result;
  }

  if (is.fail()) {
    throw std::invalid_argument(s.append(" is not convertable to bool"));
  }

  return result;
}

CallbackReturn CybergearActuator::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "PLUGIN ON CONFIGURE START");

  // read hardware params
  can_interface_ = info_.hardware_parameters["can_interface"];

  double timeout_sec = std::stod(info_.hardware_parameters["timeout_sec"]);
  timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec));

  use_bus_time_ = stob(info_.hardware_parameters["use_bus_time"]);
  double interval_sec = std::stod(info_.hardware_parameters["interval_sec"]);
  interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(interval_sec));

  RCLCPP_INFO(get_logger(), "interface: %s", can_interface_.c_str());
  RCLCPP_INFO(get_logger(), "timeout(s): %f", timeout_sec);
  RCLCPP_INFO(get_logger(), "use bus time: %s",
              use_bus_time_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "interval(s): %f", interval_sec);

  cybergear_driver_core::CybergearPacketParam params;
  params.device_id = std::stoi(info_.hardware_parameters["device_id"]);
  params.primary_id = std::stoi(info_.hardware_parameters["primary_id"]);
  params.max_position = std::stof(info_.hardware_parameters["max_position"]);
  params.min_position = std::stof(info_.hardware_parameters["min_position"]);
  params.max_velocity = std::stof(info_.hardware_parameters["max_velocity"]);
  params.min_velocity = std::stof(info_.hardware_parameters["min_velocity"]);
  params.max_effort = std::stof(info_.hardware_parameters["max_effort"]);
  params.min_effort = std::stof(info_.hardware_parameters["min_effort"]);
  params.max_gain_kp = std::stof(info_.hardware_parameters["max_gain_kp"]);
  params.min_gain_kp = std::stof(info_.hardware_parameters["min_gain_kp"]);
  params.max_gain_kd = std::stof(info_.hardware_parameters["max_gain_kd"]);
  params.min_gain_kd = std::stof(info_.hardware_parameters["min_gain_kd"]);
  params.max_current = std::stof(info_.hardware_parameters["max_current"]);
  params.min_current = std::stof(info_.hardware_parameters["min_current"]);
  params.temperature_scale =
      std::stof(info_.hardware_parameters["temperature_scale"]);

  RCLCPP_INFO(get_logger(), "device_id: %d", params.device_id);
  RCLCPP_INFO(get_logger(), "primary_id: %d", params.primary_id);
  RCLCPP_INFO(get_logger(), "max_position: %f", params.max_position);
  RCLCPP_INFO(get_logger(), "min_position: %f", params.min_position);
  RCLCPP_INFO(get_logger(), "max_velocity: %f", params.max_velocity);
  RCLCPP_INFO(get_logger(), "min_velocity: %f", params.min_velocity);
  RCLCPP_INFO(get_logger(), "max_effort: %f", params.max_effort);
  RCLCPP_INFO(get_logger(), "min_effort: %f", params.min_effort);
  RCLCPP_INFO(get_logger(), "max_gain_kp: %f", params.max_gain_kp);
  RCLCPP_INFO(get_logger(), "min_gain_kp: %f", params.min_gain_kp);
  RCLCPP_INFO(get_logger(), "max_gain_kd: %f", params.max_gain_kd);
  RCLCPP_INFO(get_logger(), "min_gain_kd: %f", params.min_gain_kd);
  RCLCPP_INFO(get_logger(), "max_current: %f", params.max_current);
  RCLCPP_INFO(get_logger(), "min_current: %f", params.min_current);
  RCLCPP_INFO(get_logger(), "temperature_scale: %f", params.temperature_scale);

  packet_ = std::make_unique<cybergear_driver_core::CybergearPacket>(params);

  try {
    sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(
        can_interface_, false);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Error opening CAN sender: %s - %s",
                 can_interface_.c_str(), ex.what());
    return CallbackReturn::FAILURE;
  }

  try {
    receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(
        can_interface_, false);
    // apply CAN filters
    receiver_->SetCanFilters(
        drivers::socketcan::SocketCanReceiver::CanFilterList(can_filters_));
    RCLCPP_DEBUG(get_logger(), "applied filters: %s", can_filters_.c_str());
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Error opening CAN receiver: %s - %s",
                 can_interface_.c_str(), ex.what());
    return CallbackReturn::FAILURE;
  }
  receiver_thread_ = std::thread(&CybergearActuator::receive, this);

  is_active_ = false;

  RCLCPP_DEBUG(get_logger(), "Cybergear driver successfully configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TODO: Send motor enable over CAN
  is_active_ = true;

  joint_states_[0] = 0;
  joint_states_[1] = 0;
  joint_states_[2] = 0;

  switchCommandInterface();

  RCLCPP_INFO(get_logger(), "Cybergear driver activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TODO: Send motor disable over CAN
  is_active_ = false;

  RCLCPP_DEBUG(get_logger(), "Cybergear driver deactivate.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (receiver_thread_.joinable()) {
    receiver_thread_.join();
  }

  RCLCPP_DEBUG(get_logger(), "Cybergear driver cleaned up.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_DEBUG(get_logger(), "Cybergear driver shutting down.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_ERROR(get_logger(), "Cybergear driver error.");
  return CallbackReturn::FAILURE;
}

CallbackReturn CybergearActuator::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(get_logger(), "PLUGIN ON INIT START");

  if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // only one joint can be controlled
  if (info.joints.size() != 1) {
    RCLCPP_FATAL(get_logger(),
                 "Hardware interface has '%zu' joints. 1 expected.",
                 info.joints.size());
    return CallbackReturn::ERROR;
  }

  const hardware_interface::ComponentInfo& joint = info.joints[0];

  // can give feedback on position, velocity and torque
  state_interface_position_ = false;
  state_interface_velocity_ = false;
  state_interface_torque_ = false;
  if (joint.state_interfaces.size() > 3) {
    RCLCPP_FATAL(get_logger(),
                 "Joint '%s' has to many state interfaces. Any combination of "
                 "'%s', '%s' and '%s' expected.",
                 joint.name.c_str(), hardware_interface::HW_IF_POSITION,
                 hardware_interface::HW_IF_VELOCITY,
                 hardware_interface::HW_IF_TORQUE);
    return CallbackReturn::ERROR;
  }

  for (const auto& state_interface : joint.state_interfaces) {
    if (state_interface.name == hardware_interface::HW_IF_POSITION) {
      state_interface_position_ = true;
    } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
      state_interface_velocity_ = true;
    } else if (state_interface.name == hardware_interface::HW_IF_TORQUE) {
      state_interface_torque_ = true;
    } else {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has incompatible state interface '%s'. Any "
                   "combination of '%s', '%s' and '%s' expected.",
                   joint.name.c_str(), state_interface.name.c_str(),
                   hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY,
                   hardware_interface::HW_IF_TORQUE);
    }
  }

  // can only be controlled by either position, velocity or torque
  if (joint.command_interfaces.size() != 1) {
    RCLCPP_FATAL(get_logger(),
                 "Joint '%s' has %zu command interfaces found. 1 expected.",
                 joint.name.c_str(), joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto& command_interface = joint.command_interfaces[0];
  if (command_interface.name == hardware_interface::HW_IF_POSITION) {
    command_type_ = CommandInterfaceType::POSITION;
  } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
    command_type_ = CommandInterfaceType::VELOCITY;
  } else if (command_interface.name == hardware_interface::HW_IF_TORQUE) {
    command_type_ = CommandInterfaceType::TORQUE;
  } else {
    RCLCPP_FATAL(get_logger(),
                 "Joint '%s' can not be controlled with '%s' command "
                 "interfaces. '%s', '%s' or '%s' expected.",
                 joint.name.c_str(), command_interface.name.c_str(),
                 hardware_interface::HW_IF_POSITION,
                 hardware_interface::HW_IF_VELOCITY,
                 hardware_interface::HW_IF_TORQUE);
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Using '%s' interface on '%s'",
              command_interface.name.c_str(), joint.name.c_str());

  joint_states_.assign(3, std::numeric_limits<double>::quiet_NaN());
  joint_command_ = std::numeric_limits<double>::quiet_NaN();
  last_joint_command_ = joint_command_;

  constexpr uint8_t kDlc = 8;
  joint_command_template_ =
      can_msgs::msg::Frame(rosidl_runtime_cpp::MessageInitialization::ZERO);
  joint_command_template_.header.frame_id = info_.joints[0].name;
  joint_command_template_.header.stamp = get_clock()->now();
  joint_command_template_.is_rtr = false;
  joint_command_template_.is_extended = false;
  joint_command_template_.is_error = false;
  joint_command_template_.dlc = kDlc;

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> CybergearActuator::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION,
      &joint_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY,
      &joint_states_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_TORQUE,
      &joint_states_[2]));

  return state_interfaces;
}

std::vector<CommandInterface> CybergearActuator::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;

  if (command_type_ == CommandInterfaceType::POSITION) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION,
        &joint_command_));
  } else if (command_type_ == CommandInterfaceType::VELOCITY) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY,
        &joint_command_));
  } else if (command_type_ == CommandInterfaceType::TORQUE) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_TORQUE,
        &joint_command_));
  }

  return command_interfaces;
}

hardware_interface::return_type CybergearActuator::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  // TODO:
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CybergearActuator::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  // TODO:
  return hardware_interface::return_type::OK;
}

return_type CybergearActuator::read(const rclcpp::Time& time,
                                    const rclcpp::Duration& period) {
  // Implement the read method getting the states from the hardware and
  // storing them to internal variables defined in export_state_interfaces.

  switch (command_type_) {
    case CommandInterfaceType::POSITION:
      joint_states_[0] = joint_command_;
      joint_states_[1] = 0;
      joint_states_[2] = 0;
      break;
    case CommandInterfaceType::VELOCITY:
      joint_states_[0] += joint_command_ * period.seconds();
      joint_states_[1] = joint_command_;
      joint_states_[2] = 0;
      break;
    case CommandInterfaceType::TORQUE:
      joint_states_[0] = 0;
      joint_states_[1] = 0;
      joint_states_[2] = joint_command_;
      break;
  }

  RCLCPP_INFO(get_logger(), "Joint command: %f", joint_command_);
  RCLCPP_INFO(get_logger(), "Cybergear joint states %f, %f, %f",
              joint_states_[0], joint_states_[1], joint_states_[2]);

  return return_type::OK;
}

return_type CybergearActuator::write(const rclcpp::Time& time,
                                     const rclcpp::Duration& period) {
  if (std::isnan(joint_command_)) return return_type::OK;

  // testing
  if (joint_command_ != joint_states_[1]) {
    joint_states_[0] = 0;
    joint_states_[1] = joint_command_;
    joint_states_[2] = 0;
  }

  can_msgs::msg::Frame msg = joint_command_template_;
  msg.header.stamp = get_clock()->now();

  cybergear_driver_core::CanFrameUniquePtr can_frame;
  switch (command_type_) {
    case CommandInterfaceType::POSITION:
      can_frame = packet_->createPositionCommand(joint_command_);
      break;
    case CommandInterfaceType::VELOCITY:
      can_frame = packet_->createVelocityCommand(joint_command_);
      break;
    case CommandInterfaceType::TORQUE:
      can_frame = packet_->createCurrentCommand(joint_command_);
      break;
  }
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;

  return send(msg);
}

void CybergearActuator::receive() {
  using drivers::socketcan::FrameType;

  drivers::socketcan::CanId receive_id{};

  can_msgs::msg::Frame frame(rosidl_runtime_cpp::MessageInitialization::ZERO);
  frame.header.frame_id = info_.joints[0].name;

  while (rclcpp::ok()) {
    if (is_active_) {
      std::this_thread::sleep_for(100ms);
      continue;
    }

    try {
      receive_id = receiver_->receive(frame.data.data(), interval_ns_);
    } catch (const std::exception& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Error receiving CAN message: %s - %s",
                           can_interface_.c_str(), ex.what());
      continue;
    }

    if (use_bus_time_) {
      frame.header.stamp =
          rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
    } else {
      frame.header.stamp = get_clock()->now();
    }

    frame.id = receive_id.identifier();
    frame.is_rtr = (receive_id.frame_type() == FrameType::REMOTE);
    frame.is_extended = receive_id.is_extended();
    frame.is_error = (receive_id.frame_type() == FrameType::ERROR);
    frame.dlc = receive_id.length();

    {
      std::lock_guard<std::mutex> guard(last_frame_mutex_);
      last_received_frame_ = frame;
    }
  }
}

// TODO: send message data directly and remove dependency on can_msgs
return_type CybergearActuator::send(const can_msgs::msg::Frame& msg) {
  using drivers::socketcan::CanId;
  using drivers::socketcan::FrameType;
  using drivers::socketcan::StandardFrame;

  RCLCPP_INFO(get_logger(), "Send can msg with id %d", msg.id);
  CanId send_id(msg.id, 0, FrameType::DATA, StandardFrame);
  RCLCPP_INFO(get_logger(), "CanId constructed");
  try {
    sender_->send(msg.data.data(), msg.dlc, send_id, timeout_ns_);
  } catch (const std::exception& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Error sending CAN message: %s - %s",
                         can_interface_.c_str(), ex.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type CybergearActuator::switchCommandInterface() {
  can_msgs::msg::Frame msg = joint_command_template_;
  msg.header.stamp = get_clock()->now();

  cybergear_driver_core::CanFrameUniquePtr can_frame;
  switch (command_type_) {
    case CommandInterfaceType::POSITION:
      can_frame = packet_->createChangeToPositionModeCommand();
      break;
    case CommandInterfaceType::VELOCITY:
      can_frame = packet_->createChangeToVelocityModeCommand();
      break;
    case CommandInterfaceType::TORQUE:
      can_frame = packet_->createChangeToCurrentModeCommand();
      break;
  }
  std::copy(can_frame->data.cbegin(), can_frame->data.cend(), msg.data.begin());
  msg.id = can_frame->id;

  RCLCPP_INFO(get_logger(), "Send change to command interface mode msg");
  return send(msg);
}

}  // namespace cybergear_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(cybergear_control::CybergearActuator,
                       hardware_interface::ActuatorInterface)
