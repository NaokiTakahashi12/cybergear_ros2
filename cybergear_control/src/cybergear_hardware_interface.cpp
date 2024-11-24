#include "cybergear_control/cybergear_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <exception>
#include <memory>
#include <string>

#include "cybergear_driver_core/cybergear_packet.hpp"
#include "cybergear_driver_core/cybergear_packet_param.hpp"
#include "cybergear_driver_core/protocol_constant.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "realtime_buffer.hpp"
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
  can_interface_ = info_.hardware_parameters["can_interface"];

  double timeout_sec = std::stod(info_.hardware_parameters["timeout_sec"]);
  timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec));

  double interval_sec = std::stod(info_.hardware_parameters["interval_sec"]);
  interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(interval_sec));

  RCLCPP_INFO(get_logger(), "interface: %s", can_interface_.c_str());
  RCLCPP_INFO(get_logger(), "timeout(s): %f", timeout_sec);
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

  RCLCPP_DEBUG(get_logger(), "Cybergear driver successfully configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TODO: Send motor enable over CAN
  is_active_ = true;

  switchCommandInterface(active_interface_);

  RCLCPP_INFO(get_logger(), "Cybergear driver activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TODO: Send motor disable over CAN
  is_active_ = false;

  // do not reset active_interface_, becase this will be reclaimed on_activate
  switchCommandInterface(cybergear_driver_core::run_modes::OPERATION);

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

  active_interface_ = cybergear_driver_core::run_modes::OPERATION;
  command_mode_ = cybergear_driver_core::run_modes::OPERATION;
  const auto& command_interface = joint.command_interfaces[0];
  if (command_interface.name == hardware_interface::HW_IF_POSITION) {
    active_interface_ = cybergear_driver_core::run_modes::POSITION;
  } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
    active_interface_ = cybergear_driver_core::run_modes::SPEED;
  } else if (command_interface.name == hardware_interface::HW_IF_TORQUE) {
    active_interface_ = cybergear_driver_core::run_modes::CURRENT;
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
  joint_commands_.assign(3, std::numeric_limits<double>::quiet_NaN());
  last_joint_commands_ = joint_commands_;

  rtb_feedback_ = realtime_tools::RealtimeBuffer<Feedback>(
      {cybergear_driver_core::CanData(), false, false, get_clock()->now()});

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> CybergearActuator::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION,
      &joint_states_[cybergear_driver_core::run_modes::POSITION]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY,
      &joint_states_[cybergear_driver_core::run_modes::SPEED]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_TORQUE,
      &joint_states_[cybergear_driver_core::run_modes::CURRENT]));

  return state_interfaces;
}

std::vector<CommandInterface> CybergearActuator::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION,
      &joint_commands_[cybergear_driver_core::run_modes::POSITION]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY,
      &joint_commands_[cybergear_driver_core::run_modes::SPEED]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_TORQUE,
      &joint_commands_[cybergear_driver_core::run_modes::CURRENT]));

  return command_interfaces;
}

hardware_interface::return_type CybergearActuator::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  // Prepare for new command modes
  std::vector<uint8_t> new_modes = {};
  for (std::string key : start_interfaces) {
    if (key ==
        info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION) {
      new_modes.push_back(cybergear_driver_core::run_modes::POSITION);
    }
    if (key ==
        info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY) {
      new_modes.push_back(cybergear_driver_core::run_modes::SPEED);
    }
    if (key == info_.joints[0].name + "/" + hardware_interface::HW_IF_TORQUE) {
      new_modes.push_back(cybergear_driver_core::run_modes::CURRENT);
    }
  }

  if (new_modes.size() > 1) {
    return hardware_interface::return_type::ERROR;
  }

  if (command_mode_ == cybergear_driver_core::run_modes::OPERATION) {
    for (const std::string& key : stop_interfaces) {
      if (key.find(info_.joints[0].name) != std::string::npos) {
        RCLCPP_ERROR(get_logger(), "Can not stop an unclaimed interface");
        return return_type::ERROR;
      }
    }
  } else {
    std::string current_interface_key = info_.joints[0].name + "/";
    switch (command_mode_) {
      case cybergear_driver_core::run_modes::POSITION:
        current_interface_key += hardware_interface::HW_IF_POSITION;
        break;
      case cybergear_driver_core::run_modes::SPEED:
        current_interface_key += hardware_interface::HW_IF_VELOCITY;
        break;
      case cybergear_driver_core::run_modes::CURRENT:
        current_interface_key += hardware_interface::HW_IF_TORQUE;
        break;
    }

    for (const std::string& key : stop_interfaces) {
      if (key.find(info_.joints[0].name) != std::string::npos) {
        if (key == current_interface_key) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "Can not stop an unclaimed interface");
        return return_type::ERROR;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CybergearActuator::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  for (const std::string& key : stop_interfaces) {
    if (key.find(info_.joints[0].name) != std::string::npos) {
      joint_commands_[command_mode_] = 0;
      active_interface_ = cybergear_driver_core::run_modes::OPERATION;
    }
  }

  for (const std::string& key : start_interfaces) {
    if (key ==
        info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION) {
      active_interface_ = cybergear_driver_core::run_modes::POSITION;
    } else if (key == info_.joints[0].name + "/" +
                          hardware_interface::HW_IF_VELOCITY) {
      active_interface_ = cybergear_driver_core::run_modes::SPEED;
    } else if (key ==
               info_.joints[0].name + "/" + hardware_interface::HW_IF_TORQUE) {
      active_interface_ = cybergear_driver_core::run_modes::CURRENT;
    }
  }

  switchCommandInterface(active_interface_);
  return hardware_interface::return_type::OK;
}

return_type CybergearActuator::read(const rclcpp::Time& time,
                                    const rclcpp::Duration& /*period*/) {
  auto feedback = rtb_feedback_.readFromRT();
  joint_states_[0] = packet_->parsePosition(feedback->data);
  joint_states_[1] = packet_->parseVelocity(feedback->data);
  joint_states_[2] = packet_->parseEffort(feedback->data);

  if (feedback->fault || feedback->error) {
    return return_type::ERROR;
  }

  // TODO: new param to define timeout time
  const auto duration = get_clock()->now() - feedback->stamp;
  RCLCPP_INFO(get_logger(), "Last feedback %f seconds old", duration.seconds());

  return return_type::OK;
}

return_type CybergearActuator::write(const rclcpp::Time& /*time*/,
                                     const rclcpp::Duration& /*period*/) {
  if (std::isnan(joint_commands_[command_mode_])) return return_type::OK;

  // the cybergear motor has its own motor controller which doesn't need to be
  // updated with the same value again.
  if (last_joint_commands_[command_mode_] == joint_commands_[command_mode_])
    return return_type::OK;

  cybergear_driver_core::CanFrame frame;
  switch (command_mode_) {
    case cybergear_driver_core::run_modes::POSITION:
      frame = packet_->createPositionCommand(
          joint_commands_[cybergear_driver_core::run_modes::POSITION]);
      break;
    case cybergear_driver_core::run_modes::SPEED:
      frame = packet_->createVelocityCommand(
          joint_commands_[cybergear_driver_core::run_modes::SPEED]);
      break;
    case cybergear_driver_core::run_modes::CURRENT:
      frame = packet_->createCurrentCommand(
          joint_commands_[cybergear_driver_core::run_modes::CURRENT]);
      break;
  }

  return send(frame);
}

void CybergearActuator::receive() {
  RCLCPP_WARN(get_logger(), "Start receiver thread");
  drivers::socketcan::CanId can_id;

  // TODO: not thread save
  const auto frame_id = packet_->frameId();
  const auto clock = get_clock();

  Feedback feedback = *rtb_feedback_.readFromNonRT();

  while (rclcpp::ok()) {
    if (is_active_) {
      std::this_thread::sleep_for(100ms);
      continue;
    }

    try {
      can_id = receiver_->receive(feedback.data.data(), interval_ns_);
    } catch (const std::exception& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Error receiving CAN message: %s - %s",
                           can_interface_.c_str(), ex.what());
      continue;
    }
    uint32_t id = can_id.identifier();
    RCLCPP_ERROR(get_logger(), "Received new can frame");

    if (!frame_id.isDevice(id)) {
      continue;
    }

    feedback.stamp = clock->now();

    // Detect fault state
    if (frame_id.isFault(id)) {
      RCLCPP_ERROR(get_logger(), "Detect fault state from cybergear");
      feedback.fault = true;
      rtb_feedback_.writeFromNonRT(feedback);
    }

    if (frame_id.isFeedback(id)) {
      if (frame_id.hasError(id)) {
        RCLCPP_ERROR(get_logger(), "Detect fault state from cybergear");
        feedback.error = true;
      }

      rtb_feedback_.writeFromNonRT(feedback);
    }
  }
}

return_type CybergearActuator::send(
    const cybergear_driver_core::CanFrame& msg) {
  using drivers::socketcan::CanId;
  using drivers::socketcan::ExtendedFrame;
  using drivers::socketcan::FrameType;

  CanId send_id(msg.id, 0, FrameType::DATA, ExtendedFrame);
  try {
    sender_->send(msg.data.data(), msg.data.size(), send_id, timeout_ns_);
  } catch (const std::exception& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Error sending CAN message: %s - %s",
                         can_interface_.c_str(), ex.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

// TODO: Evaluate if successful
return_type CybergearActuator::switchCommandInterface(
    uint8_t new_command_mode) {
  RCLCPP_INFO(get_logger(), "Send reset torque id");
  cybergear_driver_core::CanFrame frame;
  frame.id = packet_->frameId().getResetTorqueId();
  send(frame);

  RCLCPP_INFO(get_logger(), "Send change run mode");
  switch (new_command_mode) {
    case cybergear_driver_core::run_modes::OPERATION:
      frame = packet_->createChangeToOperationModeCommand();
      break;
    case cybergear_driver_core::run_modes::POSITION:
      frame = packet_->createChangeToPositionModeCommand();
      break;
    case cybergear_driver_core::run_modes::SPEED:
      frame = packet_->createChangeToVelocityModeCommand();
      break;
    case cybergear_driver_core::run_modes::CURRENT:
      frame = packet_->createChangeToCurrentModeCommand();
      break;
  }
  send(frame);

  if (new_command_mode != cybergear_driver_core::run_modes::OPERATION) {
    RCLCPP_INFO(get_logger(), "Send enable torque");
    frame.id = packet_->frameId().getEnableTorqueId();
    send(frame);
  }

  command_mode_ = new_command_mode;
  return return_type::OK;
}

}  // namespace cybergear_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(cybergear_control::CybergearActuator,
                       hardware_interface::ActuatorInterface)
