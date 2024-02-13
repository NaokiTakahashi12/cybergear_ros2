#pragma once

namespace cybergear_socketcan_driver
{
namespace commands
{
constexpr uint8_t INFO = 0;
constexpr uint8_t COMMAND = 1;
constexpr uint8_t FEEDBACK = 2;
constexpr uint8_t ENABLE_TORQUE = 3;
constexpr uint8_t RESET_TORQUE = 4;
constexpr uint8_t ZEROING = 5;
constexpr uint8_t CHANGE_DEVICE_ID = 6;
constexpr uint8_t READ_PARAMETER = 17;
constexpr uint8_t WRITE_PARAMETER = 18;
constexpr uint8_t FAULT_FEEDBACK = 21;
constexpr uint8_t CHANGE_BAUDRATE = 22;
}  // namespace commands

namespace feedback_mode_state
{
constexpr uint8_t RESET = 0;
constexpr uint8_t CALI = 1;
constexpr uint8_t RUN = 2;
}  // namespace feedback_mode_state

namespace CybergearStatusOffset
{
constexpr uint8_t DEVICE_ID = 0;
constexpr uint8_t FAULT = 8;
constexpr uint8_t MODE = 2;
}  // namespace status_offset
}  // namespace cybergear_socketcan_driver
