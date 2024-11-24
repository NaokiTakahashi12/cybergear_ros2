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

#pragma once

#include <cstdint>

namespace cybergear_driver_core
{
namespace commands
{
constexpr uint8_t INFO = 0;
constexpr uint8_t COMMAND = 1;
constexpr uint8_t FEEDBACK = 2;
constexpr uint8_t ENABLE_TORQUE = 3;
constexpr uint8_t RESET_TORQUE = 4;
constexpr uint8_t ZEROING = 6;
constexpr uint8_t CHANGE_DEVICE_ID = 7;
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

namespace status_offset
{
constexpr uint8_t DEVICE_ID = 0;
constexpr uint8_t FAULT = 8;
constexpr uint8_t MODE = 2;
}  // namespace status_offset

// TODO(Naoki Takahashi) address constant header
namespace ram_parameters
{
constexpr uint16_t RUN_MODE = 0x7005;
constexpr uint16_t IQ_REF = 0x7006;
constexpr uint16_t SPEED_REF = 0x700a;
constexpr uint16_t LIMIT_TORQUE = 0x700b;
constexpr uint16_t CURRENT_KP = 0x7010;
constexpr uint16_t CURRENT_KI = 0x7011;
constexpr uint16_t CURRENT_FILTER_GAIN = 0x7014;
constexpr uint16_t DEST_POSITION_REF = 0x7016;
constexpr uint16_t LIMIT_SPEED = 0x7017;
constexpr uint16_t LIMIT_CURRENT = 0x7018;
constexpr uint16_t MECH_POSISION = 0x7019;
constexpr uint16_t IQF = 0x701a;
constexpr uint16_t MECH_VELOCITY = 0x701b;
constexpr uint16_t VBUS = 0x701c;
constexpr uint16_t ROTATION_COUNT = 0x701d;
constexpr uint16_t POSITION_KP = 0x701e;
constexpr uint16_t SPEED_KP = 0x701f;
constexpr uint16_t SPEED_KI = 0x7020;
}  // namespace ram_parameters

namespace run_modes
{
constexpr uint8_t OPERATION = 0;
constexpr uint8_t POSITION = 1;
constexpr uint8_t SPEED = 2;
constexpr uint8_t CURRENT = 3;
}  // namespace run_modes
}  // namespace cybergear_driver_core
