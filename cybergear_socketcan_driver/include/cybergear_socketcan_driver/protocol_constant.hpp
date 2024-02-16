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

namespace status_offset
{
constexpr uint8_t DEVICE_ID = 0;
constexpr uint8_t FAULT = 8;
constexpr uint8_t MODE = 2;
}  // namespace status_offset
}  // namespace cybergear_socketcan_driver
