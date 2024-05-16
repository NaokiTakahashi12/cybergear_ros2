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

#include <stdexcept>

#include "protocol_constant.hpp"

namespace cybergear_driver_core
{
class CybergearFrameId
{
private:
  static constexpr uint32_t FRAME_TYPE_MASK = 0xff000000;
  static constexpr uint32_t DEVICE_ID_MASK = 0x0000ff00;
  static constexpr uint32_t FEEDBACK_ERROR_MASK = 0x003f0000;
  static constexpr uint32_t FEEDBACK_MODE_STATE_MASK = 0x00c00000;

  static constexpr uint8_t FRAME_TYPE_OFFSET = 24;
  static constexpr uint8_t MODE_STATE_OFFSET = 22;
  static constexpr uint8_t DEVICE_ID_OFFSET = 8;

public:
  explicit CybergearFrameId(const uint8_t device_id, const uint8_t primary_id)
  : m_device_id(device_id),
    m_primary_id(primary_id)
  {
    if (device_id == primary_id) {
      throw std::invalid_argument("Please set different id: device vs primary(host)");
    }
  }
  ~CybergearFrameId() {}

  bool isInfo(const uint32_t id)
  {
    const uint8_t frame_type_id = (id & FRAME_TYPE_MASK) >> FRAME_TYPE_OFFSET;
    return frame_type_id == commands::INFO;
  }

  bool isFeedback(const uint32_t id)
  {
    const uint8_t frame_type_id = (id & FRAME_TYPE_MASK) >> FRAME_TYPE_OFFSET;
    return frame_type_id == commands::FEEDBACK;
  }
  bool isFault(const uint32_t id)
  {
    const uint8_t frame_type_id = (id & FRAME_TYPE_MASK) >> FRAME_TYPE_OFFSET;
    return frame_type_id == commands::FAULT_FEEDBACK;
  }

  uint8_t getFrameId(const uint32_t id)
  {
    return (id & DEVICE_ID_MASK) >> DEVICE_ID_OFFSET;
  }

  bool isDevice(const uint32_t id)
  {
    return getFrameId(id) == m_device_id;
  }

  bool hasError(const uint32_t id)
  {
    return (id & FEEDBACK_ERROR_MASK) > 0;
  }

  bool isResetMode(const uint32_t id)
  {
    const uint8_t mode = (id & FEEDBACK_MODE_STATE_MASK) >> MODE_STATE_OFFSET;
    return mode == feedback_mode_state::RESET;
  }
  bool isCaliMode(const uint32_t id)
  {
    const uint8_t mode = (id & FEEDBACK_MODE_STATE_MASK) >> MODE_STATE_OFFSET;
    return mode == feedback_mode_state::CALI;
  }
  bool isRunningMode(const uint32_t id)
  {
    const uint8_t mode = (id & FEEDBACK_MODE_STATE_MASK) >> MODE_STATE_OFFSET;
    return mode == feedback_mode_state::RUN;
  }

  uint32_t getInfoId(const uint8_t device_id)
  {
    return frameId(
      device_id, commands::INFO, m_primary_id);
  }

  uint32_t getInfoId()
  {
    return getInfoId(m_device_id);
  }
  uint32_t getCommandId(const uint16_t effort_limit)
  {
    return frameId(
      m_device_id, commands::COMMAND, effort_limit);
  }
  uint32_t getFeedbackId()
  {
    return frameId(
      m_device_id, commands::FEEDBACK, m_primary_id);
  }
  uint32_t getEnableTorqueId()
  {
    return frameId(
      m_device_id, commands::ENABLE_TORQUE, m_primary_id);
  }
  uint32_t getResetTorqueId()
  {
    return frameId(
      m_device_id, commands::RESET_TORQUE, m_primary_id);
  }
  uint32_t getZeroPositionId()
  {
    return frameId(
      m_device_id, commands::ZEROING, m_primary_id);
  }
  uint32_t getReadParameterId()
  {
    return frameId(
      m_device_id, commands::READ_PARAMETER, m_primary_id);
  }
  uint32_t getWriteParameterId()
  {
    return frameId(
      m_device_id, commands::WRITE_PARAMETER, m_primary_id);
  }

private:
  const uint8_t m_device_id;
  const uint8_t m_primary_id;

  uint32_t frameId(const uint8_t dev_id, const uint8_t cmd_id, const uint16_t opt)
  {
    return static_cast<uint32_t>(cmd_id << 24 | opt << 8 | dev_id);
  }
};
}  // namespace cybergear_driver_core
