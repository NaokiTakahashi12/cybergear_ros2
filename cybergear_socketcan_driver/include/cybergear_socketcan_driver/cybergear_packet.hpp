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

#include <memory>
#include <array>
#include <algorithm>

#include "cybergear_packet_param.hpp"
#include "protocol_constant.hpp"
#include "cybergear_frame_id.hpp"
#include "scaled_float_byte_converter.hpp"
#include "bounded_float_byte_converter.hpp"

namespace cybergear_socketcan_driver
{
struct MoveParam
{
  MoveParam()
  : position(0.0),
    velocity(0.0),
    effort(0.0),
    kp(0.0),
    kd(0.0) {}
  float position;
  float velocity;
  float effort;
  float kp;
  float kd;
};

using CanData = std::array<uint8_t, 8>;
struct CanFrame
{
  uint32_t id;
  CanData data;
};

using CanFrameUniquePtr = std::unique_ptr<CanFrame>;

class CybergearPacket
{
public:
  explicit CybergearPacket(const CybergearPacketParam & param)
  : m_frame_id(nullptr),
    m_anguler_position_converter(nullptr),
    m_anguler_velocity_converter(nullptr),
    m_anguler_effort_converter(nullptr),
    m_pid_kp_converter(nullptr),
    m_pid_kd_converter(nullptr),
    m_temperature_converter(nullptr)
  {
    m_frame_id = std::make_unique<CybergearFrameId>(
      param.device_id,
      param.primary_id);
    m_anguler_position_converter = std::make_unique<BoundedFloatByteConverter>(
      param.max_position,
      param.min_position);
    m_anguler_velocity_converter = std::make_unique<BoundedFloatByteConverter>(
      param.max_velocity,
      param.min_velocity);
    m_anguler_effort_converter = std::make_unique<BoundedFloatByteConverter>(
      param.max_effort,
      param.min_effort);
    m_pid_kp_converter = std::make_unique<BoundedFloatByteConverter>(
      param.max_gain_kp,
      param.min_gain_kp);
    m_pid_kd_converter = std::make_unique<BoundedFloatByteConverter>(
      param.max_gain_kd,
      param.min_gain_kd);
    m_temperature_converter = std::make_unique<ScaledFloatByteConverter>(
      param.temperature_scale);
  }

  ~CybergearPacket() {}

  CanFrameUniquePtr createMoveCommand(const MoveParam & param)
  {
    auto can_frame = std::make_unique<CanFrame>();

    const auto cmd_pos = m_anguler_position_converter->toByte(param.position);
    const auto cmd_vel = m_anguler_velocity_converter->toByte(param.velocity);
    const auto cmd_effort = m_anguler_effort_converter->toDoubleByte(param.effort);
    const auto kp_gain = m_pid_kp_converter->toByte(param.kp);
    const auto kd_gain = m_pid_kd_converter->toByte(param.kd);

    std::copy(cmd_pos.cbegin(), cmd_pos.cend(), can_frame->data.begin());
    std::copy(cmd_vel.cbegin(), cmd_vel.cend(), can_frame->data.begin() + 2);
    std::copy(kp_gain.cbegin(), kp_gain.cend(), can_frame->data.begin() + 4);
    std::copy(kd_gain.cbegin(), kd_gain.cend(), can_frame->data.begin() + 6);

    can_frame->id = m_frame_id->getCommandId(cmd_effort);

    return can_frame;
  }

  CanFrameUniquePtr createPositionCommand(const float position, const float kp, const float kd)
  {
    MoveParam param;
    param.position = position;
    param.kp = kp;
    param.kd = kd;
    return createMoveCommand(param);
  }

  //! @todo kp
  CanFrameUniquePtr createVelocityCommand(const float velocity, const float kp)
  {
    MoveParam param;
    param.velocity = velocity;
    param.kp = kp;
    return createMoveCommand(param);
  }

  CanFrameUniquePtr createEffortCommand(const float effort)
  {
    MoveParam param;
    param.velocity = effort;
    return createMoveCommand(param);
  }

  float persePosition(const CanData & data) const
  {
    return m_anguler_position_converter->toFloat<8>(data, 0);
  }

  float perseVelocity(const CanData & data) const
  {
    return m_anguler_velocity_converter->toFloat<8>(data, 2);
  }

  float perseEffort(const CanData & data) const
  {
    return m_anguler_effort_converter->toFloat<8>(data, 4);
  }

  float perseTemperature(const CanData & data) const
  {
    return m_temperature_converter->toFloat<8>(data, 6);
  }

private:
  std::unique_ptr<CybergearFrameId> m_frame_id;
  std::unique_ptr<BoundedFloatByteConverter> m_anguler_position_converter;
  std::unique_ptr<BoundedFloatByteConverter> m_anguler_velocity_converter;
  std::unique_ptr<BoundedFloatByteConverter> m_anguler_effort_converter;
  std::unique_ptr<BoundedFloatByteConverter> m_pid_kp_converter;
  std::unique_ptr<BoundedFloatByteConverter> m_pid_kd_converter;
  std::unique_ptr<ScaledFloatByteConverter> m_temperature_converter;
};
}  // namespace cybergear_socketcan_driver
