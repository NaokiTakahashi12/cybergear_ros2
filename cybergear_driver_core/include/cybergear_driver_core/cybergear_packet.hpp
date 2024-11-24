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

#include <algorithm>
#include <array>

#include "bounded_float_byte_converter.hpp"
#include "cybergear_frame_id.hpp"
#include "cybergear_packet_param.hpp"
#include "protocol_constant.hpp"
#include "scaled_float_byte_converter.hpp"

namespace cybergear_driver_core
{
struct MoveParam
{
  MoveParam()
  : position(0.0),
    velocity(0.0),
    effort(0.0),
    kp(0.0),
    kd(0.0)
  {}

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

class CybergearPacket
{
public:
  explicit CybergearPacket(const CybergearPacketParam & param)
  : frame_id_(param.device_id, param.primary_id),
    anguler_position_converter_(param.max_position, param.min_position),
    anguler_velocity_converter_(param.max_velocity, param.min_velocity),
    anguler_effort_converter_(param.max_effort, param.min_effort),
    pid_kp_converter_(param.max_gain_kp, param.min_gain_kp),
    pid_kd_converter_(param.max_gain_kd, param.min_gain_kd),
    motor_current_converter_(param.max_current, param.min_current),
    temperature_converter_(param.temperature_scale) {
  }

  ~CybergearPacket() {}

  CybergearFrameId frameId()
  {
    return frame_id_;
  }

  CanFrame createGetParamCommand(const uint16_t param_index)
  {
    CanFrame can_frame;
    can_frame.id = frame_id_.getReadParameterId();
    return can_frame;
  }

  CanFrame createMoveCommand(const MoveParam & param)
  {
    CanFrame can_frame;

    const auto cmd_pos = anguler_position_converter_.toTwoBytes(param.position);
    const auto cmd_vel = anguler_velocity_converter_.toTwoBytes(param.velocity);
    const auto cmd_effort = anguler_effort_converter_.toDoubleByte(param.effort);
    const auto kp_gain = pid_kp_converter_.toTwoBytes(param.kp);
    const auto kd_gain = pid_kd_converter_.toTwoBytes(param.kd);

    std::copy(cmd_pos.cbegin(), cmd_pos.cend(), can_frame.data.begin());
    std::copy(cmd_vel.cbegin(), cmd_vel.cend(), can_frame.data.begin() + 2);
    std::copy(kp_gain.cbegin(), kp_gain.cend(), can_frame.data.begin() + 4);
    std::copy(kd_gain.cbegin(), kd_gain.cend(), can_frame.data.begin() + 6);

    can_frame.id = frame_id_.getCommandId(cmd_effort);

    return can_frame;
  }

  CanFrame createWriteParameter(const uint16_t index, const std::array<uint8_t, 4> & param)
  {
    CanFrame can_frame;

    can_frame.data[0] = index & 0x00ff;
    can_frame.data[1] = index >> 8;
    std::copy(param.cbegin(), param.cend(), can_frame.data.begin() + 4);

    can_frame.id = frame_id_.getWriteParameterId();

    return can_frame;
  }

  CanFrame createChangeRunMode(const uint8_t mode_id)
  {
    std::array<uint8_t, 4> param;
    param[0] = mode_id;
    return createWriteParameter(ram_parameters::RUN_MODE, param);
  }

  CanFrame createPositionWithGainCommand(
    const float position, const float kp, const float kd)
  {
    MoveParam param;
    param.position = position;
    param.kp = kp;
    param.kd = kd;
    return createMoveCommand(param);
  }

  CanFrame createZeroPosition()
  {
    CanFrame can_frame;
    can_frame.data[0] = 1;
    can_frame.id = frame_id_.getZeroPositionId();
    return can_frame;
  }

  CanFrame createPositionCommand(const float position)
  {
    const auto param = anguler_position_converter_.toFourBytes(position);
    return createWriteParameter(ram_parameters::DEST_POSITION_REF, param);
  }

  CanFrame createVelocityCommand(const float velocity)
  {
    const auto param = anguler_velocity_converter_.toFourBytes(velocity);
    return createWriteParameter(ram_parameters::SPEED_REF, param);
  }

  CanFrame createCurrentCommand(const float current)
  {
    const auto param = motor_current_converter_.toFourBytes(current);
    return createWriteParameter(ram_parameters::IQ_REF, param);
  }

  CanFrame createChangeToOperationModeCommand()
  {
    return createChangeRunMode(run_modes::OPERATION);
  }

  CanFrame createChangeToPositionModeCommand()
  {
    return createChangeRunMode(run_modes::POSITION);
  }

  CanFrame createChangeToVelocityModeCommand()
  {
    return createChangeRunMode(run_modes::SPEED);
  }

  CanFrame createChangeToCurrentModeCommand()
  {
    return createChangeRunMode(run_modes::CURRENT);
  }

  float parsePosition(const CanData & data) const
  {
    return anguler_position_converter_.toFloat<8>(data, 0);
  }

  float parseVelocity(const CanData & data) const
  {
    return anguler_velocity_converter_.toFloat<8>(data, 2);
  }

  float parseEffort(const CanData & data) const
  {
    return anguler_effort_converter_.toFloat<8>(data, 4);
  }

  float parseTemperature(const CanData & data) const
  {
    return temperature_converter_.toFloat<8>(data, 6);
  }

private:
  CybergearFrameId frame_id_;
  BoundedFloatByteConverter anguler_position_converter_;
  BoundedFloatByteConverter anguler_velocity_converter_;
  BoundedFloatByteConverter anguler_effort_converter_;
  BoundedFloatByteConverter pid_kp_converter_;
  BoundedFloatByteConverter pid_kd_converter_;
  BoundedFloatByteConverter motor_current_converter_;
  ScaledFloatByteConverter temperature_converter_;
};
}  // namespace cybergear_driver_core
