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

#include "cybergear_packet_param.hpp"
#include "protocol_constant.hpp"
#include "cybergear_frame_id.hpp"
#include "scaled_float_byte_converter.hpp"
#include "bounded_float_byte_converter.hpp"

namespace cybergear_socketcan_driver
{
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
