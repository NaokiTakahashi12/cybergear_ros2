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

namespace cybergear_driver_core
{
struct CybergearPacketParam
{
  int device_id, primary_id;
  float max_position, min_position;  //!< Angular position [rad]
  float max_velocity, min_velocity;  //!< Angular velocity [rad/s]
  float max_effort, min_effort;      //!< Effort [N/m]
  float max_gain_kp, min_gain_kp;
  float max_gain_kd, min_gain_kd;
  float max_current, min_current;  //!< Motor current [A]
  float temperature_scale;

  CybergearPacketParam()
  : device_id(127),
    primary_id(0),
    max_position(12.56637061),
    min_position(-12.56637061),
    max_velocity(30.0),
    min_velocity(-30.0),
    max_effort(12.0),
    min_effort(-12.0),
    max_gain_kp(500.0),
    min_gain_kp(0.0),
    max_gain_kd(5.0),
    min_gain_kd(0.0),
    max_current(23.0),
    min_current(-23.0),
    temperature_scale(0.1)
  {}
};
}  // namespace cybergear_driver_core
