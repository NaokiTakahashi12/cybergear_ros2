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

#include <array>
#include <cstdint>

namespace cybergear_driver_core
{
// TODO(Naoki Takahashi) endian support
class ScaledFloatByteConverter
{
public:
  explicit ScaledFloatByteConverter(const float scale)
  : scale_(scale)
  {}

  ~ScaledFloatByteConverter() {}

  void setScale(const float scale)
  {
    scale_ = scale;
  }

  uint16_t toDoubleByte(const float value)
  {
    return static_cast<uint16_t>(value / scale_);
  }

  std::array<uint8_t, 2> toTwoBytes(const float value)
  {
    const uint16_t scaled_double_byte = toDoubleByte(value);
    return {
      static_cast<uint8_t>((scaled_double_byte & 0xff00) >> 8),
      static_cast<uint8_t>(scaled_double_byte & 0x00ff)};
  }

  template <unsigned int Size>
  float toFloat(const std::array<uint8_t, Size> & data, const unsigned int offset) const
  {
    const uint16_t raw_data = data[0 + offset] << 8 | data[1 + offset];
    return scale_ * static_cast<float>(raw_data);
  }

private:
  float scale_;
};
}  // namespace cybergear_driver_core
