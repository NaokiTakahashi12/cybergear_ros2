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

#include <cstring>
#include <algorithm>
#include <array>
#include <stdexcept>

namespace cybergear_driver_core
{
// TODO(Naoki Takahashi) endian support
class BoundedFloatByteConverter
{
public:
  explicit BoundedFloatByteConverter(const float max, const float min)
  {
    setRange(max, min);
  }

  ~BoundedFloatByteConverter() {}

  void setRange(const float max, const float min)
  {
    max_ = max;
    min_ = min;
    updateRange();
  }

  float toClampedFloat(const float value) const
  {
    return std::max(min_, std::min(max_, value));
  }

  uint16_t toDoubleByte(const float value) const
  {
    const float clamped_value = toClampedFloat(value);
    return static_cast<uint16_t>((clamped_value - min_) * byte_scale_);
  }

  std::array<uint8_t, 2> toTwoBytes(const float value) const
  {
    const uint16_t scaled_double_byte = toDoubleByte(value);
    return {
      static_cast<uint8_t>((scaled_double_byte & 0xff00) >> 8),
      static_cast<uint8_t>(scaled_double_byte & 0x00ff)};
  }

  std::array<uint8_t, 4> toFourBytes(const float value) const
  {
    constexpr size_t kArrayLength = 4;
    const float clamped_float = toClampedFloat(value);
    std::array<uint8_t, kArrayLength> raw_bytes;
    std::memcpy(raw_bytes.data(), &clamped_float, sizeof(uint8_t) * kArrayLength);
    return raw_bytes;
  }

  template <unsigned int Size>
  float toFloat(const std::array<uint8_t, Size> & data, const unsigned int offset) const
  {
    const uint16_t raw_data = data[0 + offset] << 8 | data[1 + offset];
    return float_scale_ * static_cast<float>(raw_data) + min_;
  }

private:
  float max_, min_;
  float float_range_;
  float float_scale_, byte_scale_;

  void updateRange()
  {
    float_range_ = max_ - min_;
    float_scale_ = float_range_ / static_cast<float>(0xffff);
    byte_scale_ = static_cast<float>(0xffff) / float_range_;

    if (float_range_ <= 0) {
      throw std::invalid_argument("Illigal float range: ZERO or NEGATIVE");
    }
  }
};
}  // namespace cybergear_driver_core
