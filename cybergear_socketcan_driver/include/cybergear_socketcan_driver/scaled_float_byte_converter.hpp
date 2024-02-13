#pragma once

#include <array>

namespace cybergear_socketcan_driver
{
// TODO endian support
class ScaledFloatByteConverter
{
public:
  explicit ScaledFloatByteConverter(const float scale)
  {
    setScale(scale);
  }

  ~ScaledFloatByteConverter() {}

  void setScale(const float scale)
  {
    m_scale = scale;
  }

  uint16_t toDoubleByte(const float value)
  {
    return static_cast<uint16_t>(value / m_scale);
  }

  std::array<uint8_t, 2> toByte(const float value)
  {
    const uint16_t scaled_double_byte = toDoubleByte(value);
    return {
      static_cast<uint8_t>((scaled_double_byte & 0xff00) >> 8),
      static_cast<uint8_t>(scaled_double_byte & 0x00ff)
    };
  }

  template<unsigned int Size>
  float toFloat(const std::array<uint8_t, Size> & data, const unsigned int offset)
  {
    const uint16_t raw_data = data[0 + offset] << 8 | data[1 + offset];
    return m_scale * static_cast<float>(raw_data);
  }

private:
  float m_scale;
};
}  // namespace cybergear_socketcan_driver
