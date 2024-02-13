#pragma once

#include <array>

namespace cybergear_socketcan_driver
{
// TODO endian support
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
    m_max = max;
    m_min = min;
    updateRange();
  }

  uint16_t toDoubleByte(const float value)
  {
    const float clamped_value = std::max(m_min, std::min(m_max, value));
    return static_cast<uint16_t>((clamped_value + m_float_half_range) * m_byte_scale);
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
    return m_float_scale * static_cast<float>(raw_data) - m_float_half_range;
  }

private:
  float m_max, m_min;
  float m_float_range, m_float_half_range;
  float m_float_scale, m_byte_scale;

  void updateRange()
  {
    m_float_range = m_max - m_min;
    m_float_half_range = m_float_range / 2;
    m_float_scale = m_float_range / static_cast<float>(0xffff);
    m_byte_scale = static_cast<float>(0xffff) / m_float_range;

    if (m_float_range <= 0) {
      throw std::invalid_argument("Illigal float range: ZERO or NEGATIVE");
    }
  }
};
}  // namespace cybergear_socketcan_driver
