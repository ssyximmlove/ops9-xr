#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <limits>

#include "odometry/odometry.hpp"

namespace Ops9
{
inline uint8_t Crc8(const uint8_t* bytes, size_t size)
{
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= bytes[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = static_cast<uint8_t>((crc >> 1) ^ ((crc & 1) ? 0x8C : 0));
  }
  return crc;
}

inline std::array<uint8_t, 14> EncodePose(const Pose& pose)
{
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  std::array<uint8_t, 14> result{};
  result[0] = 0x5C;
  const float values[] = {pose.x, pose.y, pose.yaw};
  for (size_t i = 0; i < 3; ++i)
  {
    uint32_t bits;
    std::memcpy(&bits, &values[i], sizeof(bits));
    for (unsigned j = 0; j < 4; ++j)
      result[1 + i * 4 + j] = static_cast<uint8_t>(bits >> (j * 8));
  }
  result[13] = Crc8(result.data(), 13);
  return result;
}

class CommandParser
{
 public:
  bool Feed(uint8_t byte, uint8_t& command)
  {
    if (byte == 0xC5)
    {
      header_ = true;
      return false;
    }
    if (!header_) return false;
    header_ = false;
    if (byte != 0x22 && (byte < 0x30 || byte > 0x33)) return false;
    command = byte;
    return true;
  }

 private:
  bool header_ = false;
};
}  // namespace Ops9
