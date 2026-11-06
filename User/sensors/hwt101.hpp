#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "odometry/odometry.hpp"

namespace Ops9
{
// Byte-stream parser: packet boundaries need not match UART DMA events.
class Hwt101
{
 public:
  bool Feed(uint8_t byte)
  {
    if (used_ == 0 && byte != 0x55) return false;
    frame_[used_++] = byte;
    if (used_ != frame_.size()) return false;
    uint8_t sum = 0;
    for (size_t i = 0; i < frame_.size() - 1; ++i) sum += frame_[i];
    const bool valid = sum == frame_[10] && frame_[1] == 0x53;
    if (valid)
    {
      const uint16_t raw =
          static_cast<uint16_t>(frame_[6]) | static_cast<uint16_t>(frame_[7]) << 8;
      const int32_t signed_raw = raw >= 0x8000 ? static_cast<int32_t>(raw) - 65536 : raw;
      yaw_ = static_cast<float>(signed_raw) * kPi / 32768.0F;
      ++valid_frames_;
      used_ = 0;
    }
    else
    {
      // Preserve a possible header inside a damaged or truncated packet.
      size_t next = 1;
      while (next < used_ && frame_[next] != 0x55) ++next;
      used_ -= next;
      for (size_t i = 0; i < used_; ++i) frame_[i] = frame_[i + next];
    }
    return valid;
  }

  float Yaw() const { return yaw_; }
  uint32_t ValidFrames() const { return valid_frames_; }

 private:
  std::array<uint8_t, 11> frame_{};
  size_t used_ = 0;
  float yaw_ = 0;
  uint32_t valid_frames_ = 0;
};
}  // namespace Ops9
