#pragma once

#include <cmath>
#include <cstdint>

namespace Ops9
{
inline constexpr float kPi = 3.14159265358979323846F;

struct Pose
{
  float x = 0;
  float y = 0;
  float yaw = 0;
};

class Odometry
{
 public:
  Odometry(float wheel_radius, float pulses_per_revolution)
      : metres_per_pulse_(2 * kPi * wheel_radius / pulses_per_revolution)
  {
  }

  void UpdateYaw(float yaw)
  {
    if (has_yaw_)
    {
      float delta = yaw - last_yaw_;
      if (delta < -kPi) delta += 2 * kPi;
      if (delta > kPi) delta -= 2 * kPi;
      pose_.yaw += delta;
    }
    else
    {
      pose_.yaw = yaw;
      has_yaw_ = true;
    }
    last_yaw_ = yaw;
  }

  void Integrate(int32_t pulses1, int32_t pulses2, uint8_t direction)
  {
    const float a = static_cast<float>(pulses1) * metres_per_pulse_;
    const float b = static_cast<float>(pulses2) * metres_per_pulse_;
    float dx = 0;
    float dy = 0;
    switch (direction)
    {
      case 0:
        dx = -b;
        dy = a;
        break;
      case 1:
        dx = a;
        dy = b;
        break;
      case 2:
        dx = b;
        dy = -a;
        break;
      case 3:
        dx = -a;
        dy = -b;
        break;
      default:
        return;
    }
    const float sine = std::sin(pose_.yaw);
    const float cosine = std::cos(pose_.yaw);
    pose_.x += dx * cosine - dy * sine;
    pose_.y += dx * sine + dy * cosine;
  }

  const Pose& GetPose() const { return pose_; }

 private:
  float metres_per_pulse_;
  float last_yaw_ = 0;
  bool has_yaw_ = false;
  Pose pose_;
};
}  // namespace Ops9
