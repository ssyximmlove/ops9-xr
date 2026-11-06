#include "application/ops9.hpp"

#include <algorithm>
#include <cmath>

namespace Ops9
{
Application::Application(LibXR::UART& imu_uart, LibXR::UART& robot_uart,
                         PulseCounter& counter1, PulseCounter& counter2, LibXR::GPIO& led,
                         LibXR::PowerManager& power, Parameters parameters)
    : imu_uart_(imu_uart),
      robot_uart_(robot_uart),
      counter1_(counter1),
      counter2_(counter2),
      led_(led),
      power_(power),
      parameters_(parameters),
      odometry_(parameters.wheel_radius, parameters.pulses_per_revolution),
      direction_(parameters.direction)
{
  REQUIRE(parameters.wheel_radius > 0 && parameters.pulses_per_revolution > 0);
  REQUIRE(parameters.sample_period_ms > 0 && parameters.report_period_ms >= 2);
  REQUIRE(parameters.direction < 4 && parameters.imu_zero_timeout_ms >= 20);
  worker_.Create(this, Run, "ops9", 2048, LibXR::Thread::Priority::HIGH);
}

void Application::PollImu(uint32_t now)
{
  uint8_t bytes[64];
  const auto count = std::min(sizeof(bytes), imu_uart_.read_port_->Size());
  if (!count) return;
  LibXR::ReadOperation operation;
  if (imu_uart_.Read({bytes, count}, operation) != LibXR::ErrorCode::OK) return;
  for (size_t i = 0; i < count; ++i)
  {
    if (imu_.Feed(bytes[i]))
    {
      last_imu_ = now;
      if (startup_ == Startup::RUN) odometry_.UpdateYaw(imu_.Yaw());
    }
  }
}

void Application::PollCommands()
{
  uint8_t bytes[32];
  const auto count = std::min(sizeof(bytes), robot_uart_.read_port_->Size());
  if (!count) return;
  LibXR::ReadOperation operation;
  if (robot_uart_.Read({bytes, count}, operation) != LibXR::ErrorCode::OK) return;
  for (size_t i = 0; i < count; ++i)
  {
    uint8_t command;
    if (!commands_.Feed(bytes[i], command)) continue;
    if (command == 0x22)
      power_.Reset();
    else
      direction_ = command - 0x30;
  }
}

bool Application::SendImu(const std::array<uint8_t, 5>& command)
{
  LibXR::WriteOperation operation;
  return imu_uart_.Write({command.data(), command.size()}, operation) ==
         LibXR::ErrorCode::OK;
}

void Application::AdvanceStartup(uint32_t now)
{
  constexpr std::array<uint8_t, 5> unlock{0xFF, 0xAA, 0x69, 0x88, 0xB5};
  constexpr std::array<uint8_t, 5> zero{0xFF, 0xAA, 0x76, 0x00, 0x00};
  constexpr std::array<uint8_t, 5> save{0xFF, 0xAA, 0x00, 0x00, 0x00};
  switch (startup_)
  {
    case Startup::UNLOCK:
      if (SendImu(unlock))
      {
        startup_time_ = now;
        startup_ = Startup::WAIT_UNLOCK;
      }
      break;
    case Startup::WAIT_UNLOCK:
      if (now - startup_time_ >= 20)
      {
        last_zero_ = now - 20;
        startup_ = Startup::ZERO;
      }
      break;
    case Startup::ZERO:
      if ((zero_sent_ && imu_.ValidFrames() != zero_frame_ &&
           std::fabs(imu_.Yaw()) < 0.001F && now - last_zero_ >= 20) ||
          now - startup_time_ >= parameters_.imu_zero_timeout_ms)
      {
        startup_ = Startup::SAVE;
      }
      else if (now - last_zero_ >= 20 && SendImu(zero))
      {
        last_zero_ = now;
        zero_frame_ = imu_.ValidFrames();
        zero_sent_ = true;
      }
      break;
    case Startup::SAVE:
      if (SendImu(save))
      {
        if (imu_.ValidFrames()) odometry_.UpdateYaw(imu_.Yaw());
        startup_ = Startup::RUN;
      }
      break;
    case Startup::RUN:
      break;
  }
}

void Application::Report()
{
  const auto frame = EncodePose(odometry_.GetPose());
  // LibXR copies accepted writes into its TX queue; no stack buffer is borrowed by DMA.
  LibXR::WriteOperation operation;
  if (robot_uart_.Write({frame.data(), frame.size()}, operation) != LibXR::ErrorCode::OK)
    ++tx_rejected_;
}

void Application::Run(Application* self)
{
  self->counter1_.Start();
  self->counter2_.Start();
  LibXR::MillisecondTimestamp wakeup = LibXR::Timebase::GetMilliseconds();
  for (;;)
  {
    const uint32_t now = LibXR::Thread::GetTime();
    self->PollImu(now);
    self->PollCommands();
    self->AdvanceStartup(now);
    const int32_t pulses1 = self->counter1_.Sample();
    const int32_t pulses2 = self->counter2_.Sample();
    if (self->startup_ == Startup::RUN)
      self->odometry_.Integrate(pulses1, pulses2, self->direction_);
    if (now - self->last_report_ >= self->parameters_.report_period_ms)
    {
      self->last_report_ = now;
      self->Report();
    }
    if (now - self->last_led_ >= (now - self->last_imu_ > 500 ? 100 : 1000))
    {
      self->last_led_ = now;
      self->led_.Write(!self->led_.Read());
    }
    LibXR::Thread::SleepUntil(wakeup, self->parameters_.sample_period_ms);
  }
}
}  // namespace Ops9
