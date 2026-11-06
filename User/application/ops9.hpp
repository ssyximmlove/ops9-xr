#pragma once

#include "board/pulse_counter.hpp"
#include "communication/protocol.hpp"
#include "power.hpp"
#include "sensors/hwt101.hpp"
#include "thread.hpp"
#include "uart.hpp"

namespace Ops9
{
struct Parameters
{
  float wheel_radius = 0.029F;
  float pulses_per_revolution = 1024.0F;
  uint32_t sample_period_ms = 1;
  uint32_t report_period_ms = 2;
  uint8_t direction = 2;
  uint32_t imu_zero_timeout_ms = 2000;
};

class Application
{
 public:
  Application(LibXR::UART& imu_uart, LibXR::UART& robot_uart, PulseCounter& counter1,
              PulseCounter& counter2, LibXR::GPIO& led, LibXR::PowerManager& power,
              Parameters parameters);

 private:
  enum class Startup
  {
    UNLOCK,
    WAIT_UNLOCK,
    ZERO,
    SAVE,
    RUN
  };
  static void Run(Application* self);
  void PollImu(uint32_t now);
  void PollCommands();
  void AdvanceStartup(uint32_t now);
  bool SendImu(const std::array<uint8_t, 5>& command);
  void Report();

  LibXR::UART& imu_uart_;
  LibXR::UART& robot_uart_;
  PulseCounter& counter1_;
  PulseCounter& counter2_;
  LibXR::GPIO& led_;
  LibXR::PowerManager& power_;
  const Parameters parameters_;
  Odometry odometry_;
  Hwt101 imu_;
  CommandParser commands_;
  LibXR::Thread worker_;
  Startup startup_ = Startup::UNLOCK;
  uint32_t startup_time_ = 0;
  uint32_t last_zero_ = 0;
  uint32_t zero_frame_ = 0;
  uint32_t last_imu_ = 0;
  uint32_t last_report_ = 0;
  uint32_t last_led_ = 0;
  uint32_t tx_rejected_ = 0;
  uint8_t direction_;
  bool zero_sent_ = false;
};
}  // namespace Ops9
