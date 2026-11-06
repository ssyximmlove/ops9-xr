#pragma once

#include "gpio.hpp"
#include "tim.h"

namespace Ops9
{
// TIM1/2 use TI1 external pulse clocks, with separate direction GPIOs.
// Subtract free-running samples instead of clearing CNT and losing an edge.
class PulseCounter
{
 public:
  PulseCounter(TIM_HandleTypeDef& timer, LibXR::GPIO& direction)
      : timer_(timer), direction_(direction)
  {
  }

  void Start()
  {
    REQUIRE(HAL_TIM_Base_Start(&timer_) == HAL_OK);
    previous_ = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&timer_));
  }

  int32_t Sample()
  {
    const auto current = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&timer_));
    const uint16_t pulses = static_cast<uint16_t>(current - previous_);
    previous_ = current;
    return direction_.Read() ? static_cast<int32_t>(pulses)
                             : -static_cast<int32_t>(pulses);
  }

 private:
  TIM_HandleTypeDef& timer_;
  LibXR::GPIO& direction_;
  uint16_t previous_ = 0;
};
}  // namespace Ops9
