#pragma once

#include "main.h"

#include <cstdint>

namespace tutrcos {
namespace peripheral {

class TIM {
public:
  TIM(TIM_HandleTypeDef *htim) : htim_{htim} {}

  bool start_timer() { return HAL_TIM_Base_Start_IT(htim_) == HAL_OK; }

  bool stop_timer() { return HAL_TIM_Base_Stop_IT(htim_) == HAL_OK; }

  bool start_pwm(uint32_t channel) {
    return HAL_TIM_PWM_Start(htim_, channel) == HAL_OK;
  }

  bool stop_pwm(uint32_t channel) {
    return HAL_TIM_PWM_Stop(htim_, channel) == HAL_OK;
  }

  bool start_encoder(uint32_t channel) {
    return HAL_TIM_Encoder_Start(htim_, channel) == HAL_OK;
  }

  bool stop_encoder(uint32_t channel) {
    return HAL_TIM_Encoder_Stop(htim_, channel) == HAL_OK;
  }

  uint32_t get_counter() { return __HAL_TIM_GET_COUNTER(htim_); }

  void set_counter(uint32_t count) { __HAL_TIM_SET_COUNTER(htim_, count); }

  uint32_t get_compare(uint32_t channel) {
    return __HAL_TIM_GET_COMPARE(htim_, channel);
  }

  void set_compare(uint32_t channel, uint32_t compare) {
    __HAL_TIM_SET_COMPARE(htim_, channel, compare);
  }

  TIM_HandleTypeDef *get_hal_handle() { return htim_; }

private:
  TIM_HandleTypeDef *htim_;
};

} // namespace peripheral
} // namespace tutrcos
