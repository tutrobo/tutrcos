#pragma once

#include "main.h"

#include <cstdint>

namespace tutrcos {
namespace peripheral {

class PWM {
public:
  /**
   * PWM初期化
   *
   * @param htim TIM_HandleTypeDefへのポインタ
   * @param channel 使用するPWMチャネル
   * @return true: 成功, false: 失敗
   */
  PWM(TIM_HandleTypeDef *htim, uint32_t channel)
      : htim_{htim}, channel_{channel} {
    if (HAL_TIM_PWM_Start(htim_, channel_) != HAL_OK) {
      Error_Handler();
    }
  }

  ~PWM() { HAL_TIM_PWM_Stop(htim_, channel_); }

  /**
   * タイマのカウンタを取得
   */
  uint32_t get_compare() { return __HAL_TIM_GET_COMPARE(htim_, channel_); }

  /**
   * タイマのカウンタを設定
   */
  void set_compare(uint32_t compare) {
    __HAL_TIM_SET_COMPARE(htim_, channel_, compare);
  }

private:
  TIM_HandleTypeDef *htim_;
  uint32_t channel_;
};

} // namespace peripheral
} // namespace tutrcos
