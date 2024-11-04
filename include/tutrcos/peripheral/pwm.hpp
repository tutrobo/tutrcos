#pragma once

#include "main.h"

#include <cstdint>

namespace tutrcos {
namespace peripheral {

/**
 * Counter Period: 1000-1, PWM周波数: 1kHz に設定して動作確認しました。
 *
 * @code{.cpp}
 * #include <tutrcos.hpp>
 *
 * extern TIM_HandleTypeDef htim2;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *
 *   PWM led(&htim2, TIM_CHANNEL_1);
 *
 *   while (true) {
 *     // だんだん明るく
 *     for (int i = 0; i < 1000 - 1; ++i) {
 *       led.set_compare(i);
 *       Thread::delay(1);
 *     }
 *
 *     // だんだん暗く
 *     for (int i = 1000 - 1; i > 0; --i) {
 *       led.set_compare(i);
 *       Thread::delay(1);
 *     }
 *   }
 * }
 * @endcode
 */
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
