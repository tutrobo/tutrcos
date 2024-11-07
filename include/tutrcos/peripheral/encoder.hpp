#pragma once

#include "main.h"

#include <cstdint>

namespace tutrcos {
namespace peripheral {

/**
 * Timers -> TIMx -> Combined Channels を Encoder Mode にしてください。
 *
 * @code{.cpp}
 * #include <cstdio>
 * #include <tutrcos.hpp>
 *
 * extern UART_HandleTypeDef huart2;
 * extern TIM_HandleTypeDef htim1;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *
 *   UART uart2(&huart2); // デバッグ出力用
 *
 *   Encoder enc(&htim1, 2048, 0.01); // PPR: 2048, 読み取り周期: 0.01s
 *
 *   while (true) {
 *     enc.update();
 *
 *     // エンコーダの回転速度と絶対位置を出力
 *     uart2.printf("%f %f\r\n", enc.get_rps(), enc.get_position());
 *
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class Encoder {
public:
  Encoder(TIM_HandleTypeDef *htim, uint16_t ppr, float period)
      : htim_{htim}, ppr_{ppr}, period_{period} {
    if (HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL) != HAL_OK) {
      Error_Handler();
    }
  }

  ~Encoder() { HAL_TIM_Encoder_Stop(htim_, TIM_CHANNEL_ALL); }

  void update() {
    int16_t delta = __HAL_TIM_GET_COUNTER(htim_);
    __HAL_TIM_SET_COUNTER(htim_, 0);

    float cpr = ppr_ * 4;
    count_ += delta;
    rps_ = delta / period_ / cpr;
    position_ = count_ / cpr;
  }

  float get_rps() { return rps_; }

  float get_rpm() { return get_rps() * 60; }

  float get_position() { return position_; }

private:
  TIM_HandleTypeDef *htim_;
  uint16_t ppr_;
  float period_;

  int64_t count_ = 0;
  float rps_;
  float position_;
};

} // namespace peripheral
} // namespace tutrcos
