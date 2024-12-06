#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "tutrcos/peripheral/uart.hpp"
#include "tutrcos/utility.hpp"

#include "encoder_base.hpp"

namespace tutrcos {
namespace module {

/**
 *
 * modeはSTS3215のworkmodeの設定に応じて選択してください。
 * シリアル通信のbaudrateが1Mbpsだと、オーバーランエラーになる可能性があります（処理能力依存）
 * 500kbps以下を推奨します。
 *
 * @code{.cpp}
 * #include <cstdio>
 * #include <tutrcos.hpp>
 * #include <tutrcos/module/sts3215.hpp>
 *
 * extern UART_HandleTypeDef huart2;
 * extern UART_HandleTypeDef huart4;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *   using namespace tutrcos::module;
 *
 *   UART uart2(&huart2); // デバッグ出力用
 *   uart2.enable_stdout();
 *
 *   UART uart3(&huart4);
 *   STS3215 sts(uart3, STS3215::Mode::PWM, 10);
 *
 *   while (true) {
 *     if(!sts.update()){ // データ送受信
 *       printf("error\r\n");
 *     }else{
 *       float Kp = 5;
 *       float x_target = 0.5f;
 *       // 現在の角度をrotationで取得
 *       float x_actual = sts.get_rotation();
 *       float error = x_target - x_actual;
 *
 *       // 出力(-1~1)を指定
 *       sts.set_ref(Kp * error);
 *
 *       // STS3215の回転速度と絶対位置を出力
 *       printf("%f %f\r\n", sts.get_rps(), sts.get_rotation());
 *     }
 *
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class STS3215 : public EncoderBase {
public:
  enum class Mode {
    RAD = 0,
    PWM = 2,
  };

  STS3215(peripheral::UART &uart, Mode mode, uint8_t id)
      : EncoderBase{ppr_}, uart_{uart}, mode_{mode}, id_{id} {}

  bool update() orverride {
    uint8_t rx_data[8] = {0};
    uart_.flush();
    if (send({0x02, 0x38, 0x02})) {
      // rx_data : 0xff 0xff id size cmd data data checksum
      if (uart_.receive(rx_data, 8, 1)) {
        uint8_t checksum = 0;
        for (uint8_t i = 2; i < 8; i++) {
          checksum += rx_data[i];
        }
        if ((rx_data[0] == 0xff) && (rx_data[1] == 0xff) &&
            (checksum == 0xff) && (rx_data[2] == id_)) {

          int16_t count = static_cast<int16_t>(rx_data[6] << 8) | rx_data[5];
          int16_t delta = count - prev_count_;

          if (delta > (ppr_ / 2)) {
            delta -= ppr_;
          } else if (delta < -(ppr_ / 2)) {
            delta += ppr_;
          }

          set_count(get_count() + delta);
          prev_count_ = count;
        }
      }
    }

    bool res = true;
    // transmit
    int16_t target = 0;
    uint8_t upper, lower;
    switch (mode_) {
    case Mode::RAD:
      ref_ = std::clamp<float>(ref_, 0, 2 * M_PI);
      target = ref_ / (2 * M_PI) * (ppr_ - 1);
      upper = static_cast<uint8_t>(target >> 8);
      lower = static_cast<uint8_t>(target);
      res = send({0x03, 0x2A, lower, upper});
      break;
    case Mode::PWM:
      ref_ = std::clamp<float>(ref_, -1, 1);
      target = static_cast<uint16_t>(abs(ref_ * 1023));
      upper = static_cast<uint8_t>(target >> 8) | ((ref_ > 0) ? 0x04 : 0);
      lower = static_cast<uint8_t>(target);
      res = send({0x03, 0x2C, lower, upper});
      break;
    }
    return res;
  }

  void set_ref(float value) { ref_ = value; }

private:
  inline static constexpr uint16_t ppr_ = 4096;
  peripheral::UART &uart_;
  Mode mode_;
  const uint8_t id_;
  float ref_ = 0;
  int16_t prev_count_ = 0;
  int16_t rpm_ = 0;
  int16_t current_ = 0;
  int16_t current_target_ = 0;

  bool send(std::vector<uint8_t> tx) {
    uint8_t size = tx.size() + 1;
    tx.insert(tx.begin(), {0xff, 0xff, id_, size});
    uint8_t checksum = 0;
    for (uint8_t i = 2, size = tx.size(); i < size; i++) {
      checksum += tx[i];
    }
    tx.emplace_back(~checksum);
    return uart_.transmit(tx.data(), size + 5, 1);
  }
};

} // namespace module
} // namespace tutrcos
