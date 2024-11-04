#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "tutrcos/peripheral/uart.hpp"
#include "tutrcos/utility.hpp"

#include <cstdio>

namespace tutrcos {
namespace module {

/**
 * @code{.cpp}
 * #include <cstdio>
 * #include <tutrcos.hpp>
 * #include <tutrcos/module/ps3.hpp>
 *
 * extern UART_HandleTypeDef huart1;
 * extern UART_HandleTypeDef huart2;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *   using namespace tutrcos::module;
 *
 *   UART uart2(&huart2);
 *   uart2.enable_printf(); // デバッグ用printf有効化
 *
 *   UART uart1(&huart1);
 *   PS3 ps3(uart1);
 *
 *   while (true) {
 *     ps3.update();
 *
 *     // DualShock 左スティックのx, y座標を出力
 *     printf("%f %f\r\n", ps3.get_axis(PS3::Axis::LEFT_X),
 *            ps3.get_axis(PS3::Axis::LEFT_Y));
 *
 *     if (ps3.get_key_down(PS3::Key::CIRCLE)) {
 *       printf("O ボタンが押されたよ\r\n");
 *     }
 *
 *     if (ps3.get_key_up(PS3::Key::CIRCLE)) {
 *       printf("O ボタンが離されたよ\r\n");
 *     }
 *
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class PS3 {
public:
  enum class Axis {
    LEFT_X,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y,
  };

  enum class Key {
    UP,
    DOWN,
    RIGHT,
    LEFT,
    TRIANGLE,
    CROSS,
    CIRCLE,
    SQUARE = 8,
    L1,
    L2,
    R1,
    R2,
    START,
    SELECT,
  };

  PS3(peripheral::UART &uart) : uart_{uart} {}

  void update() {
    keys_prev_ = keys_;

    for (size_t i = 0; i < 8; ++i) {
      if (buf_[0] != 0x80) {
        if (!uart_.receive(buf_.data(), 1, 0)) {
          return;
        }
      }

      if (buf_[0] == 0x80) {
        if (!uart_.receive(buf_.data() + 1, 7, 0)) {
          return;
        }

        uint8_t checksum = 0;
        for (size_t i = 1; i < 7; ++i) {
          checksum += buf_[i];
        }

        if ((checksum & 0x7F) == buf_[7]) {
          keys_ = (buf_[1] << 8) | buf_[2];
          if ((keys_ & 0x03) == 0x03) {
            keys_ &= ~0x03;
            keys_ |= 1 << 13;
          }
          if ((keys_ & 0x0C) == 0x0C) {
            keys_ &= ~0x0C;
            keys_ |= 1 << 14;
          }
          for (size_t i = 0; i < 4; ++i) {
            axes_[i] = (static_cast<float>(buf_[i + 3]) - 64) / 64;
          }
        }

        buf_.fill(0);
      }
    }
  }

  float get_axis(Axis axis) { return axes_[utility::to_underlying(axis)]; }

  bool get_key(Key key) {
    return (keys_ & (1 << utility::to_underlying(key))) != 0;
  }

  bool get_key_down(Key key) {
    return ((keys_ ^ keys_prev_) & keys_ &
            (1 << utility::to_underlying(key))) != 0;
  }

  bool get_key_up(Key key) {
    return ((keys_ ^ keys_prev_) & keys_prev_ &
            (1 << utility::to_underlying(key))) != 0;
  }

private:
  peripheral::UART &uart_;
  std::array<uint8_t, 8> buf_{};
  std::array<float, 4> axes_{};
  uint16_t keys_;
  uint16_t keys_prev_;
};

} // namespace module
} // namespace tutrcos
