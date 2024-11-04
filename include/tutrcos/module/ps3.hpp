#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "tutrcos/peripheral/uart.hpp"
#include "tutrcos/utility.hpp"

namespace tutrcos {
namespace module {

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

    uint8_t checksum = 0;

    if (!uart_.receive(buf_.data(), buf_.size(), 0)) {
      return;
    }

    for (size_t i = 0; i < 8; ++i) {
      if (buf_[i] == 0x80) {
        if (i > 0) {
          // 1周分受信
          for (size_t j = i; j < 8; ++j) {
            buf_[j - i] = buf_[j];
          }
          if (!uart_.receive(buf_.data() + 8 - i, i, 0)) {
            return;
          }
        }

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
        return;
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
