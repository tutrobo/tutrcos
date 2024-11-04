#pragma once

#include <array>
#include <cstdint>

#include "tutrcos/core.hpp"
#include "tutrcos/peripheral/uart.hpp"

namespace tutrcos {
namespace module {

class BNO055 {
public:
  /**
   * BNO055初期化
   *
   * @param uart tutrclib::UARTへのポインタ
   * @param timeout タイムアウト(ms)
   * @return true: 成功, false: 失敗
   */
  BNO055(peripheral::UART &uart, uint32_t timeout = 500) : uart_{uart} {
    uint32_t start = core::Kernel::get_ticks();
    while (core::Kernel::get_ticks() - start < timeout) {
      uint8_t data = 0x00;
      if (!write_reg(0x3D, &data, 1)) {
        continue;
      }
      data = 0x04;
      if (!write_reg(0x3B, &data, 1)) {
        continue;
      }
      data = 0x08;
      if (!write_reg(0x3D, &data, 1)) {
        continue;
      }
      return;
    }
    Error_Handler();
  }

  /**
   * BNO055からデータを受信する
   */
  void update() {
    std::array<int16_t, 3> data;
    if (read_reg(0x1A, reinterpret_cast<uint8_t *>(data.data()), 6)) {
      euler_x_ = data[0] / 900.0f;
      euler_y_ = data[1] / 900.0f;
      euler_z_ = data[2] / 900.0f;
    }
  }

  /**
   * x軸中心のオイラー角(rad)を取得する
   */
  float get_euler_x() { return euler_x_; }

  /**
   * y軸中心のオイラー角(rad)を取得する
   */
  float get_euler_y() { return euler_y_; }

  /**
   * z軸中心のオイラー角(rad)を取得する
   */
  float get_euler_z() { return euler_z_; }

private:
  peripheral::UART &uart_;
  float euler_x_ = 0;
  float euler_y_ = 0;
  float euler_z_ = 0;

  bool write_reg(uint8_t addr, uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x00, addr, size};
    uart_.flush();
    if (!uart_.transmit(buf.data(), 4)) {
      return false;
    }
    if (!uart_.transmit(data, size)) {
      return false;
    }
    if (!uart_.receive(buf.data(), 2, 5)) {
      return false;
    }
    return buf[0] == 0xEE && buf[1] == 0x01;
  }

  bool read_reg(uint8_t addr, uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x01, addr, size};
    uart_.flush();
    if (!uart_.transmit(buf.data(), 4)) {
      return false;
    }
    if (!uart_.receive(buf.data(), 2, 5)) {
      return false;
    }
    if (buf[0] != 0xBB || buf[1] != size) {
      return false;
    }
    return uart_.receive(data, size, 5);
  }
};

} // namespace module
} // namespace tutrcos
