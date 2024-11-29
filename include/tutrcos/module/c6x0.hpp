#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "tutrcos/peripheral/can_base.hpp"
#include "tutrcos/utility.hpp"

#include "encoder_base.hpp"

namespace tutrcos {
namespace module {

/**
 * FDCAN の Classic CAN モードを用いて C610
 * と通信を行う場合は、`tutrcos::peripheral::FDCAN` を用いて構築してください。
 *
 * @code{.cpp}
 * #include <tutrcos.hpp>
 * #include <tutrcos/module/c610.hpp>
 *
 * extern UART_HandleTypeDef huart2;
 * extern CAN_HandleTypeDef hcan1;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *   using namespace tutrcos::module;
 *
 *   UART uart2(&huart2); // デバッグ出力用
 *
 *   CAN can1(&hcan1);
 *   C610 c610(can1);
 *
 *   while (true) {
 *     c610.update(); // データ送受信
 *
 *     float Kp = 100;
 *     float v_target = 100.0f;
 *     // 現在の速度をrpsで取得
 *     float v_actual = c610.get_rps(C610::ID::ID1);
 *     float error = v_target - v_actual;
 *
 *     // 電流値をmAで指定
 *     c610.set_current(C610::ID::ID1, Kp * error);
 *
 *     // M2006の回転速度と絶対位置を出力
 *     uart2.printf("%f %f\r\n", c610.get_rps(C610::ID::ID1),
 *                  c610.get_position(C610::ID::ID1));
 *
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class C6x0 {
public:
  enum class Type {
    C610,
    C620,
  };

  enum class ID {
    ID1,
    ID2,
    ID3,
    ID4,
    ID5,
    ID6,
    ID7,
    ID8,
  };

  class Motor : public EncoderBase {
  public:
    float get_rps() override { return get_rpm() / 60; }

    float get_rpm() override { return rpm_; }

    int16_t get_current() { return current_; }

    void set_current(int16_t current) { current_target_ = current; }

  private:
    int16_t prev_count_ = 0;
    int16_t rpm_ = 0;
    int16_t current_ = 0;
    int16_t current_target_ = 0;

    Motor() : EncoderBase{8192} {}

    Motor(const Motor &) = delete;
    Motor &operator=(const Motor &) = delete;
    Motor(Motor &&) = delete;
    Motor &operator=(Motor &&) = delete;

    friend class C6x0;
  };

  C6x0(peripheral::CANBase &can, Type type) : can_{can}, type_{type} {}

  void update() {
    peripheral::CANMessage msg;
    while (can_.receive(msg, 0)) {
      for (size_t i = 0; i < 8; ++i) {
        if (msg.id == 0x201 + i) {
          int16_t count = static_cast<int16_t>(msg.data[0] << 8 | msg.data[1]);
          int16_t delta = count - motors_[i].prev_count_;
          if (delta > 4096) {
            delta -= 8192;
          } else if (delta < -4096) {
            delta += 8192;
          }
          motors_[i].set_count(motors_[i].get_count() + delta);
          motors_[i].prev_count_ = count;

          motors_[i].rpm_ =
              static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);

          int16_t current =
              static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
          switch (type_) {
          case Type::C610:
            motors_[i].current_ = current;
            break;
          case Type::C620:
            motors_[i].current_ = current * 25000 / 16384;
            break;
          }
          break;
        }
      }
    }

    msg.id_type = peripheral::CANIDType::STANDARD;
    msg.id = 0x200;
    msg.dlc = 8;
    for (size_t i = 0; i < 4; ++i) {
      int16_t current_target;
      switch (type_) {
      case Type::C610:
        current_target = motors_[i].current_target_;
        break;
      case Type::C620:
        current_target = motors_[i].current_target_ * 16384 / 25000;
        break;
      }
      msg.data[i * 2] = current_target >> 8;
      msg.data[i * 2 + 1] = current_target;
    }
    can_.transmit(msg, 0);
    msg.id = 0x1FF;
    for (size_t i = 0; i < 4; ++i) {
      int16_t current_target;
      switch (type_) {
      case Type::C610:
        current_target = motors_[i + 4].current_target_;
        break;
      case Type::C620:
        current_target = motors_[i + 4].current_target_ * 16384 / 25000;
        break;
      }
      msg.data[i * 2] = current_target >> 8;
      msg.data[i * 2 + 1] = current_target;
    }
    can_.transmit(msg, 0);
  }

  Motor &get_motor(ID id) { return motors_[utility::to_underlying(id)]; }

private:
  peripheral::CANBase &can_;
  Type type_;
  std::array<Motor, 8> motors_ = {};
};

} // namespace module
} // namespace tutrcos
