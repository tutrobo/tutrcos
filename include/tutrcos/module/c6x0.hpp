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
 * #include <cstdio>
 * #include <tutrcos.hpp>
 * #include <tutrcos/module/c6x0.hpp>
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
 *   uart2.enable_stdout();
 *
 *   CAN can1(&hcan1);
 *   C6x0::Manager c610_manager(can1);
 *   C6x0 c610_1(c610_manager, C6x0::Type::C610, C6x0::ID::_1);
 *
 *   while (true) {
 *     c610_manager.update(); // データ送受信
 *
 *     float Kp = 100;
 *     float v_target = 100.0f;
 *     // 現在の速度をrpsで取得
 *     float v_actual = c610_1.get_rps();
 *     float error = v_target - v_actual;
 *
 *     // 電流値をmAで指定
 *     c610_1.set_current(Kp * error);
 *
 *     // M2006の回転速度と絶対位置を出力
 *     printf("%f %f\r\n", c610_1.get_rps(), c610_1.get_rotation());
 *
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class C6x0 : public EncoderBase {
public:
  enum class Type {
    C610,
    C620,
  };

  enum class ID {
    _1,
    _2,
    _3,
    _4,
    _5,
    _6,
    _7,
    _8,
  };

  class Manager {
  public:
    Manager(peripheral::CANBase &can) : can_{can} {
      can.add_rx_queue(0x200, 0x200, rx_queue_);
    }

    bool update() {
      peripheral::CANBase::Message msg;
      while (rx_queue_.pop(msg, 0)) {
        if (msg.id >= 0x201 && msg.id < 0x201 + 8) {
          C6x0 *motor = motors_[msg.id - 0x201];
          if (motor) {
            int16_t count =
                static_cast<int16_t>(msg.data[0] << 8 | msg.data[1]);
            int16_t delta = count - motor->prev_count_;
            if (delta > 4096) {
              delta -= 8192;
            } else if (delta < -4096) {
              delta += 8192;
            }
            motor->set_count(motor->get_count() + delta);
            motor->prev_count_ = count;

            motor->rpm_ = static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);
            motor->current_ =
                static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
          }
        }
      }

      msg.data.fill(0);
      msg.id_type = peripheral::CANBase::IDType::STANDARD;
      msg.id = 0x200;
      msg.dlc = 8;
      for (size_t i = 0; i < 4; ++i) {
        if (motors_[i]) {
          msg.data[i * 2] = motors_[i]->target_current_ >> 8;
          msg.data[i * 2 + 1] = motors_[i]->target_current_;
        }
      }
      if (!can_.transmit(msg, 0)) {
        return false;
      }

      msg.data.fill(0);
      msg.id = 0x1FF;
      for (size_t i = 0; i < 4; ++i) {
        if (motors_[i + 4]) {
          msg.data[i * 2] = motors_[i + 4]->target_current_ >> 8;
          msg.data[i * 2 + 1] = motors_[i + 4]->target_current_;
        }
      }
      return can_.transmit(msg, 0);
    }

  private:
    peripheral::CANBase &can_;
    core::Queue<peripheral::CANBase::Message> rx_queue_{64};
    std::array<C6x0 *, 8> motors_{};

    friend C6x0;
  };

  C6x0(Manager &manager, Type type, ID id)
      : EncoderBase{8192}, manager_{manager}, type_{type}, id_{id} {
    TUTRCOS_VERIFY(manager_.motors_[utility::to_underlying(id_)] == nullptr);
    manager_.motors_[utility::to_underlying(id_)] = this;
  }

  ~C6x0() { manager_.motors_[utility::to_underlying(id_)] = nullptr; }

  float get_rps() override { return get_rpm() / 60; }

  float get_rpm() override { return rpm_; }

  int16_t get_current() {
    switch (type_) {
    case Type::C610:
      return current_;
    case Type::C620:
      return current_ * 25000 / 16384;
    }
  }

  void set_current(int16_t current) {
    switch (type_) {
    case Type::C610:
      target_current_ = current;
      break;
    case Type::C620:
      target_current_ = current * 16384 / 25000;
      break;
    }
  }

private:
  Manager &manager_;
  Type type_;
  ID id_;

  int16_t prev_count_ = 0;
  int16_t rpm_ = 0;
  int16_t current_ = 0;
  int16_t target_current_ = 0;
};

} // namespace module
} // namespace tutrcos
