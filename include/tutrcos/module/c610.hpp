#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "tutrcos/peripheral/can_base.hpp"
#include "tutrcos/utility.hpp"

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
 * extern CAN_HandleTypeDef hcan1;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *   using namespace tutrcos::module;
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
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class C610 {
public:
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

  C610(peripheral::CANBase &can) : can_{can} {}

  void update() {
    peripheral::CANMessage msg;
    for (size_t i = can_.available(); i > 0; --i) {
      if (can_.receive(msg, 0)) {
        for (size_t i = 0; i < 8; ++i) {
          if (msg.id == 0x201 + i) {
            int16_t angle =
                static_cast<int16_t>(msg.data[0] << 8 | msg.data[1]);
            int16_t delta = angle - prev_angle_[i];
            if (delta > 4096) {
              delta -= 8192;
            } else if (delta < -4096) {
              delta += 8192;
            }
            position_[i] += delta;
            prev_angle_[i] = angle;

            rpm_[i] = static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);
            current_[i] = static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
            break;
          }
        }
      }
    }

    msg.id_type = peripheral::CANIDType::STANDARD;
    msg.id = 0x200;
    msg.dlc = 8;
    for (size_t i = 0; i < 4; ++i) {
      msg.data[i * 2] = current_target_[i] >> 8;
      msg.data[i * 2 + 1] = current_target_[i];
    }
    can_.transmit(msg);
    msg.id = 0x1FF;
    for (size_t i = 0; i < 4; ++i) {
      msg.data[i * 2] = current_target_[i + 4] >> 8;
      msg.data[i * 2 + 1] = current_target_[i + 4];
    }
    can_.transmit(msg);
  }

  float get_rpm(ID id) { return rpm_[utility::to_underlying(id)]; }

  float get_rps(ID id) { return get_rpm(id) / 60.0f; }

  float get_position(ID id) {
    return position_[utility::to_underlying(id)] / 8192.0f;
  }

  void set_position(ID id, float position) {
    position_[utility::to_underlying(id)] = position * 8192;
  }

  // -10000 ~ 10000 mA
  int16_t get_current(ID id) { return current_[utility::to_underlying(id)]; }

  void set_current(ID id, int16_t current) {
    current_target_[utility::to_underlying(id)] = current;
  }

private:
  peripheral::CANBase &can_;

  std::array<int16_t, 8> prev_angle_ = {};
  std::array<int64_t, 8> position_ = {};
  std::array<int16_t, 8> rpm_ = {};
  std::array<int16_t, 8> current_ = {};
  std::array<int16_t, 8> current_target_ = {};
};

} // namespace module
} // namespace tutrcos
