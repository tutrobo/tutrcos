#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <typeinfo>

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
 *
 * @endcode
 */
namespace {
#define CMD_POSITION 1
#define CMD_RESPONSE 2
#define CMD_ENABLE 3
#define CMD_RESET 4
#define CMD_SET_MECH_POSITION_TO_ZERO 6
#define CMD_CHANGE_CAN_ID 7
#define CMD_RAM_READ 17
#define CMD_RAM_WRITE 18
#define CMD_GET_MOTOR_FAIL 21

#define ADDR_RUN_MODE 0x7005
#define ADDR_IQ_REF 0x7006
#define ADDR_SPEED_REF 0x700A
#define ADDR_LIMIT_TORQUE 0x700B
#define ADDR_CURRENT_KP 0x7010
#define ADDR_CURRENT_KI 0x7011
#define ADDR_CURRENT_FILTER_GAIN 0x7014
#define ADDR_LOC_REF 0x7016
#define ADDR_LIMIT_SPEED 0x7017
#define ADDR_LIMIT_CURRENT 0x7018
#define ADDR_MECH_POS 0x7019
#define ADDR_IQF 0x701A
#define ADDR_MECH_VEL 0x701B
#define ADDR_VBUS 0x701C
#define ADDR_ROTATION 0x701D
#define ADDR_LOC_KP 0x701E
#define ADDR_SPD_KP 0x701F
#define ADDR_SPD_KI 0x7020

#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define VEL_MIN -30.0f
#define VEL_MAX 30.0f
#define CUR_MIN -27.0f
#define CUR_MAX 27.0f
#define TOR_MIN -12.0f
#define TOR_MAX 12.0f

#define CURRENT_FILTER_GAIN_MIN 0.0f
#define CURRENT_FILTER_GAIN_MAX 1.0f

#define CUR_KP_MAX 200.0f
#define CUR_KP_MIN 0.0f
#define CUR_KI_MAX 200.0f
#define CUR_KI_MIN 0.0f

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define T_MIN -12.0f
#define T_MAX 12.0f
#define IQ_MIN -27.0f
#define IQ_MAX 27.0f
#define CURRENT_FILTER_GAIN_MIN 0.0f
#define CURRENT_FILTER_GAIN_MAX 1.0f

#define DEFAULT_CURRENT_KP 0.125f
#define DEFAULT_CURRENT_KI 0.0158f
#define DEFAULT_CURRENT_FINTER_GAIN 0.1f
#define DEFAULT_POSITION_KP 30.0f
#define DEFAULT_VELOCITY_KP 2.0f
#define DEFAULT_VELOCITY_KI 0.002f
#define DEFAULT_VELOCITY_LIMIT 2.0f
#define DEFAULT_CURRENT_LIMIT 27.0f
#define DEFAULT_TORQUE_LIMIT 12.0f

#define RET_CYBERGEAR_OK 0x00
#define RET_CYBERGEAR_MSG_NOT_AVAIL 0x01
#define RET_CYBERGEAR_INVALID_CAN_ID 0x02
#define RET_CYBERGEAR_INVALID_PACKET 0x03

#define CW 1
#define CCW -1
} // namespace

class Cybergear : public EncoderBase {
public:
  enum class MotorMode : uint8_t {
    NONE = 0,
    RAD = 1,
    RADPS = 2,
    CURRENT = 3,
  };

  enum class TurnMode {
    SINGLE_TURN,
    MULTI_TURN,
  };

  Cybergear(peripheral::CAN &can, uint8_t motor_id, uint8_t master_id,
            TurnMode turnmode = TurnMode::MULTI_TURN)
      : EncoderBase(1 << 14), can_{can}, motor_id_{motor_id},
        master_id_{master_id}, turnmode_{turnmode} {
    reset();
    set_motor_mode(MotorMode::CURRENT);
    enable();
  }

  virtual ~Cybergear() {}

  bool update() override {
    float rad;
    if (!read_paramater(ADDR_MECH_POS, rad)) {
      return false;
    }
    int16_t count = (rad / (2 * M_PI)) * get_cpr();
    int16_t delta = count - prev_count_;

    switch (turnmode_) {
    case TurnMode::SINGLE_TURN: {
      set_count(count);
      break;
    }
    case TurnMode::MULTI_TURN: {
      if (delta > (get_cpr() / 2)) {
        delta -= get_cpr();
      } else if (delta < -(get_cpr() / 2)) {
        delta += get_cpr();
      }
      set_count(get_count() + delta);
      break;
    }
    }
    prev_count_ = count;
    return true;
  }

  bool transmit() {
    enable();
    transmit_current_ref(current_);
    return true;
  }

  void set_current(float ampare) { current_ = ampare; }
  float get_current() { return current_; }

  void stop() { reset(); }

  void enable() {
    stop_flag = false;
    send(motor_id_, CMD_ENABLE, master_id_, std::array<uint8_t, 8U>{0x00});
  }

  void reset() {
    stop_flag = true;
    send(motor_id_, CMD_RESET, master_id_, std::array<uint8_t, 8U>{0x00});
  }

  void set_motor_mode(MotorMode mode) {
    if (mode == MotorMode::NONE)
      return;
    std::array<uint8_t, 8U> data = {0x00};
    data[0] = ADDR_RUN_MODE & 0x00FF;
    data[1] = ADDR_RUN_MODE >> 8;
    data[4] = utility::to_underlying(mode);
    send(motor_id_, CMD_RAM_WRITE, master_id_, data);
  }

  template <class T> bool read_paramater(uint16_t index, T &dest) {
    std::array<uint8_t, 8U> data; // = {0x00};
    data.fill(0);
    data[0] = static_cast<uint8_t>(index);
    data[1] = static_cast<uint8_t>(index >> 8);
    if (!send(motor_id_, CMD_RAM_READ, master_id_, data)) {
      return false;
    }
    return receive(dest, CMD_RAM_READ, index, 2);
  }

private:
  peripheral::CAN &can_;
  const uint8_t motor_id_, master_id_;
  TurnMode turnmode_;

  unsigned long send_count_;
  bool stop_flag = true;
  float current_;
  int16_t prev_count_ = 0;

  bool transmit_current_ref(float value) {
    return write_float(motor_id_, ADDR_IQ_REF, value, CUR_MIN, CUR_MAX);
  }

  bool send(uint8_t can_id, uint8_t cmd_id, uint16_t option,
            const std::array<uint8_t, 8U> &data) {
    uint32_t id = cmd_id << 24 | option << 8 | can_id;
    ++send_count_;
    peripheral::CANBase::Message msg;
    msg.id_type = peripheral::CANBase::IDType::EXTENDED;
    msg.id = id;
    msg.data = data;
    msg.dlc = 8;
    return can_.transmit(msg, 1);
  }

  template <class T>
  bool receive(T &dest, uint8_t cmd, uint16_t index, uint32_t timeout) {
    peripheral::CANBase::Message msg;
    uint32_t start = core::Kernel::get_ticks();
    uint32_t elapsed = start;
    while (true) {
      bool res = can_.receive(msg, 0);
      if (res) {
        uint16_t id[] = {
            static_cast<uint16_t>(msg.id & 0xffff),
            static_cast<uint16_t>((motor_id_ << 8) | master_id_),
        };
        uint8_t rx_cmd = (msg.id & 0x3F000000) >> 24;
        if ((cmd == CMD_RAM_READ) && (rx_cmd == cmd) && (id[0] == id[1])) {
          // switch (cmd) {
          // case CMD_RESPONSE: // 応答
          //   process_motor_packet(msg.data);
          //   return true;
          //   break;
          // case CMD_RAM_READ: //
          if (process_read_parameter_packet(dest, index, msg.data)) {
            return true;
          }
          //   break;
          // default:
          //   break;
          // }
        }
      }
      elapsed = core::Kernel::get_ticks();
      if ((elapsed - start) >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }
    return false;
  }

  template <class T>
  bool process_read_parameter_packet(T &dest, uint16_t index,
                                     const std::array<uint8_t, 8U> &data) {
    uint16_t rx_index = data[1] << 8 | data[0];
    if (rx_index == index) {
      switch (index) {
      case ADDR_RUN_MODE:
        memcpy(&dest, &data[4], sizeof(uint8_t));
        break;
      case ADDR_ROTATION:
        memcpy(&dest, &data[4], sizeof(int16_t));
        break;
      default:
        memcpy(&dest, &data[4], sizeof(float));
        break;
      }
      return true;
    } else {
      return false;
    }
  }

  bool write_float(uint8_t can_id, uint16_t addr, float value, float min,
                   float max) {
    std::array<uint8_t, 8U> data = {0x00};
    data[0] = addr & 0x00FF;
    data[1] = addr >> 8;

    union Val {
      uint8_t data[4];
      float f;
    } val;
    val.f = (max < value) ? max : (min > value) ? min : value;
    data[4] = val.data[0];
    data[5] = val.data[1];
    data[6] = val.data[2];
    data[7] = val.data[3];

    return send(can_id, CMD_RAM_WRITE, master_id_, data);
  }

  int float2unit(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    x = std::clamp<float>(x, x_min, x_max);
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
  }

  float uint2float(uint16_t x, float x_min, float x_max) {
    uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    return (float)x / type_max * span + x_min;
  }
};

} // namespace module
} // namespace tutrcos

// bool setPosRef(float value) {
//   return write_float(motor_id_, ADDR_LOC_REF, value, POS_MIN, POS_MAX);
// }
// bool setVelRef(float value) {
//   return write_float(motor_id_, ADDR_SPEED_REF, value, VEL_MIN, VEL_MAX);
// }

// void setMechPos2Zero() {
//   std::array<uint8_t, 8U> data = {0x00};
//   data[0] = 0x01;
//   send(motor_id_, CMD_SET_MECH_POSITION_TO_ZERO, master_id_, data);
// }

// void changeId(uint8_t can_id) {
//   std::array<uint8_t, 8U> data = {0x00};
//   uint16_t option = can_id << 8 | master_id_;
//   send(motor_id_, CMD_CHANGE_CAN_ID, option, data);
// }

// void process_motor_packet(const std::array<uint8_t, 8U> &data) {
// motor_status_.raw_position = data[1] | data[0] << 8;
// motor_status_.raw_velocity = data[3] | data[2] << 8;
// motor_status_.raw_effort = data[5] | data[4] << 8;
// motor_status_.raw_temperature = data[7] | data[6] << 8;

// motor_status_.motor_id = motor_id_;
// motor_status_.position =
//     uint2float(motor_status_.raw_position, P_MIN, P_MAX);
// motor_status_.velocity =
//     uint2float(motor_status_.raw_velocity, V_MIN, V_MAX);
// motor_status_.effort = uint2float(motor_status_.raw_effort, T_MIN,
// T_MAX); motor_status_.temperature = motor_status_.raw_temperature;
// }