#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

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
  typedef struct {
    unsigned long stamp_usec; //< timestamp
    uint8_t motor_id;         //!< motor id
    float position;           //!< encoder position (-4pi to 4pi)
    float velocity;           //!< motor velocity (-30rad/s to 30rad/s)
    float effort;             //!< motor effort (-12Nm - 12Nm)
    float temperature;        //!< temperature
    uint16_t raw_position;    //!< raw position (for sync data)
    uint16_t raw_velocity;    //!< raw velocity (for sync data)
    uint16_t raw_effort;      //!< raw effort (for sync data)
    uint16_t raw_temperature; //!< raw temperature (for sync data)
  } MotorStatus;

  typedef struct {
    bool encoder_not_calibrated;
    bool over_current_phase_a;
    bool over_current_phase_b;
    bool over_voltage;
    bool under_voltage;
    bool driver_chip;
    bool motor_over_tempareture;
  } MotorFault;

  typedef struct {
    unsigned long stamp_usec;
    uint16_t run_mode;
    float iq_ref;
    float spd_ref;
    float limit_torque;
    float cur_kp;
    float cur_ki;
    float cur_filt_gain;
    float loc_ref;
    float limit_spd;
    float limit_cur;
    float mech_pos;
    float iqf;
    float mech_vel;
    float vbus;
    int16_t rotation;
    float loc_kp;
    float spd_kp;
    float spd_ki;
  } MotorParameter;

  Cybergear(peripheral::CAN &can, uint8_t motor_id, uint8_t master_id)
      : EncoderBase(1),
        can_{
            can,
        },
        motor_id_{motor_id}, master_id_{master_id} {
    reset();
    set_motor_mode(MotorMode::CURRENT);
    enable();
  }

  virtual ~Cybergear() {}

  bool update() override {
    // if(stop_flag) {
    //   set_motor_mode(MotorMode::CURRENT);
    //   enable();
    // }
    enable();
    setCurRef(current_);
    return true;
  }

  void set_current(float ampare) { current_ = ampare; }

  void stop() {
    // setMode(MotorMode::NONE);
    reset();
    // setCurrent(0);
  }

  // void get_mech_position() { readRam(ADDR_MECH_POS); }

  void readRam(uint16_t index) {
    peripheral::CANBase::Message msg;
    msg.data = {0};
    std::array<uint8_t, 8U> data = {0x00};
    data[0] = static_cast<uint8_t>(index);
    data[1] = static_cast<uint8_t>(index >> 8);
    // while(can_.receive(msg, 1));
    // send(motor_id_, CMD_RAM_READ, master_id_, data);
    // if(can_.receive(msg, 20)){
    //   process_receive_data(msg);
    // } else {
    //   printf("timeout\r\n");
    // }
    send(motor_id_, CMD_RAM_READ, master_id_, data);
    while (can_.receive(msg, 10)) {
      // printf("id:%x / ", msg.id);
      // printf("dlc:%d / ", msg.dlc);
      // for(uint8_t d : msg.data) printf("%02x, ", d);
      // printf("\n");
      process_receive_data(msg);
    }
  }

  void process_receive_data(const peripheral::CANBase::Message &msg) {
    if (static_cast<uint16_t>(msg.id & 0xffff) ==
        static_cast<uint16_t>((motor_id_ << 8) | master_id_)) {
      uint8_t packet_type = (msg.id & 0x3F000000) >> 24;
      switch (packet_type) {
      case CMD_RESPONSE:
        // process_motor_packet(msg.data);
        // printf("response\r\n");
        break;
      case CMD_RAM_READ:
        process_read_parameter_packet(msg.data);
        break;
      // case CMD_GET_MOTOR_FAIL:
      //   break;
      default:
        printf("invalid response 0x%x\r\n", packet_type);
        break;
      }
    }
  }

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
    // uint8_t data[8] = {0x00};
    std::array<uint8_t, 8U> data = {0x00};
    data[0] = ADDR_RUN_MODE & 0x00FF;
    data[1] = ADDR_RUN_MODE >> 8;
    data[4] = utility::to_underlying(mode);
    send(motor_id_, CMD_RAM_WRITE, master_id_, data);
  }

private:
  void setPosRef(float value) {
    writeFloat(motor_id_, ADDR_LOC_REF, value, POS_MIN, POS_MAX);
  }
  void setVelRef(float value) {
    writeFloat(motor_id_, ADDR_SPEED_REF, value, VEL_MIN, VEL_MAX);
  }
  void setCurRef(float value) {
    writeFloat(motor_id_, ADDR_IQ_REF, value, CUR_MIN, CUR_MAX);
  }

  bool transmit(peripheral::CANBase::IDType idtype, uint32_t id,
                const std::array<uint8_t, 8U> &data, uint32_t timeout) {
    peripheral::CANBase::Message msg;
    msg.id_type = idtype;
    msg.id = id;
    msg.data = data;
    msg.dlc = 8;
    return can_.transmit(msg, timeout);
  }

  void process_motor_packet(const std::array<uint8_t, 8U> &data) {
    motor_status_.raw_position = data[1] | data[0] << 8;
    motor_status_.raw_velocity = data[3] | data[2] << 8;
    motor_status_.raw_effort = data[5] | data[4] << 8;
    motor_status_.raw_temperature = data[7] | data[6] << 8;

    // convert motor data
    // motor_status_.stamp_usec = micros();
    motor_status_.motor_id = motor_id_;
    motor_status_.position =
        uint2float(motor_status_.raw_position, P_MIN, P_MAX);
    motor_status_.velocity =
        uint2float(motor_status_.raw_velocity, V_MIN, V_MAX);
    motor_status_.effort = uint2float(motor_status_.raw_effort, T_MIN, T_MAX);
    motor_status_.temperature = motor_status_.raw_temperature;
  }

  void process_read_parameter_packet(const std::array<uint8_t, 8U> &data) {
    uint16_t index = data[1] << 8 | data[0];
    uint8_t uint8_data;
    memcpy(&uint8_data, &data[4], sizeof(uint8_t));
    int16_t int16_data;
    memcpy(&int16_data, &data[4], sizeof(int16_t));
    float float_data;
    memcpy(&float_data, &data[4], sizeof(float));

    for (uint8_t d : data)
      printf("%02x, ", d);
    printf("data : %02x, %04x, %f / ", uint8_data, (uint16_t)int16_data,
           float_data);
    printf("\r\n");
    return;

    switch (index) {
    case ADDR_RUN_MODE:
      motor_param_.run_mode = uint8_data;
      printf("ADDR_RUN_MODE = [0x%02x]\n", uint8_data);
      break;
    case ADDR_IQ_REF:
      motor_param_.iq_ref = float_data;
      printf("ADDR_IQ_REF = %f\r\n", float_data);
      break;
    case ADDR_SPEED_REF:
      motor_param_.spd_ref = float_data;
      printf("ADDR_SPEED_REF = %f\r\n", float_data);
      break;
    case ADDR_LIMIT_TORQUE:
      motor_param_.limit_torque = float_data;
      printf("ADDR_LIMIT_TORQUE = %f\r\n", float_data);
      break;
    case ADDR_CURRENT_KP:
      motor_param_.cur_kp = float_data;
      printf("ADDR_CURRENT_KP = %f\r\n", float_data);
      break;
    case ADDR_CURRENT_KI:
      motor_param_.cur_ki = float_data;
      printf("ADDR_CURRENT_KI = %f\r\n", float_data);
      break;
    case ADDR_CURRENT_FILTER_GAIN:
      motor_param_.cur_filt_gain = float_data;
      printf("ADDR_CURRENT_FILTER_GAIN = %f\r\n", float_data);
      break;
    case ADDR_LOC_REF:
      motor_param_.loc_ref = float_data;
      printf("ADDR_LOC_REF = %f\r\n", float_data);
      break;
    case ADDR_LIMIT_SPEED:
      motor_param_.limit_spd = float_data;
      printf("ADDR_LIMIT_SPEED = %f\r\n", float_data);
      break;
    case ADDR_LIMIT_CURRENT:
      motor_param_.limit_cur = float_data;
      printf("ADDR_LIMIT_CURRENT = %f\r\n", float_data);
      break;
    case ADDR_MECH_POS:
      motor_param_.mech_pos = float_data;
      printf("ADDR_MECH_POS = %f\r\n", float_data);
      break;
    case ADDR_IQF:
      motor_param_.iqf = float_data;
      printf("ADDR_IQF = %f\r\n", float_data);
      break;
    case ADDR_MECH_VEL:
      motor_param_.mech_vel = float_data;
      printf("ADDR_MECH_VEL = %f\r\n", float_data);
      break;
    case ADDR_VBUS:
      motor_param_.vbus = float_data;
      printf("ADDR_VBUS = %f\r\n", float_data);
      break;
    case ADDR_ROTATION:
      motor_param_.rotation = int16_data;
      printf("ADDR_ROTATION = %d\r\n", int16_data);
      break;
    case ADDR_LOC_KP:
      motor_param_.loc_kp = float_data;
      printf("ADDR_LOC_KP = %f\r\n", float_data);
      break;
    case ADDR_SPD_KP:
      motor_param_.spd_kp = float_data;
      printf("ADDR_SPD_KP = %f\r\n", float_data);
      break;
    case ADDR_SPD_KI:
      motor_param_.spd_ki = float_data;
      printf("ADDR_SPD_KI = %f\r\n", float_data);
      break;
    default:
      printf("Unknown index = 0x%04x\r\n", index);
      // is_updated = false;
      break;
    }
    // if (is_updated) {
    //   motor_param_.stamp_usec = micros();
    // }
  }

  void send(uint8_t can_id, uint8_t cmd_id, uint16_t option,
            const std::array<uint8_t, 8U> &data) {
    uint32_t id = cmd_id << 24 | option << 8 | can_id;
    transmit(peripheral::CANBase::IDType::EXTENDED, id, data, 1);
    ++send_count_;
  }

  void writeFloat(uint8_t can_id, uint16_t addr, float value, float min,
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

    send(can_id, CMD_RAM_WRITE, master_id_, data);
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

  void processPacket(const std::array<uint8_t, 8U> &data) {
    motor_status_.raw_position = data[1] | data[0] << 8;
    motor_status_.raw_velocity = data[3] | data[2] << 8;
    motor_status_.raw_effort = data[5] | data[4] << 8;
    motor_status_.raw_temperature = data[7] | data[6] << 8;

    // convert motor data
    // motor_status_.stamp_usec = micros();
    motor_status_.motor_id = motor_id_;
    motor_status_.position =
        uint2float(motor_status_.raw_position, POS_MIN, POS_MAX);
    motor_status_.velocity =
        uint2float(motor_status_.raw_velocity, VEL_MIN, VEL_MAX);
    motor_status_.effort =
        uint2float(motor_status_.raw_effort, TOR_MIN, TOR_MAX);
    motor_status_.temperature = motor_status_.raw_temperature;
  }

  peripheral::CAN &can_;
  MotorStatus motor_status_;
  MotorParameter motor_param_;

  const uint8_t motor_id_, master_id_;
  uint8_t receive_buffer_[64];
  unsigned long send_count_;
  bool stop_flag = true;
  float current_;
};

// void set_limit_speed(float speed) { writeFloat(motor_id_, ADDR_LIMIT_SPEED,
// speed, 0.0f, VEL_MAX); } void set_limit_current(float current) {
// writeFloat(motor_id_, ADDR_LIMIT_CURRENT, current, 0.0f, CUR_MAX); } void
// set_current_kp(float kp) { writeFloat(motor_id_, ADDR_CURRENT_KP, kp, 0.0f,
// CUR_KP_MAX); } void set_current_ki(float ki) { writeFloat(motor_id_,
// ADDR_CURRENT_KI, ki, 0.0f, CUR_KI_MAX); } void set_current_filter_gain(float
// gain) { writeFloat(motor_id_, ADDR_CURRENT_FILTER_GAIN, gain,
// CURRENT_FILTER_GAIN_MIN, CURRENT_FILTER_GAIN_MAX); } void
// set_limit_torque(float torque) { writeFloat(motor_id_, ADDR_LIMIT_TORQUE,
// torque, 0.0f, TOR_MAX); } void set_position_kp(float kp) {
// writeFloat(motor_id_, ADDR_LOC_KP, kp, 0.0f, 200.0f); } void
// set_velocity_kp(float kp) { writeFloat(motor_id_, ADDR_SPD_KP, kp, 0.0f,
// 200.0f); } void set_velocity_ki(float ki) { writeFloat(motor_id_,
// ADDR_SPD_KI, ki, 0.0f, 200.0f); } void get_mech_position() {
// readRam(ADDR_MECH_POS); } void get_mech_velocity() { readRam(ADDR_MECH_VEL);
// } void get_vbus() { readRam(ADDR_VBUS); } void get_rotation() {
// readRam(ADDR_ROTATION); }

// void motor_control(float position, float speed, float torque, float kp, float
// kd) {
//   std::array<uint8_t, 8U> data
//   data[0] = float2unit(position, POS_MIN, POS_MAX, 16) >> 8;
//   data[1] = float2unit(position, POS_MIN, POS_MAX, 16);
//   data[2] = float2unit(speed, VEL_MIN, VEL_MAX, 16) >> 8;
//   data[3] = float2unit(speed, VEL_MIN, VEL_MAX, 16);
//   data[4] = float2unit(kp, KP_MIN, KP_MAX, 16) >> 8;
//   data[5] = float2unit(kp, KP_MIN, KP_MAX, 16);
//   data[6] = float2unit(kd, KD_MIN, KD_MAX, 16) >> 8;
//   data[7] = float2unit(kd, KD_MIN, KD_MAX, 16);

//   uint16_t data_torque = float2unit(torque, TOR_MIN, TOR_MAX, 16);
//   send(motor_id_, CMD_POSITION, data_torque, data);
// }

// void dump_motor_param() {
//     const std::vector<uint16_t> index_array = {
//         ADDR_RUN_MODE,
//         ADDR_IQ_REF,
//         ADDR_SPEED_REF,
//         ADDR_LIMIT_TORQUE,
//         ADDR_CURRENT_KP,
//         ADDR_CURRENT_KI,
//         ADDR_CURRENT_FILTER_GAIN,
//         ADDR_LOC_REF,
//         ADDR_LIMIT_SPEED,
//         ADDR_LIMIT_CURRENT,
//         ADDR_MECH_POS,
//         ADDR_IQF,
//         ADDR_MECH_VEL,
//         ADDR_VBUS,
//         ADDR_ROTATION,
//         ADDR_LOC_KP,
//         ADDR_SPD_KP,
//         ADDR_SPD_KI
//     };

//     for (auto index : index_array){
//         read_ram_data(index);
//         // delay(1);
//     }
// }
// uint8_t getMode() const { return run_mode_; }

// uint8_t getId() const { return motor_id_; }

// bool process_packet() {
//   bool check_update = false;
//   while (true) {
//     if (receive_motor_data(motor_status_)) {
//       check_update = true;
//     } else {
//       break;
//     }
//   }
//   return check_update;
// }

// bool update_motor_status(unsigned long id, const uint8_t * data, unsigned
// long len){
//   uint8_t receive_can_id = id & 0xff;
//   if ( receive_can_id != master_id_ ) {
//     return false;
//   }
//   uint8_t motor_can_id = (id & 0xff00) >> 8;
//   if ( motor_can_id != motor_id_ ) {
//     return false;
//   }
//   // check packet type
//   uint8_t packet_type = (id & 0x3F000000) >> 24;
//   if (packet_type == CMD_RESPONSE) {
//     process_motor_packet(data, len);
//   } else if (packet_type == CMD_RAM_READ){
//     process_read_parameter_packet(data, len);
//   } else if (packet_type == CMD_GET_MOTOR_FAIL) {
//     // NOT IMPLEMENTED
//   } else {
//     printf("invalid command response [0x%x]\n", packet_type);
//     print_can_packet(id, data, len);
//     return false;
//   }
//   return true;
// }

// bool receive_motor_data(MotorStatus & mot) {
//   // receive data
//   unsigned long id;
//   uint8_t len;
//   if (!can_->read_message(id, receive_buffer_, len)) {
//     printf("received data is not available\r\n");
//     return false;
//   }
//   // if id is not mine
//   uint8_t receive_can_id = id & 0xff;
//   if ( receive_can_id != master_id_ ) {
//     printf("Invalid master can id. Expected=[0x%02x] Actual=[0x%02x]
//     Raw=[%x]\r\n", master_id_, receive_can_id, id); return false;
//   }
//   uint8_t motor_can_id = (id & 0xff00) >> 8;
//   if ( motor_can_id != motor_id_ ) {
//     printf("Invalid target can id. Expected=[0x%02x] Actual=[0x%02x]
//     Raw=[%x]\r\n", motor_id_, motor_can_id, id); return false;
//   }
//   // parse packet --------------
//   return update_motor_status(id, receive_buffer_, len);
// }

} // namespace module
} // namespace tutrcos
