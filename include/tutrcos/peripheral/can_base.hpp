#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tutrcos {
namespace peripheral {

enum class CANIDType {
  STANDARD,
  EXTENDED,
};

struct CANMessage {
  CANIDType id_type;
  uint32_t id;
  uint8_t dlc;
  std::array<uint8_t, 8> data;
};

class CANBase {
public:
  virtual ~CANBase() {}
  virtual bool transmit(const CANMessage &msg) = 0;
  virtual bool receive(CANMessage &msg, uint32_t timeout) = 0;
};

} // namespace peripheral
} // namespace tutrcos
