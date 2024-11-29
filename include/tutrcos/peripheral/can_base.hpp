#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tutrcos {
namespace peripheral {

class CANBase {
public:
  enum class IDType {
    STANDARD,
    EXTENDED,
  };

  struct Message {
    IDType id_type;
    uint32_t id;
    uint8_t dlc;
    std::array<uint8_t, 8> data;
  };

  virtual ~CANBase() {}
  virtual bool transmit(const Message &msg, uint32_t timeout) = 0;
  virtual bool receive(Message &msg, uint32_t timeout) = 0;
};

} // namespace peripheral
} // namespace tutrcos
