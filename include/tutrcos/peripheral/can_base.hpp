#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "tutrcos/core/queue.hpp"

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
  virtual void add_rx_queue(uint32_t id, uint32_t mask,
                            core::Queue<Message> &queue) = 0;
};

} // namespace peripheral
} // namespace tutrcos
