#pragma once

#include <cstddef>
#include <cstdint>

#include "tutrcos/core.hpp"
#include "tutrcos/peripheral/uart.hpp"

namespace tutrcos {
namespace module {

struct SerialMessage {
  uint8_t id;
  std::array<uint8_t, 8> data;
};

class SerialMessageController {
public:
  SerialMessageController(peripheral::UART &uart) : uart_{uart} {}

  void update() {
    while (true) {
      for (size_t i = 0; i < 12; ++i) {
        if (rx_buf_[0] & 0x80 || !uart_.receive(rx_buf_.data(), 1, 0)) {
          break;
        }
      }
      if (rx_buf_[0] & 0x80 || !uart_.receive(rx_buf_.data() + 1, 11, 0)) {
        return;
      }

      SerialMessage msg;
      if (decode_message(msg, rx_buf_.data())) {
        rx_queue_.push(msg, 0);
      }

      rx_buf_.fill(0);
    }
  }

  bool transmit(const SerialMessage &msg, uint32_t timeout) {
    encode_message(msg, tx_buf_.data());
    return uart_.transmit(tx_buf_.data(), tx_buf_.size(), timeout);
  }

  bool receive(SerialMessage &msg, uint32_t timeout) {
    return rx_queue_.pop(msg, timeout);
  }

private:
  peripheral::UART &uart_;
  std::array<uint8_t, 12> tx_buf_{};
  std::array<uint8_t, 12> rx_buf_{};
  core::Queue<SerialMessage> rx_queue_{64};

  void encode_message(const SerialMessage &msg, uint8_t *bytes) {
    bytes[0] = 0b10000000 | (msg.id & 0b1111111);
    uint64_t buf = 0;
    for (size_t i = 0; i < 8; ++i) {
      buf |= static_cast<uint64_t>(msg.data[i]) << (8 * i);
    }
    for (size_t i = 1; i < 11; ++i) {
      bytes[i] = buf & 0b1111111;
      buf >>= 7;
    }
    bytes[11] = checksum(bytes);
  }

  bool decode_message(SerialMessage &msg, const uint8_t *bytes) {
    msg.id = bytes[0] & 0b1111111;
    uint64_t buf = 0;
    for (size_t i = 0; i < 10; ++i) {
      buf |= static_cast<uint64_t>(bytes[i + 1]) << (7 * i);
    }
    for (size_t i = 0; i < 8; ++i) {
      msg.data[i] = buf & 0xFF;
      buf >>= 8;
    }
    return bytes[11] == checksum(bytes);
  }

  uint8_t checksum(const uint8_t *bytes) {
    uint8_t res = 0;
    for (size_t i = 0; i < 11; ++i) {
      res += bytes[i];
    }
    return res & 0b1111111;
  }
};

} // namespace module
} // namespace tutrcos
