#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "cobs.h"

#include "tutrcos/core.hpp"
#include "tutrcos/peripheral/uart.hpp"

namespace tutrcos {
namespace module {

struct COBSBridgeMessage {
  uint8_t id;
  std::vector<uint8_t> data;
};

class COBSBridge {
public:
  COBSBridge(peripheral::UART &uart) : uart_{uart} {}

  bool transmit(const COBSBridgeMessage &msg, uint32_t timeout) {
    // id(1 byte) + data(n byte) + checksum(1 byte) + delimiter(1 byte)
    std::vector<uint8_t> src(msg.data.size() + 1);
    src[0] = msg.id;
    std::copy(msg.data.begin(), msg.data.end(), src.begin() + 1);
    src.push_back(checksum(src.data(), src.size()));

    std::vector<uint8_t> dest;
    if (!utility::cobs_encode(src, dest)) {
      return false;
    }

    return uart_.transmit(dest.data(), dest.size(), timeout);
  }

  bool receive(COBSBridgeMessage &msg, uint32_t timeout) {
    uint32_t start = core::Kernel::get_ticks();
    while (!read_to_end()) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }

    std::vector<uint8_t> dest;
    if (!utility::cobs_decode(buf_, dest)) {
      buf_.clear();
      return false;
    }

    if (checksum(dest.data(), dest.size() - 1) != dest[dest.size()]) {
      buf_.clear();
      return false;
    }

    msg.id = dest[0];
    msg.data.resize(dest.size() - 1);
    std::copy(dest.begin() + 1, dest.end(), msg.data.begin());
    buf_.clear();
    return true;
  }

private:
  peripheral::UART &uart_;
  std::vector<uint8_t> buf_;

  bool read_to_end() {
    uint8_t tmp;
    while (uart_.receive(&tmp, 1, 0)) {
      buf_.push_back(tmp);
      if (tmp == 0x00) {
        return true;
      }
    }
    return false;
  }

  uint8_t checksum(const uint8_t *data, size_t size) {
    uint8_t res = 0;
    for (size_t i = 0; i < size; ++i) {
      res += data[i];
    }
    return res;
  }
};

} // namespace module
} // namespace tutrcos
