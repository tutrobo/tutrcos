#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
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
    tx_buf_.resize(COBS_ENCODE_DST_BUF_LEN_MAX(msg.data.size()) + 3);
    cobs_encode_result res = cobs_encode(tx_buf_.data() + 1, tx_buf_.size() - 3,
                                         msg.data.data(), msg.data.size());
    if (res.status != COBS_ENCODE_OK) {
      return false;
    }
    tx_buf_[0] = msg.id;
    tx_buf_[res.out_len + 1] = checksum(tx_buf_.data(), res.out_len + 1);
    tx_buf_[res.out_len + 2] = 0; // delimiter
    return uart_.transmit(tx_buf_.data(), res.out_len + 3, timeout);
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

    if (checksum(rx_buf_.data(), rx_buf_.size() - 2) !=
        rx_buf_[rx_buf_.size() - 2]) {
      rx_buf_.clear();
      return false;
    }
    msg.data.resize(COBS_DECODE_DST_BUF_LEN_MAX(rx_buf_.size() - 3));
    cobs_decode_result res =
        cobs_decode(msg.data.data(), msg.data.size(), rx_buf_.data() + 1,
                    rx_buf_.size() - 3);
    if (res.status != COBS_DECODE_OK) {
      rx_buf_.clear();
      return false;
    }
    msg.id = rx_buf_[0];
    rx_buf_.clear();
    msg.data.resize(res.out_len);
    return true;
  }

private:
  peripheral::UART &uart_;
  std::vector<uint8_t> tx_buf_;
  std::vector<uint8_t> rx_buf_;

  bool read_to_end() {
    uint8_t tmp;
    while (uart_.receive(&tmp, 1, 0)) {
      rx_buf_.push_back(tmp);
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
    return 0x80 | (res & 0x7F);
  }
};

} // namespace module
} // namespace tutrcos
