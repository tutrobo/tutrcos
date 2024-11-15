#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "cobs.h"

#include "tutrcos/core.hpp"
#include "tutrcos/peripheral/uart.hpp"

namespace tutrcos {
namespace module {

class COBSBridge {
public:
  COBSBridge(peripheral::UART &uart) : uart_{uart} {}

  bool transmit(const std::vector<uint8_t> &data, uint32_t timeout) {
    std::vector<uint8_t> buf(COBS_ENCODE_DST_BUF_LEN_MAX(data.size()));
    cobs_encode_result res =
        cobs_encode(buf.data(), buf.size(), data.data(), data.size());
    if (res.status != COBS_ENCODE_OK) {
      return false;
    }
    return uart_.transmit(buf.data(), res.out_len, timeout);
  }

  bool receive(std::vector<uint8_t> &data, uint32_t timeout) {
    uint32_t start = core::Kernel::get_ticks();
    while (rx_buf_.empty() || rx_buf_.back() != 0x00) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      uint8_t tmp;
      if (uart_.receive(&tmp, 1, 1)) {
        rx_buf_.push_back(tmp);
      }
    }
    data.resize(COBS_DECODE_DST_BUF_LEN_MAX(rx_buf_.size()));
    cobs_decode_result res =
        cobs_decode(data.data(), data.size(), rx_buf_.data(), rx_buf_.size());
    rx_buf_.clear();
    if (res.status != COBS_ENCODE_OK) {
      return false;
    }
    data.resize(res.out_len);
    return true;
  }

private:
  peripheral::UART &uart_;
  std::vector<uint8_t> rx_buf_;
};

} // namespace module
} // namespace tutrcos
