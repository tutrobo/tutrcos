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

class COBSBridge {
public:
  COBSBridge(peripheral::UART &uart) : uart_{uart} {}

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    if (tx_buf_.size() < COBS_ENCODE_DST_BUF_LEN_MAX(size) + 1) {
      tx_buf_.resize(COBS_ENCODE_DST_BUF_LEN_MAX(size) + 1);
    }
    cobs_encode_result res =
        cobs_encode(tx_buf_.data(), tx_buf_.size(), data, size);
    if (res.status != COBS_ENCODE_OK) {
      return false;
    }
    tx_buf_[res.out_len] = 0;
    return uart_.transmit(tx_buf_.data(), res.out_len + 1, timeout);
  }

  std::optional<size_t> receive(uint8_t *data, size_t size, uint32_t timeout) {
    uint32_t start = core::Kernel::get_ticks();
    while (!read_to_end()) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return std::nullopt;
      }
      core::Thread::delay(1);
    }

    if (size < COBS_ENCODE_DST_BUF_LEN_MAX(rx_buf_.size())) {
      return std::nullopt;
    }
    cobs_decode_result res =
        cobs_decode(data, size, rx_buf_.data(), rx_buf_.size());
    rx_buf_.clear();
    if (res.status != COBS_DECODE_OK) {
      return std::nullopt;
    }
    return res.out_len;
  }

private:
  peripheral::UART &uart_;
  std::vector<uint8_t> tx_buf_;
  std::vector<uint8_t> rx_buf_;

  bool read_to_end() {
    uint8_t tmp;
    while (uart_.receive(&tmp, 1, 0)) {
      if (tmp == 0x00) {
        return true;
      }
      rx_buf_.push_back(tmp);
    }
    return false;
  }
};

} // namespace module
} // namespace tutrcos
