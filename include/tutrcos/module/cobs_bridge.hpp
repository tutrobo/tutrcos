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

  template <class T>
  bool transmit(uint8_t id, const T &data, uint32_t timeout) {
    // id(1 byte) + data(n byte) + checksum(1 byte) + delimiter(1 byte)
    size_t max_encoded_size = COBS_ENCODE_DST_BUF_LEN_MAX(sizeof(T));
    if (tx_buf_.size() < max_encoded_size + 3) {
      tx_buf_.resize(max_encoded_size + 3);
    }
    cobs_encode_result res =
        cobs_encode(tx_buf_.data() + 1, tx_buf_.size() - 3, &data, sizeof(T));
    if (res.status != COBS_ENCODE_OK) {
      return false;
    }
    tx_buf_[0] = id;
    tx_buf_[res.out_len + 1] = 0xFF; // checksum
    tx_buf_[res.out_len + 2] = 0;    // delimiter
    return uart_.transmit(tx_buf_.data(), res.out_len + 3, timeout);
  }

  template <class T> bool receive(uint8_t &id, T &data, uint32_t timeout) {
    uint32_t start = core::Kernel::get_ticks();
    while (!read_to_end()) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }

    size_t max_decoded_size = COBS_ENCODE_DST_BUF_LEN_MAX(rx_buf_.size() - 3);
    /* if (sizeof(T) < COBS_ENCODE_DST_BUF_LEN_MAX(rx_buf_.size())) {
      return std::nullopt;
    } */
    cobs_decode_result res =
        cobs_decode(&data, sizeof(T), rx_buf_.data() + 1, rx_buf_.size() - 3);
    id = rx_buf_[0];
    rx_buf_.clear();
    if (res.status != COBS_DECODE_OK) {
      return false;
    }
    return true;
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
