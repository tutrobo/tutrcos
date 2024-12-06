#pragma once

#include "main.h"

#include <array>
#include <cstdint>

#include "tutrcos/peripheral/uart.hpp"

#include "encoder_base.hpp"

namespace tutrcos {
namespace module {

class AMT21 : public EncoderBase {
public:
  enum class Resolution : uint8_t {
    _12 = 12,
    _14 = 14,
  };

  enum class Mode {
    SINGLE_TURN,
    MULTI_TURN,
  };

  AMT21(peripheral::UART &uart, Resolution resolution, Mode mode,
        uint8_t address)
      : EncoderBase{1 << utility::to_underlying(resolution)}, uart_{uart},
        resolution_{resolution}, mode_{mode}, address_{address} {}

  bool update() override {
    uint16_t cpr = 1 << utility::to_underlying(resolution_);
    uint16_t response;
    if (!send_command(0x00, reinterpret_cast<uint8_t *>(&response))) {
      return false;
    }

    int16_t count = response & (cpr - 1);
    switch (mode_) {
    case Mode::SINGLE_TURN: {
      set_count(count);
      break;
    }
    case Mode::MULTI_TURN: {
      int16_t delta = count - prev_count_;
      if (delta > (cpr / 2)) {
        delta -= cpr;
      } else if (delta < -(cpr / 2)) {
        delta += cpr;
      }
      set_count(get_count() + delta);
      prev_count_ = count;
      break;
    }
    }
    return true;
  }

  bool set_zero_point() {
    if (!send_extended_command(0x5E)) {
      return false;
    }
    prev_count_ = 0;
    set_count(0);
    return true;
  }

private:
  peripheral::UART &uart_;
  Resolution resolution_;
  Mode mode_;
  uint8_t address_;
  int16_t prev_count_ = 0;

  bool send_command(uint8_t command, uint8_t *response) {
    uint8_t data = address_ | command;
    uart_.flush();
    if (!uart_.transmit(&data, 1, 1)) {
      return false;
    }
    uart_.receive(reinterpret_cast<uint8_t *>(response), 2, 1);
    return checksum(response[0], response[1]);
  }

  bool send_extended_command(uint8_t command) {
    std::array<uint8_t, 2> data{static_cast<uint8_t>(address_ | 0x02), command};
    if (!uart_.transmit(data.data(), data.size(), 1)) {
      return false;
    }
    return true;
  }

  bool checksum(uint8_t l, uint8_t h) {
    bool k1 = !(bit(h, 5) ^ bit(h, 3) ^ bit(h, 1) ^ bit(l, 7) ^ bit(l, 5) ^
                bit(l, 3) ^ bit(l, 1));
    bool k0 = !(bit(h, 4) ^ bit(h, 2) ^ bit(h, 0) ^ bit(l, 6) ^ bit(l, 4) ^
                bit(l, 2) ^ bit(l, 0));
    return (k1 == bit(h, 7)) && (k0 == bit(h, 6));
  }

  bool bit(uint8_t x, uint8_t i) { return ((x >> i) & 1) == 1; }
};

} // namespace module
} // namespace tutrcos
