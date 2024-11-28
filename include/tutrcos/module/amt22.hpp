#pragma once

#include "main.h"

#include <array>
#include <cstdint>

#include "tutrcos/peripheral/gpio.hpp"
#include "tutrcos/peripheral/spi.hpp"

#include "encoder_base.hpp"

namespace tutrcos {
namespace module {

class AMT22 : public EncoderBase {
public:
  enum class Resolution : uint8_t {
    _12 = 12,
    _14 = 14,
  };

  AMT22(peripheral::SPI &spi, peripheral::GPIO &cs, Resolution resolution,
        bool multi_turn, float dt)
      : EncoderBase{1 << utility::to_underlying(resolution), dt}, spi_{spi},
        cs_{cs}, resolution_{resolution}, multi_turn_{multi_turn} {
    cs_.write(true);
  }

  bool update() {
    uint16_t cpr = 1 << utility::to_underlying(resolution_);
    if (multi_turn_) {
      std::array<uint8_t, 4> tx{0x00, 0xA0, 0x00, 0x00};
      std::array<uint16_t, 2> rx{};
      if (!send_command(tx.data(), reinterpret_cast<uint8_t *>(rx.data()),
                        tx.size())) {
        return false;
      }

      int16_t count = rx[1] & (cpr - 1);
      int16_t rotation = rx[0] & ((1 << 14) - 1);
      set_count(rotation * cpr + count);
    } else {
      std::array<uint8_t, 2> tx{0x00, 0x00};
      uint16_t rx;
      if (!send_command(tx.data(), reinterpret_cast<uint8_t *>(&rx),
                        tx.size())) {
        return false;
      }

      int16_t count = rx & (cpr - 1);
      if ((count - prev_count_) < -(cpr / 2)) {
        rotation_++;
      } else if ((count - prev_count_) > (cpr / 2)) {
        rotation_--;
      }
      prev_count_ = count;
      set_count(rotation_ * cpr + count);
    }
    return true;
  }

  bool set_zero_point() {
    std::array<uint8_t, 2> tx{0x00, 0x70};
    std::array<uint8_t, 2> rx{};
    if (!send_command(tx.data(), rx.data(), tx.size())) {
      return false;
    }
    prev_count_ = 0;
    rotation_ = 0;
    set_count(0);
    return true;
  }

private:
  peripheral::SPI &spi_;
  peripheral::GPIO &cs_;
  Resolution resolution_;
  bool multi_turn_;
  int16_t prev_count_ = 0;
  int64_t rotation_ = 0;

  bool send_command(const uint8_t *tx_data, uint8_t *rx_data, size_t size) {
    cs_.write(false);
    for (size_t i = 0; i < size; ++i) {
      if (!spi_.transmit_receive(&tx_data[i], &rx_data[size - i - 1], 1, 5)) {
        return false;
      }
    }
    cs_.write(true);
    for (size_t i = 0; i < size; i += 2) {
      if (!checksum(rx_data[i], rx_data[i + 1])) {
        return false;
      }
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
