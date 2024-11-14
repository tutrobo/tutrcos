#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>

#include "tutrcos/core.hpp"

namespace tutrcos {
namespace peripheral {

class I2CMaster {
public:
  I2CMaster(I2C_HandleTypeDef *hi2c) : hi2c_{hi2c} {}

  bool transmit(uint16_t address, uint8_t *data, size_t size,
                uint32_t timeout) {
    std::lock_guard lock{mtx_};
    uint32_t start = core::Kernel::get_ticks();
    while (HAL_I2C_Master_Transmit_IT(hi2c_, address << 1, data, size) !=
           HAL_OK) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }
    return true;
  }

  bool receive(uint16_t address, uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    uint32_t start = core::Kernel::get_ticks();
    while (HAL_I2C_Master_Receive_IT(hi2c_, address << 1, data, size) !=
           HAL_OK) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }
    return true;
  }

private:
  I2C_HandleTypeDef *hi2c_;
  core::Mutex mtx_;
};

} // namespace peripheral
} // namespace tutrcos
