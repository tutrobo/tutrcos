#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>
#include <mutex>

#include "tutrcos/core.hpp"
#include "tutrcos/utility.hpp"

namespace tutrcos {
namespace peripheral {

class SPI {
public:
  SPI(SPI_HandleTypeDef *hspi) : hspi_{hspi} {}

  ~SPI() { TUTRCOS_VERIFY(HAL_SPI_Abort(hspi_) == HAL_OK); }

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    if (HAL_SPI_Transmit_IT(hspi_, data, size) != HAL_OK) {
      return false;
    }
    uint32_t start = core::Kernel::get_ticks();
    while (hspi_->State != HAL_SPI_STATE_READY) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }
    return true;
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    if (HAL_SPI_Receive_IT(hspi_, data, size) != HAL_OK) {
      return false;
    }
    uint32_t start = core::Kernel::get_ticks();
    while (hspi_->State != HAL_SPI_STATE_READY) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }
    return true;
  }

  bool transmit_receive(const uint8_t *tx_data, uint8_t *rx_data, size_t size,
                        uint32_t timeout) {
    std::lock_guard lock{mtx_};
    if (HAL_SPI_TransmitReceive_IT(hspi_, tx_data, rx_data, size) != HAL_OK) {
      return false;
    }
    uint32_t start = core::Kernel::get_ticks();
    while (hspi_->State != HAL_SPI_STATE_READY) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::delay(1);
    }
    return true;
  }

  SPI_HandleTypeDef *get_hal_handle() { return hspi_; }

private:
  SPI_HandleTypeDef *hspi_;
  core::Mutex mtx_;
};

} // namespace peripheral
} // namespace tutrcos
