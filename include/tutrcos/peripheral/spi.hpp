#pragma once

#include "main.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>

#include "tutrcos/core.hpp"

namespace tutrcos {
namespace peripheral {

class SPI {
public:
  SPI(SPI_HandleTypeDef *hspi) : hspi_{hspi} { get_instances()[hspi_] = this; }

  ~SPI() { get_instances().erase(hspi_); }

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    thread_id_ = core::Thread::get_id();
    if (HAL_SPI_Transmit_IT(hspi_, data, size) != HAL_OK) {
      return false;
    }
    uint32_t start = core::Kernel::get_ticks();
    while (hspi_->State != HAL_SPI_STATE_READY) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::wait(1);
    }
    return true;
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    thread_id_ = core::Thread::get_id();
    if (HAL_SPI_Receive_IT(hspi_, data, size) != HAL_OK) {
      return false;
    }
    uint32_t start = core::Kernel::get_ticks();
    while (hspi_->State != HAL_SPI_STATE_READY) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::wait(1);
    }
    return true;
  }

  bool transmit_receive(const uint8_t *tx_data, uint8_t *rx_data, size_t size,
                        uint32_t timeout) {
    std::lock_guard lock{mtx_};
    thread_id_ = core::Thread::get_id();
    if (HAL_SPI_TransmitReceive_IT(hspi_, tx_data, rx_data, size) != HAL_OK) {
      return false;
    }
    uint32_t start = core::Kernel::get_ticks();
    while (hspi_->State != HAL_SPI_STATE_READY) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      core::Thread::wait(1);
    }
    return true;
  }

private:
  SPI_HandleTypeDef *hspi_;
  std::atomic<core::Thread::Id> thread_id_{nullptr};
  core::Mutex mtx_;

  static inline std::map<SPI_HandleTypeDef *, SPI *> &get_instances() {
    static std::map<SPI_HandleTypeDef *, SPI *> instances;
    return instances;
  }

  friend void ::HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
  friend void ::HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
  friend void ::HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
};

} // namespace peripheral
} // namespace tutrcos
