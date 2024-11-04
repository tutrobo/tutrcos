#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>

#include "tutrcos/core.hpp"

extern "C" int _write(int file, char *ptr, int len);

namespace tutrcos {
namespace peripheral {

class UART {
public:
  UART(UART_HandleTypeDef *huart, size_t rx_queue_size = 64)
      : huart_{huart}, rx_queue_{rx_queue_size} {
    get_instances()[huart_] = this;
    if (HAL_UART_Receive_IT(huart_, &rx_buf_, 1) != HAL_OK) {
      Error_Handler();
    }
  }

  ~UART() { HAL_UART_Abort(huart_); }

  bool transmit(const uint8_t *data, size_t size) {
    std::lock_guard lock{tx_mutex_};
    tx_sem_.acquire();
    if (HAL_UART_Transmit_IT(huart_, data, size) != HAL_OK) {
      return false;
    }
    return true;
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{rx_mutex_};
    uint32_t start = core::Kernel::get_ticks();
    while (available() < size) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        break;
      }
      rx_sem_.try_acquire(timeout - elapsed);
    }
    if (available() < size) {
      return false;
    }
    for (size_t i = 0; i < size; ++i) {
      rx_queue_.pop(data[i], 0);
    }
    return true;
  }

  size_t available() {
    std::lock_guard lock{rx_mutex_};
    return rx_queue_.size();
  }

  void flush() {
    std::lock_guard lock{rx_mutex_};
    rx_queue_.clear();
  }

  void enable_printf() { get_uart_printf() = this; }

private:
  UART_HandleTypeDef *huart_;
  core::Mutex tx_mutex_;
  core::Mutex rx_mutex_;
  core::Semaphore tx_sem_{1, 1};
  core::Semaphore rx_sem_{1, 1};
  core::Queue<uint8_t> rx_queue_;
  uint8_t rx_buf_;

  static inline std::map<UART_HandleTypeDef *, UART *> &get_instances() {
    static std::map<UART_HandleTypeDef *, UART *> instances;
    return instances;
  }

  static inline UART *&get_uart_printf() {
    static UART *uart = nullptr;
    return uart;
  }

  friend void ::HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
  friend void ::HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
  friend void ::HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
  friend int ::_write(int file, char *ptr, int len);
};

} // namespace peripheral
} // namespace tutrcos
