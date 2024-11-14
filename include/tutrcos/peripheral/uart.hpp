#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <mutex>
#include <vector>

#include "tutrcos/core.hpp"

extern "C" int _write(int file, char *ptr, int len);

namespace tutrcos {
namespace peripheral {

/**
 * Connectivity -> USART(UART)x -> NVIC Settings -> USART(UART)x global
 * interrupt を有効化してください。
 *
 * @code{.cpp}
 * #include <tutrcos.hpp>
 *
 * extern UART_HandleTypeDef huart2;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *
 *   UART uart2(&huart2);
 *
 *   while (true) {
 *     // 7バイト送信
 *     uint8_t data[] = {'h', 'e', 'l', 'l', 'o', '\r', '\n'};
 *     uart2.transmit(data, sizeof(data));
 *
 *     // 1バイト受信
 *     char c;
 *     if (uart2.receive((uint8_t *)&c, 1, Kernel::MAX_DELAY)) {
 *       uart2.printf("入力した文字: %c\r\n", c);
 *     }
 *
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
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

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    uint32_t start = core::Kernel::get_ticks();
    if (HAL_UART_Transmit_IT(huart_, data, size) != HAL_OK) {
      return false;
    }
    while (huart_->gState != HAL_UART_STATE_READY) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      if (HAL_UART_GetError(huart_) != HAL_UART_ERROR_NONE) {
        HAL_UART_Abort_IT(huart_);
        return false;
      }
      core::Thread::delay(1);
    }
    return true;
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    uint32_t start = core::Kernel::get_ticks();
    while (rx_queue_.size() < size) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      if (HAL_UART_GetError(huart_) != HAL_UART_ERROR_NONE) {
        HAL_UART_Abort_IT(huart_);
        return false;
      }
      core::Thread::delay(1);
    }
    for (size_t i = 0; i < size; ++i) {
      rx_queue_.pop(data[i], 0);
    }
    return true;
  }

  void flush() {
    std::lock_guard lock{mtx_};
    rx_queue_.clear();
  }

  template <class... Args> bool printf(const char *fmt, Args... args) {
    size_t size = std::snprintf(nullptr, 0, fmt, args...);
    if (size + 1 > printf_buf_.size()) {
      printf_buf_.resize(size + 1);
    }
    std::snprintf(reinterpret_cast<char *>(printf_buf_.data()), size + 1, fmt,
                  args...);
    return transmit(printf_buf_.data(), size, core::Kernel::MAX_DELAY);
  }

private:
  UART_HandleTypeDef *huart_;
  core::Mutex mtx_;
  core::Queue<uint8_t> rx_queue_;
  uint8_t rx_buf_;
  std::vector<uint8_t> printf_buf_;

  static inline std::map<UART_HandleTypeDef *, UART *> &get_instances() {
    static std::map<UART_HandleTypeDef *, UART *> instances;
    return instances;
  }

  friend void ::HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
};

} // namespace peripheral
} // namespace tutrcos
