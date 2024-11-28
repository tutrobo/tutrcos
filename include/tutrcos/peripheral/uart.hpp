#pragma once

#include "main.h"

#include <atomic>
#include <cassert>
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
 *     uart2.transmit(data, sizeof(data), Kernel::MAX_DELAY);
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
    assert(HAL_UART_Receive_IT(huart_, &rx_buf_, 1) == HAL_OK);
  }

  ~UART() {
    get_instances().erase(huart_);
    HAL_UART_Abort_IT(huart_);
  }

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    thread_id_ = core::Thread::get_id();
    if (HAL_UART_Transmit_IT(huart_, data, size) != HAL_OK) {
      return false;
    }
    uint32_t start = core::Kernel::get_ticks();
    while (huart_->gState != HAL_UART_STATE_READY) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      if (huart_->gState == HAL_UART_STATE_ERROR) {
        assert(HAL_UART_Abort(huart_) == HAL_OK);
        assert(HAL_UART_Receive_IT(huart_, &rx_buf_, 1) == HAL_OK);
        return false;
      }
      core::Thread::wait(1);
    }
    return true;
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    thread_id_ = core::Thread::get_id();
    uint32_t start = core::Kernel::get_ticks();
    while (rx_queue_.size() < size) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      if (huart_->gState == HAL_UART_STATE_ERROR) {
        assert(HAL_UART_Abort(huart_) == HAL_OK);
        assert(HAL_UART_Receive_IT(huart_, &rx_buf_, 1) == HAL_OK);
        return false;
      }
      core::Thread::wait(1);
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

  void enable_stdout() { get_uart_stdout() = this; }

  template <class... Args> bool printf(const char *fmt, Args... args) {
    size_t size = std::snprintf(nullptr, 0, fmt, args...);
    printf_buf_.resize(size + 1);
    std::snprintf(reinterpret_cast<char *>(printf_buf_.data()), size + 1, fmt,
                  args...);
    return transmit(printf_buf_.data(), size, core::Kernel::MAX_DELAY);
  }

private:
  std::atomic<core::Thread::Id> thread_id_{nullptr};
  UART_HandleTypeDef *huart_;
  core::Mutex mtx_;
  core::Queue<uint8_t> rx_queue_;
  uint8_t rx_buf_;
  std::vector<uint8_t> printf_buf_;

  static inline std::map<UART_HandleTypeDef *, UART *> &get_instances() {
    static std::map<UART_HandleTypeDef *, UART *> instances;
    return instances;
  }

  static inline UART *&get_uart_stdout() {
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
