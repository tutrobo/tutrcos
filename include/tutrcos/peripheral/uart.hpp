#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include "tutrcos/core.hpp"
#include "tutrcos/utility.hpp"

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
    auto uart =
        std::find(get_instances().begin(), get_instances().end(), nullptr);
    TUTRCOS_VERIFY(uart != get_instances().end());
    *uart = this;
    TUTRCOS_VERIFY(HAL_UART_Receive_IT(huart_, &rx_buf_, 1) == HAL_OK);
  }

  ~UART() {
    TUTRCOS_VERIFY(HAL_UART_Abort(huart_) == HAL_OK);
    auto uart = std::find(get_instances().begin(), get_instances().end(), this);
    TUTRCOS_VERIFY(uart != get_instances().end());
    *uart = nullptr;
  }

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    if (HAL_UART_Transmit_IT(huart_, data, size) != HAL_OK) {
      return false;
    }
    uint32_t start = core::Kernel::get_ticks();
    while (huart_->gState != HAL_UART_STATE_READY) {
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
    uint32_t start = core::Kernel::get_ticks();
    while (rx_queue_.size() < size) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
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

  void enable_stdout() { get_uart_stdout() = this; }

  UART_HandleTypeDef *get_hal_handle() { return huart_; }

private:
  UART_HandleTypeDef *huart_;
  core::Mutex mtx_;
  core::Queue<uint8_t> rx_queue_;

  static inline uint8_t rx_buf_;

  static inline std::array<UART *, 20> &get_instances() {
    static std::array<UART *, 20> instances{};
    return instances;
  }

  static inline UART *&get_uart_stdout() {
    static UART *uart = nullptr;
    return uart;
  }

  friend void ::HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
  friend void ::HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
  friend int ::_write(int file, char *ptr, int len);
};

} // namespace peripheral
} // namespace tutrcos
