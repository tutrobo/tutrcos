#pragma once

#include "main.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <vector>

#include "tutrcos/core.hpp"
#include "tutrcos/utility.hpp"

#include "peripheral_table.hpp"

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
      : huart_{huart}, rx_buf_(rx_queue_size) {
    get_instances().set(huart_, this);
    TUTRCOS_VERIFY(HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buf_.data(),
                                                rx_buf_.size()) == HAL_OK);
  }

  ~UART() {
    TUTRCOS_VERIFY(HAL_UART_Abort(huart_) == HAL_OK);
    get_instances().erase(huart_);
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
      sem_.try_acquire(1);
    }
    return true;
  }

  bool transmit(const std::vector<uint8_t> &data, uint32_t timeout) {
    return transmit(data.data(), data.size(), timeout);
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    uint32_t start = core::Kernel::get_ticks();
    while ((rx_tail_ + rx_buf_.size() - rx_head_) % rx_buf_.size() < size) {
      uint32_t elapsed = core::Kernel::get_ticks() - start;
      if (elapsed >= timeout) {
        return false;
      }
      sem_.try_acquire(1);
    }
    for (size_t i = 0; i < size; ++i) {
      data[i] = rx_buf_[rx_head_++];
      if (rx_head_ == rx_buf_.size()) {
        rx_head_ = 0;
      }
    }
    return true;
  }

  void flush() {
    std::lock_guard lock{mtx_};
    rx_head_ = rx_tail_;
  }

  void enable_stdout() { get_uart_stdout() = this; }

private:
  UART_HandleTypeDef *huart_;
  core::Mutex mtx_;
  core::Semaphore sem_{1, 0};
  std::vector<uint8_t> rx_buf_;
  size_t rx_head_ = 0;
  std::atomic<size_t> rx_tail_ = 0;

  static inline PeripheralTable<UART_HandleTypeDef *, UART, 32> &
  get_instances() {
    static PeripheralTable<UART_HandleTypeDef *, UART, 32> instances;
    return instances;
  }

  static inline UART *&get_uart_stdout() {
    static UART *uart = nullptr;
    return uart;
  }

  friend void ::HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
  friend void ::HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size);
  friend void ::HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
  friend int ::_write(int file, char *ptr, int len);
};

} // namespace peripheral
} // namespace tutrcos
