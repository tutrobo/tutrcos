#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "tutrcos/peripheral/uart.hpp"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    uart->sem_.release();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    uart->rx_tail_ = Size;
    uart->sem_.release();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    __disable_irq();
    TUTRCOS_VERIFY(HAL_UART_Abort(huart) == HAL_OK);
    uart->rx_head_ = 0;
    uart->rx_tail_ = 0;
    __enable_irq();
    uart->sem_.release();
    TUTRCOS_VERIFY(HAL_UARTEx_ReceiveToIdle_DMA(huart, uart->rx_buf_.data(),
                                                uart->rx_buf_.size()) ==
                   HAL_OK);
  }
}

int _write(int, char *ptr, int len) {
  auto uart = tutrcos::peripheral::UART::get_uart_stdout();
  if (uart) {
    uart->transmit(reinterpret_cast<uint8_t *>(ptr), len,
                   tutrcos::core::Kernel::MAX_DELAY);
    return len;
  }
  return -1;
}

#endif
