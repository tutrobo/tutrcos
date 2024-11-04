#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "tutrcos/peripheral/uart.hpp"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    uart->tx_sem_.release();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    uart->rx_queue_.push(uart->rx_buf_, 0);
    uart->rx_sem_.release();
    HAL_UART_Receive_IT(huart, &uart->rx_buf_, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    HAL_UART_Abort(huart);
    uart->tx_sem_.release();
    uart->rx_sem_.release();
    HAL_UART_Receive_IT(huart, &uart->rx_buf_, 1);
  }
}

int _write(int, char *ptr, int len) {
  auto uart = tutrcos::peripheral::UART::get_uart_printf();
  if (uart) {
    uart->transmit(reinterpret_cast<uint8_t *>(ptr), len);
    return len;
  }
  return -1;
}

#endif
