#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "tutrcos/peripheral/uart.hpp"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    uart->rx_queue_.push(uart->rx_buf_, 0);
    HAL_UART_Receive_IT(uart->huart_, &uart->rx_buf_, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  HAL_UART_Abort_IT(huart);
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    HAL_UART_Receive_IT(uart->huart_, &uart->rx_buf_, 1);
  }
}

#endif
