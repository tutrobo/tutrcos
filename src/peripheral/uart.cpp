#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "tutrcos/peripheral/uart.hpp"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (auto uart = tutrcos::peripheral::UART::get_instances().get(huart)) {
    uart->sem_.release();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (auto uart = tutrcos::peripheral::UART::get_instances().get(huart)) {
    uart->rx_queue_.push(uart->rx_buf_, 0);
    uart->sem_.release();
    TUTRCOS_VERIFY(HAL_UART_Receive_IT(huart, &uart->rx_buf_, 1) == HAL_OK);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  TUTRCOS_VERIFY(HAL_UART_Abort_IT(huart) == HAL_OK);
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
  if (auto uart = tutrcos::peripheral::UART::get_instances().get(huart)) {
    uart->sem_.release();
    TUTRCOS_VERIFY(HAL_UART_Receive_IT(huart, &uart->rx_buf_, 1) == HAL_OK);
  }
}

int _write(int, char *ptr, int len) {
  if (auto uart = tutrcos::peripheral::UART::get_uart_stdout()) {
    uart->transmit(reinterpret_cast<uint8_t *>(ptr), len,
                   tutrcos::core::Kernel::MAX_DELAY);
    return len;
  }
  return -1;
}

#endif
