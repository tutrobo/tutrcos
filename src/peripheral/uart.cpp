#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "tutrcos/peripheral/uart.hpp"

using tutrcos::peripheral::UART;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  for (UART *uart : UART::get_instances()) {
    if (uart->huart_ == huart) {
      uart->rx_queue_.push(UART::rx_buf_, 0);
      break;
    }
  }
  TUTRCOS_VERIFY(HAL_UART_Receive_IT(huart, &UART::rx_buf_, 1) == HAL_OK);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  TUTRCOS_VERIFY(HAL_UART_Abort_IT(huart) == HAL_OK);
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
  TUTRCOS_VERIFY(HAL_UART_Receive_IT(huart, &UART::rx_buf_, 1) == HAL_OK);
}

int _write(int, char *ptr, int len) {
  if (auto uart = UART::get_uart_stdout()) {
    if (__get_IPSR()) { // for debug
      if (HAL_UART_Transmit(uart->huart_, reinterpret_cast<uint8_t *>(ptr), len,
                            HAL_MAX_DELAY) == HAL_OK) {
        return len;
      }
    } else {
      if (uart->transmit(reinterpret_cast<uint8_t *>(ptr), len,
                         tutrcos::core::Kernel::MAX_DELAY)) {
        return len;
      }
    }
  }
  return -1;
}

#endif
