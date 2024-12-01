#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "tutrcos/peripheral/uart.hpp"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (auto uart = tutrcos::peripheral::UART::get_instances().get(huart)) {
    uart->sem_.release();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (auto uart = tutrcos::peripheral::UART::get_instances().get(huart)) {
    __disable_irq();
    uart->rx_tail_ = Size;
    __enable_irq();
    uart->sem_.release();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  HAL_UART_AbortTransmit_IT(huart);
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart) {
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE) != RESET) {
    __HAL_UART_CLEAR_PEFLAG(huart);
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET) {
    __HAL_UART_CLEAR_FEFLAG(huart);
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET) {
    __HAL_UART_CLEAR_NEFLAG(huart);
  } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
  if (auto uart = tutrcos::peripheral::UART::get_instances().get(huart)) {
    uart->sem_.release();
    TUTRCOS_VERIFY(HAL_UARTEx_ReceiveToIdle_DMA(huart, uart->rx_buf_.data(),
                                                uart->rx_buf_.size()) ==
                   HAL_OK);
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
