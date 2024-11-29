#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "tutrcos/peripheral/uart.hpp"

#include "tutrcos/peripheral/tim.hpp"
extern tutrcos::peripheral::TIM *tim;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    uart->sem_.release();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  tim->set_counter(0);
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    for (uint16_t i = 0; i < Size; ++i) {
      uart->rx_queue_.push(uart->rx_buf_[i], 0);
    }
    uart->sem_.release();
    TUTRCOS_VERIFY(HAL_UARTEx_ReceiveToIdle_IT(huart, uart->rx_buf_.data(),
                                               uart->rx_buf_.size()) == HAL_OK);
  }
  printf("%d %d\r\n", (int)tim->get_counter(), (int)Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    TUTRCOS_VERIFY(HAL_UART_Abort(huart) == HAL_OK);
    uart->sem_.release();
    TUTRCOS_VERIFY(HAL_UARTEx_ReceiveToIdle_IT(huart, uart->rx_buf_.data(),
                                               uart->rx_buf_.size()) == HAL_OK);
  }
}

int _write(int, char *ptr, int len) {
  auto uart = tutrcos::peripheral::UART::get_uart_stdout();
  if (uart) {
    HAL_UART_Transmit(uart->huart_, reinterpret_cast<uint8_t *>(ptr), len,
                      tutrcos::core::Kernel::MAX_DELAY);
    return len;
  }
  return -1;
}

#endif
