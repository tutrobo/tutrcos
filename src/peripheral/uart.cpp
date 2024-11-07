#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "tutrcos/peripheral/uart.hpp"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    uart->tx_res_.push(true, 0);
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
  if (huart->gState != HAL_UART_STATE_READY) {
    HAL_UART_AbortTransmit_IT(huart);
  }
  if (huart->RxState != HAL_UART_STATE_READY) {
    HAL_UART_AbortReceive_IT(huart);
  }
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    uart->tx_res_.push(false, 0);
  }
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart) {
  auto itr = tutrcos::peripheral::UART::get_instances().find(huart);
  if (itr != tutrcos::peripheral::UART::get_instances().end()) {
    auto uart = itr->second;
    HAL_UART_Receive_IT(huart, &uart->rx_buf_, 1);
  }
}

#endif
