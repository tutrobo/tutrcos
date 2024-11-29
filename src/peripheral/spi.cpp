#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED

#include "tutrcos/peripheral/spi.hpp"

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  auto itr = tutrcos::peripheral::SPI::get_instances().find(hspi);
  if (itr != tutrcos::peripheral::SPI::get_instances().end()) {
    auto spi = itr->second;
    spi->sem_.release();
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  auto itr = tutrcos::peripheral::SPI::get_instances().find(hspi);
  if (itr != tutrcos::peripheral::SPI::get_instances().end()) {
    auto spi = itr->second;
    spi->sem_.release();
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  auto itr = tutrcos::peripheral::SPI::get_instances().find(hspi);
  if (itr != tutrcos::peripheral::SPI::get_instances().end()) {
    auto spi = itr->second;
    spi->sem_.release();
  }
}

#endif
