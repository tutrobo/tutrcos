#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED

#include "tutrcos/peripheral/spi.hpp"

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (auto spi = tutrcos::peripheral::SPI::get_instances().get(hspi)) {
    spi->sem_.release();
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (auto spi = tutrcos::peripheral::SPI::get_instances().get(hspi)) {
    spi->sem_.release();
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (auto spi = tutrcos::peripheral::SPI::get_instances().get(hspi)) {
    spi->sem_.release();
  }
}

#endif
