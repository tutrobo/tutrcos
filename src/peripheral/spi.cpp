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

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  TUTRCOS_VERIFY(HAL_SPI_Abort_IT(hspi) == HAL_OK);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi) {
  if (auto spi = tutrcos::peripheral::SPI::get_instances().get(hspi)) {
    spi->sem_.release();
  }
}

#endif
