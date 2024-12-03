#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED

#include "tutrcos/peripheral/spi.hpp"

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  TUTRCOS_VERIFY(HAL_SPI_Abort_IT(hspi) == HAL_OK);
}

#endif
