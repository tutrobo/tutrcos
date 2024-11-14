#include "main.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include "tutrcos/peripheral/i2c_master.hpp"

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_Master_Abort_IT(hi2c);
}

#endif
