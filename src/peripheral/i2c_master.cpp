#include "main.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include "tutrcos/peripheral/i2c_master.hpp"

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  auto itr = tutrcos::peripheral::I2CMaster::get_instances().find(hi2c);
  if (itr != tutrcos::peripheral::I2CMaster::get_instances().end()) {
    auto i2c_master = itr->second;
    i2c_master->tx_sem_.release();
  }
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  auto itr = tutrcos::peripheral::I2CMaster::get_instances().find(hi2c);
  if (itr != tutrcos::peripheral::I2CMaster::get_instances().end()) {
    auto i2c_master = itr->second;
    i2c_master->rx_sem_.release();
  }
}

#endif
