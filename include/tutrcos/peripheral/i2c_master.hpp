#pragma once

#include "main.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>

#include "tutrcos/core.hpp"

namespace tutrcos {
namespace peripheral {

class I2CMaster {
public:
  I2CMaster(I2C_HandleTypeDef *hi2c) : hi2c_{hi2c} {
    get_instances()[hi2c_] = this;
  }

  bool transmit(uint16_t address, uint8_t *data, size_t size) {
    std::lock_guard lock{mtx_};
    if (HAL_I2C_Master_Transmit_IT(hi2c_, address << 1, data, size) != HAL_OK) {
      return false;
    }
    bool res;
    if (!res_.pop(res, core::Kernel::MAX_DELAY)) {
      return false;
    }
    return res;
  }

  bool receive(uint16_t address, uint8_t *data, size_t size, uint32_t timeout) {
    std::lock_guard lock{mtx_};
    if (HAL_I2C_Master_Receive_IT(hi2c_, address << 1, data, size) != HAL_OK) {
      return false;
    }
    bool res;
    if (!res_.pop(res, timeout)) {
      return false;
    }
    return res;
  }

private:
  I2C_HandleTypeDef *hi2c_;
  core::Mutex mtx_;
  core::Queue<bool> res_{1};

  static inline std::map<I2C_HandleTypeDef *, I2CMaster *> &get_instances() {
    static std::map<I2C_HandleTypeDef *, I2CMaster *> instances;
    return instances;
  }

  friend void ::HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
  friend void ::HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
  friend void ::HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
};

} // namespace peripheral
} // namespace tutrcos
