#pragma once

#include "main.h"

#include <cstdint>
#include <functional>
#include <map>

namespace tutrcos {
namespace peripheral {

class GPIO {
public:
  GPIO(GPIO_TypeDef *port, uint16_t pin) : port_{port}, pin_{pin} {
    get_instances()[pin_] = this;
  }

  void write(bool state) {
    HAL_GPIO_WritePin(port_, pin_, static_cast<GPIO_PinState>(state));
  }

  bool read() { return HAL_GPIO_ReadPin(port_, pin_); }

  void toggle() { HAL_GPIO_TogglePin(port_, pin_); }

  void set_exti_callback(std::function<void()> &&callback) {
    exti_callback_ = std::move(callback);
  }

private:
  GPIO_TypeDef *port_;
  uint16_t pin_;
  std::function<void()> exti_callback_;

  static inline std::map<uint16_t, GPIO *> &get_instances() {
    static std::map<uint16_t, GPIO *> instances;
    return instances;
  }

  friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
};

} // namespace peripheral
} // namespace tutrcos
