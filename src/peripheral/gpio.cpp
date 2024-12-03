#include "main.h"

#ifdef HAL_GPIO_MODULE_ENABLED

#include "tutrcos/peripheral/gpio.hpp"

using tutrcos::peripheral::GPIO;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  for (GPIO *gpio : GPIO::get_instances()) {
    if (gpio->pin_ == GPIO_Pin && gpio->callback_) {
      gpio->callback_();
      break;
    }
  }
}

#endif
