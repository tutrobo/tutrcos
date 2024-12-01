#include "main.h"

#ifdef HAL_GPIO_MODULE_ENABLED

#include "tutrcos/peripheral/gpio.hpp"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (auto gpio = tutrcos::peripheral::GPIO::get_instances().get(GPIO_Pin)) {
    if (gpio->callback_) {
      gpio->callback_();
    }
  }
}

#endif
