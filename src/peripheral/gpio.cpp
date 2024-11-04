#include "main.h"

#ifdef HAL_GPIO_MODULE_ENABLED

#include "tutrcos/peripheral/gpio.hpp"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  auto itr = tutrcos::peripheral::GPIO::get_instances().find(GPIO_Pin);
  if (itr != tutrcos::peripheral::GPIO::get_instances().end()) {
    auto gpio = itr->second;
    if (gpio->exti_callback_) {
      gpio->exti_callback_();
    }
  }
}

#endif
