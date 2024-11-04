#pragma once

#include "main.h"

#include <cstdint>
#include <functional>
#include <map>

namespace tutrcos {
namespace peripheral {

/**
 * `GPIO::set_exti_callback` を使う場合は System Core -> GPIO -> NVIC -> EXTI
 * line[x:x] interrupts を有効化してください。
 *
 * @code{.cpp}
 * #include <tutrcos.hpp>
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *
 *   GPIO led(LD2_GPIO_Port, LD2_Pin);
 *   GPIO button(B1_GPIO_Port, B1_Pin);
 *
 *   button.set_exti_callback([&] {
 *     // GPIO EXTI 割り込み
 *   });
 *
 *   while (true) {
 *     led.write(!button.read()); // ボタンを押している間LED点灯
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
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
