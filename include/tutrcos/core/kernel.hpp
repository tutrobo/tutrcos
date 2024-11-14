#pragma once

#include <cstdint>

#include <cmsis_os2.h>

namespace tutrcos {
namespace core {

class Kernel {
public:
  static constexpr uint32_t MAX_DELAY = osWaitForever;

  static inline uint32_t get_ticks() { return osKernelGetTickCount(); }
};

} // namespace core
} // namespace tutrcos
