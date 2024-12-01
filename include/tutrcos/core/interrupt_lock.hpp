#pragma once

#include "cmsis_gcc.h"

namespace tutrcos {
namespace core {

class InterruptLock {
public:
  void lock() const { __disable_irq(); }
  void unlock() const { __enable_irq(); }
};

inline constexpr InterruptLock interrupt_lock;

} // namespace core
} // namespace tutrcos
