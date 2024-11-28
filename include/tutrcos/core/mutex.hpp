#pragma once

#include <cstdint>
#include <memory>
#include <type_traits>

#include "cmsis_os2.h"

namespace tutrcos {
namespace core {

class Mutex {
private:
  struct Deleter {
    void operator()(osMutexId_t mutex_id) { osMutexDelete(mutex_id); }
  };

  using MutexId = std::unique_ptr<std::remove_pointer_t<osMutexId_t>, Deleter>;

public:
  Mutex() {
    osMutexAttr_t attr = {};
    attr.attr_bits = osMutexPrioInherit;
    mutex_id_ = MutexId{osMutexNew(&attr)};
  }

  bool try_lock(uint32_t timeout) {
    return osMutexAcquire(mutex_id_.get(), timeout) == osOK;
  }

  void lock() { try_lock(osWaitForever); }

  void unlock() { osMutexRelease(mutex_id_.get()); }

private:
  MutexId mutex_id_;
};

} // namespace core
} // namespace tutrcos
