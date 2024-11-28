#pragma once

#include <cstdint>
#include <memory>
#include <type_traits>

#include "cmsis_os2.h"

namespace tutrcos {
namespace core {

class Semaphore {
private:
  struct Deleter {
    void operator()(osSemaphoreId_t queue_id) { osSemaphoreDelete(queue_id); }
  };

  using SemaphoreIdUniquePtr =
      std::unique_ptr<std::remove_pointer_t<osSemaphoreId_t>, Deleter>;

public:
  Semaphore(uint32_t max, uint32_t initial) {
    semaphore_id_ = SemaphoreIdUniquePtr{osSemaphoreNew(max, initial, nullptr)};
  }

  bool try_acquire(uint32_t timeout) {
    return osSemaphoreAcquire(semaphore_id_.get(), timeout) == osOK;
  }

  void acquire() { try_acquire(osWaitForever); }

  void release() { osSemaphoreRelease(semaphore_id_.get()); }

private:
  SemaphoreIdUniquePtr semaphore_id_;
};

} // namespace core
} // namespace tutrcos
