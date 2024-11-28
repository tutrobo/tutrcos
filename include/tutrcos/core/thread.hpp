#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <type_traits>

#include "cmsis_os2.h"

namespace tutrcos {
namespace core {

class Thread {
private:
  struct Deleter {
    void operator()(osThreadId_t thread_id) { osThreadTerminate(thread_id); }
  };

  using ThreadIdUniquePtr =
      std::unique_ptr<std::remove_pointer_t<osThreadId_t>, Deleter>;

public:
  using ThreadId = osThreadId_t;

  Thread(std::function<void()> &&func) : func_{std::move(func)} {
    osThreadAttr_t attr = {};
    attr.stack_size = STACK_SIZE;
    attr.priority = PRIORITY;
    thread_id_ = ThreadIdUniquePtr{osThreadNew(func_internal, this, &attr)};
  }

  static inline void yield() { osThreadYield(); }

  static inline void delay(uint32_t ticks) { osDelay(ticks); }

  static inline void delay_until(uint32_t ticks) { osDelayUntil(ticks); }

  static inline ThreadId get_id() { return osThreadGetId(); }

  static inline void notify(ThreadId id) { osThreadFlagsSet(id, 1); }

  static inline bool wait(uint32_t timeout) {
    return (osThreadFlagsWait(1, osFlagsWaitAny, timeout) & osFlagsError) ==
           osFlagsError;
  }

private:
  static constexpr uint32_t STACK_SIZE = 4096;
  static constexpr osPriority_t PRIORITY = osPriorityNormal;

  ThreadIdUniquePtr thread_id_;
  std::function<void()> func_;

  static inline void func_internal(void *thread) {
    reinterpret_cast<tutrcos::core::Thread *>(thread)->func_();
  }
};

} // namespace core
} // namespace tutrcos
