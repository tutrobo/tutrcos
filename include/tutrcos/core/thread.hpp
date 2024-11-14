#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <type_traits>

#include <cmsis_os2.h>

extern "C" void tutrcos_Thread_func_impl(void *thread);

namespace tutrcos {
namespace core {

class Thread {
private:
  struct Deleter {
    void operator()(osThreadId_t thread_id) { osThreadTerminate(thread_id); }
  };

  using ThreadId =
      std::unique_ptr<std::remove_pointer_t<osThreadId_t>, Deleter>;

public:
  Thread(std::function<void()> &&func) : func_{std::move(func)} {
    osThreadAttr_t attr = {};
    attr.stack_size = STACK_SIZE;
    attr.priority = PRIORITY;
    thread_id_ = ThreadId{osThreadNew(tutrcos_Thread_func_impl, this, &attr)};
  }

  static inline void yield() { osThreadYield(); }

  static inline void delay(uint32_t ticks) { osDelay(ticks); }

  static inline void delay_until(uint32_t ticks) { osDelayUntil(ticks); }

private:
  static constexpr uint32_t STACK_SIZE = 4096;
  static constexpr osPriority_t PRIORITY = osPriorityNormal;

  ThreadId thread_id_;
  std::function<void()> func_;

  friend void ::tutrcos_Thread_func_impl(void *thread);
};

} // namespace core
} // namespace tutrcos
