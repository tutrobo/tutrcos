#include "tutrcos/core.hpp"

void tutrcos_core_Thread_func(void *thread) {
  reinterpret_cast<tutrcos::core::Thread *>(thread)->func_();
}
