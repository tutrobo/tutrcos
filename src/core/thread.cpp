#include "tutrcos/core.hpp"

void tutrcos_Thread_func_impl(void *thread) {
  reinterpret_cast<tutrcos::core::Thread *>(thread)->func_();
}
