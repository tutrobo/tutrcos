#pragma once

#include "main.h"

#include <cstdio>
#include <type_traits>

#define TUTRCOS_ASSERT(expr, ...)                                              \
  if (!(expr)) {                                                               \
    printf(__FILE__ ":%d: assertion failed\r\n", __LINE__);                    \
    Error_Handler();                                                           \
  }

namespace tutrcos {
namespace utility {

template <class T>
constexpr std::underlying_type_t<T> to_underlying(T value) noexcept {
  return static_cast<std::underlying_type_t<T>>(value);
}

} // namespace utility
} // namespace tutrcos
