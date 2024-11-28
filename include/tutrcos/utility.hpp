#pragma once

#include "main.h"

#include <cstdio>
#include <type_traits>

#define STRINGIFY(n) #n
#define TOSTRING(n) STRINGIFY(n)

#define TUTRCOS_ASSERT(expr, ...)                                              \
  if (!(expr)) {                                                               \
    puts(__FILE__ ":" TOSTRING(__LINE__) ": assertion failed");                \
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
