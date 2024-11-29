#pragma once

#include "main.h"

#include <string>
#include <type_traits>

namespace tutrcos {
namespace utility {

template <class T>
constexpr std::underlying_type_t<T> to_underlying(T value) noexcept {
  return static_cast<std::underlying_type_t<T>>(value);
}

template <class... Args> std::string format(const char *fmt, Args... args) {
  size_t size = std::snprintf(nullptr, 0, fmt, args...);
  std::string buf(size, '\0');
  std::snprintf(buf.data(), size + 1, fmt, args...);
  return buf;
}

} // namespace utility
} // namespace tutrcos
