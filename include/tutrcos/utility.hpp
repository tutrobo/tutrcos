#pragma once

#include <type_traits>

namespace tutrcos {
namespace utility {

template <class T>
constexpr std::underlying_type_t<T> to_underlying(T value) noexcept {
  return static_cast<std::underlying_type_t<T>>(value);
}

} // namespace utility
} // namespace tutrcos
