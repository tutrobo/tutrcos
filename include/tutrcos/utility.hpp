#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <type_traits>
#include <vector>

#include "cobs.h"

#ifdef NDEBUG
#define TUTRCOS_VERIFY(expr) ((void)(expr))
#else
#define TUTRCOS_VERIFY(expr) assert((expr))
#endif

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

inline bool cobs_encode(const std::vector<uint8_t> &src,
                        std::vector<uint8_t> &dest) {
  dest.resize(COBS_ENCODE_DST_BUF_LEN_MAX(src.size()));
  cobs_encode_result res =
      ::cobs_encode(dest.data(), dest.size(), src.data(), src.size());
  if (res.status != COBS_ENCODE_OK) {
    return false;
  }
  dest.resize(res.out_len + 1);
  dest[res.out_len] = 0;
  return true;
}

inline bool cobs_decode(const std::vector<uint8_t> &src,
                        std::vector<uint8_t> &dest) {
  dest.resize(COBS_DECODE_DST_BUF_LEN_MAX(src.size()));
  cobs_decode_result res =
      ::cobs_decode(dest.data(), dest.size(), src.data(), src.size());
  if (res.status != COBS_DECODE_OK &&
      res.status != COBS_DECODE_ZERO_BYTE_IN_INPUT) {
    return false;
  }
  dest.resize(res.out_len);
  return true;
}

} // namespace utility
} // namespace tutrcos
