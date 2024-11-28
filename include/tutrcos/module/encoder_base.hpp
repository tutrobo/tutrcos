#pragma once

#include <cassert>
#include <cstdint>

namespace tutrcos {
namespace module {

class EncoderBase {
public:
  EncoderBase(int64_t cpr, float dt) : cpr_{cpr}, dt_{dt} {}

  int64_t get_count() { return count_; }

  float get_rotation() { return static_cast<float>(count_) / cpr_; }

  float get_rps() { return delta_ / dt_ / cpr_; }

  float get_rpm() { return get_rps() * 60; }

protected:
  void set_count(int64_t count) {
    delta_ = count - count_;
    count_ = count;
  }

private:
  int64_t cpr_;
  float dt_;
  int64_t count_ = 0;
  int64_t delta_ = 0;
};

} // namespace module
} // namespace tutrcos
