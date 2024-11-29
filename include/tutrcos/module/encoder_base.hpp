#pragma once

#include <algorithm>
#include <cstdint>

namespace tutrcos {
namespace module {

class EncoderBase {
public:
  EncoderBase(int64_t cpr) : cpr_{cpr} {}
  virtual ~EncoderBase() {}
  virtual int64_t get_count() { return count_; }
  virtual float get_rotation() { return static_cast<float>(count_) / cpr_; }
  virtual float get_rps() { return delta_ / dt_ / cpr_; }
  virtual float get_rpm() { return get_rps() * 60; }

protected:
  void set_count(int64_t count) {
    delta_ = count - count_;
    count_ = count;
    uint32_t tick = core::Kernel::get_ticks();
    dt_ = std::min<uint32_t>(tick - prev_tick_, 1) / 1000.0f;
    prev_tick_ = tick;
  }

private:
  int64_t cpr_;
  uint32_t prev_tick_;
  float dt_;
  int64_t count_ = 0;
  int64_t delta_ = 0;
};

} // namespace module
} // namespace tutrcos
