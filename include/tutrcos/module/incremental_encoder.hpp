#pragma once

#include <cassert>
#include <cstdint>

#include "tutrcos/peripheral/tim.hpp"

#include "encoder_base.hpp"

namespace tutrcos {
namespace module {

class IncrementalEncoder : public EncoderBase {
public:
  IncrementalEncoder(peripheral::TIM &tim, int16_t ppr, float dt)
      : EncoderBase{ppr * 4}, tim_{tim} {
    assert(tim_.start_encoder(TIM_CHANNEL_ALL));
  }

  ~IncrementalEncoder() { assert(tim_.stop_encoder(TIM_CHANNEL_ALL)); }

  void update() {
    int16_t delta = tim_.get_counter();
    tim_.set_counter(0);
    count_ += delta;
    set_count(count_);
  }

private:
  peripheral::TIM &tim_;
  int64_t count_ = 0;
};

} // namespace module
} // namespace tutrcos
