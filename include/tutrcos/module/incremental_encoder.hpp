#pragma once

#include <cstdint>

#include "tutrcos/peripheral/tim.hpp"
#include "tutrcos/utility.hpp"

#include "encoder_base.hpp"

namespace tutrcos {
namespace module {

class IncrementalEncoder : public EncoderBase {
public:
  IncrementalEncoder(peripheral::TIM &tim, int16_t ppr)
      : EncoderBase{ppr * 4}, tim_{tim} {
    TUTRCOS_VERIFY(tim_.start_encoder(TIM_CHANNEL_ALL));
  }

  ~IncrementalEncoder() { TUTRCOS_VERIFY(tim_.stop_encoder(TIM_CHANNEL_ALL)); }

  bool update() override {
    int16_t delta = tim_.get_counter();
    tim_.set_counter(0);
    count_ += delta;
    set_count(count_);
    return true;
  }

private:
  peripheral::TIM &tim_;
  int64_t count_ = 0;
};

} // namespace module
} // namespace tutrcos
