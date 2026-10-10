// Single-channel wheel encoder (direction blind: counts only increase).
#pragma once

#include <stdint.h>

#include "hal/encoders.h"

namespace micromouse {

class Encoder {
 public:
  explicit constexpr Encoder(const hal::EncoderChannel channel)
      : channel_(channel) {}

  /// Enables the input and the edge interrupt.
  void begin() const { hal::encoderBegin(channel_); }

  /// Edges since the last reset (atomic read).
  int32_t count() const { return hal::encoderCount(channel_); }

  void reset() const { hal::encoderReset(channel_); }

 private:
  hal::EncoderChannel channel_;
};

}  // namespace micromouse
