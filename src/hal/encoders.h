// Wheel encoder edge counters, incremented from external interrupts.
#pragma once

#include <stdint.h>

namespace micromouse::hal {

enum class EncoderChannel : uint8_t { kLeft = 0, kRight = 1 };

/// Enables the pull-up on the channel's pin (config/pins.h) and attaches
/// the RISING-edge interrupt. The count starts at 0.
void encoderBegin(EncoderChannel channel);

/// Edge count since the last reset, read atomically (the 32-bit counter
/// is updated from an ISR on an 8-bit CPU).
int32_t encoderCount(EncoderChannel channel);

/// Resets the count to 0, atomically.
void encoderReset(EncoderChannel channel);

}  // namespace micromouse::hal
