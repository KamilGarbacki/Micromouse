// Monotonic time and blocking delays (wraps the Arduino core).
#pragma once

#include <stdint.h>

namespace micromouse::hal {

/// Microseconds since boot; wraps every ~71 minutes (use unsigned deltas).
uint32_t nowUs();

/// Milliseconds since boot; wraps every ~49 days (use unsigned deltas).
uint32_t nowMs();

/// Blocks for `ms` milliseconds.
void delayMs(uint32_t ms);

}  // namespace micromouse::hal
