#include "hal/time.h"

#include <Arduino.h>

namespace micromouse::hal {

uint32_t nowUs() { return ::micros(); }

uint32_t nowMs() { return ::millis(); }

void delayMs(const uint32_t ms) { ::delay(ms); }

}  // namespace micromouse::hal
