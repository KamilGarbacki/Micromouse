// Constant tables in program memory (header-only, no hardware access). On
// the AVR a constant aggregate is otherwise copied into RAM (.data) at
// start-up; MM_FLASH puts it into flash and loadFlash() copies it onto the
// stack where it is needed. On the host (unit tests) both are plain memory,
// so pure modules can use them.
#pragma once

#include <string.h>

#if defined(__AVR__)
#include <avr/pgmspace.h>
#define MM_FLASH PROGMEM
#else
#define MM_FLASH
#endif

namespace micromouse::util {

/// Copy of a constant object declared with MM_FLASH.
template <typename T>
T loadFlash(const T& object) {
  T copy;
#if defined(__AVR__)
  memcpy_P(&copy, &object, sizeof(T));
#else
  memcpy(&copy, &object, sizeof(T));
#endif
  return copy;
}

}  // namespace micromouse::util
