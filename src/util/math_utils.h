// Small numeric helpers. Named to avoid Arduino's min/max/abs/constrain
// macros (avr-gcc has no <algorithm>).
#pragma once

namespace micromouse::util {

template <typename T>
constexpr T clamp(const T value, const T lo, const T hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

template <typename T>
constexpr T absValue(const T value) {
  return value < 0 ? -value : value;
}

template <typename T>
constexpr T minOf(const T a, const T b) {
  return b < a ? b : a;
}

}  // namespace micromouse::util
