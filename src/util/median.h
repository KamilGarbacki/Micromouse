// Median of a small sample set.
#pragma once

#include <stdint.h>

namespace micromouse::util {

/// Sorts `values[0..count)` in place (insertion sort, fine for a handful of
/// samples) and returns the median; for an even count the lower middle.
/// Returns T{} if `count` is 0.
template <typename T>
T medianInPlace(T* values, const uint8_t count) {
  if (count == 0) return T{};
  for (uint8_t i = 1; i < count; ++i) {
    const T key = values[i];
    uint8_t j = i;
    while (j > 0 && key < values[j - 1]) {
      values[j] = values[j - 1];
      --j;
    }
    values[j] = key;
  }
  return values[(count - 1) / 2];
}

}  // namespace micromouse::util
