// Encoder health check of the move loop (pure).
//
// Both wheels always turn in a move (the wheel floor keeps each above its
// breakaway PWM), and the heading loop keeps their counts close. A wheel
// whose encoder stays (nearly) silent while the other one counts is a dead,
// unplugged or misaligned encoder: the AVERAGE distance is then half the
// truth and the robot would drive 2 cells per move.
#pragma once

#include <stdint.h>

namespace micromouse {

/// True if the encoder pair looks broken: once the larger of the two
/// (direction-blind) counts exceeds `minTravelTicks`, the smaller one is
/// below `minRatio` of it. Never true before that much travel, so the
/// breakaway of one wheel slightly before the other is not a fault.
constexpr bool encoderPairFaulty(const int32_t left, const int32_t right,
                                 const int32_t minTravelTicks,
                                 const float minRatio) {
  const int32_t high = left > right ? left : right;
  const int32_t low = left > right ? right : left;
  return high > minTravelTicks &&
         static_cast<float>(low) < minRatio * static_cast<float>(high);
}

}  // namespace micromouse
