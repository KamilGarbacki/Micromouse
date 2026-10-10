// Wall detection with the three ultrasonic sensors (robot stopped).
#pragma once

#include <stdint.h>

#include "drivers/ultrasonic_sensor.h"
#include "navigation/pose.h"

namespace micromouse {

/// One mapping reading per sensor.
struct WallReading {
  Relative side;        ///< sensor direction relative to the heading
  uint16_t medianCm;    ///< median distance; kFarCm if no echo
  bool wall;            ///< medianCm < kWallThresholdCm
  bool anyEcho;         ///< at least one of the samples got an echo
};

/// Pings every sensor kSonarSamples times, round-robin with
/// kInterPingGapMs of silence between any two pings (no crosstalk), and
/// classifies the median of each sensor. `sensors` and `sides` have
/// kSonarCount entries. Blocks for about
/// kSonarCount * kSonarSamples * (kInterPingGapMs + ~3 ms).
void senseWalls(const UltrasonicSensor* const sensors[kSonarCount],
                const Relative sides[kSonarCount],
                WallReading out[kSonarCount]);

/// Value used for "no echo within range" so that it sorts as far away.
constexpr uint16_t kFarCm = 0xFFFF;

}  // namespace micromouse
