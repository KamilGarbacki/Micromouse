// Wall detection and ranging with the three ultrasonic sensors (robot
// stopped).
#pragma once

#include <stdint.h>

#include "drivers/ultrasonic_sensor.h"
#include "navigation/pose.h"

namespace micromouse {

/// Value used for "no echo within range" so that it sorts as far away.
constexpr uint16_t kFarUs = 0xFFFF;

/// One mapping reading per sensor.
struct WallReading {
  Relative side;      ///< sensor direction relative to the heading
  uint16_t medianUs;  ///< median round-trip echo time; kFarUs if no echo
  bool wall;          ///< closer than kWallThresholdCm
  bool anyEcho;       ///< at least one of the samples got an echo
};

/// Distance [mm] of a round-trip echo time with the configured speed of
/// sound (config::kSonarUsPerCm); 0 for no echo (0 or kFarUs).
float echoToMm(uint16_t echoUs);

/// Pings every sensor kSonarSamples times, round-robin with
/// kInterPingGapMs of silence between any two pings (no crosstalk), and
/// classifies the median echo time of each sensor. `sensors` and `sides`
/// have kSonarCount entries. Blocks for about
/// kSonarCount * kSonarSamples * (kInterPingGapMs + ~3 ms).
void senseWalls(const UltrasonicSensor* const sensors[kSonarCount],
                const Relative sides[kSonarCount],
                WallReading out[kSonarCount]);

/// Precise ranging for alignment and learning: the sensors whose bit is set
/// in `mask` (bit i = sensors[i]) are pinged kRangingSamples times,
/// round-robin with kInterPingGapMs gaps; out[i] is the median distance
/// [mm], 0 for no echo or not pinged.
void rangeSonars(const UltrasonicSensor* const sensors[kSonarCount],
                 uint8_t mask, float out[kSonarCount]);

}  // namespace micromouse
