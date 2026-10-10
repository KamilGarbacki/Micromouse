#include "app/wall_sensing.h"

#include "config/config.h"
#include "hal/time.h"
#include "util/median.h"

namespace micromouse {

void senseWalls(const UltrasonicSensor* const sensors[kSonarCount],
                const Relative sides[kSonarCount],
                WallReading out[kSonarCount]) {
  uint16_t samples[kSonarCount][config::kSonarSamples] = {};

  for (uint8_t round = 0; round < config::kSonarSamples; ++round) {
    for (uint8_t s = 0; s < kSonarCount; ++s) {
      const uint16_t cm = sensors[s]->pingCm(config::kMappingMaxCm);
      // NO_ECHO means nothing within range: sort it as "far", so a single
      // dropout cannot drag the median to 0.
      samples[s][round] = cm == UltrasonicSensor::kNoEcho ? kFarCm : cm;
      hal::delayMs(config::kInterPingGapMs);
    }
  }

  for (uint8_t s = 0; s < kSonarCount; ++s) {
    const uint16_t median =
        util::medianInPlace(samples[s], config::kSonarSamples);
    // Sorted ascending: the smallest sample is "far" only if all are.
    const bool anyEcho = samples[s][0] != kFarCm;
    out[s] = WallReading{sides[s], median,
                         median < config::kWallThresholdCm, anyEcho};
  }
}

}  // namespace micromouse
