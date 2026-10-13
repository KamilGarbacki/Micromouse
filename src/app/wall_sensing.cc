#include "app/wall_sensing.h"

#include "config/config.h"
#include "hal/time.h"
#include "util/median.h"

namespace micromouse {

namespace {

constexpr float kWallThresholdUs =
    static_cast<float>(config::kWallThresholdCm) * config::kSonarUsPerCm;

/// One ping as an echo time; NO_ECHO is sorted as "far", so a single
/// dropout cannot drag the median to 0.
uint16_t pingSample(const UltrasonicSensor& sensor) {
  const uint16_t us = sensor.pingUs(config::kMappingMaxCm);
  return us == UltrasonicSensor::kNoEcho ? kFarUs : us;
}

}  // namespace

float echoToMm(const uint16_t echoUs) {
  if (echoUs == 0 || echoUs == kFarUs) return 0.0f;
  return static_cast<float>(echoUs) * 10.0f / config::kSonarUsPerCm;
}

void senseWalls(const UltrasonicSensor* const sensors[kSonarCount],
                const Relative sides[kSonarCount],
                WallReading out[kSonarCount]) {
  uint16_t samples[kSonarCount][config::kSonarSamples] = {};

  for (uint8_t round = 0; round < config::kSonarSamples; ++round) {
    for (uint8_t s = 0; s < kSonarCount; ++s) {
      samples[s][round] = pingSample(*sensors[s]);
      hal::delayMs(config::kInterPingGapMs);
    }
  }

  for (uint8_t s = 0; s < kSonarCount; ++s) {
    const uint16_t median =
        util::medianInPlace(samples[s], config::kSonarSamples);
    // Sorted ascending: the smallest sample is "far" only if all are.
    const bool anyEcho = samples[s][0] != kFarUs;
    out[s] = WallReading{sides[s], median,
                         static_cast<float>(median) < kWallThresholdUs,
                         anyEcho};
  }
}

void rangeSonars(const UltrasonicSensor* const sensors[kSonarCount],
                 const uint8_t mask, float out[kSonarCount]) {
  uint16_t samples[kSonarCount][config::kRangingSamples] = {};
  for (uint8_t round = 0; round < config::kRangingSamples; ++round) {
    for (uint8_t s = 0; s < kSonarCount; ++s) {
      if ((mask & (1u << s)) == 0) continue;
      samples[s][round] = pingSample(*sensors[s]);
      hal::delayMs(config::kInterPingGapMs);
    }
  }
  for (uint8_t s = 0; s < kSonarCount; ++s) {
    out[s] = (mask & (1u << s)) == 0
                 ? 0.0f
                 : echoToMm(util::medianInPlace(samples[s],
                                                config::kRangingSamples));
  }
}

}  // namespace micromouse
