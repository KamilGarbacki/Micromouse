// Umbrella header for all configuration, plus compile-time sanity checks.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config/features.h"
#include "config/geometry.h"
#include "config/maze_config.h"
#include "config/pins.h"
#include "config/tuning.h"

namespace micromouse::config::detail {

constexpr bool pinsUnique(const uint8_t* pins, const size_t count) {
  for (size_t i = 0; i < count; ++i) {
    for (size_t j = i + 1; j < count; ++j) {
      if (pins[i] == pins[j]) return false;
    }
  }
  return true;
}

constexpr bool isUnoPwmPin(const uint8_t pin) {
  return pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 ||
         pin == 11;
}

constexpr bool isUnoInterruptPin(const uint8_t pin) {
  return pin == 2 || pin == 3;
}

constexpr bool cellInMaze(const Cell cell) {
  return cell.row < kMazeRows && cell.col < kMazeCols;
}

// Ideal-plant length of the accel ramp (distance) plus the decel ramp from
// cruise down to creep speed, in ticks.
constexpr float rampTicks(const float cruise, const float creep,
                          const float accel, const float decel) {
  return cruise * cruise / (2.0f * accel) +
         (cruise * cruise - creep * creep) / (2.0f * decel);
}

// Blocking time of one NewPing ping [us]. Normal: echo window + typical
// sensor start delay. Worst case (sensor never / very late starting the
// echo): NewPing's MAX_SENSOR_DELAY start timeout + echo window, plus the
// echo window again if the echo starts at the very end.
constexpr uint32_t kSonarStartDelayUs = 500;
constexpr uint32_t kNewPingMaxSensorDelayUs = 5800;  // NewPing.h
constexpr uint32_t echoWindowUs(const uint16_t maxCm) {
  return (maxCm + 1u) * kNewPingUsPerCm;
}
constexpr uint32_t pingBlockingUs(const uint16_t maxCm) {
  return echoWindowUs(maxCm) + kSonarStartDelayUs;
}
constexpr uint32_t pingWorstCaseUs(const uint16_t maxCm) {
  return kNewPingMaxSensorDelayUs + 2u * echoWindowUs(maxCm);
}

}  // namespace micromouse::config::detail

namespace micromouse::config {

// --- Pins ------------------------------------------------------------------
static_assert(detail::pinsUnique(pins::kAllPins,
                                 sizeof(pins::kAllPins) /
                                     sizeof(pins::kAllPins[0])),
              "pin assigned twice");
static_assert(detail::isUnoPwmPin(pins::kLeftMotorPwm) &&
                  detail::isUnoPwmPin(pins::kRightMotorPwm),
              "motor PWM pins must be hardware PWM pins");
static_assert(detail::isUnoInterruptPin(pins::kLeftEncoder) &&
                  detail::isUnoInterruptPin(pins::kRightEncoder),
              "encoders must be on external-interrupt pins D2/D3");

// --- Maze ------------------------------------------------------------------
static_assert(kMazeRows > 0 && kMazeCols > 0, "maze must not be empty");
static_assert(kMazeRows <= kMazeMaxRows && kMazeCols <= kMazeMaxCols,
              "maze exceeds the static storage capacity");
static_assert(static_cast<uint16_t>(kMazeMaxRows) * kMazeMaxCols <= 255,
              "cell index / flood distance are 8 bit: at most 255 cells");
static_assert(detail::cellInMaze(kStartCell), "start cell outside the maze");
static_assert(detail::cellInMaze(kGoalCell), "goal cell outside the maze");

// --- Geometry --------------------------------------------------------------
// (derivation checks: geometry.h)
static_assert(kTicksPerCellMin < kTicksPerCell &&
                  kTicksPerCell < kTicksPerCellMax &&
                  kTicksPer90Min < kTicksPer90 && kTicksPer90 < kTicksPer90Max,
              "defaults must lie inside the learning bounds");

// --- Control ---------------------------------------------------------------
static_assert(kControlPeriodUs > 0, "control period must be positive");
static_assert(kSpeedFilter > 0.0f && kSpeedFilter <= 1.0f,
              "speed filter coefficient must be in (0, 1]");
static_assert(kHeadingCarryLimit >= 0, "carry limit must not be negative");
static_assert(kHeadingCarryRampPeriods > 0, "ramp needs at least 1 period");
static_assert(kRollingExitTpp >= 0.0f && kRollingExitTpp < kRollingEnterTpp,
              "stiction hysteresis: exit threshold must be below enter");
static_assert(kWheelFloorFraction >= 0.0f && kWheelFloorFraction <= 1.0f,
              "wheel floor is a fraction of kineticPwm");
// Tick quantisation: both creep speeds (an alignment move runs entirely at
// creep) must lie clearly above the rolling threshold, which is >= 1 tick
// per period. If a geometry / resolution change trips this, raise the
// creep speed or lengthen the speed measurement.
static_assert(forward::kCreepSpeed >= 1.2f * kRollingEnterTpp &&
                  turn::kCreepSpeed >= 1.2f * kRollingEnterTpp,
              "creep speed too close to the rolling threshold (tpp)");
static_assert(kEncoderFaultMinRatio > 0.0f && kEncoderFaultMinRatio < 1.0f,
              "encoder fault ratio must be in (0, 1)");
// The check must be armed in every cell move and every spin, and the
// healthy wheel's threshold must be resolvable (>= ~10 ticks).
static_assert(kEncoderFaultMinTravelTicks * kEncoderFaultMinRatio >= 10.0f &&
                  kEncoderFaultMinTravelTicks < kTicksPer90Min / 2,
              "encoder fault travel threshold out of range");
static_assert(kTurnLearnMaxCarryTicks <= kHeadingCarryLimit,
              "turn-learning carry gate must lie inside the carry limit");

namespace detail {
constexpr bool profileValid(const float cruise, const float creep,
                            const float accel, const float decel,
                            const int32_t creepTicks, const int32_t brakeLead,
                            const int32_t maxBrakeLead) {
  return cruise > 0.0f && creep > 0.0f && creep <= cruise && accel > 0.0f &&
         decel > 0.0f && creepTicks >= 0 && brakeLead >= 0 &&
         brakeLead <= maxBrakeLead;
}
constexpr bool motorDefaultsValid(const float staticPwm,
                                  const float kineticPwm,
                                  const float ffOffset, const float slope) {
  return kineticPwm >= 0.0f && kineticPwm <= staticPwm &&
         staticPwm <= static_cast<float>(kMaxPwm) &&
         ffOffset >= kMinFfOffsetPwm &&
         ffOffset <= static_cast<float>(kMaxPwm) && slope >= kMinFfSlope &&
         slope <= kMaxFfSlope;
}
}  // namespace detail

static_assert(detail::profileValid(forward::kCruiseSpeed,
                                   forward::kCreepSpeed, forward::kAccel,
                                   forward::kDecel, forward::kCreepTicks,
                                   forward::kBrakeLeadTicks,
                                   forward::kMaxBrakeLeadTicks),
              "invalid forward profile");
static_assert(detail::profileValid(turn::kCruiseSpeed, turn::kCreepSpeed,
                                   turn::kAccel, turn::kDecel,
                                   turn::kCreepTicks, turn::kBrakeLeadTicks,
                                   turn::kMaxBrakeLeadTicks),
              "invalid turn profile");
static_assert(detail::motorDefaultsValid(
                  forward::kDefaultStaticPwm, forward::kDefaultKineticPwm,
                  forward::kDefaultFfOffsetPwm, forward::kDefaultFfSlope),
              "invalid forward motor defaults");
static_assert(detail::motorDefaultsValid(
                  turn::kDefaultStaticPwm, turn::kDefaultKineticPwm,
                  turn::kDefaultFfOffsetPwm, turn::kDefaultFfSlope),
              "invalid turn motor defaults");
static_assert(kForwardLoadScale > 0.0f && kTurnLoadScale > 0.0f,
              "load scale must be positive");

// The profile checks use the WORST case of the run-time values: the
// smallest learnable tick counts and the largest learnable brake leads.
static_assert(forward::kCreepTicks + forward::kMaxBrakeLeadTicks <
                  kTicksPerCellMin,
              "creep + brake lead must fit in the shortest learnable cell");
// A turn target is clamped to >= half the nominal 90 deg (see
// heading::turnTarget), so the creep phase must fit in that.
static_assert(turn::kCreepTicks + turn::kMaxBrakeLeadTicks <
                  kTicksPer90Min / 2,
              "creep + brake lead must fit in the shortest turn");
static_assert(detail::rampTicks(turn::kCruiseSpeed, turn::kCreepSpeed,
                                turn::kAccel, turn::kDecel) +
                      turn::kCreepTicks + turn::kMaxBrakeLeadTicks <
                  kTicksPer90Min / 2,
              "the shortest turn is too short to reach cruise speed");
static_assert(detail::rampTicks(forward::kCruiseSpeed, forward::kCreepSpeed,
                                forward::kAccel, forward::kDecel) +
                      forward::kCreepTicks + forward::kMaxBrakeLeadTicks <
                  kTicksPerCellMin,
              "a cell is too short to reach cruise speed; lower it");

// --- Front-wall alignment ----------------------------------------------------
static_assert(frontWallNominalMm(kMinCellSizeMm) > 20.0f,
              "front sensor too close to the wall when centred (< 2 cm, "
              "below the ultrasonic minimum range)");
static_assert(kAlignDeadbandMm >= 0.0f &&
                  kAlignMaxCorrectionMm > kAlignDeadbandMm &&
                  kAlignMaxErrorMm >= kAlignMaxCorrectionMm,
              "alignment bounds inconsistent");
static_assert(frontWallNominalMm(kMaxCellSizeMm) + kAlignMaxErrorMm <
                  kWallThresholdCm * 10.0f,
              "alignment window must lie inside the wall threshold");
static_assert(kAlignMaxIterations >= 1, "align at least once");

// --- Online learning ---------------------------------------------------------
static_assert(kScaleLearnGain > 0.0f && kScaleLearnGain <= 1.0f &&
                  kPitchLearnGain > 0.0f && kPitchLearnGain <= 1.0f &&
                  kTurnLearnGain > 0.0f && kTurnLearnGain <= 1.0f &&
                  kCurveLearnGain > 0.0f && kCurveLearnGain <= 1.0f &&
                  kBrakeLeadGain > 0.0f && kBrakeLeadGain <= 1.0f,
              "learning gains must be in (0, 1]");
static_assert(kScaleLearnOutlierFraction > 0.0f &&
                  kPitchLearnOutlierFraction > 0.0f &&
                  kTurnLearnOutlierFraction > 0.0f,
              "outlier gate must be positive");
static_assert(kMaxWallOffsetMm > 0.0f && kPitchJointForget > 0.0f &&
                  kPitchJointForget <= 1.0f && kPitchMinDecorrelation > 0.0f &&
                  kPitchMinDecorrelation < 1.0f &&
                  kPitchProvisionalCells >= 1 &&
                  kPitchMinRows >= 2 && kPitchMinRows <= 7,
              "joint pitch / offset estimator parameters");
static_assert(kLearnFrontMaxMm <= kMappingMaxCm * 10.0f,
              "front ruler range must lie inside the mapping range");
static_assert(kLearnSaveEveryUpdates >= 1, "save period >= 1");

// --- Self-calibration / menu -------------------------------------------------
static_assert(kSelfCalLevels >= 3 && kSelfCalLevels <= 8,
              "use 3..8 feed-forward levels");
static_assert(kSelfCalTopPwm <= kMaxPwm && kSelfCalMaxStartPwm < kSelfCalTopPwm,
              "self-calibration PWM range");
static_assert(kSelfCalMovingTicks >= 1 && kSelfCalMovingSteps >= 1 &&
                  kSelfCalStartPwm >= 1,
              "self-calibration moving threshold");
static_assert(kRangingSamples >= 1 && kRangingSamples <= 7 &&
                  (kRangingSamples % 2) == 1,
              "use an odd number of 1..7 pings for ranging");
static_assert(kAirTemperatureC > -20.0f && kAirTemperatureC < 50.0f,
              "air temperature out of range");

// --- Sensing ---------------------------------------------------------------
static_assert(kSonarSamples >= 1 && kSonarSamples <= 7 &&
                  (kSonarSamples % 2) == 1,
              "use an odd number of 1..7 pings for the median");
static_assert(kWallThresholdCm > 0 && kWallThresholdCm <= kMappingMaxCm,
              "wall threshold must be within the mapping range");
static_assert(detail::pingBlockingUs(kCenteringPingMaxCm) <= 2500,
              "centering ping must block <= ~2.5 ms");
static_assert(detail::pingBlockingUs(kCenteringPingMaxCm) <
                  kControlPeriodUs / 2,
              "centering ping must leave time for the control loop");
// A dead sensor may block much longer; it must still not exceed one period
// (the loop then overruns, which is handled, but never skips a whole period).
static_assert(detail::pingWorstCaseUs(kCenteringPingMaxCm) < kControlPeriodUs,
              "dead-sensor centering ping must fit in one control period");
static_assert(kCenteringPingPeriods >= 1, "ping at least every period");
static_assert(kCenteringFilter > 0.0f && kCenteringFilter <= 1.0f,
              "centering filter coefficient must be in (0, 1]");

// --- Exploration ------------------------------------------------------------
static_assert(kWallFlipObservations >= 1 && kWallFlipObservations <= 4,
              "wall flip needs 1..4 observations (2-bit counter)");
static_assert(kSensorFailurePasses >= 1, "sensor failure threshold >= 1");
static_assert(kExplorationBudgetFactor >= 1 &&
                  static_cast<uint32_t>(kMazeMaxRows) * kMazeMaxCols *
                          kExplorationBudgetFactor <=
                      0xFFFFu,
              "exploration budget must fit in 16 bits");
static_assert(kCenteringMaxTicks >= 0.0f && kCenteringGainTicksPerCm >= 0.0f,
              "centering gain / bound must not be negative");
static_assert(kCenteringNominalLeftCm + kCenteringMaxDeviationCm <=
                      kCenteringPingMaxCm &&
                  kCenteringNominalRightCm + kCenteringMaxDeviationCm <=
                      kCenteringPingMaxCm,
              "centering ping range must cover nominal + max deviation");

}  // namespace micromouse::config
