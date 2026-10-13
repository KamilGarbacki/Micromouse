// Host tests: geometry derivation, calibration store (CRC, validation),
// feed-forward fit, online estimators, brake lead and front alignment.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "config/config.h"
#include "control/calibration.h"
#include "control/ff_fit.h"
#include "control/heading_carry.h"
#include "control/learning.h"
#include "control/motion_controller.h"
#include "util/crc16.h"

using namespace micromouse;
using calibration::Validity;
using learning::LearnOutcome;
using learning::LearnParams;
using learning::RobustEstimator;

namespace {

constexpr float kPi = 3.14159265f;

LearnParams scaleParams() {
  return {config::kMmPerTickMin, config::kMmPerTickMax,
          config::kScaleLearnOutlierFraction, config::kScaleLearnGain,
          config::kScaleLearnMinSamples, config::kLearnReacquireCount};
}

LearnParams pitchParams() {
  return {config::kMinCellSizeMm, config::kMaxCellSizeMm,
          config::kPitchLearnOutlierFraction, config::kPitchLearnGain,
          config::kPitchLearnMinSamples, config::kLearnReacquireCount};
}

LearnParams turnParams() {
  return {config::kTicksPer90Min, config::kTicksPer90Max,
          config::kTurnLearnOutlierFraction, config::kTurnLearnGain,
          config::kTurnLearnMinSamples, config::kLearnReacquireCount};
}

constexpr float kT90 = static_cast<float>(config::kTicksPer90);

}  // namespace

void setUp() {}
void tearDown() {}

// --- Geometry --------------------------------------------------------------

void test_geometry_derivation() {
  // 8 pulses x 120:1 = 960 ticks per wheel revolution (confirmed on the
  // robot: with 16 it drove exactly 2 cells per move).
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 960.0f, config::kTicksPerWheelRev);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, kPi * 65.0f / 960.0f, config::kMmPerTick);
  // 220 / (pi * 65 / 960) = 1034.3 -> 1034.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1034.3f, config::kTicksPerCellExact);
  TEST_ASSERT_EQUAL_INT32(1034, config::kTicksPerCell);
  // pi * 150 / 4 / 0.21271 = 553.8 -> 554 (the original hand-tuned value).
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 553.8f, config::kTicksPer90Exact);
  TEST_ASSERT_EQUAL_INT32(554, config::kTicksPer90);
  // Wheel scale -5 % / +3 %, pitch 210..222 mm, turn +-15 %.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, config::kMmPerTick * 0.95f,
                           config::kMmPerTickMin);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, config::kMmPerTick * 1.03f,
                           config::kMmPerTickMax);
  // Worst-case ticks per cell: 210 / (1.03 m) .. 222 / (0.95 m).
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 958.5f, config::kTicksPerCellMin);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 1098.6f, config::kTicksPerCellMax);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 470.8f, config::kTicksPer90Min);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 636.9f, config::kTicksPer90Max);
}

void test_profile_scales_from_physical_units() {
  // The mm values at 0.2127 mm/tick (half the phase-2 tick numbers, which
  // assumed 0.1064 mm/tick).
  TEST_ASSERT_EQUAL_INT32(60, config::forward::kCreepTicks);
  TEST_ASSERT_EQUAL_INT32(8, config::forward::kBrakeLeadTicks);
  TEST_ASSERT_EQUAL_INT32(24, config::forward::kMaxBrakeLeadTicks);
  TEST_ASSERT_EQUAL_INT32(30, config::turn::kCreepTicks);
  TEST_ASSERT_EQUAL_INT32(5, config::turn::kBrakeLeadTicks);
  TEST_ASSERT_EQUAL_INT32(14, config::turn::kMaxBrakeLeadTicks);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 4.0f, config::forward::kCruiseSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.5f, config::forward::kCreepSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f, config::turn::kCruiseSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.41f, config::turn::kCreepSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, config::forward::kAccel);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.15f, config::forward::kDecel);
  // Centred front distance: 220 / 2 - 0 / 2 - 60 = 50 mm; 45 at 210 mm.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, config::kFrontWallNominalMm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 45.0f, config::frontWallNominalMm(210.0f));
}

void test_physical_tunables_at_the_true_resolution() {
  // Units policy (tuning.h): the physical gains / thresholds reproduce the
  // per-tick values the robot ran with at the TRUE 960 ticks per rev.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.27f, config::kMmPerSPerTpp);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.08125f, config::kDegPerTickDiff);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.25f, config::forward::kDefaultFfSlope);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.25f, config::turn::kDefaultFfSlope);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, config::kSpeedKp);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 20.0f, config::kSpeedKi);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.6f, config::kHeadingKp);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.5f, config::kHeadingKi);
  TEST_ASSERT_EQUAL_INT32(62, config::kHeadingCarryLimit);  // 5 deg
  TEST_ASSERT_EQUAL_INT32(40, config::kTurnLearnMaxCarryTicks);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.0f, config::kCenteringGainTicksPerCm);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, config::kCenteringMaxTicks);
  TEST_ASSERT_EQUAL_INT32(94, config::kEncoderFaultMinTravelTicks);  // 20 mm
  // The slope bounds still contain the defaults.
  TEST_ASSERT_TRUE(config::kMinFfSlope < 1.0f && config::kMaxFfSlope > 60.0f);
}

// --- CRC / calibration store -------------------------------------------------

void test_crc16_check_value() {
  const char* text = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, util::crc16(text, 9));
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, util::crc16(text, 0));
}

void test_store_defaults_are_valid() {
  TEST_ASSERT_EQUAL(Validity::kValid,
                    calibration::validate(calibration::defaultMotor()));
  TEST_ASSERT_EQUAL(Validity::kValid,
                    calibration::validate(calibration::defaultLearned()));
  const Calibration cal = calibration::defaults();
  TEST_ASSERT_FALSE(cal.motorsCalibrated);
  TEST_ASSERT_EQUAL_INT32(1034, calibration::ticksPerCell(cal));
  TEST_ASSERT_EQUAL_INT32(554, calibration::ticksPer90(cal));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 8.0f, cal.learned.brakeLeadForward);
}

void test_store_detects_corrupt_byte() {
  MotorRecord m = calibration::defaultMotor();
  reinterpret_cast<uint8_t*>(&m)[20] ^= 0x01;  // inside wheel[0]
  TEST_ASSERT_EQUAL(Validity::kBadCrc, calibration::validate(m));
  LearnedRecord l = calibration::defaultLearned();
  reinterpret_cast<uint8_t*>(&l)[13] ^= 0x10;  // inside cellPitchMm
  TEST_ASSERT_EQUAL(Validity::kBadCrc, calibration::validate(l));
  l = calibration::defaultLearned();
  l.crc ^= 0x8000;
  TEST_ASSERT_EQUAL(Validity::kBadCrc, calibration::validate(l));
}

void test_store_rejects_wrong_magic_version_and_key() {
  LearnedRecord l = calibration::defaultLearned();
  memset(&l, 0xFF, sizeof(l));  // erased EEPROM
  TEST_ASSERT_EQUAL(Validity::kEmpty, calibration::validate(l));
  l = calibration::defaultLearned();
  l.magic = 0x434D;  // the phase-3 v1 combined record ("MC")
  TEST_ASSERT_EQUAL(Validity::kBadMagic, calibration::validate(l));

  l = calibration::defaultLearned();
  l.version = 3;  // pre-M-1: joint bit without a row count must not be trusted
  l.flags = calibration::kLearnedFlagJoint;
  l.crc = calibration::computeCrc(l);
  TEST_ASSERT_EQUAL(Validity::kBadVersion, calibration::validate(l));

  l = calibration::defaultLearned();
  l.version = 2;  // without the front-offset error
  l.crc = calibration::computeCrc(l);
  TEST_ASSERT_EQUAL(Validity::kBadVersion, calibration::validate(l));

  l = calibration::defaultLearned();
  l.geometryTicksPer90 = 1108;  // made with the wrong 16 pulses per rev
  l.crc = calibration::computeCrc(l);
  TEST_ASSERT_EQUAL(Validity::kGeometryChanged, calibration::validate(l));
  l = calibration::defaultLearned();
  l.geometryTicksPerCell = 2069;  // ditto
  l.crc = calibration::computeCrc(l);
  TEST_ASSERT_EQUAL(Validity::kGeometryChanged, calibration::validate(l));

  // The motor record is keyed on the ticks per wheel revolution only: a
  // record calibrated with the wrong 16 pulses (1920) is rejected.
  MotorRecord m = calibration::defaultMotor();
  m.ticksPerWheelRev = 1920;
  m.crc = calibration::computeCrc(m);
  TEST_ASSERT_EQUAL(Validity::kGeometryChanged, calibration::validate(m));
  m = calibration::defaultMotor();
  m.version = 2;
  m.crc = calibration::computeCrc(m);
  TEST_ASSERT_EQUAL(Validity::kBadVersion, calibration::validate(m));
}

void test_store_rejects_out_of_range_fields() {
  LearnedRecord l = calibration::defaultLearned();
  l.cellPitchMm = 205.0f;  // below the 210 mm minimum
  calibration::seal(l);
  TEST_ASSERT_EQUAL(Validity::kOutOfRange, calibration::validate(l));

  l = calibration::defaultLearned();
  l.mmPerTick = config::kMmPerTick * 1.1f;  // beyond the +3 % tyre bound
  calibration::seal(l);
  TEST_ASSERT_EQUAL(Validity::kOutOfRange, calibration::validate(l));

  l = calibration::defaultLearned();
  l.brakeLeadTurn = -1.0f;
  calibration::seal(l);
  TEST_ASSERT_EQUAL(Validity::kOutOfRange, calibration::validate(l));

  l = calibration::defaultLearned();
  l.wallOffsetMm = 20.0f;  // beyond +-15 mm
  calibration::seal(l);
  TEST_ASSERT_EQUAL(Validity::kOutOfRange, calibration::validate(l));

  l = calibration::defaultLearned();
  l.pitchSums.v[learning::PitchSums::kBB] = -1.0f;  // sum of squares < 0
  calibration::seal(l);
  TEST_ASSERT_EQUAL(Validity::kOutOfRange, calibration::validate(l));

  l = calibration::defaultLearned();
  l.flags = 0x80;  // unknown flag
  calibration::seal(l);
  TEST_ASSERT_EQUAL(Validity::kOutOfRange, calibration::validate(l));

  MotorRecord m = calibration::defaultMotor();
  m.wheel[1].kineticPwm = m.wheel[1].staticPwm + 1.0f;  // kinetic > static
  calibration::seal(m);
  TEST_ASSERT_EQUAL(Validity::kOutOfRange, calibration::validate(m));

  m = calibration::defaultMotor();
  m.wheel[0].ffSlopePwmPerTpp = NAN;
  calibration::seal(m);
  TEST_ASSERT_EQUAL(Validity::kOutOfRange, calibration::validate(m));
}

void test_store_effective_values_and_seed_restart() {
  Calibration cal = calibration::defaults();
  // A 210 mm maze (learned) with nominal wheels: 210 / 0.21271 = 987.3.
  cal.learned.cellPitchMm = 210.0f;
  cal.learned.pitchSamples = config::kPitchLearnMinSamples - 1;
  TEST_ASSERT_EQUAL_INT32(1034, calibration::ticksPerCell(cal));  // not yet
  cal.learned.pitchSamples = config::kPitchLearnMinSamples;
  TEST_ASSERT_EQUAL_INT32(987, calibration::ticksPerCell(cal));
  // Plus a compressed tyre (-2 % mm per tick): 210 / (0.98 m) = 1007.4.
  cal.learned.mmPerTick = config::kMmPerTick * 0.98f;
  cal.learned.scaleSamples = config::kScaleLearnMinSamples;
  TEST_ASSERT_EQUAL_INT32(1007, calibration::ticksPerCell(cal));
  // An interrupted seed (samples below the minimum) restarts after boot.
  cal.learned.turnSamples = config::kTurnLearnMinSamples - 1;
  calibration::restartIncompleteSeeds(cal.learned);
  TEST_ASSERT_EQUAL_UINT16(0, cal.learned.turnSamples);
  TEST_ASSERT_EQUAL_UINT16(config::kPitchLearnMinSamples,
                           cal.learned.pitchSamples);
}

void test_wheel_model_defaults_and_floor_load() {
  Calibration cal = calibration::defaults();
  // Not self-calibrated: the phase-2 defaults of each move kind.
  WheelCalibration f =
      calibration::wheelModel(cal, Wheel::kLeft, MoveKind::kForward);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 35.0f, f.staticPwm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 25.0f, f.ffOffsetPwm);
  WheelCalibration t =
      calibration::wheelModel(cal, Wheel::kRight, MoveKind::kTurn);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 30.0f, t.staticPwm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 22.0f, t.kineticPwm);
  // Self-calibrated: per wheel, turns add the configured floor load.
  cal.motorsCalibrated = true;
  cal.motor.wheel[0] = {20.0f, 15.0f, 14.0f, 6.0f};
  cal.motor.wheel[1] = {24.0f, 18.0f, 16.0f, 7.0f};
  f = calibration::wheelModel(cal, Wheel::kRight, MoveKind::kForward);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f,
                           24.0f * config::kForwardLoadScale +
                               config::kForwardLoadOffsetPwm,
                           f.staticPwm);
  t = calibration::wheelModel(cal, Wheel::kLeft, MoveKind::kTurn);
  TEST_ASSERT_FLOAT_WITHIN(
      1e-4f, 14.0f * config::kTurnLoadScale + config::kTurnLoadOffsetPwm,
      t.ffOffsetPwm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.0f, t.ffSlopePwmPerTpp);  // unchanged
}

// --- Feed-forward fit --------------------------------------------------------

void test_ff_fit_exact_line() {
  const float x[] = {2.0f, 4.0f, 6.0f, 8.0f, 10.0f};
  float y[5];
  for (int i = 0; i < 5; ++i) y[i] = 20.0f + 6.0f * x[i];
  const LineFit fit = fitLine(x, y, 5);
  TEST_ASSERT_TRUE(fit.valid);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f, fit.offset);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.0f, fit.slope);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, fit.r2);
}

void test_ff_fit_least_squares_and_degenerate() {
  // Symmetric noise around y = 10 + 5x: same line, r2 < 1.
  const float x[] = {1.0f, 2.0f, 3.0f, 4.0f};
  const float y[] = {15.5f, 19.5f, 25.5f, 29.5f};
  const LineFit fit = fitLine(x, y, 4);
  TEST_ASSERT_TRUE(fit.valid);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.8f, fit.slope);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.5f, fit.offset);
  TEST_ASSERT_TRUE(fit.r2 > 0.98f && fit.r2 < 1.0f);
  const float same[] = {3.0f, 3.0f, 3.0f};
  TEST_ASSERT_FALSE(fitLine(same, y, 3).valid);
  TEST_ASSERT_FALSE(fitLine(x, y, 1).valid);
}

// --- Robust estimator --------------------------------------------------------

void test_estimator_seeds_with_median_within_bounds() {
  RobustEstimator est;
  float value = kT90;
  uint16_t n = 0;
  // Seeding is gated by the hard bounds only: 620 (12 % off the default
  // 554, beyond the 10 % tracking gate) is accepted; 700 (> +15 %) is not.
  TEST_ASSERT_EQUAL(LearnOutcome::kOutlier,
                    est.update(value, n, 700.0f, kT90, turnParams()));
  TEST_ASSERT_EQUAL(LearnOutcome::kSeeding,
                    est.update(value, n, 620.0f, kT90, turnParams()));
  TEST_ASSERT_EQUAL(LearnOutcome::kSeeding,
                    est.update(value, n, 575.0f, kT90, turnParams()));
  TEST_ASSERT_EQUAL(LearnOutcome::kSeeding,
                    est.update(value, n, 622.0f, kT90, turnParams()));
  TEST_ASSERT_EQUAL_FLOAT(kT90, learning::learnedOrPrior(
                                    value, n, kT90,
                                    config::kTurnLearnMinSamples));
  TEST_ASSERT_EQUAL(LearnOutcome::kAccepted,
                    est.update(value, n, 619.0f, kT90, turnParams()));
  // Median of {620, 575, 622, 619}: the lower middle, 619 - the wild
  // 575 does not pull it down.
  TEST_ASSERT_EQUAL_UINT16(config::kTurnLearnMinSamples, n);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 619.0f, value);
}

void test_estimator_tracks_the_10_to_15_percent_band() {
  // A true ticks/90 of 620 (+12 %) was locked out by the old +-10 % gate
  // around the default; now it converges and stays.
  RobustEstimator est;
  float value = kT90;
  uint16_t n = 0;
  const float noise[] = {+1.5f, -2.0f, +0.5f, -1.0f, +2.5f};
  for (int i = 0; i < 60; ++i) {
    est.update(value, n, 620.0f + noise[i % 5], kT90, turnParams());
  }
  TEST_ASSERT_FLOAT_WITHIN(1.5f, 620.0f,
                           learning::learnedOrPrior(
                               value, n, kT90, config::kTurnLearnMinSamples));
}

void test_estimator_rejects_isolated_outliers_and_invalid() {
  RobustEstimator est;
  float value = 2000.0f;
  uint16_t n = 10;  // tracking
  const float prior = 2069.0f;
  const LearnParams p = {1900.0f, 2200.0f, 0.04f, 0.15f, 3, 4};
  TEST_ASSERT_EQUAL(LearnOutcome::kOutlier,
                    est.update(value, n, 2150.0f, prior, p));
  TEST_ASSERT_EQUAL(LearnOutcome::kAccepted,
                    est.update(value, n, 2010.0f, prior, p));  // resets run
  TEST_ASSERT_EQUAL(LearnOutcome::kOutlier,
                    est.update(value, n, 2150.0f, prior, p));
  TEST_ASSERT_EQUAL(LearnOutcome::kInvalid,
                    est.update(value, n, NAN, prior, p));
  TEST_ASSERT_EQUAL(LearnOutcome::kInvalid,
                    est.update(value, n, 0.0f, prior, p));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2001.5f, value);
  TEST_ASSERT_EQUAL_UINT16(11, n);
}

void test_estimator_recovers_from_lock_in() {
  // A bad early estimate (575) with the truth at 500: every sample is an
  // outlier for the 10 % gate (|500 - 575| = 75 > 57.5).
  RobustEstimator est;
  float value = 575.0f;
  uint16_t n = 20;
  const float truth = 500.0f;  // -13 % from 575: outside the gate
  LearnOutcome last = LearnOutcome::kOutlier;
  int steps = 0;
  const float noise[] = {+2.0f, -1.5f, +1.0f, -2.5f};
  while (last != LearnOutcome::kReacquired && steps < 10) {
    last = est.update(value, n, truth + noise[steps % 4], kT90, turnParams());
    ++steps;
  }
  TEST_ASSERT_EQUAL(LearnOutcome::kReacquired, last);
  TEST_ASSERT_EQUAL_INT(config::kLearnReacquireCount, steps);
  TEST_ASSERT_FLOAT_WITHIN(2.5f, truth, value);
  TEST_ASSERT_EQUAL_UINT16(config::kTurnLearnMinSamples, n);
  // Disagreeing outliers (spread > the gate) do NOT re-acquire.
  RobustEstimator other;
  float v2 = 575.0f;
  uint16_t n2 = 20;
  const float spread[] = {475.0f, 650.0f, 475.0f, 650.0f, 475.0f, 650.0f};
  for (const float s : spread) {
    TEST_ASSERT_EQUAL(LearnOutcome::kOutlier,
                      other.update(v2, n2, s, kT90, turnParams()));
  }
  TEST_ASSERT_EQUAL_FLOAT(575.0f, v2);
}

void test_estimator_stale_eeprom_start() {
  // The stored wheel scale says -4 % (old tyres); new tyres are at +2 %. The
  // stored record is valid and applied, but after kLearnReacquireCount
  // consistent samples the estimator re-seeds from them.
  const float m0 = config::kMmPerTick;
  Calibration cal = calibration::defaults();
  cal.learned.mmPerTick = m0 * 0.96f;
  cal.learned.scaleSamples = 40;
  RobustEstimator est;
  for (int i = 0; i < config::kLearnReacquireCount; ++i) {
    const LearnOutcome o =
        est.update(cal.learned.mmPerTick, cal.learned.scaleSamples,
                   m0 * (1.02f + 0.001f * (i % 2)), m0, scaleParams());
    TEST_ASSERT_TRUE(o == LearnOutcome::kOutlier ||
                     o == LearnOutcome::kReacquired);
  }
  TEST_ASSERT_FLOAT_WITHIN(m0 * 0.002f, m0 * 1.02f,
                           calibration::mmPerTick(cal));
}

void test_estimator_clamped_to_bounds() {
  RobustEstimator est;
  float value = 215.0f;
  uint16_t n = 10;
  // Samples always 2.5 % below the estimate (inside the 3 % gate) walk it
  // down, but never below 210 mm.
  for (int i = 0; i < 200; ++i) {
    est.update(value, n, value * 0.975f, config::kCellSizeMm, pitchParams());
    TEST_ASSERT_TRUE(value >= config::kMinCellSizeMm - 1e-3f);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, config::kMinCellSizeMm, value);
}

// --- Wheel scale and cell pitch ----------------------------------------------

void test_scale_sample_from_front_distances() {
  // 2069 ticks moved the robot 220 mm: 0.10633 mm per tick.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 220.0f / 2069.0f,
                           learning::mmPerTickSample(270.0f, 50.0f, 2069));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, learning::mmPerTickSample(50.0f, 60.0f, 2069));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, learning::mmPerTickSample(270.0f, 0.0f, 2069));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, learning::mmPerTickSample(270.0f, 50.0f, 0));
}

namespace {

/// The TRUE geometry of a simulated maze and robot. The firmware only knows
/// the configured wall thickness / sensor offset (config/geometry.h).
struct Truth {
  float pitch;     ///< P [mm]
  float wall;      ///< w [mm]
  float sensor;    ///< s [mm]
  /// delta = (w_c - w) / 2 + (s_c - s): configured minus true.
  float offset() const {
    return (config::kWallThicknessMm - wall) / 2.0f +
           (config::kFrontSensorToCenterMm - sensor);
  }
};

/// Front reading [mm] of the robot centre at `centreMm` along the axis
/// (cell boundaries at multiples of P), facing the wall `cells + 1`
/// boundaries ahead of the start cell's rear boundary.
float frontReading(const Truth& t, const float centreMm, const int cells) {
  return (cells + 1) * t.pitch - t.wall / 2.0f - centreMm - t.sensor;
}

/// Centre position after aligning on the rear wall (boundary 0, face at
/// w / 2) with the final reading `finalFrontMm`.
float centreAfterAlignment(const Truth& t, const float finalFrontMm) {
  return t.wall / 2.0f + finalFrontMm + t.sensor;
}

learning::PitchParams jointParams() {
  return {config::kMinCellSizeMm, config::kMaxCellSizeMm,
          config::kMaxWallOffsetMm, config::kPitchLearnOutlierFraction,
          config::kPitchJointForget, config::kPitchMinDecorrelation,
          config::kLearnReacquireCount, config::kPitchMinRows};
}

/// Simulated pitch learning: runs of `cells` straight cells from an aligned
/// reference (or the start cell), with the alignment and the drive using
/// the values in force, as the firmware does.
struct PitchSim {
  Truth truth;
  learning::PitchSums sums = {};
  float pitch = config::kCellSizeMm;
  float offset = 0.0f;
  uint8_t state = 0;  // learning::kPitchJointBit + rows
  learning::PitchOffsetEstimator estimator;

  bool joint() const { return learning::pitchJointInForce(state); }
  float pitchInForce() const {
    return joint() ? pitch : config::kCellSizeMm;
  }
  float offsetInForce() const { return joint() ? offset : 0.0f; }

  LearnOutcome run(const int cells, const bool aligned, const float noise) {
    // Reference: aligned to the target in force (+ a residual), or the
    // start cell (placed truly centred).
    const float target =
        config::frontWallNominalMm(pitchInForce()) + offsetInForce();
    const float rf = target + 0.7f * noise;
    const float start =
        aligned ? centreAfterAlignment(truth, rf) : truth.pitch / 2.0f;
    // The drive: cells * ticks per cell in force (wheel scale exact).
    const float driven = cells * pitchInForce() + 1.3f * noise;
    const float re = frontReading(truth, start + driven, cells) + noise;
    const learning::PitchObservation o = learning::pitchObservation(
        driven, re, static_cast<uint8_t>(cells), aligned, rf,
        config::kWallThicknessMm, config::kFrontSensorToCenterMm);
    return estimator.update(sums, pitch, offset, state, o, jointParams());
  }
};

}  // namespace

void test_pitch_observation_is_exact_for_true_geometry() {
  // a * P + b * delta = y must hold for the TRUE P and delta, whatever the
  // configured wall thickness / sensor offset (they do NOT cancel).
  const Truth truths[] = {{220.0f, 12.0f, 60.0f},   // delta = -6
                          {212.0f, 0.0f, 52.0f},    // delta = +8
                          {216.0f, 8.0f, 63.0f}};   // delta = -7
  for (const Truth& t : truths) {
    for (int cells = 1; cells <= 3; ++cells) {
      // Start cell.
      const float driven = cells * 220.0f + 2.0f;
      float re = frontReading(t, t.pitch / 2.0f + driven, cells);
      learning::PitchObservation o = learning::pitchObservation(
          driven, re, static_cast<uint8_t>(cells), false, 0.0f,
          config::kWallThicknessMm, config::kFrontSensorToCenterMm);
      TEST_ASSERT_FLOAT_WITHIN(1e-2f, o.y, o.a * t.pitch + o.b * t.offset());
      TEST_ASSERT_FLOAT_WITHIN(1e-6f, cells + 0.5f, o.a);
      TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, o.b);
      // Aligned reference with an arbitrary final reading (residual).
      const float rf = 47.3f;
      re = frontReading(t, centreAfterAlignment(t, rf) + driven, cells);
      o = learning::pitchObservation(driven, re,
                                     static_cast<uint8_t>(cells), true, rf,
                                     config::kWallThicknessMm,
                                     config::kFrontSensorToCenterMm);
      TEST_ASSERT_FLOAT_WITHIN(1e-2f, o.y, o.a * t.pitch + o.b * t.offset());
      TEST_ASSERT_FLOAT_WITHIN(1e-6f, cells + 1.0f, o.a);
      TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, o.b);
    }
  }
}

void test_pitch_and_offset_converge_jointly() {
  // Real 12 mm walls, configured 0 (delta = -6), 214 mm pitch: assuming
  // delta = 0 would learn 214 + 2 * (-6) / 2 = 208 from aligned 1-cell
  // runs; the joint solve recovers both.
  PitchSim sim;
  sim.truth = {214.0f, 12.0f, 60.0f};
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -6.0f, sim.truth.offset());
  const int cells[] = {1, 2, 3, 1, 2};
  const bool aligned[] = {false, true, true, true, true};
  const float noise[] = {+1.0f, -0.8f, +0.5f, -1.2f, +0.3f, -0.4f, +0.9f};
  for (int i = 0; i < 40; ++i) {
    sim.run(cells[i % 5], aligned[i % 5] || i > 0, noise[i % 7]);
  }
  TEST_ASSERT_TRUE(sim.joint());
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 214.0f, sim.pitch);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -6.0f, sim.offset);
  // With the learned delta the alignment target is the TRUE centred reading.
  const float trueCentred =
      sim.truth.pitch / 2.0f - sim.truth.wall / 2.0f - sim.truth.sensor;
  TEST_ASSERT_FLOAT_WITHIN(1.0f, trueCentred,
                           config::frontWallNominalMm(sim.pitch) + sim.offset);
}

void test_pitch_provisional_until_conditioned() {
  // Only aligned 2-cell runs: all observations alike, P and delta cannot be
  // separated - no joint solution; the provisional pitch (delta = 0
  // assumed, robust estimator on y / a) is biased by b * delta / a.
  PitchSim sim;
  sim.truth = {214.0f, 12.0f, 60.0f};  // delta = -6
  RobustEstimator provisional;
  float provPitch = config::kCellSizeMm;
  uint16_t provSamples = 0;
  for (int i = 0; i < 6; ++i) {
    TEST_ASSERT_EQUAL(LearnOutcome::kSeeding, sim.run(2, true, 0.0f));
    TEST_ASSERT_FALSE(sim.joint());
    const float y = (sim.sums.v[learning::PitchSums::kAY]) /
                    (sim.sums.v[learning::PitchSums::kAA]);  // = y / a
    provisional.update(provPitch, provSamples, y, config::kCellSizeMm,
                       pitchParams());
  }
  // y / a = P + b delta / a = 214 + 2 * (-6) / 3 = 210.
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 210.0f, provPitch);
  // Runs of other lengths make the solve conditioned: joint, unbiased.
  sim.run(1, true, 0.0f);
  sim.run(3, true, 0.0f);
  TEST_ASSERT_TRUE(sim.joint());
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 214.0f, sim.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -6.0f, sim.offset);
}

void test_pitch_estimator_reacquires_after_maze_change() {
  PitchSim sim;
  sim.truth = {220.0f, 0.0f, 60.0f};  // delta = 0
  const int cells[] = {1, 2, 3};
  for (int i = 0; i < 12; ++i) sim.run(cells[i % 3], true, 0.0f);
  TEST_ASSERT_TRUE(sim.joint());
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 220.0f, sim.pitch);
  // Another maze: 210 mm pitch, 10 mm walls (delta -5). Every observation
  // is outside the gate; after kLearnReacquireCount the estimator restarts.
  sim.truth = {210.0f, 10.0f, 60.0f};
  LearnOutcome last = LearnOutcome::kOutlier;
  int steps = 0;
  while (last != LearnOutcome::kReacquired && steps < 10) {
    last = sim.run(cells[steps % 3], true, 0.0f);
    ++steps;
  }
  TEST_ASSERT_EQUAL(LearnOutcome::kReacquired, last);
  TEST_ASSERT_EQUAL_INT(config::kLearnReacquireCount, steps);
  TEST_ASSERT_FALSE(sim.joint());
  // The restart is seeded with the observation that triggered it.
  TEST_ASSERT_EQUAL_UINT8(1, learning::pitchRows(sim.state));
  TEST_ASSERT_TRUE(sim.sums.v[learning::PitchSums::kAA] > 0.0f);
  for (int i = 0; i < 12; ++i) sim.run(cells[i % 3], true, 0.0f);
  TEST_ASSERT_TRUE(sim.joint());
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 210.0f, sim.pitch);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -5.0f, sim.offset);
}

void test_pitch_joint_needs_min_rows() {
  // Two separable rows (aligned 1 and 2 cells) would already solve, but
  // P would only be known to ~4 mm: kPitchMinRows (3) rows are required.
  PitchSim sim;
  sim.truth = {214.0f, 12.0f, 60.0f};
  TEST_ASSERT_EQUAL(LearnOutcome::kSeeding, sim.run(1, true, 0.0f));
  TEST_ASSERT_EQUAL(LearnOutcome::kSeeding, sim.run(2, true, 0.0f));
  TEST_ASSERT_FALSE(sim.joint());
  TEST_ASSERT_EQUAL_UINT8(2, learning::pitchRows(sim.state));
  float p = 0.0f;
  float d = 0.0f;
  TEST_ASSERT_TRUE(learning::solvePitch(sim.sums, 0.02f, p, d));  // could
  TEST_ASSERT_EQUAL(LearnOutcome::kAccepted, sim.run(3, true, 0.0f));
  TEST_ASSERT_TRUE(sim.joint());
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 214.0f, sim.pitch);
  // Rows saturate at 7.
  for (int i = 0; i < 10; ++i) sim.run(1 + i % 3, true, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(7, learning::pitchRows(sim.state));
}

void test_pitch_solve_needs_distinct_ratio() {
  // An aligned run of 2 cells (a / b = 3 / 2) is collinear with a start
  // run of 1 cell (1.5 / 1): a different n alone does not separate P and
  // delta.
  learning::PitchSums sums = {};
  const learning::PitchObservation rows[] = {{3.0f, 2.0f, 630.0f},
                                             {1.5f, 1.0f, 315.0f}};
  for (const learning::PitchObservation& o : rows) {
    sums.v[learning::PitchSums::kAA] += o.a * o.a;
    sums.v[learning::PitchSums::kAB] += o.a * o.b;
    sums.v[learning::PitchSums::kBB] += o.b * o.b;
    sums.v[learning::PitchSums::kAY] += o.a * o.y;
    sums.v[learning::PitchSums::kBY] += o.b * o.y;
  }
  float p = 0.0f;
  float d = 0.0f;
  TEST_ASSERT_FALSE(learning::solvePitch(sums, 0.02f, p, d));
}

void test_pitch_solve_needs_distinct_runs() {
  learning::PitchSums sums = {};
  float p = 0.0f;
  float d = 0.0f;
  TEST_ASSERT_FALSE(learning::solvePitch(sums, 0.02f, p, d));
  // Two identical rows: singular.
  sums.v[learning::PitchSums::kAA] = 2 * 9.0f;
  sums.v[learning::PitchSums::kAB] = 2 * -6.0f;
  sums.v[learning::PitchSums::kBB] = 2 * 4.0f;
  TEST_ASSERT_FALSE(learning::solvePitch(sums, 0.02f, p, d));
}

// --- Ticks per 90 estimator --------------------------------------------------

namespace {

constexpr float kD = 220.0f;  // driven distance [mm]

/// Side readings of a straight move during which the robot holds the TRUE
/// heading eps (encoder heading 0): the lateral position changes by
/// eps * D, which the left sensor sees as +, the right one as -.
learning::WallDrift straightAlongWall(const float eps, const int8_t sigma) {
  learning::WallDrift d = {};
  d.startMm = 60.0f;
  d.endMm = 60.0f + static_cast<float>(sigma) * eps * kD;
  d.sigma = sigma;
  d.distanceMm = kD;
  d.leverMm = 40.0f;
  return d;
}

float turnSample(const int8_t sign, const int8_t sigma, const float tTrue) {
  learning::TurnScaleLearner learner;
  float sample = 0.0f;
  TEST_ASSERT_FALSE(learner.onMeasurement(
      learning::invisibleHeadingRad(straightAlongWall(0.0f, sigma)), kD,
      sample));
  learner.onTurn(sign, kT90, kT90);
  const float eps = sign * (kPi / 2.0f) * (kT90 / tTrue - 1.0f);
  const float measured =
      learning::invisibleHeadingRad(straightAlongWall(eps, sigma));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, eps, measured);
  TEST_ASSERT_TRUE(learner.onMeasurement(measured, kD, sample));
  return sample;
}

}  // namespace

void test_turn_sample_signs_all_cases() {
  const float truths[] = {1080.0f, 1140.0f};
  const int8_t signs[] = {1, -1};
  const int8_t sides[] = {1, -1};
  for (const float truth : truths) {
    for (const int8_t sign : signs) {
      for (const int8_t side : sides) {
        TEST_ASSERT_FLOAT_WITHIN(0.5f, truth, turnSample(sign, side, truth));
      }
    }
  }
  // CW over-rotation drifts the robot to the RIGHT (eps > 0): the left
  // distance grows, the right one shrinks; fewer ticks per 90 deg.
  const learning::WallDrift left = straightAlongWall(0.02f, 1);
  const learning::WallDrift right = straightAlongWall(0.02f, -1);
  TEST_ASSERT_TRUE(left.endMm > left.startMm);
  TEST_ASSERT_TRUE(right.endMm < right.startMm);
  TEST_ASSERT_TRUE(learning::ticksPer90Sample(0.02f, 1, kT90, kT90) < kT90);
  TEST_ASSERT_TRUE(learning::ticksPer90Sample(-0.02f, -1, kT90, kT90) < kT90);
  TEST_ASSERT_TRUE(learning::ticksPer90Sample(-0.02f, 1, kT90, kT90) > kT90);
  // TurnDirection -> sign used by the app glue.
  TEST_ASSERT_EQUAL_INT8(1, turnSign(TurnDirection::kClockwise));
  TEST_ASSERT_EQUAL_INT8(-1, turnSign(TurnDirection::kCounterClockwise));
}

void test_turn_drift_subtracts_encoder_visible_heading() {
  const float eps = -0.015f;
  const float h0 = 0.03f;  // rad, encoder-visible
  const float sCorr = 40.0f;
  const float lever = 40.0f;
  const float encIntegral = h0 * sCorr / 2.0f;
  const float lateral = encIntegral + eps * kD;
  const int8_t sigmas[] = {1, -1};
  for (const int8_t sigma : sigmas) {
    learning::WallDrift d = {};
    d.sigma = sigma;
    d.distanceMm = kD;
    d.leverMm = lever;
    d.encStartRad = h0;
    d.encEndRad = 0.0f;
    d.encIntegralMmRad = encIntegral;
    d.startMm = 60.0f + sigma * (lever * (h0 + eps));
    d.endMm = 60.0f + sigma * (lateral + lever * eps);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, eps, learning::invisibleHeadingRad(d));
  }
}

void test_turn_learner_needs_baseline_and_single_turn() {
  learning::TurnScaleLearner learner;
  float sample = 0.0f;
  learner.onTurn(1, kT90, kT90);
  TEST_ASSERT_FALSE(learner.onMeasurement(0.01f, kD, sample));
  learner.onTurn(1, kT90, kT90);
  learner.onTurn(-1, kT90, kT90);
  TEST_ASSERT_FALSE(learner.onMeasurement(0.02f, kD, sample));  // two turns
  learner.onTurn(1, kT90, kT90);
  TEST_ASSERT_TRUE(learner.onMeasurement(0.03f, kD, sample));
  TEST_ASSERT_TRUE(sample < kT90);
  learner.reset();
  learner.onTurn(1, kT90, kT90);
  TEST_ASSERT_FALSE(learner.onMeasurement(0.03f, kD, sample));
}

void test_turn_learner_subtracts_straight_line_curvature() {
  // Wheel-diameter mismatch: the robot curves clockwise by c rad per mm on
  // every straight; the turn itself is exact (T_true = T_used).
  const float c = 4.0e-5f;  // 0.5 deg per cell
  learning::TurnScaleLearner learner;
  float sample = 0.0f;
  float eps = 0.0f;
  // Straights without turns: eps grows by c * distance (midpoint to
  // midpoint = kD for consecutive cells).
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_FALSE(learner.onMeasurement(eps, kD, sample));
    eps += c * kD;
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, c, learner.curvatureRadPerMm());
  // One unmeasured straight, then an exact CW turn, then a measurement:
  // eps grew only by curvature over 2 * kD.
  learner.onStraight(kD);
  eps += c * kD;
  learner.onTurn(1, kT90, kT90);
  TEST_ASSERT_TRUE(learner.onMeasurement(eps, kD, sample));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, kT90, sample);
  // Without the subtraction the sample would be biased by ~0.6 %.
  TEST_ASSERT_TRUE(learning::ticksPer90Sample(c * 2 * kD, 1, kT90, kT90) <
                   kT90 - 5.0f);
}

namespace {

/// Pose of a differential-drive robot in a corridor frame: x lateral (+
/// right), y along the corridor, theta + clockwise from +y.
struct SimPose {
  double x;
  double y;
  double theta;
};

/// Side sensor reading [mm] of the left (sigma +1) or right (-1) wall at
/// x = -/+ halfWidth, sensor `lever` ahead of the centre, beam
/// perpendicular to the heading (exact trigonometry, not small-angle).
float sideReading(const SimPose& p, const int8_t sigma, const double lever,
                  const double halfWidth) {
  const double sx = p.x + lever * sin(p.theta);
  const double perpendicular = sigma > 0 ? sx + halfWidth : halfWidth - sx;
  return static_cast<float>(perpendicular / cos(p.theta));
}

struct SimResult {
  float eps;
  float moveMm;
};

/// Drives `ticks` per wheel straight with the encoder heading starting at
/// `carry` [L - R ticks], which the heading loop removes linearly over the
/// first `corrTicks`; integrates the TRUE pose with the true geometry and
/// returns what the app glue computes (wallDriftOf + invisibleHeadingRad).
SimResult simulateStraight(SimPose& pose, const int32_t carry,
                           const int32_t ticks, const int32_t corrTicks,
                           const bool leftWall, const double mTrue,
                           const double tTrue, const float mEst,
                           const float tUsed) {
  const double lever = config::kSideSensorAheadOfCenterMm;
  const double halfWidth = 100.0;
  const int8_t sigma = leftWall ? 1 : -1;
  const float start = sideReading(pose, sigma, lever, halfWidth);
  const double radPerDiffTick = kPi / (4.0 * tTrue);  // true m / b
  double integral = 0.0;
  int32_t diff = 0;  // L - R
  for (int32_t s = 1; s <= ticks; ++s) {
    const int32_t target =
        s >= corrTicks ? -carry : static_cast<int32_t>(-carry * s / corrTicks);
    const int32_t step = target - diff;
    diff = target;
    pose.theta += step * radPerDiffTick;
    pose.x += mTrue * sin(pose.theta);
    pose.y += mTrue * cos(pose.theta);
    integral += static_cast<double>(carry + diff);
  }
  const float end = sideReading(pose, sigma, lever, halfWidth);
  // Counts as the encoders see them: L - R = diff, (L + R) / 2 = ticks.
  const learning::StraightMove move = {carry, ticks + diff / 2,
                                       ticks - diff / 2,
                                       static_cast<float>(integral)};
  const learning::WallDrift drift = learning::wallDriftOf(
      move, leftWall, start, end, mEst, tUsed,
      config::kSideSensorAheadOfCenterMm);
  return {learning::invisibleHeadingRad(drift),
          learning::straightDistanceMm(move, mEst)};
}

/// Full chain on the true geometry: straight along a wall, a turn of
/// `direction` with N ticks per wheel, straight along a wall in the new
/// corridor. Returns the ticks-per-90 sample of the learner.
float simulateTurnLearning(const TurnDirection direction, const bool wallLeft1,
                           const bool wallLeft2, const double tTrue) {
  const double mTrue = config::kMmPerTick * 1.01;  // tyre 1 % larger
  const float mEst = config::kMmPerTick;
  const float tUsed = kT90;
  const int32_t cell = config::kTicksPerCell;
  learning::TurnScaleLearner learner;
  float sample = 0.0f;

  SimPose pose = {3.0, 0.0, 0.004};  // a little off-centre and rotated
  const SimResult a = simulateStraight(pose, 0, cell, 1, wallLeft1, mTrue,
                                       tTrue, mEst, tUsed);
  learner.onMeasurement(a.eps, a.moveMm, sample);

  // Turn: N = T_used + 5 ticks per wheel (brake overshoot), visible to the
  // encoders as carry; truly rotated sign * pi/2 * N / T_true.
  const int8_t sign = turnSign(direction);
  const int32_t n = static_cast<int32_t>(tUsed) + 5;
  const int32_t carry = heading::carryAfterTurn(
      0, sign, n, n, static_cast<int32_t>(tUsed), config::kHeadingCarryLimit);
  learner.onTurn(sign, static_cast<float>(n), tUsed);
  // New corridor frame: heading relative to its axis.
  pose.theta += sign * (kPi / 2.0) * n / tTrue - sign * (kPi / 2.0);
  pose.x = -2.0;
  pose.y = 0.0;

  const SimResult b = simulateStraight(pose, carry, cell, 300, wallLeft2,
                                       mTrue, tTrue, mEst, tUsed);
  TEST_ASSERT_TRUE(learner.onMeasurement(b.eps, b.moveMm, sample));
  return sample;
}

}  // namespace

void test_turn_learning_geometric_simulation() {
  // True ticks per 90: 540 (robot over-rotates) and 568 (under-rotates),
  // CW and CCW, walls left and right, on exact trigonometry, with a carry
  // from the turn overshoot, a 1 % wheel-scale error and the sensor lever.
  const double truths[] = {540.0, 568.0};
  const TurnDirection dirs[] = {TurnDirection::kClockwise,
                                TurnDirection::kCounterClockwise};
  for (const double truth : truths) {
    for (const TurnDirection dir : dirs) {
      TEST_ASSERT_FLOAT_WITHIN(
          2.0f, truth, simulateTurnLearning(dir, true, true, truth));
      TEST_ASSERT_FLOAT_WITHIN(
          2.0f, truth, simulateTurnLearning(dir, false, true, truth));
      TEST_ASSERT_FLOAT_WITHIN(
          2.0f, truth, simulateTurnLearning(dir, true, false, truth));
      TEST_ASSERT_FLOAT_WITHIN(
          2.0f, truth, simulateTurnLearning(dir, false, false, truth));
    }
  }
}

// --- Brake lead --------------------------------------------------------------

void test_brake_lead_adaptation() {
  TEST_ASSERT_FLOAT_WITHIN(
      1e-4f, 16.0f, learning::adaptBrakeLead(15.0f, 5, 0.2f, 47.0f, 141));
  TEST_ASSERT_FLOAT_WITHIN(
      1e-4f, 14.0f, learning::adaptBrakeLead(15.0f, -5, 0.2f, 47.0f, 141));
  TEST_ASSERT_FLOAT_WITHIN(
      1e-4f, 0.0f, learning::adaptBrakeLead(0.5f, -10, 0.2f, 47.0f, 141));
  TEST_ASSERT_FLOAT_WITHIN(
      1e-4f, 47.0f, learning::adaptBrakeLead(46.0f, 100, 0.2f, 47.0f, 141));
  // A slip / bump (|over| > maxAbsOver) is ignored.
  TEST_ASSERT_FLOAT_WITHIN(
      1e-4f, 15.0f, learning::adaptBrakeLead(15.0f, 500, 0.2f, 47.0f, 141));
  float lead = 15.0f;
  for (int i = 0; i < 60; ++i) {
    const int32_t over = static_cast<int32_t>(25.0f - lead);  // true 25
    lead = learning::adaptBrakeLead(lead, over, 0.2f, 47.0f, 141);
  }
  TEST_ASSERT_FLOAT_WITHIN(1.5f, 25.0f, lead);
}

// --- Front-wall alignment ----------------------------------------------------

void test_alignment_target_math() {
  const learning::AlignParams p = {50.0f, 3.0f, 30.0f, 60.0f};
  const float mmPerTick = config::kMmPerTick;
  // 10 mm / 0.21271 = 47.0, -5 mm = -23.5 -> -24, clamp 30 mm = 141.0.
  TEST_ASSERT_EQUAL_INT32(47, learning::alignmentTicks(60.0f, mmPerTick, p));
  TEST_ASSERT_EQUAL_INT32(-24, learning::alignmentTicks(45.0f, mmPerTick, p));
  TEST_ASSERT_EQUAL_INT32(0, learning::alignmentTicks(52.0f, mmPerTick, p));
  TEST_ASSERT_EQUAL_INT32(141, learning::alignmentTicks(95.0f, mmPerTick, p));
  TEST_ASSERT_EQUAL_INT32(-141, learning::alignmentTicks(5.0f, mmPerTick, p));
  TEST_ASSERT_EQUAL_INT32(0, learning::alignmentTicks(200.0f, mmPerTick, p));
  TEST_ASSERT_EQUAL_INT32(0, learning::alignmentTicks(0.0f, mmPerTick, p));
}

void test_alignment_brake_lead_never_swallows_the_move() {
  // The learned forward lead (it brakes from the same creep speed) ...
  TEST_ASSERT_EQUAL_INT32(15, learning::alignmentBrakeLead(100, 15));
  TEST_ASSERT_EQUAL_INT32(15, learning::alignmentBrakeLead(-100, 15));
  // ... but at most half the move: a 3 mm correction (28 ticks) still
  // moves, forward or backward.
  TEST_ASSERT_EQUAL_INT32(14, learning::alignmentBrakeLead(28, 15));
  TEST_ASSERT_EQUAL_INT32(14, learning::alignmentBrakeLead(-28, 47));
  TEST_ASSERT_EQUAL_INT32(3, learning::alignmentBrakeLead(7, 15));
}

void test_turn_carry_clip_detection() {
  // Over-rotated by 70 ticks (L + R - 2 N): beyond the 60-tick carry limit,
  // so the carry is clipped and the learner must skip this turn.
  const int32_t unclipped =
      heading::carryAfterTurnUnclamped(0, 1, 1143, 1143, 1108);
  TEST_ASSERT_EQUAL_INT32(70, unclipped);
  TEST_ASSERT_EQUAL_INT32(60, heading::carryAfterTurn(0, 1, 1143, 1143, 1108,
                                                      60));
  TEST_ASSERT_EQUAL_INT32(
      -10, heading::carryAfterTurnUnclamped(0, -1, 1113, 1113, 1108));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_geometry_derivation);
  RUN_TEST(test_profile_scales_from_physical_units);
  RUN_TEST(test_physical_tunables_at_the_true_resolution);
  RUN_TEST(test_crc16_check_value);
  RUN_TEST(test_store_defaults_are_valid);
  RUN_TEST(test_store_detects_corrupt_byte);
  RUN_TEST(test_store_rejects_wrong_magic_version_and_key);
  RUN_TEST(test_store_rejects_out_of_range_fields);
  RUN_TEST(test_store_effective_values_and_seed_restart);
  RUN_TEST(test_wheel_model_defaults_and_floor_load);
  RUN_TEST(test_ff_fit_exact_line);
  RUN_TEST(test_ff_fit_least_squares_and_degenerate);
  RUN_TEST(test_estimator_seeds_with_median_within_bounds);
  RUN_TEST(test_estimator_tracks_the_10_to_15_percent_band);
  RUN_TEST(test_estimator_rejects_isolated_outliers_and_invalid);
  RUN_TEST(test_estimator_recovers_from_lock_in);
  RUN_TEST(test_estimator_stale_eeprom_start);
  RUN_TEST(test_estimator_clamped_to_bounds);
  RUN_TEST(test_scale_sample_from_front_distances);
  RUN_TEST(test_pitch_observation_is_exact_for_true_geometry);
  RUN_TEST(test_pitch_and_offset_converge_jointly);
  RUN_TEST(test_pitch_provisional_until_conditioned);
  RUN_TEST(test_pitch_estimator_reacquires_after_maze_change);
  RUN_TEST(test_pitch_joint_needs_min_rows);
  RUN_TEST(test_pitch_solve_needs_distinct_ratio);
  RUN_TEST(test_pitch_solve_needs_distinct_runs);
  RUN_TEST(test_turn_sample_signs_all_cases);
  RUN_TEST(test_turn_drift_subtracts_encoder_visible_heading);
  RUN_TEST(test_turn_learner_needs_baseline_and_single_turn);
  RUN_TEST(test_turn_learner_subtracts_straight_line_curvature);
  RUN_TEST(test_turn_learning_geometric_simulation);
  RUN_TEST(test_brake_lead_adaptation);
  RUN_TEST(test_alignment_target_math);
  RUN_TEST(test_alignment_brake_lead_never_swallows_the_move);
  RUN_TEST(test_turn_carry_clip_detection);
  return UNITY_END();
}
