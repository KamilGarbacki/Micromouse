// Robot and maze geometry, measured with a ruler, and the encoder tick
// counts derived from it at compile time.
//
// This file is the SINGLE SOURCE OF TRUTH for the default tick counts. There
// are no override constants: if a derived number is wrong, a measurement
// below is wrong. At run time the values learned while driving (see
// control/calibration.h, README "Calibration") refine ticks per cell and per
// 90 deg, but only within the learning bounds defined at the end of this
// file.
#pragma once

#include <stdint.h>

namespace micromouse::config {

constexpr float kPi = 3.14159265f;

// --- Encoder -----------------------------------------------------------------
// [pulses] RISING edges of ONE hall channel per MOTOR-SHAFT revolution, as
// counted by hal/encoders.cc (one channel, RISING only, direction blind).
//
// 8 - CONFIRMED BY THE REAL ROBOT: with 16 configured, every one-cell move
// drove exactly two cells (the counts were half of what the firmware
// believed). 8 RISING edges x 120:1 = 960 ticks per wheel revolution,
// 1034 ticks per cell and 554 per 90 deg. It also matches the history:
// the original hand-tuned firmware (RISING, one channel) turned with 554
// ticks per 90 deg, and its 2068 ticks per cell (= 2 cells at 960) is what
// made it "drive too far". The phase-0 firmware counted CHANGE (both edges)
// and turned with 1107 = 2 x 554.
// Re-check on any new hardware with the 'e' encoder check (README step 1):
// turn a wheel by exactly one revolution; ~960 ticks -> 8, ~1920 -> 16.
// One wheel near 0 = dead / unplugged encoder (the move loop then faults
// with ENCODER_FAULT, see tuning.h kEncoderFaultMinRatio).
// Everything else (ticks per cell / per 90 deg, profile distances and
// speeds, gains and thresholds - see tuning.h) is given in physical units
// and derived from this, so it scales automatically.
constexpr uint16_t kEncoderPulsesPerMotorRev = 8;
constexpr uint16_t kGearRatio = 120;  // [-] FIT0458 gearbox, 120:1

// --- Wheels (measured) -------------------------------------------------------
constexpr float kWheelDiameterMm = 65.0f;  // [mm] outer tyre diameter
// [mm] Tyre width. The contact point can sit anywhere across it, so the
// effective wheelbase of a spin is uncertain by about +-kWheelWidthMm / 2
// (137..163 mm), plus tyre scrub. That is why the turn-learning bounds below
// are wide.
constexpr float kWheelWidthMm = 26.0f;
constexpr float kWheelShaftMm = 6.0f;  // [mm] shaft diameter (reference only)
// [mm] Distance between the MIDDLES of the two wheels (tyre centre lines).
constexpr float kWheelbaseMm = 150.0f;

// --- Maze (measured) ---------------------------------------------------------
// [mm] Cell pitch of the maze (wall centre line to wall centre line). About
// 220 mm, never below 210 mm, and not exactly known: it is the DEFAULT of the
// learned pitch (README "Calibration"), which stays within
// [kMinCellSizeMm, kMaxCellSizeMm].
constexpr float kCellSizeMm = 220.0f;
constexpr float kMinCellSizeMm = 210.0f;
constexpr float kMaxCellSizeMm = 222.0f;
// [mm] Wall thickness. MEASURE IT. 0 = treat walls as infinitely thin (the
// front-wall alignment then stops kWallThicknessMm / 2 too close).
constexpr float kWallThicknessMm = 0.0f;

// --- Sensor mounting (measure with a ruler; README step 2) -------------------
// [mm] From the rotation centre (midpoint between the wheel contact points)
// forward to the FRONT ultrasonic sensor's face. PLACEHOLDER - the user MUST
// measure it: the front-wall alignment centres the robot with it.
constexpr float kFrontSensorToCenterMm = 60.0f;
// [mm] How far the SIDE sensors sit ahead of the rotation centre. PLACEHOLDER
// - measure it: a heading change swings the side sensors by this lever arm,
// which the turn learning subtracts.
constexpr float kSideSensorAheadOfCenterMm = 40.0f;

// --- Derived defaults --------------------------------------------------------
/// [ticks] per wheel revolution (8 * 120 = 960 with the values above).
constexpr float kTicksPerWheelRev =
    static_cast<float>(kEncoderPulsesPerMotorRev) * kGearRatio;
/// [mm] wheel travel per tick (pi * 65 / 960 = 0.2127 mm).
constexpr float kMmPerTick = kPi * kWheelDiameterMm / kTicksPerWheelRev;
/// [ticks] exact (unrounded) per default cell: 220 / 0.2127 = 1034.3.
constexpr float kTicksPerCellExact = kCellSizeMm / kMmPerTick;
/// [ticks] exact per 90 deg spin in place: each wheel travels a quarter of
/// the turning circle, pi * 150 / 4 = 117.8 mm -> 553.8.
constexpr float kTicksPer90Exact = kPi * kWheelbaseMm / 4.0f / kMmPerTick;

/// Default encoder ticks (per wheel) for one cell straight ahead (1034).
constexpr int32_t kTicksPerCell =
    static_cast<int32_t>(kTicksPerCellExact + 0.5f);
/// Default encoder ticks (per wheel) for a 90 degree spin in place (554).
constexpr int32_t kTicksPer90 = static_cast<int32_t>(kTicksPer90Exact + 0.5f);

// --- Learning bounds ---------------------------------------------------------
// Ticks per cell = cell pitch / wheel scale, and both are learned separately:
//  - Wheel scale (mm of travel per tick): -5 % / +3 % of the nominal. Lower:
//    the tyre compresses under the robot's weight (effective rolling radius
//    1..3 % smaller) plus +-1 mm (1.5 %) measuring a 65 mm tyre. Upper: the
//    measuring tolerance only (a loaded tyre does not grow).
//  - Cell pitch: [kMinCellSizeMm, kMaxCellSizeMm] (see above).
constexpr float kWheelScaleLowerFraction = 0.05f;
constexpr float kWheelScaleUpperFraction = 0.03f;
// Ticks per 90 deg: +-15 %. Effective wheelbase 137..163 mm (+-8.7 %, tyre
// width) plus scrub.
constexpr float kTurnLearnFraction = 0.15f;

constexpr float kMmPerTickMin = kMmPerTick * (1.0f - kWheelScaleLowerFraction);
constexpr float kMmPerTickMax = kMmPerTick * (1.0f + kWheelScaleUpperFraction);
/// Worst-case range of the effective ticks per cell (smallest pitch with the
/// largest scale, and vice versa), for the compile-time profile checks.
constexpr float kTicksPerCellMin = kMinCellSizeMm / kMmPerTickMax;
constexpr float kTicksPerCellMax = kMaxCellSizeMm / kMmPerTickMin;
constexpr float kTicksPer90Min = kTicksPer90Exact * (1.0f - kTurnLearnFraction);
constexpr float kTicksPer90Max = kTicksPer90Exact * (1.0f + kTurnLearnFraction);

// --- Compile-time checks of the derivation ----------------------------------
static_assert(kEncoderPulsesPerMotorRev > 0 && kGearRatio > 0,
              "encoder resolution must be positive");
static_assert(kWheelDiameterMm > 0.0f && kWheelbaseMm > kWheelWidthMm &&
                  kCellSizeMm >= kMinCellSizeMm &&
                  kCellSizeMm <= kMaxCellSizeMm && kMinCellSizeMm > 0.0f,
              "geometry must be positive and consistent");
static_assert(kWallThicknessMm >= 0.0f && kWallThicknessMm < kMinCellSizeMm / 4,
              "wall thickness out of range");
static_assert(kFrontSensorToCenterMm > 0.0f &&
                  kFrontSensorToCenterMm < kMinCellSizeMm / 2,
              "front sensor must sit inside the half cell");
static_assert(kTicksPerCell > 100 && kTicksPerCell < 20000 &&
                  kTicksPer90 > 50 && kTicksPer90 < 10000,
              "derived tick counts are implausible: check the geometry");
static_assert(kTicksPerCell - kTicksPerCellExact <= 0.5f &&
                  kTicksPerCellExact - kTicksPerCell <= 0.5f &&
                  kTicksPer90 - kTicksPer90Exact <= 0.5f &&
                  kTicksPer90Exact - kTicksPer90 <= 0.5f,
              "rounding of the derived tick counts");
static_assert(kWheelScaleLowerFraction > 0.0f &&
                  kWheelScaleLowerFraction < 0.5f &&
                  kWheelScaleUpperFraction > 0.0f &&
                  kWheelScaleUpperFraction < 0.5f &&
                  kTurnLearnFraction > 0.0f && kTurnLearnFraction < 0.5f,
              "learning bounds must be in (0, 50 %)");

}  // namespace micromouse::config
