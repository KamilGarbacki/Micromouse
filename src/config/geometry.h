// Robot and maze geometry, and the encoder tick counts derived from it.
#pragma once

#include <stdint.h>

namespace micromouse::config {

constexpr float kPi = 3.14159265f;

// --- Physical dimensions ---------------------------------------------------
// NOTE: these were back-derived from the phase-1 calibrated tick counts
// (2068 ticks per cell, 554 per 90 deg) and are NOT measured. Measure the real
// robot and maze and update them; the override constants below keep the
// calibrated values in force meanwhile.
constexpr float kWheelDiameterMm = 65.0f;  // [mm] TT wheel
// [ticks] RISING edges of ONE hall channel per motor-shaft revolution.
constexpr uint16_t kEncoderCountsPerMotorRev = 16;
constexpr uint16_t kGearRatio = 120;  // [-] FIT0458 gearbox
// [mm] Effective track width for spins in place (tyre scrub makes it differ
// from the measured wheel spacing, so it is best calibrated).
constexpr float kWheelbaseMm = 75.0f;
constexpr float kCellSizeMm = 220.0f;  // [mm] wall-to-wall pitch of the maze

// --- Derived ---------------------------------------------------------------
constexpr float kTicksPerWheelRev =
    static_cast<float>(kEncoderCountsPerMotorRev) * kGearRatio;
constexpr float kMmPerTick = kPi * kWheelDiameterMm / kTicksPerWheelRev;
constexpr int32_t kDerivedTicksPerCell =
    static_cast<int32_t>(kCellSizeMm / kMmPerTick + 0.5f);
// In a spin each wheel travels a quarter of the turning circle per 90 deg.
constexpr int32_t kDerivedTicksPer90 =
    static_cast<int32_t>(kPi * kWheelbaseMm / 4.0f / kMmPerTick + 0.5f);

// --- Calibration overrides (0 = use the derived value) ---------------------
// Measured values win over theory: tyre squish and scrub are not modelled.
// See README "Calibration", step 5. With the placeholder geometry above the
// derived values are 2069 / 554; the 1-tick difference to the calibrated
// 2068 is rounding of the (back-derived) geometry, not an error.
constexpr int32_t kTicksPerCellOverride = 2068;  // [ticks per wheel]
constexpr int32_t kTicksPer90Override = 554;     // [ticks per wheel]

/// Encoder ticks (per wheel) for one cell straight ahead.
constexpr int32_t kTicksPerCell = kTicksPerCellOverride > 0
                                      ? kTicksPerCellOverride
                                      : kDerivedTicksPerCell;
/// Encoder ticks (per wheel) for a 90 degree spin in place.
constexpr int32_t kTicksPer90 =
    kTicksPer90Override > 0 ? kTicksPer90Override : kDerivedTicksPer90;

}  // namespace micromouse::config
