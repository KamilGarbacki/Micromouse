// All tunables of the robot, with units. Calibrate them on the real robot in
// the order given in README.md ("Calibration"); the step numbers below refer
// to that procedure.
//
// Speeds are in "ticks per control period" (tpp): with a 10 ms period,
// 1 tpp = 100 encoder ticks per second (average of both wheels).
#pragma once

#include <stdint.h>

#include "config/geometry.h"

namespace micromouse::config {

// --- Control loop ----------------------------------------------------------
// The move loop runs at a FIXED period, so every gain means the same thing
// regardless of how long an iteration takes.
constexpr uint32_t kControlPeriodUs = 10000;  // [us] 100 Hz
constexpr float kControlPeriodS = kControlPeriodUs / 1000000.0f;  // [s]

// --- Motor model -----------------------------------------------------------
constexpr int16_t kMaxPwm = 255;  // [PWM] analogWrite() full scale
// base PWM = stiction offset + kFfPwmPerTpp * targetSpeed + speed PI, where
// the offset is the profile's staticPwm while the robot is at rest and its
// kineticPwm while rolling. (step 3)
// PROVISIONAL: 5.25 = (35 + 4 * 8 - 25) / 8 reproduces the phase-1 cruise
// PWM (35 + 4 PWM/tpp at 8 tpp) with kineticPwm 25, but at creep speed
// (3 tpp) it gives ~6 PWM less than phase 1 (40.75 vs 47); the speed PI
// makes up the difference. A single line cannot match both points with the
// lower offset, so this MUST be recalibrated against kineticPwm (step 3).
constexpr float kFfPwmPerTpp = 5.25f;  // [PWM per tpp]
// Stiction hysteresis on the filtered average speed: the robot counts as
// rolling (kinetic offset) once it reaches kRollingEnterTpp, and as stopped
// (static offset) again only below kRollingExitTpp, so a single tick of gear
// backlash cannot toggle the offset. [tpp]
constexpr float kRollingEnterTpp = 1.0f;
constexpr float kRollingExitTpp = 0.25f;
// Per-wheel PWM multipliers so that open-loop both wheels run equally fast;
// the heading loop then only corrects small residuals. [-] (step 4)
constexpr float kLeftPwmTrim = 1.00f;
constexpr float kRightPwmTrim = 1.00f;
// Neither wheel is commanded below kWheelFloorFraction * (staticPwm at rest,
// kineticPwm while rolling); the heading correction is shifted onto the
// other wheel instead (prevents one wheel stalling into short brake and
// stick-slipping, and keeps both wheels above breakaway at the start). [-]
constexpr float kWheelFloorFraction = 1.0f;

// --- Forward speed PI (average measured speed -> PWM) ----------------------
constexpr float kSpeedKp = 3.0f;          // [PWM per tpp]
constexpr float kSpeedKi = 20.0f;         // [PWM per (tpp * s)]
constexpr float kSpeedILimit = 60.0f;     // [PWM] max integral share
constexpr float kSpeedOutLimit = 80.0f;   // [PWM] max total PI output
// Low-pass on the measured speed (tick quantisation): 1 = no filter. [-]
constexpr float kSpeedFilter = 0.5f;

// --- Heading hold (PID on leftTicks - rightTicks) --------------------------
// error = leftTicks - rightTicks (+ carried heading, + optional centering)
// [ticks]; the output is PWM subtracted from the left wheel and added to the
// right one. (step 8)
constexpr float kHeadingKp = 0.6f;          // [PWM per tick]
constexpr float kHeadingKi = 1.5f;          // [PWM per (tick * s)]
constexpr float kHeadingKd = 0.0f;          // [PWM per (tick / s)] noisy
constexpr float kHeadingILimit = 20.0f;     // [PWM] max integral share
constexpr float kHeadingOutLimit = 30.0f;   // [PWM] max total correction
// Residual heading error carried into the next move is clamped to this, so
// one bad reading (wheel slip) cannot cause a big swerve. [ticks]
constexpr int32_t kHeadingCarryLimit = 60;
// The carried heading error is ramped into the setpoint over this many
// control periods instead of being applied as a step (avoids saturating the
// heading loop at the start of every move). [periods]
constexpr uint8_t kHeadingCarryRampPeriods = 10;

// --- Velocity profiles -----------------------------------------------------
// Speed ramps up at `accel`, cruises, slows down at constant `decel` so that
// it reaches creepSpeed `creepTicks` before the brake point, creeps, then
// brakes `brakeLeadTicks` before the target (the robot coasts the rest).
namespace forward {
constexpr float kCruiseSpeed = 8.0f;  // [tpp] = 800 ticks/s (step 6)
constexpr float kCreepSpeed = 3.0f;   // [tpp] final approach (step 6)
constexpr float kAccel = 0.4f;        // [tpp per period] 0 -> 8 tpp in 200 ms
constexpr float kDecel = 0.3f;        // [tpp per period]
constexpr int32_t kCreepTicks = 120;  // [ticks] driven at creep speed
constexpr int32_t kBrakeLeadTicks = 15;  // [ticks] starting guess (step 7)
constexpr float kStaticPwm = 35.0f;   // [PWM] breakaway from rest (step 1)
constexpr float kKineticPwm = 25.0f;  // [PWM] keeps it rolling (step 2)
}  // namespace forward

namespace turn {
constexpr float kCruiseSpeed = 6.0f;     // [tpp]
constexpr float kCreepSpeed = 2.5f;      // [tpp]
constexpr float kAccel = 0.3f;           // [tpp per period]
constexpr float kDecel = 0.2f;           // [tpp per period]
constexpr int32_t kCreepTicks = 60;      // [ticks]
constexpr int32_t kBrakeLeadTicks = 10;  // [ticks] starting guess (step 7)
constexpr float kStaticPwm = 30.0f;      // [PWM] measure spinning on floor
constexpr float kKineticPwm = 22.0f;     // [PWM] starting guess (step 2)
}  // namespace turn

// --- Stopping / safety -----------------------------------------------------
// After braking, wait until neither encoder has ticked for kSettleQuietMs so
// the next move (or sensor reading) starts at rest: late ticks would be
// counted as progress by the direction-blind encoders.
constexpr uint32_t kSettleQuietMs = 60;    // [ms]
constexpr uint32_t kSettleTimeoutMs = 500;  // [ms] then flagged, not fatal
// A move fails (FAULT) if the average position makes no progress for this
// long, or if the whole move takes longer than kMoveTimeoutMs.
constexpr uint32_t kStallTimeoutMs = 1500;  // [ms]
constexpr uint32_t kMoveTimeoutMs = 15000;  // [ms]
// Pause after every completed cell before sensing (chassis stops rocking).
constexpr uint32_t kPostMoveDelayMs = 150;  // [ms]

// --- Wall sensing (robot stopped, used for mapping) ------------------------
// A wall is present if the median of kSonarSamples pings is below this.
constexpr uint16_t kWallThresholdCm = 20;  // [cm]
// NewPing gives up after this distance, bounding each ping to
// ~(max + 1) * 57 us plus the sensor's start delay. 2 cells is plenty. [cm]
constexpr uint16_t kMappingMaxCm =
    static_cast<uint16_t>(2.0f * kCellSizeMm / 10.0f);  // 2 cells = 44 cm
constexpr uint8_t kSonarSamples = 3;  // [-] odd, pings per sensor (median)
// Silence between any two pings (round-robin over the sensors), so the echo
// of one sensor is not heard by the next. Raise towards 60 if noisy. [ms]
constexpr uint32_t kInterPingGapMs = 30;

// --- Optional side-wall centering (see features.h, README) -----------------
// During straight moves one side sensor is pinged per control period,
// alternating left/right, only where the map says a wall is present.
// Nominal sensor-to-wall distance with the robot centred in the cell. [cm]
constexpr float kCenteringNominalLeftCm = 6.0f;
constexpr float kCenteringNominalRightCm = 6.0f;
// Heading-error ticks added per cm of lateral offset. [ticks per cm]
constexpr float kCenteringGainTicksPerCm = 8.0f;
// Bound of the centering term added to the heading error. [ticks]
constexpr float kCenteringMaxTicks = 40.0f;
// Readings further than this from nominal are implausible (wall gap, post,
// angled echo) and ignored. [cm]
constexpr float kCenteringMaxDeviationCm = 5.0f;
// Ping range during moves: a normal ping blocks <= ~(20 + 1) * 57 us +
// ~0.5 ms start delay ~= 1.7 ms. Worst case (sensor dead or unplugged):
// NewPing waits up to its 5.8 ms start timeout + 2 echo windows ~= 8.2 ms,
// still inside the 10 ms period (config.h asserts both). [cm]
constexpr uint16_t kCenteringPingMaxCm = 20;
// A side sensor is pinged every kCenteringPingPeriods control periods,
// left and right offset by kCenteringPingPeriods (2: L, -, R, -, ...), so
// there are >= 20 ms between any two centering pings (no crosstalk). [-]
constexpr uint8_t kCenteringPingPeriods = 2;
// Low-pass on each side's centering term (1 = no filter). [-]
constexpr float kCenteringFilter = 0.5f;

// --- Exploration robustness ------------------------------------------------
// An already-known wall (or opening) is flipped only after this many
// consecutive contradicting observations; unknown -> known is immediate.
// 1 = latest reading wins. Range 1..4. Prevents livelock on readings that
// flicker around kWallThresholdCm. [-]
constexpr uint8_t kWallFlipObservations = 2;
// A sensor is declared failed (FAULT SENSOR_FAILURE) after this many
// consecutive SENSE passes in which it got NO echo on ANY sample although
// the map already has a wall right beside the robot on that side (e.g. the
// outer boundary). Plain "no echo" is normal in open corridors and is not
// counted; any echo resets the count. [passes]
constexpr uint8_t kSensorFailurePasses = 3;
// Exploration gives up (FAULT EXPLORATION_LIMIT) after more than
// kExplorationBudgetFactor * (rows * cols) cells driven. [-]
constexpr uint8_t kExplorationBudgetFactor = 4;

}  // namespace micromouse::config
