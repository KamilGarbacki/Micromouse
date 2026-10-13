// All tunables of the robot, with units. Normally none of them needs to be
// touched: the motor model comes from the self-calibration ('c') and the
// tick counts / brake leads are learned while driving (README
// "Calibration"). The README troubleshooting table says which knob to turn
// for which symptom.
//
// Control-loop speeds are in "ticks per control period" (tpp): with a 10 ms
// period, 1 tpp = 100 encoder ticks per second (average of both wheels),
// i.e. 21.3 mm/s of wheel speed at 960 ticks per wheel revolution.
//
// UNITS POLICY: every tunable is given in PHYSICAL units (mm, mm/s, mm/s^2,
// degrees of heading, PWM per mm/s, PWM per degree) and converted to
// encoder units at compile time with the helpers below, so the encoder
// resolution (geometry.h kEncoderPulsesPerMotorRev) enters in ONE place.
// Deliberate exceptions, in ticks because they are ABOUT tick quantisation:
// the stiction hysteresis (kRollingEnterTpp / kRollingExitTpp) and the
// self-calibration motion detection (kSelfCalMovingTicks).
//
// The physical values of the gains and thresholds equal what the robot
// actually ran with (the tick numbers below at the TRUE 960 ticks per
// revolution): the encoder hardware never changed, only the configured
// resolution, so a gain in "PWM per tick" always meant the same PWM per
// physical tick. The profile speeds / distances keep their mm values (with
// the wrong 16 pulses the robot physically drove them 2x fast and 2x far).
#pragma once

#include <stdint.h>

#include "config/geometry.h"

namespace micromouse::config {

// --- Control loop ----------------------------------------------------------
// The move loop runs at a FIXED period, so every gain means the same thing
// regardless of how long an iteration takes.
constexpr uint32_t kControlPeriodUs = 10000;  // [us] 100 Hz
constexpr float kControlPeriodS = kControlPeriodUs / 1000000.0f;  // [s]

// --- Unit conversions (physical -> encoder units) ----------------------------
/// [mm/s] of wheel speed per tpp (21.27 at 960 ticks per revolution).
constexpr float kMmPerSPerTpp = kMmPerTick / kControlPeriodS;
/// [deg] of heading per tick of left-minus-right wheel difference: a
/// difference of d mm of wheel travel turns the robot by d / wheelbase rad
/// (0.0812 deg at 960 ticks per revolution).
constexpr float kDegPerTickDiff = kMmPerTick / kWheelbaseMm * (180.0f / kPi);

/// [tpp] for a wheel speed in mm/s.
constexpr float tppFromMmPerS(const float mmPerS) {
  return mmPerS / kMmPerSPerTpp;
}
/// [tpp per period] for an acceleration in mm/s^2.
constexpr float tppPerPeriodFromMmPerS2(const float mmPerS2) {
  return mmPerS2 * kControlPeriodS / kMmPerSPerTpp;
}
/// [ticks] for a wheel travel in mm (rounded).
constexpr int32_t ticksFromMm(const float mm) {
  return static_cast<int32_t>(mm / kMmPerTick + 0.5f);
}
/// [L - R ticks] for a heading angle in degrees (unrounded).
constexpr float tickDiffFromDeg(const float deg) {
  return deg / kDegPerTickDiff;
}
/// A gain per (mm/s) of speed error -> per tpp.
constexpr float perTppFromPerMmPerS(const float gain) {
  return gain * kMmPerSPerTpp;
}
/// A gain per degree of heading error -> per L - R tick.
constexpr float perTickDiffFromPerDeg(const float gain) {
  return gain * kDegPerTickDiff;
}

// --- Motor model -----------------------------------------------------------
constexpr int16_t kMaxPwm = 255;  // [PWM] analogWrite() full scale
// Per-wheel feed-forward (control/calibration.h): for each wheel
//   PWM = offset + slope * targetSpeed + speed PI,
// where offset is the wheel's staticPwm while the robot is at rest and its
// fitted ffOffsetPwm once rolling. The per-wheel values come from the motor
// self-calibration ('c', robot lifted) stored in EEPROM; they replace the
// old LEFT/RIGHT_PWM_TRIM (a wheel needing less PWM simply has a smaller
// fit). Without a stored self-calibration both wheels use the defaults
// below (the phase-2 hand-tuned values, so behaviour is unchanged).
// The PWM values do not depend on the encoder. The slope is a property of
// the motor: 0.247 PWM per mm/s = 5.25 PWM per (real, 960-per-rev) tpp,
// the value the robot ran with. It must NOT be doubled for the resolution
// fix: the tick did not change, only its configured length.
namespace forward {
constexpr float kDefaultStaticPwm = 35.0f;   // [PWM] breakaway from rest
constexpr float kDefaultKineticPwm = 25.0f;  // [PWM] keeps it rolling
constexpr float kDefaultFfOffsetPwm = 25.0f;  // [PWM] FF intercept rolling
constexpr float kDefaultFfSlopePerMmPerS = 0.247f;  // [PWM per mm/s]
constexpr float kDefaultFfSlope =
    perTppFromPerMmPerS(kDefaultFfSlopePerMmPerS);  // [PWM per tpp] (5.25)
}  // namespace forward
namespace turn {
constexpr float kDefaultStaticPwm = 30.0f;
constexpr float kDefaultKineticPwm = 22.0f;
constexpr float kDefaultFfOffsetPwm = 22.0f;
constexpr float kDefaultFfSlopePerMmPerS = 0.247f;
constexpr float kDefaultFfSlope =
    perTppFromPerMmPerS(kDefaultFfSlopePerMmPerS);
}  // namespace turn
// Floor load on top of the LIFTED self-calibration (applied only when a
// self-calibration is stored): offset' = offset * scale + offsetPwm for
// staticPwm, kineticPwm and the FF intercept; the slope is unchanged.
// Driving straight on the floor adds little friction (the speed PI covers
// it). Spinning in place scrubs the tyres sideways, a roughly constant
// extra torque, hence the extra PWM for turns. Raise kTurnLoadOffsetPwm if
// turns start hesitantly or end in STALL; lower it if they jerk.
constexpr float kForwardLoadScale = 1.0f;      // [-]
constexpr float kForwardLoadOffsetPwm = 0.0f;  // [PWM]
constexpr float kTurnLoadScale = 1.0f;         // [-]
constexpr float kTurnLoadOffsetPwm = 8.0f;     // [PWM]
// Stiction hysteresis on the filtered average speed: the robot counts as
// rolling (kinetic offset) once it reaches kRollingEnterTpp, and as stopped
// (static offset) again only below kRollingExitTpp, so a single tick of gear
// backlash cannot toggle the offset. In ticks on purpose: it is about tick
// quantisation (enter = the first period with a tick, filtered; 1 tpp =
// 21 mm/s at 960 ticks per revolution - unchanged physically by the
// resolution fix). config.h asserts that both creep speeds stay >= 1.2x
// the enter threshold (an alignment move runs entirely at creep). [tpp]
constexpr float kRollingEnterTpp = 1.0f;
constexpr float kRollingExitTpp = 0.25f;
// Neither wheel is commanded below kWheelFloorFraction * (its staticPwm at
// rest, its kineticPwm while rolling); the heading correction is shifted
// onto the other wheel instead (prevents one wheel stalling into short brake
// and stick-slipping, and keeps both wheels above breakaway at the start).
constexpr float kWheelFloorFraction = 1.0f;  // [-]

// --- Forward speed PI (average measured speed -> PWM) ----------------------
// Physical gains (the robot's per-tick values in brackets, at 960 ticks
// per revolution: unchanged by the resolution fix, see the units policy).
constexpr float kSpeedKpPerMmPerS = 0.141f;  // [PWM per mm/s] (3.0 per tpp)
constexpr float kSpeedKiPerMm = 0.94f;       // [PWM per mm] (20 per tpp*s)
constexpr float kSpeedKp = perTppFromPerMmPerS(kSpeedKpPerMmPerS);
constexpr float kSpeedKi = perTppFromPerMmPerS(kSpeedKiPerMm);
constexpr float kSpeedILimit = 60.0f;     // [PWM] max integral share
constexpr float kSpeedOutLimit = 80.0f;   // [PWM] max total PI output
// Low-pass on the measured speed (tick quantisation): 1 = no filter. At
// the slowest creep (turn, 1.41 tpp) the per-period count alternates 1/2;
// this EMA keeps the ripple at about +-0.25 tpp (+-0.75 PWM through Kp),
// with a zero-mean error for the integrator. [-]
constexpr float kSpeedFilter = 0.5f;

// --- Heading hold (PID on leftTicks - rightTicks) --------------------------
// error = leftTicks - rightTicks (+ carried heading, + optional centering)
// [ticks]; the output is PWM subtracted from the left wheel and added to the
// right one. Gains per DEGREE of heading (straight move: L - R ticks x
// kDegPerTickDiff); per-tick values at 960 ticks per revolution in brackets.
constexpr float kHeadingKpPerDeg = 7.4f;   // [PWM per deg] (0.6 per tick)
constexpr float kHeadingKiPerDeg = 18.5f;  // [PWM per deg*s] (1.5)
constexpr float kHeadingKdPerDeg = 0.0f;   // [PWM per deg/s] noisy
constexpr float kHeadingKp = perTickDiffFromPerDeg(kHeadingKpPerDeg);
constexpr float kHeadingKi = perTickDiffFromPerDeg(kHeadingKiPerDeg);
constexpr float kHeadingKd = perTickDiffFromPerDeg(kHeadingKdPerDeg);
constexpr float kHeadingILimit = 20.0f;     // [PWM] max integral share
constexpr float kHeadingOutLimit = 30.0f;   // [PWM] max total correction
// Residual heading error carried into the next move is clamped to this, so
// one bad reading (wheel slip) cannot cause a big swerve. [deg]
constexpr float kHeadingCarryLimitDeg = 5.0f;
/// [L - R ticks] (62 at 960 ticks per revolution).
constexpr int32_t kHeadingCarryLimit =
    static_cast<int32_t>(tickDiffFromDeg(kHeadingCarryLimitDeg) + 0.5f);
// The carried heading error is ramped into the setpoint over this many
// control periods instead of being applied as a step (avoids saturating the
// heading loop at the start of every move). [periods]
constexpr uint8_t kHeadingCarryRampPeriods = 10;

// --- Velocity profiles -----------------------------------------------------
// Speed ramps up at `accel`, cruises, slows down at constant `decel` so that
// it reaches creepSpeed `creep` before the brake point, creeps, then brakes
// `brakeLead` before the target (the robot coasts the rest).
//
// Given in PHYSICAL units (mm, mm/s, mm/s^2 of wheel travel) and converted
// with the geometry. Tick values at 960 ticks per revolution (0.2127 mm per
// tick) in brackets. NOTE: these mm values were set when the resolution was
// wrongly believed to be 1920; with that error the real robot drove them
// 2x fast as well as 2x far (cruise ~170 mm/s). Now they are what they say:
// expect the robot to move at HALF the speed it did before the fix. The
// brake lead here is only the start value: it is learned per move kind at
// run time (README "Calibration") within [0, kMaxBrakeLeadMm].

namespace forward {
constexpr float kCruiseMmPerS = 85.0f;   // [mm/s] (4.0 tpp)
constexpr float kCreepMmPerS = 32.0f;    // [mm/s] (1.5 tpp) final approach
constexpr float kAccelMmPerS2 = 425.0f;  // [mm/s^2] (0.2 tpp/period)
constexpr float kDecelMmPerS2 = 320.0f;  // [mm/s^2] (0.15 tpp/period)
constexpr float kCreepMm = 12.8f;        // [mm] (60 ticks) at creep speed
constexpr float kBrakeLeadMm = 1.6f;     // [mm] (8 ticks) start value
constexpr float kMaxBrakeLeadMm = 5.0f;  // [mm] (24 ticks) learning bound

constexpr float kCruiseSpeed = tppFromMmPerS(kCruiseMmPerS);       // [tpp]
constexpr float kCreepSpeed = tppFromMmPerS(kCreepMmPerS);         // [tpp]
constexpr float kAccel = tppPerPeriodFromMmPerS2(kAccelMmPerS2);  // [tpp/p]
constexpr float kDecel = tppPerPeriodFromMmPerS2(kDecelMmPerS2);  // [tpp/p]
constexpr int32_t kCreepTicks = ticksFromMm(kCreepMm);
constexpr int32_t kBrakeLeadTicks = ticksFromMm(kBrakeLeadMm);
constexpr int32_t kMaxBrakeLeadTicks = ticksFromMm(kMaxBrakeLeadMm);
}  // namespace forward

namespace turn {
// Wheel speeds / distances of the spin (each wheel's own travel).
constexpr float kCruiseMmPerS = 64.0f;   // [mm/s] (3.0 tpp)
// 30 (was 26.6 = 1.25 tpp): margin over the rolling threshold (1 tpp) and
// over breakaway at turn creep on the default motor model.
constexpr float kCreepMmPerS = 30.0f;    // [mm/s] (1.41 tpp, see kSpeedFilter)
constexpr float kAccelMmPerS2 = 320.0f;  // [mm/s^2] (0.15 tpp/period)
constexpr float kDecelMmPerS2 = 213.0f;  // [mm/s^2] (0.1 tpp/period)
constexpr float kCreepMm = 6.4f;         // [mm] (30 ticks)
constexpr float kBrakeLeadMm = 1.1f;     // [mm] (5 ticks) start value
constexpr float kMaxBrakeLeadMm = 3.0f;  // [mm] (14 ticks) learning bound

constexpr float kCruiseSpeed = tppFromMmPerS(kCruiseMmPerS);
constexpr float kCreepSpeed = tppFromMmPerS(kCreepMmPerS);
constexpr float kAccel = tppPerPeriodFromMmPerS2(kAccelMmPerS2);
constexpr float kDecel = tppPerPeriodFromMmPerS2(kDecelMmPerS2);
constexpr int32_t kCreepTicks = ticksFromMm(kCreepMm);
constexpr int32_t kBrakeLeadTicks = ticksFromMm(kBrakeLeadMm);
constexpr int32_t kMaxBrakeLeadTicks = ticksFromMm(kMaxBrakeLeadMm);
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
// Encoder health: a dead / unplugged encoder (or one with a broken magnet)
// halves the AVERAGE distance and the robot drives 2 cells per move, just
// like a wrong kEncoderPulsesPerMotorRev - while the stall check still sees
// progress. Once either wheel has counted more than
// kEncoderFaultMinTravelMm, the other must have counted at least
// kEncoderFaultMinRatio of it, else the move stops with ENCODER_FAULT.
// A healthy pair stays near 1 (the heading loop keeps L ~ R; at the start
// one wheel may break away a few ticks earlier). [mm], [-]
constexpr float kEncoderFaultMinTravelMm = 20.0f;
constexpr float kEncoderFaultMinRatio = 0.3f;
/// [ticks] (94 at 960 ticks per revolution).
constexpr int32_t kEncoderFaultMinTravelTicks =
    ticksFromMm(kEncoderFaultMinTravelMm);
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

// NewPing's fixed round-trip time per cm (US_ROUNDTRIP_CM, ~32 degC). Only
// NewPing's range limit uses it. [us per cm]
constexpr uint32_t kNewPingUsPerCm = 57;
// Speed of sound -> echo time per cm. The ultrasonic sensor is the ruler of
// the ticks-per-cell learning, so its scale matters: c = 331.3 + 0.606 * T
// [m/s], i.e. about +0.17 % per degC. Set the typical room temperature of
// the maze. [degC]
constexpr float kAirTemperatureC = 20.0f;
/// [us per cm] round-trip echo time (58.2 at 20 degC).
constexpr float kSonarUsPerCm = 20000.0f / (331.3f + 0.606f * kAirTemperatureC);
// Precise ranging while stopped (front-wall alignment and online learning):
// median of this many pings per sensor, round-robin with kInterPingGapMs.
constexpr uint8_t kRangingSamples = 3;  // [-] odd

// --- Optional side-wall centering (see features.h, README) -----------------
// During straight moves one side sensor is pinged per control period,
// alternating left/right, only where the map says a wall is present.
// Nominal sensor-to-wall distance with the robot centred in the cell. [cm]
constexpr float kCenteringNominalLeftCm = 6.0f;
constexpr float kCenteringNominalRightCm = 6.0f;
// Heading setpoint offset per cm of lateral offset. [deg per cm]
constexpr float kCenteringGainDegPerCm = 0.65f;
// Bound of the centering term added to the heading error. [deg]
constexpr float kCenteringMaxDeg = 3.25f;
/// [L - R ticks per cm] (8.0 at 960 ticks per revolution).
constexpr float kCenteringGainTicksPerCm =
    tickDiffFromDeg(kCenteringGainDegPerCm);
/// [L - R ticks] (40 at 960 ticks per revolution).
constexpr float kCenteringMaxTicks = tickDiffFromDeg(kCenteringMaxDeg);
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

// --- Front-wall alignment (features.h: MM_FRONT_ALIGN) ---------------------
// After a straight move with a wall ahead, the robot creeps forward or back
// until the front reading equals the distance of a centred robot. Removes
// accumulated longitudinal error.
/// [mm] front-sensor reading of a robot centred in a cell of `pitchMm`.
constexpr float frontWallNominalMm(const float pitchMm) {
  return pitchMm / 2.0f - kWallThicknessMm / 2.0f - kFrontSensorToCenterMm;
}
/// The default (with kCellSizeMm); at run time the learned pitch is used.
constexpr float kFrontWallNominalMm = frontWallNominalMm(kCellSizeMm);
constexpr float kAlignDeadbandMm = 3.0f;        // [mm] closer: no move
constexpr float kAlignMaxCorrectionMm = 30.0f;  // [mm] max one adjustment
// A reading further than this from nominal is not "the wall right ahead"
// (or is implausible): no alignment. [mm]
constexpr float kAlignMaxErrorMm = 60.0f;
constexpr uint8_t kAlignMaxIterations = 2;  // [-] measure-adjust rounds
// The alignment moves brake with the learned forward lead, capped at half
// the move (learning::alignmentBrakeLead), and the position is re-measured
// after every adjustment.

// --- Online learning (features.h: MM_LEARNING) -----------------------------
// See README "Calibration" for what is learned and how it is gated. Every
// learned value uses the same robust estimator (control/learning.h): before
// minSamples it only checks the hard bounds and seeds with the MEDIAN of the
// first minSamples samples; afterwards samples beyond outlierFraction of the
// estimate are rejected and the rest enter an EMA with the gain; after
// kLearnReacquireCount consecutive outliers that agree with each other the
// estimate is re-seeded from them (hardware changed / bad early estimate).
constexpr uint8_t kLearnReacquireCount = 4;  // [-] <= learning::kLearnBuffer
// Wheel scale (mm per tick) from the front-distance change of a straight
// move towards a wall.
constexpr float kScaleLearnOutlierFraction = 0.04f;
constexpr float kScaleLearnGain = 0.15f;
constexpr uint16_t kScaleLearnMinSamples = 3;
// Cell pitch P and the front-offset error delta (configured minus true
// wall thickness / 2 + sensor offset) from the front readings after cells
// driven from a known reference (an alignment, or the start cell); see
// control/learning.h. Joint least squares with forgetting:
constexpr float kPitchLearnOutlierFraction = 0.03f;  // gate per cell
constexpr float kPitchJointForget = 0.9f;  // [-] per observation
// Normalised determinant 1 - corr^2 of the normal equations below which P
// and delta cannot be separated: all observations have (nearly) the same
// ratio a / b - a different n is not enough (an aligned run of 2k cells is
// collinear with a start run of k cells). [-]
constexpr float kPitchMinDecorrelation = 0.02f;
// Accepted observations since a restart before the joint solution is put
// in force (two rows alone give P only to ~4 mm). [-]
constexpr uint8_t kPitchMinRows = 3;
constexpr float kMaxWallOffsetMm = 15.0f;  // [mm] bound of |delta|
// Until the joint solve is conditioned, a PROVISIONAL pitch (delta = 0
// assumed, robust estimator) is learned from runs of at least this many
// cells, where a wrong delta matters least.
constexpr uint8_t kPitchProvisionalCells = 2;
constexpr float kPitchLearnGain = 0.15f;
constexpr uint16_t kPitchLearnMinSamples = 3;
// Front readings usable as a ruler: [min, max] distance. [mm]
constexpr float kLearnFrontMinMm = 30.0f;
constexpr float kLearnFrontMaxMm = 400.0f;
// The reading at the wall right ahead after a move only needs the sensor's
// minimum range: a robot that overdrives (pitch smaller than believed)
// ends closer than nominal. [mm]
constexpr float kLearnFrontNearMinMm = 20.0f;
// Ticks per 90 deg from side-wall drift after a turn.
constexpr float kTurnLearnOutlierFraction = 0.10f;
constexpr float kTurnLearnGain = 0.10f;
constexpr uint16_t kTurnLearnMinSamples = 4;
// Straight-line curvature (wheel diameter mismatch) [rad per mm], estimated
// from drift measurements with no turn in between and subtracted from the
// turn samples: EMA gain, samples before use, plausibility bound.
constexpr float kCurveLearnGain = 0.2f;
constexpr uint8_t kCurveLearnMinSamples = 3;
constexpr float kMaxCurveRadPerMm = 2.0e-4f;  // 2.5 deg per 220 mm cell
// Side readings usable for the drift measurement. [mm]
constexpr float kLearnSideMinMm = 20.0f;
constexpr float kLearnSideMaxMm = 150.0f;
// |d_end - d_start| above this: wall gap / post / bad echo. [mm]
constexpr float kLearnMaxSideDriftMm = 25.0f;
// The encoder-visible heading (carry) at the start of the straight move
// must be below this, so that errors in its conversion stay small. [deg]
constexpr float kTurnLearnMaxCarryDeg = 3.25f;
/// [L - R ticks] (40 at 960 ticks per revolution).
constexpr int32_t kTurnLearnMaxCarryTicks =
    static_cast<int32_t>(tickDiffFromDeg(kTurnLearnMaxCarryDeg) + 0.5f);
// Brake lead adaptation: lead += gain * over after every settled move.
constexpr float kBrakeLeadGain = 0.2f;  // [-]
// |over| above this is a slip / bump, not a brake-lead error (3x the
// largest learnable lead, so it follows the mm bounds above). [ticks]
constexpr int32_t kForwardBrakeMaxOverTicks = 3 * forward::kMaxBrakeLeadTicks;
constexpr int32_t kTurnBrakeMaxOverTicks = 3 * turn::kMaxBrakeLeadTicks;

// --- Calibration store (EEPROM) ---------------------------------------------
// Two independent records, each with its own magic, version, key and CRC:
// the motor model (written by 'c') and the learned geometry. [byte]
constexpr uint16_t kMotorRecordAddress = 0;
constexpr uint16_t kLearnedRecordAddress = 64;
// Wear policy: the learned record is written on GOAL_REACHED and after every
// kLearnSaveEveryUpdates accepted scale/pitch/turn samples, never per move.
// The write only touches bytes that changed. [-]
constexpr uint8_t kLearnSaveEveryUpdates = 8;

// --- Startup menu ------------------------------------------------------------
// Wait this long for a mode key at boot, then RUN. [ms]
constexpr uint32_t kModeSelectTimeoutMs = 3000;
// Boot: print the previous run's log (README "Run log"), then wait this
// long before overwriting it with a new run (START RUN), so the user has
// time to read it or lift the robot. 0 = start immediately. [ms]
constexpr uint32_t kPreRunDelayMs = 2000;
// Encoder check: print interval of the L/R counts. [ms]
constexpr uint32_t kEncoderCheckPrintMs = 200;

// --- Motor self-calibration ('c', robot lifted) -----------------------------
// staticPwm: PWM ramps up from kSelfCalStartPwm by 1 every kSelfCalStepMs
// until at least kSelfCalMovingTicks ticks appear in one step.
// Moving = at least kSelfCalMovingTicks in kSelfCalMovingSteps consecutive
// steps (a single burst can be gearbox backlash). In ticks on purpose (it
// is about counting enough edges): 3 ticks = 1.1 deg of wheel rotation =
// 0.64 mm per 100 ms (6 mm/s), physically the same as before the
// resolution fix, still well above backlash and far below any speed the
// robot drives at.
constexpr int16_t kSelfCalStartPwm = 1;            // [PWM]
constexpr int16_t kSelfCalMaxStartPwm = 150;       // [PWM] give up above
constexpr uint32_t kSelfCalStepMs = 100;           // [ms]
constexpr int32_t kSelfCalMovingTicks = 3;         // [ticks] per step
constexpr uint8_t kSelfCalMovingSteps = 2;         // [-]
// Feed-forward line: kSelfCalLevels steady-state levels from
// staticPwm + kSelfCalLowAboveStaticPwm up to kSelfCalTopPwm.
constexpr uint8_t kSelfCalLevels = 5;              // [-] 3..8
constexpr int16_t kSelfCalLowAboveStaticPwm = 10;  // [PWM]
constexpr int16_t kSelfCalTopPwm = 160;            // [PWM]
constexpr uint32_t kSelfCalSettleMs = 700;         // [ms] per level
constexpr uint32_t kSelfCalMeasureMs = 1000;       // [ms] per level
constexpr float kSelfCalMinR2 = 0.95f;             // [-] fit quality
// kineticPwm: from the lowest level the PWM ramps down by 1 every
// kSelfCalKineticStepMs until fewer than kSelfCalMovingTicks appear.
constexpr uint32_t kSelfCalKineticStepMs = 150;    // [ms]
// Margins added to the measured thresholds (robustness). [PWM]
constexpr float kSelfCalStaticMarginPwm = 3.0f;
constexpr float kSelfCalKineticMarginPwm = 2.0f;
// Plausible ranges of a stored wheel model (validation). [PWM per mm/s]
// (0.43 .. 64 PWM per tpp at 960 ticks per revolution)
constexpr float kMinFfSlopePerMmPerS = 0.02f;
constexpr float kMaxFfSlopePerMmPerS = 3.0f;
constexpr float kMinFfSlope = perTppFromPerMmPerS(kMinFfSlopePerMmPerS);
constexpr float kMaxFfSlope = perTppFromPerMmPerS(kMaxFfSlopePerMmPerS);
constexpr float kMinFfOffsetPwm = -60.0f;  // [PWM]

}  // namespace micromouse::config
