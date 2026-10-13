#include "control/motion_controller.h"

#include "config/config.h"
#include "control/encoder_health.h"
#include "control/heading_carry.h"
#include "control/learning.h"
#include "control/stiction.h"
#include "control/wheel_mixer.h"
#include "hal/log.h"
#include "hal/time.h"
#include "util/flash.h"
#include "util/math_utils.h"

namespace micromouse {

namespace {

namespace cfg = config;
namespace log = hal::log;
using config::LogLevel;
using util::loadFlash;

// The parameter sets live in flash: on the AVR a constant aggregate would
// otherwise be copied into RAM (.data) at start-up. loadFlash() copies one
// onto the stack for the duration of a move.
const MoveParams kForwardMove MM_FLASH = {
    {cfg::forward::kCruiseSpeed, cfg::forward::kCreepSpeed,
     cfg::forward::kAccel, cfg::forward::kDecel, cfg::forward::kCreepTicks,
     cfg::forward::kBrakeLeadTicks},
    MoveKind::kForward,
};

const MoveParams kTurnMove MM_FLASH = {
    {cfg::turn::kCruiseSpeed, cfg::turn::kCreepSpeed, cfg::turn::kAccel,
     cfg::turn::kDecel, cfg::turn::kCreepTicks, cfg::turn::kBrakeLeadTicks},
    MoveKind::kTurn,
};


constexpr float kMaxPwm = static_cast<float>(cfg::kMaxPwm);
constexpr int8_t kForward = 1;
constexpr int8_t kBackward = -1;
constexpr uint8_t kMaxQuarterTurns = 2;

/// Calibration diagnostics of one move (cruise-phase averages).
struct MoveStats {
  float cruisePwmSum = 0.0f;
  float cruiseSpeedSum = 0.0f;
  uint16_t cruiseSamples = 0;
  uint16_t overruns = 0;
};

void logMove(const int32_t target, const int32_t left, const int32_t right,
             const Status status, const bool settled, const MoveStats& stats) {
  if (isOk(status)) {
    log::fmt(F("move"));
  } else {
    log::fmt(F("move FAILED status=%u"), static_cast<unsigned int>(status));
  }
  log::fmt(F(" target=%l L=%l R=%l over=%l"), static_cast<long>(target),
           static_cast<long>(left), static_cast<long>(right),
           static_cast<long>((left + right) / 2 - target));
  if (stats.cruiseSamples > 0) {
    log::fmt(F(" cruisePwm=%2 cruiseV=%2"),
             static_cast<double>(stats.cruisePwmSum / stats.cruiseSamples),
             static_cast<double>(stats.cruiseSpeedSum / stats.cruiseSamples));
  }
  if (stats.overruns > 0) {
    log::fmt(F(" overruns=%u"), static_cast<unsigned int>(stats.overruns));
  }
  if (!settled) log::fmt(F(" settle=TIMEOUT"));
  log::println();
}

/// Learned brake lead [ticks, float] -> profile ticks.
int32_t roundTicks(const float ticks) {
  return static_cast<int32_t>(ticks + 0.5f);
}

}  // namespace

void MotionController::begin() {
  leftMotor_.begin();
  rightMotor_.begin();
  leftEncoder_.begin();
  rightEncoder_.begin();
  headingCarry_ = 0;
}

void MotionController::stop() const {
  leftMotor_.brake();
  rightMotor_.brake();
}

void MotionController::driveWheel(const Motor& motor, const float pwm,
                                  const int8_t direction) {
  // The magnitude never goes negative: a single-channel encoder counts up
  // whichever way the wheel turns, so a reversed wheel would make the
  // controller run away.
  const int16_t magnitude = util::clamp<int16_t>(
      static_cast<int16_t>(pwm + 0.5f), 0, Motor::kMaxDuty);
  motor.setDuty(static_cast<int16_t>(direction * magnitude));
}

bool MotionController::waitForStandstill() const {
  const uint32_t start = hal::nowMs();
  uint32_t lastChange = start;
  int32_t lastLeft = leftEncoder_.count();
  int32_t lastRight = rightEncoder_.count();

  while (true) {
    const uint32_t now = hal::nowMs();
    if (now - lastChange >= cfg::kSettleQuietMs) return true;
    if (now - start >= cfg::kSettleTimeoutMs) return false;
    const int32_t left = leftEncoder_.count();
    const int32_t right = rightEncoder_.count();
    if (left != lastLeft || right != lastRight) {
      lastLeft = left;
      lastRight = right;
      lastChange = hal::nowMs();
    }
  }
}

MoveReport MotionController::move(const int32_t targetTicks,
                                  const int8_t dirLeft, const int8_t dirRight,
                                  const MoveParams& params,
                                  const int32_t carriedHeading,
                                  const CenteringWalls* centering) {
  MoveReport result = {Status::kOk, true, targetTicks, 0, 0, carriedHeading,
                       0.0f};
  if (targetTicks <= 0) return result;

  // Motor model of each wheel for this kind of move (self-calibration plus
  // floor load, or the defaults).
  const WheelCalibration wheelL =
      calibration::wheelModel(calibration_, Wheel::kLeft, params.kind);
  const WheelCalibration wheelR =
      calibration::wheelModel(calibration_, Wheel::kRight, params.kind);

  leftEncoder_.reset();
  rightEncoder_.reset();
  headingPid_.reset();
  speedPid_.reset();
  centering_.reset();
  const bool centeringOn = cfg::kWallCenteringEnabled && centering != nullptr;
  if (centeringOn) centering_.setWalls(centering->current);
  const int32_t halfway = targetTicks / 2;

  MotionProfile profile(params.profile, targetTicks);
  // Carried heading error added per period while it is ramped in.
  const float carryStep = static_cast<float>(carriedHeading) /
                          cfg::kHeadingCarryRampPeriods;

  float measuredSpeed = 0.0f;
  int32_t prevTravelled = 0;
  PidClamp speedClamp = {false, false};
  PidClamp headingClamp = {false, false};
  uint8_t rampPeriods = 0;
  bool rolling = false;
  MoveStats stats;

  const uint32_t startMs = hal::nowMs();
  uint32_t lastProgressMs = startMs;
  uint32_t nextUs = hal::nowUs();

  while (true) {
    // Fixed-rate loop: wait for the start of the next control period.
    while (static_cast<int32_t>(hal::nowUs() - nextUs) < 0) {
    }
    nextUs += cfg::kControlPeriodUs;
    const uint32_t periodStartUs = hal::nowUs();
    if (static_cast<int32_t>(periodStartUs - nextUs) > 0) {
      // Overran by more than a period: restart the schedule so the next
      // period is a full one instead of starting immediately.
      nextUs = periodStartUs + cfg::kControlPeriodUs;
      ++stats.overruns;
    }

    result.left = leftEncoder_.count();
    result.right = rightEncoder_.count();

    // Stop on the AVERAGE distance of both wheels, both wheels at once.
    const int32_t travelled = (result.left + result.right) / 2;
    // Encoder heading integral (learning): heading x distance increment.
    result.headingIntegral +=
        static_cast<float>(carriedHeading + result.left - result.right) *
        static_cast<float>(travelled - prevTravelled);
    if (profile.brakePointReached(travelled)) {
      prevTravelled = travelled;
      break;
    }

    const uint32_t nowMs = hal::nowMs();
    if (travelled != prevTravelled) lastProgressMs = nowMs;
    if (nowMs - lastProgressMs > cfg::kStallTimeoutMs) {
      result.status = Status::kStall;
      prevTravelled = travelled;
      break;
    }
    if (nowMs - startMs > cfg::kMoveTimeoutMs) {
      result.status = Status::kMoveTimeout;
      prevTravelled = travelled;
      break;
    }
    // A silent encoder halves the average distance (the robot would drive
    // 2 cells) while the stall check above still sees progress.
    if (encoderPairFaulty(result.left, result.right,
                          cfg::kEncoderFaultMinTravelTicks,
                          cfg::kEncoderFaultMinRatio)) {
      result.status = Status::kEncoderFault;
      prevTravelled = travelled;
      break;
    }

    // Measured average speed [tpp], low-pass filtered.
    measuredSpeed +=
        cfg::kSpeedFilter *
        (static_cast<float>(travelled - prevTravelled) - measuredSpeed);
    prevTravelled = travelled;

    const float targetSpeed = profile.next(travelled);

    // Forward speed: per-wheel stiction offset + per-wheel feed-forward +
    // one PI on the average wheel speed. The full breakaway offset applies
    // only at rest; once rolling (with hysteresis) the fitted intercept lets
    // the PI actually control the creep speed.
    rolling = updateRolling(rolling, measuredSpeed, cfg::kRollingEnterTpp,
                            cfg::kRollingExitTpp);
    const float offsetL =
        stictionPwm(rolling, wheelL.staticPwm, wheelL.ffOffsetPwm);
    const float offsetR =
        stictionPwm(rolling, wheelR.staticPwm, wheelR.ffOffsetPwm);
    const float pi = speedPid_.update(targetSpeed - measuredSpeed, speedClamp);
    float baseL = offsetL + wheelL.ffSlopePwmPerTpp * targetSpeed + pi;
    float baseR = offsetR + wheelR.ffSlopePwmPerTpp * targetSpeed + pi;
    bool floored = false;
    if (baseL < offsetL) {
      baseL = offsetL;
      floored = true;
    }
    if (baseR < offsetR) {
      baseR = offsetR;
      floored = true;
    }

    if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
      if (targetSpeed >= params.profile.cruiseSpeed) {
        stats.cruisePwmSum += 0.5f * (baseL + baseR);
        stats.cruiseSpeedSum += measuredSpeed;
        ++stats.cruiseSamples;
      }
    }

    // Heading hold: positive error = left wheel ahead -> slow left, speed
    // up right. Magnitudes are used, so the same law works for spins. The
    // carried error is ramped in rather than applied as a step.
    if (rampPeriods < cfg::kHeadingCarryRampPeriods) ++rampPeriods;
    const float carry = carryStep * rampPeriods;
    float headingError =
        carry + static_cast<float>(result.left - result.right);
    if (centeringOn) {
      if (travelled >= halfway) centering_.setWalls(centering->next);
      headingError += centering_.correctionTicks();
    }
    const float correction = headingPid_.update(headingError, headingClamp);

    // Per-wheel floor: never below breakaway at rest, kinetic once rolling.
    const WheelMix mix = mixWheels(
        baseL, baseR, correction,
        stictionPwm(rolling, wheelL.staticPwm, wheelL.kineticPwm) *
            cfg::kWheelFloorFraction,
        stictionPwm(rolling, wheelR.staticPwm, wheelR.kineticPwm) *
            cfg::kWheelFloorFraction,
        kMaxPwm);
    speedClamp = mix.speedClamp;
    speedClamp.low = speedClamp.low || floored;
    headingClamp = mix.headingClamp;

    driveWheel(leftMotor_, mix.left, dirLeft);
    driveWheel(rightMotor_, mix.right, dirRight);

    // Optional wall centering: one side ping per period, AFTER the outputs
    // are written, so the ping never delays actuation. Its reading is used
    // from the next period on.
    if (centeringOn) {
      const Side side = centering_.nextSide();
      if (side != Side::kNone) {
        const UltrasonicSensor& sonar =
            side == Side::kLeft ? leftSonar_ : rightSonar_;
        const uint16_t echoUs = sonar.pingUs(cfg::kCenteringPingMaxCm);
        centering_.addReading(side,
                              static_cast<float>(echoUs) / cfg::kSonarUsPerCm);
      }
    }
  }

  stop();
  if constexpr (cfg::logEnabled(LogLevel::kError)) {
    // Printed after braking (Serial may block). Which wheel is silent:
    if (result.status == Status::kEncoderFault) {
      log::fmtln(F("ENCODER FAULT L=%l R=%l"),
                 static_cast<long>(result.left),
                 static_cast<long>(result.right));
    }
  }
  result.settled = waitForStandstill();
  result.left = leftEncoder_.count();
  result.right = rightEncoder_.count();
  // The coast after braking also moves (and may turn) the robot.
  const int32_t finalTravelled = (result.left + result.right) / 2;
  result.headingIntegral +=
      static_cast<float>(carriedHeading + result.left - result.right) *
      static_cast<float>(finalTravelled - prevTravelled);

  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    logMove(targetTicks, result.left, result.right, result.status,
            result.settled, stats);
  }
  return result;
}

void MotionController::logCarry(const bool settled) const {
  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    log::fmtln(F("H=%l%s"), static_cast<long>(headingCarry_),
               settled ? F("") : F(" (unsettled)"));
  }
}

Status MotionController::driveForward(const CenteringWalls& walls,
                                      MoveReport& report) {
  // Start with the heading error left over from previous moves so it is
  // corrected instead of accumulating cell after cell.
  MoveParams params = loadFlash(kForwardMove);
  params.profile.brakeLeadTicks =
      roundTicks(calibration_.learned.brakeLeadForward);
  report = move(calibration::ticksPerCell(calibration_), kForward, kForward,
                params, headingCarry_, &walls);
  // Also after a settle timeout: the move has already acted on the old
  // carry, so the counts at the timeout are the best estimate there is.
  headingCarry_ =
      heading::carryAfterForward(headingCarry_, report.left, report.right,
                                 cfg::kHeadingCarryLimit);
  logCarry(report.settled);
  return report.status;
}

Status MotionController::turn(const TurnDirection direction,
                              const uint8_t quarterTurns, MoveReport& report) {
  report = MoveReport{Status::kOk, true, 0, 0, 0, headingCarry_, 0.0f};
  if (quarterTurns == 0) return Status::kOk;
  if (quarterTurns > kMaxQuarterTurns) {
    report.status = Status::kInvalidArgument;
    return report.status;
  }

  const int32_t nominal = static_cast<int32_t>(quarterTurns) *
                          calibration::ticksPer90(calibration_);
  const int8_t sign = static_cast<int8_t>(direction);
  const int32_t target = heading::turnTarget(nominal, sign, headingCarry_);

  MoveParams params = loadFlash(kTurnMove);
  params.profile.brakeLeadTicks =
      roundTicks(calibration_.learned.brakeLeadTurn);
  report = move(target, sign, static_cast<int8_t>(-sign), params, 0, nullptr);
  report.carryBefore = headingCarry_;
  headingCarry_ = heading::carryAfterTurn(headingCarry_, sign, report.left,
                                         report.right, nominal,
                                         cfg::kHeadingCarryLimit);
  logCarry(report.settled);
  return report.status;
}

Status MotionController::adjustStraight(const int32_t signedTicks,
                                        MoveReport& report) {
  const int8_t dir = signedTicks >= 0 ? kForward : kBackward;
  const int32_t ticks = signedTicks >= 0 ? signedTicks : -signedTicks;
  // The whole adjustment at creep speed (cruise = creep). It brakes from
  // creep speed like a cell move, so the learned forward lead applies, but
  // at most half the move so that a tiny correction is not swallowed.
  MoveParams params = loadFlash(kForwardMove);
  params.profile.cruiseSpeed = params.profile.creepSpeed;
  params.profile.creepTicks = 0;
  params.profile.brakeLeadTicks = learning::alignmentBrakeLead(
      ticks, roundTicks(calibration_.learned.brakeLeadForward));
  // Carry 0 as the setpoint: equal L/R, so the adjustment itself does not
  // rotate the robot. The forward carry ramp would steer with the wrong
  // sign in reverse, and the next cell move corrects the carry anyway.
  report = move(ticks, dir, dir, params, 0, nullptr);
  report.carryBefore = headingCarry_;
  headingCarry_ = heading::carryAfterStraight(
      headingCarry_, dir, report.left, report.right, cfg::kHeadingCarryLimit);
  logCarry(report.settled);
  return report.status;
}

}  // namespace micromouse
