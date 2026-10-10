#include "control/motion_controller.h"

#include <avr/pgmspace.h>

#include "config/config.h"
#include "control/heading_carry.h"
#include "control/stiction.h"
#include "control/wheel_mixer.h"
#include "hal/log.h"
#include "hal/time.h"
#include "util/math_utils.h"

namespace micromouse {

namespace {

namespace cfg = config;
namespace log = hal::log;
using config::LogLevel;

constexpr PidGains kHeadingGains = {cfg::kHeadingKp, cfg::kHeadingKi,
                                    cfg::kHeadingKd, cfg::kHeadingILimit,
                                    cfg::kHeadingOutLimit};
constexpr PidGains kSpeedGains = {cfg::kSpeedKp, cfg::kSpeedKi, 0.0f,
                                  cfg::kSpeedILimit, cfg::kSpeedOutLimit};

// The parameter sets live in flash: on the AVR a constant aggregate would
// otherwise be copied into RAM (.data) at start-up. load() copies one onto
// the stack for the duration of a move.
const MoveParams kForwardMove PROGMEM = {
    {cfg::forward::kCruiseSpeed, cfg::forward::kCreepSpeed,
     cfg::forward::kAccel, cfg::forward::kDecel, cfg::forward::kCreepTicks,
     cfg::forward::kBrakeLeadTicks},
    cfg::forward::kStaticPwm,
    cfg::forward::kKineticPwm,
};

const MoveParams kTurnMove PROGMEM = {
    {cfg::turn::kCruiseSpeed, cfg::turn::kCreepSpeed, cfg::turn::kAccel,
     cfg::turn::kDecel, cfg::turn::kCreepTicks, cfg::turn::kBrakeLeadTicks},
    cfg::turn::kStaticPwm,
    cfg::turn::kKineticPwm,
};

const CenteringParams kCenteringParams PROGMEM = {
    cfg::kCenteringNominalLeftCm, cfg::kCenteringNominalRightCm,
    cfg::kCenteringGainTicksPerCm, cfg::kCenteringMaxTicks,
    cfg::kCenteringMaxDeviationCm, cfg::kCenteringFilter,
    cfg::kCenteringPingPeriods};

template <typename T>
T load(const T& flashObject) {
  T copy;
  memcpy_P(&copy, &flashObject, sizeof(T));
  return copy;
}

constexpr float kMaxPwm = static_cast<float>(cfg::kMaxPwm);
constexpr int8_t kForward = 1;
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
    log::print(F("move target="));
  } else {
    log::print(F("move FAILED status="));
    log::print(static_cast<unsigned int>(status));
    log::print(F(" target="));
  }
  log::print(target);
  log::print(F(" L="));
  log::print(left);
  log::print(F(" R="));
  log::print(right);
  log::print(F(" over="));
  log::print((left + right) / 2 - target);
  if (stats.cruiseSamples > 0) {
    log::print(F(" cruisePwm="));
    log::print(stats.cruisePwmSum / stats.cruiseSamples);
    log::print(F(" cruiseV="));
    log::print(stats.cruiseSpeedSum / stats.cruiseSamples);
  }
  if (stats.overruns > 0) {
    log::print(F(" overruns="));
    log::print(static_cast<unsigned int>(stats.overruns));
  }
  if (!settled) log::print(F(" settle=TIMEOUT"));
  log::println();
}

}  // namespace

MotionController::MotionController(const Motor& leftMotor,
                                   const Motor& rightMotor,
                                   const Encoder& leftEncoder,
                                   const Encoder& rightEncoder,
                                   const UltrasonicSensor& leftSonar,
                                   const UltrasonicSensor& rightSonar)
    : leftMotor_(leftMotor),
      rightMotor_(rightMotor),
      leftEncoder_(leftEncoder),
      rightEncoder_(rightEncoder),
      leftSonar_(leftSonar),
      rightSonar_(rightSonar),
      headingPid_(kHeadingGains, cfg::kControlPeriodS),
      speedPid_(kSpeedGains, cfg::kControlPeriodS),
      centering_(load(kCenteringParams)) {}

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

MotionController::MoveResult MotionController::move(
    const int32_t targetTicks, const int8_t dirLeft, const int8_t dirRight,
    const MoveParams& params, const int32_t carriedHeading,
    const CenteringWalls* centering) {
  MoveResult result = {Status::kOk, true, 0, 0};
  if (targetTicks <= 0) return result;

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

    result.ticksLeft = leftEncoder_.count();
    result.ticksRight = rightEncoder_.count();

    // Stop on the AVERAGE distance of both wheels, both wheels at once.
    const int32_t travelled = (result.ticksLeft + result.ticksRight) / 2;
    if (profile.brakePointReached(travelled)) break;

    const uint32_t nowMs = hal::nowMs();
    if (travelled != prevTravelled) lastProgressMs = nowMs;
    if (nowMs - lastProgressMs > cfg::kStallTimeoutMs) {
      result.status = Status::kStall;
      break;
    }
    if (nowMs - startMs > cfg::kMoveTimeoutMs) {
      result.status = Status::kMoveTimeout;
      break;
    }

    // Measured average speed [tpp], low-pass filtered.
    measuredSpeed +=
        cfg::kSpeedFilter *
        (static_cast<float>(travelled - prevTravelled) - measuredSpeed);
    prevTravelled = travelled;

    const float targetSpeed = profile.next(travelled);

    // Forward speed: stiction offset + feed-forward + PI on the average
    // wheel speed. The full breakaway offset applies only at rest; once
    // rolling (with hysteresis) the lower kinetic offset lets the PI
    // actually control the creep speed.
    rolling = updateRolling(rolling, measuredSpeed, cfg::kRollingEnterTpp,
                            cfg::kRollingExitTpp);
    const float offset =
        stictionPwm(rolling, params.staticPwm, params.kineticPwm);
    float basePwm = offset + cfg::kFfPwmPerTpp * targetSpeed +
                    speedPid_.update(targetSpeed - measuredSpeed, speedClamp);
    const bool floored = basePwm < offset;
    if (floored) basePwm = offset;

    if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
      if (targetSpeed >= params.profile.cruiseSpeed) {
        stats.cruisePwmSum += basePwm;
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
        carry + static_cast<float>(result.ticksLeft - result.ticksRight);
    if (centeringOn) {
      if (travelled >= halfway) centering_.setWalls(centering->next);
      headingError += centering_.correctionTicks();
    }
    const float correction = headingPid_.update(headingError, headingClamp);

    // Per-wheel floor: never below breakaway at rest, kinetic once rolling.
    const WheelMix mix =
        mixWheels(basePwm, correction, cfg::kLeftPwmTrim, cfg::kRightPwmTrim,
                  offset * cfg::kWheelFloorFraction, kMaxPwm);
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
        centering_.addReading(
            side, static_cast<float>(echoUs) / UltrasonicSensor::kUsPerCm);
      }
    }
  }

  stop();
  result.settled = waitForStandstill();
  result.ticksLeft = leftEncoder_.count();
  result.ticksRight = rightEncoder_.count();

  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    logMove(targetTicks, result.ticksLeft, result.ticksRight, result.status,
            result.settled, stats);
  }
  return result;
}

void MotionController::logCarry(const bool settled) const {
  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    log::print(F("H="));
    log::print(headingCarry_);
    if (!settled) log::print(F(" (unsettled)"));
    log::println();
  }
}

Status MotionController::driveForward(const CenteringWalls& walls) {
  // Start with the heading error left over from previous moves so it is
  // corrected instead of accumulating cell after cell.
  const MoveParams params = load(kForwardMove);
  const MoveResult result = move(cfg::kTicksPerCell, kForward, kForward,
                                 params, headingCarry_, &walls);
  // Also after a settle timeout: the move has already acted on the old
  // carry, so the counts at the timeout are the best estimate there is.
  headingCarry_ =
      heading::carryAfterForward(headingCarry_, result.ticksLeft,
                                 result.ticksRight, cfg::kHeadingCarryLimit);
  logCarry(result.settled);
  return result.status;
}

Status MotionController::turn(const TurnDirection direction,
                              const uint8_t quarterTurns) {
  if (quarterTurns == 0) return Status::kOk;
  if (quarterTurns > kMaxQuarterTurns) return Status::kInvalidArgument;

  const int32_t nominal = static_cast<int32_t>(quarterTurns) * cfg::kTicksPer90;
  const int8_t sign = static_cast<int8_t>(direction);
  const int32_t target = heading::turnTarget(nominal, sign, headingCarry_);

  const MoveParams params = load(kTurnMove);
  const MoveResult result = move(target, sign, static_cast<int8_t>(-sign),
                                 params, 0, nullptr);
  headingCarry_ = heading::carryAfterTurn(headingCarry_, sign,
                                         result.ticksLeft, result.ticksRight,
                                         nominal, cfg::kHeadingCarryLimit);
  logCarry(result.settled);
  return result.status;
}

}  // namespace micromouse
