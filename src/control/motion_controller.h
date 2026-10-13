// Closed-loop cell moves and spins in place (the 100 Hz move loop).
#pragma once

#include <stdint.h>

#include "config/config.h"
#include "control/calibration.h"
#include "control/motion_profile.h"
#include "control/pid.h"
#include "control/wall_centering.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/ultrasonic_sensor.h"
#include "util/status.h"

namespace micromouse {

/// Velocity profile of one kind of move; the motor model of each wheel
/// comes from the calibration (calibration::wheelModel) for `kind`.
struct MoveParams {
  ProfileParams profile;
  MoveKind kind;
};

/// What a move did, for logging and the online learning.
struct MoveReport {
  Status status;
  bool settled;          ///< false: encoders still ticking at the timeout
  int32_t target;        ///< commanded ticks per wheel
  int32_t left;          ///< final left count (direction blind)
  int32_t right;         ///< final right count
  int32_t carryBefore;   ///< heading carry at the start [L - R ticks]
  /// Sum over the move of the encoder heading (carryBefore + L - R) times
  /// the average-distance increment [ticks^2]; meaningful for forward moves.
  float headingIntegral;
};

enum class TurnDirection : int8_t { kClockwise = 1, kCounterClockwise = -1 };

/// +1 clockwise, -1 counter-clockwise: the sign convention of the heading
/// carry and of control/learning.h.
constexpr int8_t turnSign(const TurnDirection direction) {
  return static_cast<int8_t>(direction);
}

/// Walls known beside the path of a one-cell move, for wall centering:
/// `current` for the first half of the move, `next` for the second half.
struct CenteringWalls {
  SideWalls current;
  SideWalls next;
};

namespace motion_detail {
constexpr PidGains kHeadingGains = {config::kHeadingKp, config::kHeadingKi,
                                    config::kHeadingKd, config::kHeadingILimit,
                                    config::kHeadingOutLimit};
constexpr PidGains kSpeedGains = {config::kSpeedKp, config::kSpeedKi, 0.0f,
                                  config::kSpeedILimit,
                                  config::kSpeedOutLimit};
constexpr CenteringParams kCenteringParams = {
    config::kCenteringNominalLeftCm,  config::kCenteringNominalRightCm,
    config::kCenteringGainTicksPerCm, config::kCenteringMaxTicks,
    config::kCenteringMaxDeviationCm, config::kCenteringFilter,
    config::kCenteringPingPeriods};
}  // namespace motion_detail

/// Drives the two wheels through fixed-period moves.
///
/// Per control period: speed profile -> per-wheel feed-forward (each wheel
/// from its own motor model) + one PI on the average speed -> per-wheel base
/// PWM; heading PID on (left - right) ticks -> differential; mixed with a
/// per-wheel floor. Tick counts, brake leads and motor models are read from
/// the run-time calibration at the start of every move. Between moves the
/// residual heading error ("carry", in left-minus-right ticks, + = rotated
/// clockwise) is kept and corrected by the next move. Blocking: returns when
/// the robot is stopped.
class MotionController {
 public:
  /// constexpr: the robot object is constant-initialised (no start-up
  /// code); nothing here touches the hardware (see begin()).
  constexpr MotionController(const Motor& leftMotor, const Motor& rightMotor,
                             const Encoder& leftEncoder,
                             const Encoder& rightEncoder,
                             const UltrasonicSensor& leftSonar,
                             const UltrasonicSensor& rightSonar,
                             const Calibration& calibration)
      : leftMotor_(leftMotor),
        rightMotor_(rightMotor),
        leftEncoder_(leftEncoder),
        rightEncoder_(rightEncoder),
        leftSonar_(leftSonar),
        rightSonar_(rightSonar),
        headingPid_(motion_detail::kHeadingGains, config::kControlPeriodS),
        speedPid_(motion_detail::kSpeedGains, config::kControlPeriodS),
        centering_(motion_detail::kCenteringParams),
        calibration_(calibration) {}

  /// Initialises motors (braked) and encoders.
  void begin();

  /// Drives one cell straight ahead. kStall / kMoveTimeout / kEncoderFault
  /// on failure.
  Status driveForward(const CenteringWalls& walls, MoveReport& report);

  /// Spins in place by `quarterTurns` x 90 degrees (1 or 2).
  /// kInvalidArgument for other values; kStall / kMoveTimeout /
  /// kEncoderFault on failure.
  Status turn(TurnDirection direction, uint8_t quarterTurns,
              MoveReport& report);

  /// Drives |signedTicks| straight at creep speed, forward (+) or backward
  /// (-), holding equal wheel ticks (no carry is applied as a setpoint),
  /// with the small alignment brake lead (learning::alignmentBrakeLead).
  /// The carry is then updated with the commanded sign
  /// (heading::carryAfterStraight): the adjustment does not change the
  /// heading the carry describes, so it is neither reset nor double counted.
  Status adjustStraight(int32_t signedTicks, MoveReport& report);

  /// Short-brakes both motors immediately.
  void stop() const;

  /// Heading error carried into the next move [left - right ticks].
  int32_t headingCarryTicks() const { return headingCarry_; }

 private:
  MoveReport move(int32_t targetTicks, int8_t dirLeft, int8_t dirRight,
                  const MoveParams& params, int32_t carriedHeading,
                  const CenteringWalls* centering);
  bool waitForStandstill() const;
  static void driveWheel(const Motor& motor, float pwm, int8_t direction);
  void logCarry(bool settled) const;

  const Motor& leftMotor_;
  const Motor& rightMotor_;
  const Encoder& leftEncoder_;
  const Encoder& rightEncoder_;
  const UltrasonicSensor& leftSonar_;
  const UltrasonicSensor& rightSonar_;
  Pid headingPid_;
  Pid speedPid_;
  WallCentering centering_;
  const Calibration& calibration_;
  int32_t headingCarry_ = 0;
};

}  // namespace micromouse
