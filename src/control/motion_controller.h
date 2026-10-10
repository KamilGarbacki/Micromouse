// Closed-loop cell moves and spins in place (the 100 Hz move loop).
#pragma once

#include <stdint.h>

#include "control/motion_profile.h"
#include "control/pid.h"
#include "control/wall_centering.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/ultrasonic_sensor.h"
#include "util/status.h"

namespace micromouse {

/// Velocity profile and stiction model of one kind of move.
struct MoveParams {
  ProfileParams profile;
  float staticPwm;   ///< [PWM] offset while (nearly) at rest: breakaway
  float kineticPwm;  ///< [PWM] offset while moving: keeps the wheels turning
};

enum class TurnDirection : int8_t { kClockwise = 1, kCounterClockwise = -1 };

/// Walls known beside the path of a one-cell move, for wall centering:
/// `current` for the first half of the move, `next` for the second half.
struct CenteringWalls {
  SideWalls current;
  SideWalls next;
};

/// Drives the two wheels through fixed-period moves.
///
/// Per control period: speed profile -> feed-forward + average-speed PI ->
/// base PWM; heading PID on (left - right) ticks -> differential; mixed with
/// a per-wheel floor. Between moves the residual heading error ("carry", in
/// left-minus-right ticks, + = rotated clockwise) is kept and corrected by
/// the next move. Blocking: returns when the robot is stopped.
class MotionController {
 public:
  MotionController(const Motor& leftMotor, const Motor& rightMotor,
                   const Encoder& leftEncoder, const Encoder& rightEncoder,
                   const UltrasonicSensor& leftSonar,
                   const UltrasonicSensor& rightSonar);

  /// Initialises motors (braked) and encoders.
  void begin();

  /// Drives one cell straight ahead. kStall / kMoveTimeout on failure.
  Status driveForward(const CenteringWalls& walls);

  /// Spins in place by `quarterTurns` x 90 degrees (1 or 2).
  /// kInvalidArgument for other values; kStall / kMoveTimeout on failure.
  Status turn(TurnDirection direction, uint8_t quarterTurns);

  /// Short-brakes both motors immediately.
  void stop() const;

  /// Heading error carried into the next move [left - right ticks].
  int32_t headingCarryTicks() const { return headingCarry_; }

 private:
  struct MoveResult {
    Status status;
    bool settled;  ///< false: encoders still ticking at kSettleTimeoutMs
    int32_t ticksLeft;
    int32_t ticksRight;
  };

  MoveResult move(int32_t targetTicks, int8_t dirLeft, int8_t dirRight,
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
  int32_t headingCarry_ = 0;
};

}  // namespace micromouse
