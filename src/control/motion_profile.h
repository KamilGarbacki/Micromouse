// Trapezoidal-with-sqrt-decel speed profile of one move, in tpp
// (encoder ticks per control period).
#pragma once

#include <stdint.h>

namespace micromouse {

/// Shape of a velocity profile.
struct ProfileParams {
  float cruiseSpeed;       ///< [tpp] top speed
  float creepSpeed;        ///< [tpp] final approach speed
  float accel;             ///< [tpp per period] speed-up rate
  float decel;             ///< [tpp per period] slow-down rate
  int32_t creepTicks;      ///< [ticks] driven at creepSpeed before braking
  int32_t brakeLeadTicks;  ///< [ticks] brake this far before the target
};

/// Target-speed generator for one move of `targetTicks`.
///
/// The speed ramps up by `accel` per period to `cruiseSpeed`, and is limited
/// by the constant-deceleration curve v = sqrt(creep^2 + 2 * decel * d) where
/// d is the distance left to the creep start (stopTicks - creepTicks). From
/// the creep start to the brake point (stopTicks) it is creepSpeed.
class MotionProfile {
 public:
  MotionProfile(const ProfileParams& params, int32_t targetTicks);

  /// Position [ticks] at which the motors are braked.
  int32_t stopTicks() const { return stopTicks_; }

  /// Position [ticks] from which the robot creeps.
  int32_t creepStartTicks() const { return creepStart_; }

  /// True once `travelled` has reached the brake point.
  bool brakePointReached(const int32_t travelled) const {
    return travelled >= stopTicks_;
  }

  /// Advances the accel ramp by one period and returns the target speed
  /// [tpp] at position `travelled`. Call exactly once per control period.
  float next(int32_t travelled);

  /// The deceleration limit at `travelled` (pure function of position).
  float decelLimit(int32_t travelled) const;

 private:
  float cruiseSpeed_;
  float creepSpeed_;
  float accel_;
  float twoDecel_;
  float creepSquared_;
  int32_t stopTicks_;
  int32_t creepStart_;
  float rampSpeed_ = 0.0f;
};

}  // namespace micromouse
