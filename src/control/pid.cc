#include "control/pid.h"

#include "util/math_utils.h"

namespace micromouse {

float Pid::update(const float error, const PidClamp external) {
  if (dt_ <= 0.0f) return 0.0f;

  const float p = kp_ * error;
  const float d = hasPrev_ ? kd_ * (error - prevError_) * invDt_ : 0.0f;
  prevError_ = error;
  hasPrev_ = true;

  if (ki_ != 0.0f) {
    const float candidate =
        util::clamp(integral_ + error * dt_, -integralMax_, integralMax_);
    const float unclamped = p + ki_ * candidate + d;
    const bool saturatedHigh = external.high || unclamped > outputLimit_;
    const bool saturatedLow = external.low || unclamped < -outputLimit_;
    // Direction in which this step's integration moves the output.
    const float push = ki_ * error;
    const bool windsUp =
        (saturatedHigh && push > 0.0f) || (saturatedLow && push < 0.0f);
    if (!windsUp) integral_ = candidate;
  }

  return util::clamp(p + ki_ * integral_ + d, -outputLimit_, outputLimit_);
}

void Pid::reset() {
  integral_ = 0.0f;
  prevError_ = 0.0f;
  hasPrev_ = false;
}

}  // namespace micromouse
