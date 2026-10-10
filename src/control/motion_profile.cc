#include "control/motion_profile.h"

#include <math.h>

namespace micromouse {

MotionProfile::MotionProfile(const ProfileParams& params,
                             const int32_t targetTicks)
    : cruiseSpeed_(params.cruiseSpeed),
      creepSpeed_(params.creepSpeed),
      accel_(params.accel),
      twoDecel_(2.0f * params.decel),
      creepSquared_(params.creepSpeed * params.creepSpeed),
      stopTicks_(targetTicks - params.brakeLeadTicks),
      creepStart_(targetTicks - params.brakeLeadTicks - params.creepTicks) {}

float MotionProfile::decelLimit(const int32_t travelled) const {
  const int32_t toCreep = creepStart_ - travelled;
  return toCreep > 0
             ? sqrtf(creepSquared_ + twoDecel_ * static_cast<float>(toCreep))
             : creepSpeed_;
}

float MotionProfile::next(const int32_t travelled) {
  rampSpeed_ += accel_;
  if (rampSpeed_ > cruiseSpeed_) rampSpeed_ = cruiseSpeed_;
  const float limit = decelLimit(travelled);
  return limit < rampSpeed_ ? limit : rampSpeed_;
}

}  // namespace micromouse
