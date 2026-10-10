// Combines base PWM and heading correction into per-wheel PWM magnitudes.
#pragma once

#include "control/pid.h"

namespace micromouse {

/// Per-wheel PWM magnitudes plus the saturation seen by each loop.
struct WheelMix {
  float left;           ///< [PWM] in [floor, maxPwm]
  float right;          ///< [PWM] in [floor, maxPwm]
  PidClamp speedClamp;  ///< saturation for the speed loop (base PWM)
  PidClamp headingClamp;  ///< saturation for the heading loop (correction)
};

/// left = base * trimLeft - correction, right = base * trimRight + correction.
/// A wheel below `floorPwm` is raised to it and the difference is added to
/// the other wheel (the differential - i.e. the heading correction - is
/// kept). If a wheel then exceeds `maxPwm`, both wheels are lowered by the
/// overflow (differential kept again) and the speed loop is told it is
/// saturated high. Only if the differential itself exceeds
/// maxPwm - floorPwm is the lower wheel held at the floor and the heading
/// loop told it is saturated (high: right wheel at max, low: left at max).
WheelMix mixWheels(float basePwm, float correction, float trimLeft,
                   float trimRight, float floorPwm, float maxPwm);

}  // namespace micromouse
