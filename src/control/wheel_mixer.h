// Combines per-wheel base PWM and heading correction into per-wheel PWM
// magnitudes.
#pragma once

#include "control/pid.h"

namespace micromouse {

/// Per-wheel PWM magnitudes plus the saturation seen by each loop.
struct WheelMix {
  float left;           ///< [PWM] in [floorLeft, maxPwm]
  float right;          ///< [PWM] in [floorRight, maxPwm]
  PidClamp speedClamp;  ///< saturation for the speed loop (base PWM)
  PidClamp headingClamp;  ///< saturation for the heading loop (correction)
};

/// left = baseLeft - correction, right = baseRight + correction (each base
/// comes from that wheel's own feed-forward). A wheel below its floor is
/// raised to it and the difference is added to the other wheel (the
/// differential - i.e. the heading correction - is kept). If a wheel then
/// exceeds `maxPwm`, both wheels are lowered by the overflow (differential
/// kept again) and the speed loop is told it is saturated high. Only if the
/// differential itself does not fit between floor and maxPwm is the lower
/// wheel held at its floor and the heading loop told it is saturated
/// (high: right wheel at max, low: left at max).
WheelMix mixWheels(float baseLeft, float baseRight, float correction,
                   float floorLeft, float floorRight, float maxPwm);

}  // namespace micromouse
