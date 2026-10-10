#include "control/wheel_mixer.h"

namespace micromouse {

WheelMix mixWheels(const float basePwm, const float correction,
                   const float trimLeft, const float trimRight,
                   const float floorPwm, const float maxPwm) {
  float left = basePwm * trimLeft - correction;
  float right = basePwm * trimRight + correction;

  // Keep both wheels turning: shift the correction onto the other wheel
  // instead of stalling one below the floor (phase-1 behaviour).
  if (left < floorPwm) {
    right += floorPwm - left;
    left = floorPwm;
  }
  if (right < floorPwm) {
    left += floorPwm - right;
    right = floorPwm;
  }

  WheelMix mix{left, right, {false, false}, {false, false}};

  // Above full scale: lower BOTH wheels by the overflow so the differential
  // (the heading correction) survives; only the common speed is limited.
  const float highest = left > right ? left : right;
  if (highest > maxPwm) {
    const float overflow = highest - maxPwm;
    mix.left -= overflow;
    mix.right -= overflow;
    mix.speedClamp.high = true;
    // The differential itself does not fit between floor and full scale:
    // the lower wheel is held at the floor and the correction is limited.
    if (mix.left < floorPwm) {
      mix.left = floorPwm;
      mix.headingClamp.high = true;  // right wheel at max: can't add more
    }
    if (mix.right < floorPwm) {
      mix.right = floorPwm;
      mix.headingClamp.low = true;  // left wheel at max: can't subtract more
    }
  }
  return mix;
}

}  // namespace micromouse
