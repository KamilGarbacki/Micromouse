#include "control/wheel_mixer.h"

namespace micromouse {

WheelMix mixWheels(const float baseLeft, const float baseRight,
                   const float correction, const float floorLeft,
                   const float floorRight, const float maxPwm) {
  float left = baseLeft - correction;
  float right = baseRight + correction;

  // Keep both wheels turning: shift the correction onto the other wheel
  // instead of stalling one below its floor (phase-1 behaviour).
  if (left < floorLeft) {
    right += floorLeft - left;
    left = floorLeft;
  }
  if (right < floorRight) {
    left += floorRight - right;
    right = floorRight;
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
    // the lower wheel is held at its floor and the correction is limited.
    if (mix.left < floorLeft) {
      mix.left = floorLeft;
      mix.headingClamp.high = true;  // right wheel at max: can't add more
    }
    if (mix.right < floorRight) {
      mix.right = floorRight;
      mix.headingClamp.low = true;  // left wheel at max: can't subtract more
    }
  }
  return mix;
}

}  // namespace micromouse
