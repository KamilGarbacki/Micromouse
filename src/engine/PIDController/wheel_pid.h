#ifndef WHEEL_PID_H
#define WHEEL_PID_H

#include <Arduino.h>

namespace micromouse {

class WheelPID {
  float kp, ki, kd;
  float integral;
  float prevErr;
  float outMin, outMax;
  float iMin, iMax;

public:
  WheelPID(float kp=0.6f, float ki=0.15f, float kd=0.0f)
    : kp(kp), ki(ki), kd(kd),
      integral(0), prevErr(0),
      outMin(-255), outMax(255),
      iMin(-400), iMax(400) {}

  void reset() { integral = 0; prevErr = 0; }

  void setOutputLimits(float mn, float mx) { outMin = mn; outMax = mx; }
  void setIntegralLimits(float mn, float mx) { iMin = mn; iMax = mx; }

  // dt w sekundach
  int update(float targetTicksPerSec, float measuredTicksPerSec, float dt) {
    const float err = targetTicksPerSec - measuredTicksPerSec;

    integral += err * dt;
    if (integral > iMax) integral = iMax;
    if (integral < iMin) integral = iMin;

    const float deriv = (dt > 0) ? (err - prevErr) / dt : 0.0f;
    prevErr = err;

    float out = kp*err + ki*integral + kd*deriv;

    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;

    return (int)out;
  }
};

} // namespace micromouse
#endif
