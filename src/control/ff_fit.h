// Least-squares straight line through measured (speed, PWM) points, for the
// per-wheel feed-forward of the motor self-calibration. Pure.
#pragma once

#include <stdint.h>

namespace micromouse {

/// y = offset + slope * x.
struct LineFit {
  float offset;
  float slope;
  float r2;    ///< coefficient of determination (1 = perfect line)
  bool valid;  ///< false for < 2 points or no spread in x
};

/// Ordinary least squares of y over x for `count` points.
LineFit fitLine(const float* x, const float* y, uint8_t count);

}  // namespace micromouse
