#include "control/ff_fit.h"

namespace micromouse {

LineFit fitLine(const float* x, const float* y, const uint8_t count) {
  LineFit fit = {0.0f, 0.0f, 0.0f, false};
  if (count < 2) return fit;
  // Centred sums (numerically safer in single precision).
  float meanX = 0.0f;
  float meanY = 0.0f;
  for (uint8_t i = 0; i < count; ++i) {
    meanX += x[i];
    meanY += y[i];
  }
  meanX /= count;
  meanY /= count;
  float sxx = 0.0f;
  float sxy = 0.0f;
  float syy = 0.0f;
  for (uint8_t i = 0; i < count; ++i) {
    const float dx = x[i] - meanX;
    const float dy = y[i] - meanY;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  if (!(sxx > 1e-9f)) return fit;
  fit.slope = sxy / sxx;
  fit.offset = meanY - fit.slope * meanX;
  fit.r2 = syy > 0.0f ? (sxy * sxy) / (sxx * syy) : 1.0f;
  fit.valid = true;
  return fit;
}

}  // namespace micromouse
