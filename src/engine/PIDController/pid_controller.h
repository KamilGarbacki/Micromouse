//
// Created by kamil on 22.08.2025.
//

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

namespace micromouse {

class PIDController {
  float prevError;
  float baseErrorWeight;
  float errorWeight;
  float offShootWeight;

 public:
  explicit PIDController(const float pE = 0, const float eW = 0.21,
                         const float oW = 0.001)
      : prevError(pE), baseErrorWeight(eW), errorWeight(eW), offShootWeight(oW) {}

  float getCorrection(float error);
  void reset();
};

}  // namespace micromouse

#endif  // PID_CONTROLLER_H
