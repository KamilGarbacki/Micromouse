//
// Created by kamil on 20.08.2025.
//

#ifndef ENGINE_H
#define ENGINE_H
#include "PIDController/pid_controller.h"
#include "engine.h"
#include "motor/motor.h"

namespace micromouse {
class Engine {
  PIDController straightPid;
  PIDController lengthPid;

  Motor* leftMotor;
  Motor* rightMotor;

 public:
  Engine()
      : leftMotor(nullptr),
        rightMotor(nullptr) {}

  void addMotor(Motor* motor);
  void driveForward();
  void turn(bool clockwise, int degrees);
  static void stopMotors();
};

}  // namespace micromouse

#endif  // ENGINE_H
