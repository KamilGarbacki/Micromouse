//
// Created by kamil on 22.08.2025.
//
#include "pid_controller.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <stdlib.h>

namespace micromouse {

float PIDController::getCorrection(float error) {
  if (error == 0) {
    return 0;
  }

  if (error * prevError < 0) {
    errorWeight = errorWeight - (abs(error) + abs(prevError)) * offShootWeight;
  }

  prevError = error;

  return errorWeight;
}

void PIDController::reset() {
  prevError = 0;
  errorWeight = baseErrorWeight;
}

}  // namespace micromouse
