//
// Created by kamil on 20.08.2025.
//

#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

#include "motor_dir.h"

using FuncPtr = void(*)();

namespace micromouse {

class Motor {
  const uint8_t speedPin;
  const uint8_t directionPin;
  const uint8_t encoderPin;
  long pings;
  const MotorMountSide type;
  FuncPtr stopMotorsFunction;

 public:
  Motor()
      : speedPin(0),
        directionPin(0),
        encoderPin(0),
        pings(0),
        type(MotorMountSide::RIGHT),
        stopMotorsFunction(nullptr) {};
  Motor(const uint8_t speedPin, const uint8_t directionPin,
        const uint8_t encoderPin, const MotorMountSide type)
      : speedPin(speedPin),
        directionPin(directionPin),
        encoderPin(encoderPin),
        pings(0),
        type(type),
        stopMotorsFunction(nullptr) {}

  void setupMotor() const;

  [[nodiscard]] uint8_t getEncoderPin() const { return encoderPin; }
  [[nodiscard]] long getEncoderPings() const { return pings; }
  [[nodiscard]] MotorMountSide getType() const { return type; }

  void setSpeed(int speed) const;
  void onValueChange(const FuncPtr onValueChange) {stopMotorsFunction = onValueChange; }

  void resetPings() { pings = 0; }
  void handleEncoder();
  void shortBreak() const;
};

}  // namespace micromouse

#endif  // MOTOR_H
