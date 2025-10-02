//
// Created by kamil on 20.08.2025.
//

#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

#include "motor_dir.h"

namespace micromouse {

class Motor {
  const uint8_t speedPin;
  const uint8_t directionPin;
  const uint8_t encoderPin;
  long encoderPings;
  const MotorMountSide type;

 public:
  Motor()
      : speedPin(0),
        directionPin(0),
        encoderPin(0),
        encoderPings(0),
        type(MotorMountSide::RIGHT) {};
  Motor(const uint8_t speedPin, const uint8_t directionPin,
        const uint8_t encoderPin, const MotorMountSide type)
      : speedPin(speedPin),
        directionPin(directionPin),
        encoderPin(encoderPin),
        encoderPings(0),
        type(type) {}

  void setupMotor() const;

  uint8_t getEncoderPin() const { return encoderPin; }
  long getEncoderPings() const { return encoderPings; }
  void resetEncoderPings() { encoderPings = 0; }
  MotorMountSide getType() const { return type; }

  void handleEncoder();
  void setSpeed(int speed) const;
  void shortBreak() const;
};

}  // namespace micromouse

#endif  // MOTOR_H
