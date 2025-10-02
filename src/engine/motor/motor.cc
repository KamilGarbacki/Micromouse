//
// Created by kamil on 20.08.2025.
//

#include "motor.h"

#include <Arduino.h>

namespace micromouse {

void Motor::setupMotor() const {
  pinMode(speedPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
}

void Motor::handleEncoder() {
  if (digitalRead(directionPin) == HIGH)
    encoderPings++;
  else
    encoderPings--;
}

void Motor::setSpeed(const int speed) const {
  if (this->getType() == MotorMountSide::LEFT && speed > 0) {
    digitalWrite(directionPin, HIGH);
    analogWrite(speedPin, speed);
  }

  if (this->getType() == MotorMountSide::LEFT && speed < 0) {
    digitalWrite(directionPin, LOW);
    analogWrite(speedPin, abs(speed));
  }

  if (this->getType() == MotorMountSide::RIGHT && speed > 0) {
    digitalWrite(directionPin, LOW);
    analogWrite(speedPin, speed);
  }

  if (this->getType() == MotorMountSide::RIGHT && speed < 0) {
    digitalWrite(directionPin, HIGH);
    analogWrite(speedPin, abs(speed));
  }

  if (speed == 0) {
    analogWrite(speedPin, 0);
  }
}

void Motor::shortBreak() const {
  digitalWrite(directionPin, HIGH);
  analogWrite(speedPin, 255);

  this->setSpeed(0);
}

}  // namespace micromouse