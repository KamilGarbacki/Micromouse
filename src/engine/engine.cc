//
// Created by kamil on 20.08.2025.
//
#include "engine.h"

#include <Arduino.h>

namespace micromouse {
#define ENCODER_TICKS_PER_CELL 2068  // Example value, calibrate to your robot
#define ENCODER_TICKS_PER_TURN 1107   // Example value, calibrate to your robot
#define FORWARD_SPEED 40            // Base speed for straight movement (0-255)
#define TURN_SPEED 30                // Base speed for turning
#define BACKOOF_SPEED 10

static Motor* leftMotorInstance = nullptr;
static Motor* rightMotorInstance = nullptr;

void handleLeftEncoder() {
  if (leftMotorInstance) leftMotorInstance->handleEncoder();
}

void handleRightEncoder() {
  if (rightMotorInstance) rightMotorInstance->handleEncoder();
}

void Engine::addMotor(Motor* motor) {
  motor->setupMotor();

  if (motor->getType() == MotorMountSide::LEFT) {
    leftMotor = motor;
    leftMotorInstance = leftMotor;

    attachInterrupt(digitalPinToInterrupt(motor->getEncoderPin()),
                    handleLeftEncoder, CHANGE);
  } else if (motor->getType() == MotorMountSide::RIGHT) {
    rightMotor = motor;
    rightMotorInstance = rightMotor;

    attachInterrupt(digitalPinToInterrupt(motor->getEncoderPin()),
                    handleRightEncoder, CHANGE);
  }
}

int getNewSpeed(int baseSpeed, int ticks, int goal) {
  return baseSpeed - ticks * (40 / goal);
}

void Engine::driveToNextCell() {
  leftMotor->resetEncoderPings();
  rightMotor->resetEncoderPings();

  straightPid.reset();

  while (true) {
    const int encoderL = abs(leftMotor->getEncoderPings());
    const int encoderR = abs(rightMotor->getEncoderPings());

    // Serial.println("----------------------------------");
    // Serial.print("Left Encoder: ");
    // Serial.println(encoderL);
    // Serial.print("Right Encoder: ");
    // Serial.println(encoderR);

    if (encoderL >= ENCODER_TICKS_PER_CELL) leftMotor->shortBreak();
    if (encoderR >= ENCODER_TICKS_PER_CELL) rightMotor->shortBreak();

    if (encoderL >= ENCODER_TICKS_PER_CELL || encoderR >= ENCODER_TICKS_PER_CELL) {
      leftMotor->shortBreak();
      rightMotor->shortBreak();
      break;
    }

    int baseSpeedL = FORWARD_SPEED;
    int baseSpeedR = FORWARD_SPEED;

    // if (encoderL >= 0.95 * ENCODER_TICKS_PER_CELL || encoderR >= 0.95 * ENCODER_TICKS_PER_CELL) {
    //   baseSpeedL = FORWARD_SPEED / 4;
    //   baseSpeedR = FORWARD_SPEED / 4;
    // } else if (encoderL >= 0.8 * ENCODER_TICKS_PER_CELL || encoderR >= 0.8 * ENCODER_TICKS_PER_CELL) {
    //   baseSpeedL = FORWARD_SPEED / 2;
    //   baseSpeedR = FORWARD_SPEED / 2;
    // } else {
    //   baseSpeedL = FORWARD_SPEED;
    //   baseSpeedR = FORWARD_SPEED;
    // }

    baseSpeedL = getNewSpeed(baseSpeedL, encoderL, ENCODER_TICKS_PER_CELL);
    baseSpeedR = getNewSpeed(baseSpeedR, encoderR, ENCODER_TICKS_PER_CELL);

    const int straightError = encoderL - encoderR;
    const float straightCorrection = straightPid.getCorrection(straightError);

    if (straightError < 0) {
      baseSpeedL += abs(straightError) * straightCorrection;
    } else if (straightError > 0) {
      baseSpeedL -= abs(straightError) * straightCorrection;
    }

    leftMotor->setSpeed(baseSpeedL);
    rightMotor->setSpeed(baseSpeedR);
  }

  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
}

void Engine::turn(const bool clockwise, int degrees) {
  leftMotor->resetEncoderPings();
  rightMotor->resetEncoderPings();

  straightPid.reset();
  lengthPid.reset();

  const int dirL = clockwise ? 1 : -1;
  const int dirR = clockwise ? -1 : 1;

  const int turns = degrees / 90;

  while (true) {
    const int encoderL = abs(leftMotor->getEncoderPings());
    const int encoderR = abs(rightMotor->getEncoderPings());

    if (encoderL >= turns * ENCODER_TICKS_PER_TURN || encoderR >= turns * ENCODER_TICKS_PER_TURN) {
      leftMotor->shortBreak();
      rightMotor->shortBreak();
      break;
    }

    int baseSpeedL = TURN_SPEED;
    int baseSpeedR = TURN_SPEED;

    // if (encoderL >= 0.95 * ENCODER_TICKS_PER_CELL ||
    //     encoderR >= 0.95 * ENCODER_TICKS_PER_CELL) {
    //   baseSpeedL = FORWARD_SPEED / 4;
    //   baseSpeedR = FORWARD_SPEED / 4;
    // } else if (encoderL >= 0.8 * ENCODER_TICKS_PER_CELL ||
    //     encoderR >= 0.8 * ENCODER_TICKS_PER_CELL) {
    //   baseSpeedL = FORWARD_SPEED / 2;
    //   baseSpeedR = FORWARD_SPEED / 2;
    // } else {
    //   baseSpeedL = FORWARD_SPEED;
    //   baseSpeedR = FORWARD_SPEED;
    // }

    baseSpeedL = getNewSpeed(baseSpeedL, encoderL, ENCODER_TICKS_PER_TURN);
    baseSpeedR = getNewSpeed(baseSpeedR, encoderR, ENCODER_TICKS_PER_TURN);

    const float straightError = encoderL - encoderR;

    const float straightCorrection = straightPid.getCorrection(straightError);

    if (straightError < 0) {
      baseSpeedL += abs(straightError) * straightCorrection;
    } else if (straightError > 0) {
      baseSpeedL -= abs(straightError) * straightCorrection;
    }



    baseSpeedL *= dirL;
    baseSpeedR *= dirR;

    leftMotor->setSpeed(baseSpeedL);
    rightMotor->setSpeed(baseSpeedR);
  }

  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
}

}  // namespace micromouse
