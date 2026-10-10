// Pin assignment: Arduino Uno + DFRobot DRI0039 (TB6612FNG) quad motor shield.
#pragma once

#include <stdint.h>

namespace micromouse::config::pins {

// DRI0039 channel map (fixed by the shield):
//   M1: PWM D3,  DIR D4      M2: PWM D11, DIR D12
//   M3: PWM D5,  DIR D8      M4: PWM D6,  DIR D7
// DIR drives IN1 and (inverted) IN2 of the TB6612FNG, so IN1 != IN2 always
// and PWM = 0 is an active SHORT BRAKE.
//
// WARNING: M1 (D3/D4) and M4 (D6/D7) share their control lines with the
// right encoder (D3) and the ultrasonic sensors (D4, D6, D7). Those shield
// channels are toggled by the encoder / sensor signals, so the M1 and M4
// motor terminals MUST stay unconnected.

// Left wheel on shield channel M3. D5 = Timer0 (fast PWM, ~976 Hz).
constexpr uint8_t kLeftMotorPwm = 5;
constexpr uint8_t kLeftMotorDir = 8;
// Right wheel on shield channel M2. D11 = Timer2, switched to fast PWM by
// the HAL so both wheels see the same ~976 Hz PWM.
constexpr uint8_t kRightMotorPwm = 11;
constexpr uint8_t kRightMotorDir = 12;

// Single-channel hall encoders, counted on RISING edges (direction blind).
// Must be the Uno's external-interrupt pins (INT0 = D2, INT1 = D3).
constexpr uint8_t kLeftEncoder = 2;
constexpr uint8_t kRightEncoder = 3;

// US-015 ultrasonic sensors (driven through NewPing).
constexpr uint8_t kLeftSonarTrig = 6;
constexpr uint8_t kLeftSonarEcho = 4;
constexpr uint8_t kFrontSonarTrig = 9;
constexpr uint8_t kFrontSonarEcho = 7;
// D13 is also the on-board LED, so the LED cannot be used as a status light.
constexpr uint8_t kRightSonarTrig = 13;
constexpr uint8_t kRightSonarEcho = 10;

/// Every pin in use, for the uniqueness check in config.h.
constexpr uint8_t kAllPins[] = {
    kLeftMotorPwm,   kLeftMotorDir,   kRightMotorPwm,  kRightMotorDir,
    kLeftEncoder,    kRightEncoder,   kLeftSonarTrig,  kLeftSonarEcho,
    kFrontSonarTrig, kFrontSonarEcho, kRightSonarTrig, kRightSonarEcho,
};

}  // namespace micromouse::config::pins
