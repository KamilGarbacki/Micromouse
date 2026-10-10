// One DC motor on a DRI0039 / TB6612FNG channel (PWM + DIR).
#pragma once

#include <stdint.h>

namespace micromouse {

/// Which side of the chassis the motor is on. The motors are mounted
/// mirrored, so "forward" is a different DIR level on each side.
enum class MountSide : uint8_t { kLeft, kRight };

class Motor {
 public:
  static constexpr int16_t kMaxDuty = 255;

  constexpr Motor(const uint8_t pwmPin, const uint8_t dirPin,
                  const MountSide side)
      : pwmPin_(pwmPin), dirPin_(dirPin), side_(side) {}

  /// Configures the pins; the motor starts short-braked.
  void begin() const;

  /// Drives with a signed duty in [-255, 255] (clamped); + = forward.
  /// 0 = short brake (DIR is left unchanged).
  void setDuty(int16_t duty) const;

  /// Active short brake. On the DRI0039, DIR drives IN1 and inverted IN2, so
  /// IN1 != IN2 always and PWM = 0 is the TB6612 short-brake state.
  void brake() const;

 private:
  uint8_t pwmPin_;
  uint8_t dirPin_;
  MountSide side_;
};

}  // namespace micromouse
