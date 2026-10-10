#include "drivers/motor.h"

#include "hal/gpio.h"
#include "util/math_utils.h"

namespace micromouse {

void Motor::begin() const {
  hal::configureOutput(dirPin_);
  hal::configureMotorPwm(pwmPin_);
}

void Motor::setDuty(int16_t duty) const {
  duty = util::clamp<int16_t>(duty, -kMaxDuty, kMaxDuty);
  if (duty == 0) {
    brake();
    return;
  }
  const bool forward = duty > 0;
  const bool dirHigh = side_ == MountSide::kLeft ? forward : !forward;
  hal::writeDigital(dirPin_, dirHigh);
  hal::writePwm(pwmPin_, static_cast<uint8_t>(forward ? duty : -duty));
}

void Motor::brake() const { hal::writePwm(pwmPin_, 0); }

}  // namespace micromouse
