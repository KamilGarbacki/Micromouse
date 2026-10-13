// Interactive calibration modes of the startup menu (blocking, Serial).
#pragma once

#include "control/calibration.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "hal/log.h"

namespace micromouse {

/// 'e': motors braked; prints the L/R tick counts, the expected ticks per
/// wheel revolution (config::kTicksPerWheelRev) and measured / expected for
/// each wheel until a key is pressed. Turn each wheel by exactly one
/// revolution: both ratios must read ~1.00.
void runEncoderCheck(const Encoder& left, const Encoder& right);

/// 'c': motor self-calibration with the robot LIFTED. Asks for 'y', then for
/// each wheel separately measures staticPwm (ramp up until the wheel moves
/// in kSelfCalMovingSteps consecutive steps), the feed-forward line
/// (steady-state speed at kSelfCalLevels PWM levels, least-squares fit) and
/// kineticPwm (ramp down from the lowest level until the wheel stops). Any
/// key aborts (motors braked). On success writes both wheel models into
/// `record` (sealed) and returns true; the caller stores it.
bool runMotorSelfCalibration(const Motor& leftMotor, const Motor& rightMotor,
                             const Encoder& leftEncoder,
                             const Encoder& rightEncoder, MotorRecord& record);

/// 'p': prints the calibration in use and how the stored records were
/// judged at boot.
void printCalibration(const Calibration& cal, calibration::Validity motor,
                      calibration::Validity learned);

/// Short name of a validation result ("VALID", "BAD_CRC", ...).
const __FlashStringHelper* validityName(calibration::Validity validity);

}  // namespace micromouse
