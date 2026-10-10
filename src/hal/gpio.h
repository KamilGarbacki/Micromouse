// Digital I/O and motor PWM outputs.
#pragma once

#include <stdint.h>

namespace micromouse::hal {

/// Configures `pin` as a push-pull output driven low.
void configureOutput(uint8_t pin);

/// Drives an output pin high or low.
void writeDigital(uint8_t pin, bool high);

/// Configures a hardware-PWM pin as a motor PWM output at duty 0 and makes
/// sure it runs at ~976 Hz. The Uno's Timer0 pins (5, 6) already run fast
/// PWM at ~976 Hz, but the Timer2 pins (3, 11) default to phase-correct
/// ~490 Hz, which gives a different average motor current for the same duty
/// (a built-in left/right mismatch). Timer2 is switched to fast PWM with the
/// prescaler unchanged; Timer0 (millis) is not touched.
///
/// WARNING: anything else that reprograms Timer2 then breaks the motor PWM,
/// in particular NewPing's ping_timer()/timer_us()/timer_ms() and tone().
/// Only NewPing's blocking ping() may be used.
void configureMotorPwm(uint8_t pin);

/// Sets the PWM duty (0..255). 0 drives the pin constantly low.
void writePwm(uint8_t pin, uint8_t duty);

}  // namespace micromouse::hal
