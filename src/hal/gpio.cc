#include "hal/gpio.h"

#include <Arduino.h>

namespace micromouse::hal {

void configureOutput(const uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void writeDigital(const uint8_t pin, const bool high) {
  digitalWrite(pin, high ? HIGH : LOW);
}

void configureMotorPwm(const uint8_t pin) {
  pinMode(pin, OUTPUT);
  analogWrite(pin, 0);  // start braked, never floating

#if defined(TCCR2A) && defined(WGM21) && defined(WGM20) && \
    defined(TIMER2A) && defined(TIMER2B)
  const uint8_t timer = digitalPinToTimer(pin);
  if (timer == TIMER2A || timer == TIMER2B) {
    TCCR2A |= _BV(WGM21) | _BV(WGM20);  // mode 3: fast PWM, TOP = 0xFF
  }
#endif
}

void writePwm(const uint8_t pin, const uint8_t duty) { analogWrite(pin, duty); }

}  // namespace micromouse::hal
