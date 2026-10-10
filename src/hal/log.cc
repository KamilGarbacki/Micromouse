#include "hal/log.h"

#include <Arduino.h>

namespace micromouse::hal::log {

void begin(const uint32_t baud) { Serial.begin(baud); }

void print(const __FlashStringHelper* text) { Serial.print(text); }
void print(const char c) { Serial.print(c); }
void print(const int value) { Serial.print(value); }
void print(const unsigned int value) { Serial.print(value); }
void print(const long value) { Serial.print(value); }
void print(const unsigned long value) { Serial.print(value); }
void print(const float value, const uint8_t decimals) {
  Serial.print(value, decimals);
}

void println() { Serial.println(); }

}  // namespace micromouse::hal::log
