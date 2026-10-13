#include "hal/log.h"

#include <Arduino.h>
#include <stdarg.h>

namespace micromouse::hal::log {

namespace {

/// Fixed-point float output (smaller than Print::printFloat).
void printFixed(float value, const uint8_t decimals) {
  if (value != value) {  // NaN
    Serial.print(F("nan"));
    return;
  }
  if (value < 0.0f) {
    Serial.print('-');
    value = -value;
  }
  uint32_t scale = 1;
  for (uint8_t i = 0; i < decimals; ++i) scale *= 10;
  if (value > 2.0e9f / static_cast<float>(scale)) {
    Serial.print(F("ovf"));
    return;
  }
  const uint32_t scaled =
      static_cast<uint32_t>(value * static_cast<float>(scale) + 0.5f);
  Serial.print(scaled / scale);
  if (decimals == 0) return;
  Serial.print('.');
  uint32_t fraction = scaled % scale;
  for (uint32_t digit = scale / 10; digit > 0; digit /= 10) {
    Serial.print(static_cast<char>('0' + fraction / digit));
    fraction %= digit;
  }
}

void vformat(const __FlashStringHelper* format, va_list args) {
  const char* p = reinterpret_cast<const char*>(format);
  while (true) {
    char c = static_cast<char>(pgm_read_byte(p++));
    if (c == '\0') return;
    if (c != '%') {
      Serial.print(c);
      continue;
    }
    c = static_cast<char>(pgm_read_byte(p++));
    switch (c) {
      case 'd':
        Serial.print(va_arg(args, int));
        break;
      case 'u':
        Serial.print(va_arg(args, unsigned int));
        break;
      case 'l':
        Serial.print(va_arg(args, long));
        break;
      case 'c':
        Serial.print(static_cast<char>(va_arg(args, int)));
        break;
      case 's':
        Serial.print(va_arg(args, const __FlashStringHelper*));
        break;
      case '0':
      case '1':
      case '2':
      case '3':
        printFixed(static_cast<float>(va_arg(args, double)),
                   static_cast<uint8_t>(c - '0'));
        break;
      case '\0':
        return;
      default:
        Serial.print(c);
        break;
    }
  }
}

}  // namespace

void begin(const uint32_t baud) { Serial.begin(baud); }

void print(const __FlashStringHelper* text) { Serial.print(text); }
void print(const char c) { Serial.print(c); }
void print(const int value) { Serial.print(value); }
void print(const unsigned int value) { Serial.print(value); }
void print(const long value) { Serial.print(value); }
void print(const unsigned long value) { Serial.print(value); }
void print(const float value, const uint8_t decimals) {
  printFixed(value, decimals);
}

void println() { Serial.println(); }

void fmt(const __FlashStringHelper* format, ...) {
  va_list args;
  va_start(args, format);
  vformat(format, args);
  va_end(args);
}

void fmtln(const __FlashStringHelper* format, ...) {
  va_list args;
  va_start(args, format);
  vformat(format, args);
  va_end(args);
  Serial.println();
}

int16_t readChar() { return static_cast<int16_t>(Serial.read()); }

void drainInput(const uint32_t quietMs) {
  uint32_t last = millis();
  while (millis() - last < quietMs) {
    if (Serial.read() >= 0) last = millis();
  }
}

}  // namespace micromouse::hal::log
