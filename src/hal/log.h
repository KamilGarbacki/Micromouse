// Serial text output. Pass constant strings with F("...") so they stay in
// flash. Guard calls with `if constexpr (config::logEnabled(...))` so that
// disabled levels are compiled out.
#pragma once

#include <WString.h>  // F() and __FlashStringHelper only (no Arduino.h)
#include <stdint.h>

namespace micromouse::hal::log {

/// Opens the serial port.
void begin(uint32_t baud);

void print(const __FlashStringHelper* text);
void print(char c);
void print(int value);
void print(unsigned int value);
void print(long value);
void print(unsigned long value);
void print(float value, uint8_t decimals = 2);

/// Ends the line ("\r\n").
void println();

}  // namespace micromouse::hal::log
