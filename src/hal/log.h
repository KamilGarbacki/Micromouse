// Serial text output. Pass constant strings with F("...") so they stay in
// flash. Guard calls with `if constexpr (config::logEnabled(...))` so that
// disabled levels are compiled out.
#pragma once

#include <stdint.h>

#if defined(ARDUINO)
#include <WString.h>  // F() and __FlashStringHelper only (no Arduino.h)
#else
// Host build (unit tests of app modules): the declarations and F() only.
// Logging calls are compiled out there (MM_LOG_LEVEL guards with
// `if constexpr`), so nothing has to be linked.
class __FlashStringHelper;
#define F(text) (reinterpret_cast<const __FlashStringHelper*>(text))
#endif

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

/// Formatted output from a flash format string (one call instead of many
/// print() calls, which keeps the logging code small). Conversions:
/// %d int, %u unsigned int, %l long, %c char, %s flash string (F()),
/// %0 .. %3 float with that many decimals. Pass exactly these types.
void fmt(const __FlashStringHelper* format, ...);
/// fmt() followed by println().
void fmtln(const __FlashStringHelper* format, ...);

/// Serial INPUT (startup mode menu): the next received byte, or -1 if none.
int16_t readChar();

/// Discards received bytes until the line has been quiet for `quietMs`
/// (swallows the CR/LF a terminal sends after a key).
void drainInput(uint32_t quietMs);

}  // namespace micromouse::hal::log
