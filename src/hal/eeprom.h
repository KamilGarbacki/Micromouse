// Byte access to the on-chip EEPROM (wraps Arduino's EEPROM.h).
#pragma once

#include <stdint.h>

namespace micromouse::hal {

/// EEPROM size of the ATmega328P [bytes].
constexpr uint16_t kEepromSize = 1024;

/// Copies `size` bytes from EEPROM `address` to `out`. Out-of-range bytes
/// read as 0xFF (erased).
void eepromRead(uint16_t address, void* out, uint16_t size);

/// Writes `size` bytes to EEPROM `address`, only touching bytes that differ
/// (EEPROM.update), to limit wear. Out-of-range bytes are ignored.
void eepromWrite(uint16_t address, const void* data, uint16_t size);

}  // namespace micromouse::hal
