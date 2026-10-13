#include "hal/eeprom.h"

#include <EEPROM.h>

namespace micromouse::hal {

void eepromRead(const uint16_t address, void* out, const uint16_t size) {
  uint8_t* bytes = static_cast<uint8_t*>(out);
  for (uint16_t i = 0; i < size; ++i) {
    const uint16_t at = static_cast<uint16_t>(address + i);
    bytes[i] = at < kEepromSize ? EEPROM.read(at) : 0xFF;
  }
}

void eepromWrite(const uint16_t address, const void* data,
                 const uint16_t size) {
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  for (uint16_t i = 0; i < size; ++i) {
    const uint16_t at = static_cast<uint16_t>(address + i);
    if (at < kEepromSize) EEPROM.update(at, bytes[i]);
  }
}

}  // namespace micromouse::hal
