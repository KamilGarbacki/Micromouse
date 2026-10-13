// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final
// XOR). Check value: "123456789" -> 0x29B1.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace micromouse::util {

constexpr uint16_t kCrc16Init = 0xFFFF;

/// Feeds one byte into the CRC.
constexpr uint16_t crc16Update(uint16_t crc, const uint8_t byte) {
  crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(byte) << 8));
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                          : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

/// CRC of `size` bytes at `data`.
inline uint16_t crc16(const void* data, const size_t size,
                      uint16_t crc = kCrc16Init) {
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < size; ++i) crc = crc16Update(crc, bytes[i]);
  return crc;
}

}  // namespace micromouse::util
