// US-015 ultrasonic distance sensor (blocking NewPing ping).
#pragma once

#include <stdint.h>

namespace micromouse {

/// The fitted sensors; pins in config/pins.h.
enum class SonarId : uint8_t { kLeft = 0, kFront = 1, kRight = 2 };

constexpr uint8_t kSonarCount = 3;

class UltrasonicSensor {
 public:
  /// Returned when there is no echo within the range.
  static constexpr uint16_t kNoEcho = 0;

  explicit constexpr UltrasonicSensor(const SonarId id) : id_(id) {}

  /// Configures the trigger/echo pins.
  void begin() const;

  /// One blocking ping, giving up beyond `maxCm`. Blocks for at most about
  /// (maxCm + 1) * 57 us after the sensor starts the ping (typically
  /// ~0.5 ms), or up to NewPing's 5.8 ms start timeout if the sensor does not
  /// respond. Returns the distance [cm] or kNoEcho. Also kNoEcho if the
  /// sensor's previous echo is still in progress.
  uint16_t pingCm(uint16_t maxCm) const;

  /// Like pingCm() but returns the round-trip echo time [us] (0 = no echo),
  /// for sub-centimetre resolution: distance_cm = us / kUsPerCm.
  uint16_t pingUs(uint16_t maxCm) const;

  /// Round-trip microseconds per centimetre (NewPing US_ROUNDTRIP_CM).
  static constexpr float kUsPerCm = 57.0f;

  SonarId id() const { return id_; }

 private:
  SonarId id_;
};

}  // namespace micromouse
