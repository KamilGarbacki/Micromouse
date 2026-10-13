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
  /// respond. Returns the round-trip echo time [us] or kNoEcho; distance_cm
  /// = us / config::kSonarUsPerCm (temperature-corrected speed of sound).
  /// Also kNoEcho if the sensor's previous echo is still in progress.
  uint16_t pingUs(uint16_t maxCm) const;

  SonarId id() const { return id_; }

 private:
  SonarId id_;
};

}  // namespace micromouse
