#include "drivers/ultrasonic_sensor.h"

#include <Arduino.h>
#include <NewPing.h>
#include <avr/pgmspace.h>

#include "config/config.h"

namespace micromouse {

namespace {

namespace pins = config::pins;

static_assert(US_ROUNDTRIP_CM == static_cast<int>(config::kNewPingUsPerCm),
              "config::kNewPingUsPerCm must match NewPing's US_ROUNDTRIP_CM");

// NewPing objects live here so that no header above the driver layer sees
// NewPing.h (and through it Arduino.h). The constructor only records the pin
// registers; the default range is irrelevant because every ping passes its
// own maximum.
NewPing gSonars[kSonarCount] = {
    NewPing(pins::kLeftSonarTrig, pins::kLeftSonarEcho,
            config::kMappingMaxCm),
    NewPing(pins::kFrontSonarTrig, pins::kFrontSonarEcho,
            config::kMappingMaxCm),
    NewPing(pins::kRightSonarTrig, pins::kRightSonarEcho,
            config::kMappingMaxCm),
};

struct SonarPins {
  uint8_t trig;
  uint8_t echo;
};

const SonarPins kSonarPins[kSonarCount] PROGMEM = {
    {pins::kLeftSonarTrig, pins::kLeftSonarEcho},
    {pins::kFrontSonarTrig, pins::kFrontSonarEcho},
    {pins::kRightSonarTrig, pins::kRightSonarEcho},
};

}  // namespace

void UltrasonicSensor::begin() const {
  const SonarPins& flash = kSonarPins[static_cast<uint8_t>(id_)];
  const uint8_t trig = pgm_read_byte(&flash.trig);
  const uint8_t echo = pgm_read_byte(&flash.echo);
  pinMode(echo, INPUT);
  digitalWrite(trig, LOW);
  pinMode(trig, OUTPUT);
}

namespace {
// NewPing's ping(max) permanently changes the object's range, so always
// pass it explicitly. 0 would mean "keep the previous range".
uint16_t rangeOrDefault(const uint16_t maxCm) {
  return maxCm > 0 ? maxCm : config::kMappingMaxCm;
}
}  // namespace

uint16_t UltrasonicSensor::pingUs(const uint16_t maxCm) const {
  return static_cast<uint16_t>(
      gSonars[static_cast<uint8_t>(id_)].ping(rangeOrDefault(maxCm)));
}

}  // namespace micromouse
