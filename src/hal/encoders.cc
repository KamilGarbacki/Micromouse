#include "hal/encoders.h"

#include <Arduino.h>
#include <util/atomic.h>

#include "config/pins.h"

namespace micromouse::hal {

namespace {

constexpr uint8_t kChannelCount = 2;

volatile int32_t gCounts[kChannelCount] = {0, 0};

void onLeftEdge() { gCounts[0] = gCounts[0] + 1; }
void onRightEdge() { gCounts[1] = gCounts[1] + 1; }

uint8_t indexOf(const EncoderChannel channel) {
  return static_cast<uint8_t>(channel);
}

}  // namespace

void encoderBegin(const EncoderChannel channel) {
  const bool left = channel == EncoderChannel::kLeft;
  const uint8_t pin =
      left ? config::pins::kLeftEncoder : config::pins::kRightEncoder;
  // Hall encoders are commonly open-collector; without a pull-up the input
  // floats and picks up motor PWM noise as phantom ticks.
  pinMode(pin, INPUT_PULLUP);
  encoderReset(channel);
  attachInterrupt(digitalPinToInterrupt(pin), left ? onLeftEdge : onRightEdge,
                  RISING);
}

int32_t encoderCount(const EncoderChannel channel) {
  int32_t count = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { count = gCounts[indexOf(channel)]; }
  return count;
}

void encoderReset(const EncoderChannel channel) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { gCounts[indexOf(channel)] = 0; }
}

}  // namespace micromouse::hal
