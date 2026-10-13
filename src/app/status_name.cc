#include "app/status_name.h"

namespace micromouse {

const __FlashStringHelper* statusName(const Status status) {
  switch (status) {
    case Status::kOk:
      return F("OK");
    case Status::kInvalidArgument:
      return F("INVALID_ARGUMENT");
    case Status::kOutOfBounds:
      return F("OUT_OF_BOUNDS");
    case Status::kNoPath:
      return F("NO_PATH");
    case Status::kStall:
      return F("STALL");
    case Status::kMoveTimeout:
      return F("MOVE_TIMEOUT");
    case Status::kNotInitialized:
      return F("NOT_INITIALIZED");
    case Status::kQueueOverflow:
      return F("QUEUE_OVERFLOW");
    case Status::kSensorFailure:
      return F("SENSOR_FAILURE");
    case Status::kExplorationLimit:
      return F("EXPLORATION_LIMIT");
    case Status::kEncoderFault:
      return F("ENCODER_FAULT");
  }
  return F("UNKNOWN");
}

}  // namespace micromouse
