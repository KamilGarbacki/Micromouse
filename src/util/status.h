// Result codes shared by all layers. The numeric value is what a FAULT
// report prints on Serial, so never renumber existing codes.
#pragma once

#include <stdint.h>

namespace micromouse {

enum class Status : uint8_t {
  kOk = 0,
  kInvalidArgument = 1,  ///< parameter outside its valid range
  kOutOfBounds = 2,      ///< cell outside the maze
  kNoPath = 3,           ///< flood fill: goal unreachable with known walls
  kStall = 4,            ///< move: no encoder progress for kStallTimeoutMs
  kMoveTimeout = 5,      ///< move: took longer than kMoveTimeoutMs
  kNotInitialized = 6,   ///< used before begin()/compute()
  kQueueOverflow = 7,    ///< internal: flood-fill queue full (a bug)
  kSensorFailure = 8,    ///< a sonar got no echo although a wall is known
  kExplorationLimit = 9, ///< more cells driven than the exploration budget
};

constexpr bool isOk(const Status status) { return status == Status::kOk; }

}  // namespace micromouse
