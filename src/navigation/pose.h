// Grid vocabulary: cells, absolute directions, relative directions, pose.
// Dependency-free leaf header (config/ may include it).
//
// Coordinate convention (used everywhere):
//   Cell{row, col}; row 0 is the TOP row, col 0 the LEFT column.
//   Direction::kUp    -> row - 1      Direction::kDown  -> row + 1
//   Direction::kRight -> col + 1      Direction::kLeft  -> col - 1
#pragma once

#include <stdint.h>

namespace micromouse {

/// Absolute grid direction. Values are clockwise quarter turns from kUp.
enum class Direction : uint8_t { kUp = 0, kRight = 1, kDown = 2, kLeft = 3 };

/// Direction relative to the robot's heading, in clockwise quarter turns.
enum class Relative : uint8_t {
  kForward = 0,
  kRight = 1,
  kBack = 2,
  kLeft = 3
};

constexpr uint8_t kDirectionCount = 4;
constexpr uint8_t kQuarterTurnMask = kDirectionCount - 1;

/// All absolute directions, clockwise from kUp.
inline constexpr Direction kAllDirections[kDirectionCount] = {
    Direction::kUp, Direction::kRight, Direction::kDown, Direction::kLeft};

/// Maze cell coordinate (see the convention above).
struct Cell {
  uint8_t row;
  uint8_t col;
};

constexpr bool operator==(const Cell a, const Cell b) {
  return a.row == b.row && a.col == b.col;
}
constexpr bool operator!=(const Cell a, const Cell b) { return !(a == b); }

/// Where the robot is and which way it faces.
struct Pose {
  Cell cell;
  Direction heading;
};

/// The absolute direction reached by turning `rel` from `heading`.
constexpr Direction rotate(const Direction heading, const Relative rel) {
  return static_cast<Direction>(
      (static_cast<uint8_t>(heading) + static_cast<uint8_t>(rel)) &
      kQuarterTurnMask);
}

/// The turn that takes `from` to `to`.
constexpr Relative relativeTurn(const Direction from, const Direction to) {
  return static_cast<Relative>((static_cast<uint8_t>(to) + kDirectionCount -
                                static_cast<uint8_t>(from)) &
                               kQuarterTurnMask);
}

constexpr Direction opposite(const Direction dir) {
  return rotate(dir, Relative::kBack);
}

/// Number of 90 degree turns needed for `rel` (0, 1 or 2).
constexpr uint8_t quarterTurns(const Relative rel) {
  return rel == Relative::kForward ? 0 : (rel == Relative::kBack ? 2 : 1);
}

/// Row change of one step in `dir` (-1, 0 or +1).
constexpr int8_t rowStep(const Direction dir) {
  return dir == Direction::kUp ? -1 : (dir == Direction::kDown ? 1 : 0);
}

/// Column change of one step in `dir` (-1, 0 or +1).
constexpr int8_t colStep(const Direction dir) {
  return dir == Direction::kLeft ? -1 : (dir == Direction::kRight ? 1 : 0);
}

}  // namespace micromouse
