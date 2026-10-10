// Breadth-first flood fill of distances to the goal, and move selection.
#pragma once

#include <stdint.h>

#include "navigation/maze.h"
#include "navigation/pose.h"
#include "util/ring_buffer.h"
#include "util/status.h"

namespace micromouse {

/// Distance in cells to the goal.
using Distance = uint8_t;

/// Flood-fill planner. Optimistic: unknown walls count as open, so the
/// distances are a lower bound that is refined as walls are discovered.
class FloodFill {
 public:
  static constexpr Distance kUnreached = 0xFF;

  /// Recomputes all distances to `goal` (BFS from the goal, blocked only by
  /// known walls). kOutOfBounds if the goal is outside `maze`.
  Status compute(const Maze& maze, Cell goal);

  /// Distance of `cell` from the last compute(); kUnreached if the cell is
  /// unreachable, outside the maze, or compute() has not succeeded.
  Distance distance(Cell cell) const;

  /// Picks the neighbour of `from` to drive to: the reachable, not walled
  /// neighbour with the smallest distance. Ties are broken by the fewest
  /// turns from `heading` (straight, then 90 deg, then 180 deg), then by the
  /// fixed order Down, Up, Right, Left.
  /// Returns kNoPath if no neighbour is closer (goal unreachable, or `from`
  /// is the goal itself), kNotInitialized if `maze` changed size since
  /// compute().
  Status bestDirection(const Maze& maze, Cell from, Direction heading,
                       Direction& out) const;

 private:
  Distance dist_[Maze::kMaxCells] = {};
  util::RingBuffer<CellIndex, Maze::kMaxCells> queue_;
  uint8_t rows_ = 0;
  uint8_t cols_ = 0;
  bool valid_ = false;
};

}  // namespace micromouse
