#include "navigation/flood_fill.h"

namespace micromouse {

namespace {
// Final tie-break order (phase-1 neighbour order, kept for determinism).
constexpr Direction kTieBreakOrder[kDirectionCount] = {
    Direction::kDown, Direction::kUp, Direction::kRight, Direction::kLeft};
}  // namespace

Status FloodFill::compute(const Maze& maze, const Cell goal) {
  valid_ = false;
  if (!maze.contains(goal)) return Status::kOutOfBounds;

  const uint16_t cells = maze.cellCount();
  for (uint16_t i = 0; i < cells; ++i) dist_[i] = kUnreached;
  queue_.clear();

  const CellIndex goalIndex = maze.indexOf(goal);
  dist_[goalIndex] = 0;
  queue_.push(goalIndex);

  CellIndex index = 0;
  while (queue_.pop(index)) {
    const Cell cell = maze.cellAt(index);
    const Distance next = static_cast<Distance>(dist_[index] + 1);
    for (const Direction dir : kAllDirections) {
      Cell other{};
      if (maze.hasWall(cell, dir) || !maze.neighbour(cell, dir, other)) {
        continue;
      }
      const CellIndex otherIndex = maze.indexOf(other);
      if (dist_[otherIndex] != kUnreached) continue;
      dist_[otherIndex] = next;
      // Every cell is queued at most once, so this cannot fail unless the
      // capacity invariant is broken.
      if (!queue_.push(otherIndex)) return Status::kQueueOverflow;
    }
  }

  rows_ = maze.rows();
  cols_ = maze.cols();
  valid_ = true;
  return Status::kOk;
}

Distance FloodFill::distance(const Cell cell) const {
  if (!valid_ || cell.row >= rows_ || cell.col >= cols_) return kUnreached;
  return dist_[cell.row * cols_ + cell.col];
}

Status FloodFill::bestDirection(const Maze& maze, const Cell from,
                                const Direction heading,
                                Direction& out) const {
  if (!valid_ || maze.rows() != rows_ || maze.cols() != cols_) {
    return Status::kNotInitialized;
  }
  if (!maze.contains(from)) return Status::kOutOfBounds;

  const Distance here = dist_[maze.indexOf(from)];
  if (here == kUnreached) return Status::kNoPath;

  bool found = false;
  Distance bestDistance = kUnreached;
  uint8_t bestTurns = 0;
  for (const Direction dir : kTieBreakOrder) {
    Cell other{};
    if (maze.hasWall(from, dir) || !maze.neighbour(from, dir, other)) {
      continue;
    }
    const Distance d = dist_[maze.indexOf(other)];
    if (d == kUnreached || d >= here) continue;
    const uint8_t turns = quarterTurns(relativeTurn(heading, dir));
    if (!found || d < bestDistance ||
        (d == bestDistance && turns < bestTurns)) {
      found = true;
      bestDistance = d;
      bestTurns = turns;
      out = dir;
    }
  }
  return found ? Status::kOk : Status::kNoPath;
}

}  // namespace micromouse
