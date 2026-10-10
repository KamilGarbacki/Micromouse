#include "navigation/maze.h"

namespace micromouse {

Status Maze::reset(const uint8_t rows, const uint8_t cols,
                   const uint8_t flipObservations) {
  rows_ = 0;
  cols_ = 0;
  for (uint16_t i = 0; i < kMaxCells; ++i) {
    sides_[i] = 0;
    pending_[i] = 0;
  }
  if (rows == 0 || cols == 0 || rows > kMaxRows || cols > kMaxCols ||
      flipObservations == 0 || flipObservations > kMaxFlipObservations) {
    return Status::kInvalidArgument;
  }
  flipObservations_ = flipObservations;
  rows_ = rows;
  cols_ = cols;

  // The outer boundary is a known wall.
  for (uint8_t row = 0; row < rows_; ++row) {
    for (uint8_t col = 0; col < cols_; ++col) {
      const Cell cell{row, col};
      Cell unused{};
      for (const Direction dir : kAllDirections) {
        if (!neighbour(cell, dir, unused)) record(indexOf(cell), dir, true);
      }
    }
  }
  return Status::kOk;
}

bool Maze::neighbour(const Cell cell, const Direction dir, Cell& out) const {
  if (!contains(cell)) return false;
  const int16_t row = static_cast<int16_t>(cell.row) + rowStep(dir);
  const int16_t col = static_cast<int16_t>(cell.col) + colStep(dir);
  if (row < 0 || col < 0 || row >= rows_ || col >= cols_) return false;
  out = Cell{static_cast<uint8_t>(row), static_cast<uint8_t>(col)};
  return true;
}

Status Maze::setWall(const Cell cell, const Direction dir,
                     const bool present) {
  if (!contains(cell)) return Status::kOutOfBounds;
  Cell other{};
  if (!neighbour(cell, dir, other)) return Status::kOk;  // boundary: fixed
  const CellIndex here = indexOf(cell);
  const CellIndex there = indexOf(other);
  const Direction back = opposite(dir);

  if (isKnown(cell, dir) && hasWall(cell, dir) != present) {
    // Contradicts a known side: flip only after enough in a row.
    const uint8_t count = static_cast<uint8_t>(pending(here, dir) + 1);
    if (count < flipObservations_) {
      setPending(here, dir, count);
      setPending(there, back, count);
      return Status::kOk;
    }
  }
  // Unknown, confirming, or enough contradictions: take the observation.
  record(here, dir, present);
  record(there, back, present);
  setPending(here, dir, 0);
  setPending(there, back, 0);
  return Status::kOk;
}

uint8_t Maze::pending(const CellIndex index, const Direction dir) const {
  return static_cast<uint8_t>((pending_[index] >> pendingShift(dir)) &
                              kPendingMask);
}

void Maze::setPending(const CellIndex index, const Direction dir,
                      const uint8_t count) {
  const uint8_t shift = pendingShift(dir);
  pending_[index] = static_cast<uint8_t>(
      (pending_[index] & ~(kPendingMask << shift)) |
      ((count & kPendingMask) << shift));
}

bool Maze::isKnown(const Cell cell, const Direction dir) const {
  if (!contains(cell)) return true;
  return (sides_[indexOf(cell)] & knownBit(dir)) != 0;
}

bool Maze::hasWall(const Cell cell, const Direction dir) const {
  if (!contains(cell)) return true;
  return (sides_[indexOf(cell)] & wallBit(dir)) != 0;
}

void Maze::record(const CellIndex index, const Direction dir,
                  const bool present) {
  uint8_t side = static_cast<uint8_t>(sides_[index] | knownBit(dir));
  if (present) {
    side = static_cast<uint8_t>(side | wallBit(dir));
  } else {
    side = static_cast<uint8_t>(side & ~wallBit(dir));
  }
  sides_[index] = side;
}

}  // namespace micromouse
