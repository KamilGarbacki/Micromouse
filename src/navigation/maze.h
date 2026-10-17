// Wall map of a rectangular grid maze with static storage.
#pragma once

#include <stdint.h>

#include "config/config.h"
#include "navigation/pose.h"
#include "util/status.h"

namespace micromouse {

/// Linear cell index: row * cols + col.
using CellIndex = uint8_t;

/// Discovered walls of the maze (Cell{row, col} convention, see pose.h).
///
/// Every cell stores one byte with a "wall" and a "known" bit per side, and
/// one byte with a 2-bit counter of pending contradicting observations per
/// side. A shared wall is always written to both cells, so both views agree.
/// The outer boundary is a wall and cannot be changed: it blocks travel in
/// every mode. With `physicalBoundary` (reset()) it is a KNOWN wall, like
/// an observed one; otherwise (open floor) it is a VIRTUAL wall: present
/// but never "known", since no sensor can see it (see hasPhysicalWall()).
/// Unknown interior walls are reported as not present (planners are
/// optimistic).
///
/// Observations are asymmetric-sticky: an unknown side becomes known at
/// once, but a known side flips only after `flipObservations` consecutive
/// contradicting observations (a confirming one resets the count), so
/// readings flickering around the threshold cannot make the map oscillate.
class Maze {
 public:
  static constexpr uint8_t kMaxRows = config::kMazeMaxRows;
  static constexpr uint8_t kMaxCols = config::kMazeMaxCols;
  static constexpr uint16_t kMaxCells =
      static_cast<uint16_t>(kMaxRows) * kMaxCols;

  static constexpr uint8_t kMaxFlipObservations = 4;  // 2-bit counter

  /// Resizes to `rows` x `cols` and forgets all interior walls.
  /// `flipObservations` (1..4, 1 = latest wins) is the number of consecutive
  /// contradicting observations needed to change a known side.
  /// `physicalBoundary`: the outer boundary is a real (known) wall; false =
  /// open floor, a virtual (wall, but not known) boundary. Returns
  /// kInvalidArgument (and becomes empty) if the size is 0 or exceeds the
  /// capacity, or `flipObservations` is out of range.
  Status reset(uint8_t rows, uint8_t cols,
               uint8_t flipObservations = config::kWallFlipObservations,
               bool physicalBoundary = true);

  uint8_t rows() const { return rows_; }
  uint8_t cols() const { return cols_; }
  uint16_t cellCount() const {
    return static_cast<uint16_t>(rows_) * cols_;
  }

  bool contains(Cell cell) const {
    return cell.row < rows_ && cell.col < cols_;
  }

  /// Writes the cell one step in `dir` into `out`; false if that leaves the
  /// maze (or `cell` is outside it).
  bool neighbour(Cell cell, Direction dir, Cell& out) const;

  /// Records an observation of the wall on side `dir` of `cell` (mirrored to
  /// the neighbour), subject to the sticky rule above. Boundary walls stay
  /// walls. kOutOfBounds if the cell is outside the maze.
  Status setWall(Cell cell, Direction dir, bool present);

  /// True if side `dir` of `cell` lies on the outer edge of the grid (no
  /// neighbour there). False for a cell outside the maze.
  bool isBoundary(const Cell cell, const Direction dir) const {
    Cell unused{};
    return contains(cell) && !neighbour(cell, dir, unused);
  }

  /// True if the side has been observed (a physical boundary always is, a
  /// virtual one never).
  bool isKnown(Cell cell, Direction dir) const;

  /// True if a wall blocks travel there (observed, or the boundary, also a
  /// virtual one). Unknown -> false (optimistic). Cells outside the maze
  /// are treated as walled in. For PLANNING.
  bool hasWall(Cell cell, Direction dir) const;

  /// True if a wall a sensor should SEE is there: a known wall (observed
  /// interior wall, or a physical boundary), never a virtual open-floor
  /// boundary. Cells outside the maze are walled in (as hasWall()). For SENSING
  /// expectations (plausibility, learning, centering, the run log).
  bool hasPhysicalWall(Cell cell, Direction dir) const;

  /// Index of a cell; precondition: contains(cell).
  CellIndex indexOf(Cell cell) const {
    return static_cast<CellIndex>(cell.row * cols_ + cell.col);
  }

  /// Inverse of indexOf(); precondition: index < cellCount().
  Cell cellAt(CellIndex index) const {
    return Cell{static_cast<uint8_t>(index / cols_),
                static_cast<uint8_t>(index % cols_)};
  }

 private:
  static constexpr uint8_t wallBit(const Direction dir) {
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(dir));
  }
  static constexpr uint8_t knownBit(const Direction dir) {
    return static_cast<uint8_t>(1u << (static_cast<uint8_t>(dir) +
                                       kDirectionCount));
  }
  static constexpr uint8_t kPendingMask = 0x3;
  static constexpr uint8_t pendingShift(const Direction dir) {
    return static_cast<uint8_t>(2u * static_cast<uint8_t>(dir));
  }
  void record(CellIndex index, Direction dir, bool present);
  uint8_t pending(CellIndex index, Direction dir) const;
  void setPending(CellIndex index, Direction dir, uint8_t count);

  uint8_t sides_[kMaxCells] = {};
  uint8_t pending_[kMaxCells] = {};
  uint8_t flipObservations_ = 1;
  uint8_t rows_ = 0;
  uint8_t cols_ = 0;
};

}  // namespace micromouse
