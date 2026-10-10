// The maze the robot runs in: size, start pose and goal.
// Coordinates follow the Cell{row, col} convention of navigation/pose.h.
#pragma once

#include <stdint.h>

#include "navigation/pose.h"

namespace micromouse::config {

// Static storage capacity. RAM cost is ~3 bytes per cell (walls, flood
// distance, queue slot); at most 255 cells (8-bit cell index / distance).
constexpr uint8_t kMazeMaxRows = 8;
constexpr uint8_t kMazeMaxCols = 8;

// The maze actually run.
constexpr uint8_t kMazeRows = 5;
constexpr uint8_t kMazeCols = 4;

constexpr Cell kStartCell = {4, 0};  // bottom-left
constexpr Direction kStartHeading = Direction::kUp;
constexpr Cell kGoalCell = {0, 3};  // top-right

}  // namespace micromouse::config
