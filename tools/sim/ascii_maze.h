// Parses the classic micromouse ASCII maze format used by the host
// simulator (see tools/sim/mazes/*.txt and the README "Simulator" section).
// Host-only tool code: not part of the firmware layering, free to use the
// standard library.
#pragma once

#include <string>
#include <vector>

#include "navigation/maze.h"
#include "navigation/pose.h"

namespace sim {

/// Result of parsing one maze file: the ground-truth wall map (every side
/// definitely known, not the robot's sticky belief) plus any S/G markers
/// found in the text.
struct ParsedMaze {
  micromouse::Maze truth;
  uint8_t rows = 0;
  uint8_t cols = 0;
  bool hasStart = false;
  bool hasGoal = false;
  micromouse::Cell start{};
  micromouse::Cell goal{};
};

/// Parses `text`. Returns true on success. On failure `errors` holds one or
/// more human-readable messages and `out` is left in an undefined state.
bool parseMazeText(const std::string& text, ParsedMaze& out,
                    std::vector<std::string>& errors);

}  // namespace sim
