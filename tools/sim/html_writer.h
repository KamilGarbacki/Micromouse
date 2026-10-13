// Renders one simulated run into a single self-contained HTML file (inline
// CSS/JS, the trace inlined as JSON, no external libraries, works from
// file://). Host-only tool code.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "navigation/pose.h"

namespace sim {

/// One frame of the animation: the SENSE/PLAN/MOVE loop produces one of
/// these per sub-step, plus a final GOAL_REACHED or FAULT frame.
struct TraceStep {
  std::string state;  ///< "SENSE", "PLAN", "MOVE", "GOAL_REACHED", "FAULT"
  micromouse::Cell cell{};
  micromouse::Direction heading = micromouse::Direction::kUp;
  uint32_t moves = 0;
  bool hasPlanned = false;
  micromouse::Direction planned = micromouse::Direction::kUp;
  /// rows*cols*4 entries, Up/Right/Down/Left per cell, row-major:
  /// 0 = unknown, 1 = known open, 2 = known wall.
  std::vector<uint8_t> wallCodes;
  /// rows*cols entries, row-major; -1 = unreached / not yet computed.
  std::vector<int16_t> flood;
  bool hasFault = false;
  int faultCode = 0;
  std::string faultName;
};

struct SimData {
  uint8_t rows = 0;
  uint8_t cols = 0;
  micromouse::Cell start{};
  micromouse::Cell goal{};
  /// rows*cols*4 entries, Up/Right/Down/Left per cell, row-major: 1 = open,
  /// 2 = wall (the ground truth has no "unknown" side).
  std::vector<uint8_t> trueWalls;
  std::vector<TraceStep> steps;

  // Metadata shown in the page header / status line.
  std::string mazeFile;
  double noise = 0.0;
  uint32_t seed = 0;
  std::string deadSensor;  // "", "L", "F" or "R"
  std::string outcomeText;
};

/// Writes the self-contained HTML visualisation to `path`. Returns false
/// (and leaves an error on stderr) if the file could not be written.
bool writeHtml(const std::string& path, const SimData& data);

}  // namespace sim
