// Exploration logic of one run: map updates, sensor plausibility, planning
// and pose tracking. Pure (no hardware), so the whole SENSE -> PLAN -> MOVE
// decision loop can be simulated on the host.
#pragma once

#include <stdint.h>

#include "navigation/flood_fill.h"
#include "navigation/maze.h"
#include "navigation/pose.h"
#include "util/status.h"

namespace micromouse {

/// One wall observation from one sensor, taken while stopped.
struct SideObservation {
  Relative side;  ///< sensor direction relative to the heading
  bool wall;      ///< classified as a wall
  bool anyEcho;   ///< at least one ping of the pass got an echo
};

struct ExplorerParams {
  uint8_t wallFlipObservations;  ///< see Maze (1..4)
  uint8_t sensorFailurePasses;   ///< implausible passes -> kSensorFailure
  uint8_t budgetFactor;          ///< max cells driven = factor * cells
};

/// Explores towards a goal with flood fill.
///
/// Sensor plausibility: a pass in which a sensor got no echo at all
/// although the map already has a wall right beside the robot on that side
/// is implausible (a working sensor would see that wall). Such an
/// observation is NOT applied to the map (a dead sensor must not erase
/// walls), and `sensorFailurePasses` of them in a row for the same sensor
/// fail with kSensorFailure. Any echo resets the count; "no echo" facing a
/// side not known to be walled is normal (open corridor) and neutral.
class Explorer {
 public:
  /// Starts a new run. kInvalidArgument / kOutOfBounds on bad parameters.
  Status reset(uint8_t rows, uint8_t cols, Pose start, Cell goal,
               const ExplorerParams& params);

  /// Changes the goal (e.g. for a return-to-start mission).
  Status setGoal(Cell goal);

  bool atGoal() const { return pose_.cell == goal_; }

  /// Applies the observations of one SENSE pass at the current pose.
  /// kSensorFailure if a sensor has been implausible too often.
  Status observe(const SideObservation* observations, uint8_t count);

  /// Recomputes the flood fill and picks the direction to drive.
  /// kExplorationLimit once more than budget() cells have been driven,
  /// kNoPath if the goal is unreachable with the known walls.
  Status plan(Direction& out);

  /// Records a completed one-cell move in `moved` (pose and counter).
  /// kOutOfBounds (pose unchanged) if that leaves the maze.
  Status advance(Direction moved);

  const Maze& maze() const { return maze_; }
  const FloodFill& flood() const { return flood_; }
  Pose pose() const { return pose_; }
  Cell goal() const { return goal_; }
  uint16_t cellsMoved() const { return cellsMoved_; }
  uint16_t budget() const { return budget_; }

 private:
  Maze maze_;
  FloodFill flood_;
  Pose pose_ = {{0, 0}, Direction::kUp};
  Cell goal_ = {0, 0};
  uint16_t cellsMoved_ = 0;
  uint16_t budget_ = 0;
  uint8_t sensorFailurePasses_ = 1;
  uint8_t implausibleStreak_[kDirectionCount] = {};  // indexed by Relative
};

}  // namespace micromouse
