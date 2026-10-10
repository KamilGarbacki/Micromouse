#include "navigation/explorer.h"

namespace micromouse {

Status Explorer::reset(const uint8_t rows, const uint8_t cols,
                       const Pose start, const Cell goal,
                       const ExplorerParams& params) {
  cellsMoved_ = 0;
  budget_ = 0;
  for (uint8_t& streak : implausibleStreak_) streak = 0;

  const Status status = maze_.reset(rows, cols, params.wallFlipObservations);
  if (!isOk(status)) return status;
  if (params.sensorFailurePasses == 0 || params.budgetFactor == 0) {
    return Status::kInvalidArgument;
  }
  if (!maze_.contains(start.cell) || !maze_.contains(goal)) {
    return Status::kOutOfBounds;
  }
  pose_ = start;
  goal_ = goal;
  sensorFailurePasses_ = params.sensorFailurePasses;
  budget_ = static_cast<uint16_t>(maze_.cellCount() * params.budgetFactor);
  return Status::kOk;
}

Status Explorer::setGoal(const Cell goal) {
  if (!maze_.contains(goal)) return Status::kOutOfBounds;
  goal_ = goal;
  return Status::kOk;
}

Status Explorer::observe(const SideObservation* const observations,
                         const uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    const SideObservation& obs = observations[i];
    const Direction dir = rotate(pose_.heading, obs.side);
    uint8_t& streak = implausibleStreak_[static_cast<uint8_t>(obs.side)];

    if (obs.anyEcho) {
      streak = 0;
    } else if (maze_.hasWall(pose_.cell, dir)) {
      // Silence facing a known wall: do not trust it, do not map it.
      if (++streak >= sensorFailurePasses_) return Status::kSensorFailure;
      continue;
    }
    const Status status = maze_.setWall(pose_.cell, dir, obs.wall);
    if (!isOk(status)) return status;
  }
  return Status::kOk;
}

Status Explorer::plan(Direction& out) {
  if (cellsMoved_ > budget_) return Status::kExplorationLimit;
  const Status status = flood_.compute(maze_, goal_);
  if (!isOk(status)) return status;
  return flood_.bestDirection(maze_, pose_.cell, pose_.heading, out);
}

Status Explorer::advance(const Direction moved) {
  Cell next{};
  if (!maze_.neighbour(pose_.cell, moved, next)) return Status::kOutOfBounds;
  pose_ = Pose{next, moved};
  if (cellsMoved_ < 0xFFFFu) ++cellsMoved_;
  return Status::kOk;
}

}  // namespace micromouse
