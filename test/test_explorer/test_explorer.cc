// Host tests: end-to-end SENSE -> PLAN -> MOVE simulation of the Explorer
// against known mazes with a fake sensor that reveals the true walls.
#include <unity.h>

#include "config/config.h"
#include "navigation/explorer.h"

using namespace micromouse;

namespace {

constexpr uint8_t kRows = 5;
constexpr uint8_t kCols = 4;
constexpr Pose kStart = {{4, 0}, Direction::kUp};
constexpr Cell kGoal = {0, 3};
// Physical boundary: the classic walled maze (the boundary echoes).
constexpr ExplorerParams kParams = {2, 3, 4, true};
// Open floor: the boundary is virtual (default of the firmware config).
constexpr ExplorerParams kOpenFloor = {2, 3, 4, false};
constexpr Relative kSides[3] = {Relative::kLeft, Relative::kForward,
                                Relative::kRight};

struct WallSpec {
  Cell cell;
  Direction dir;
};

/// The real maze: all listed walls present, everything else open.
Maze makeTruth(const WallSpec* walls, const uint8_t count) {
  Maze truth;
  truth.reset(kRows, kCols, 1);
  for (uint8_t r = 0; r < kRows; ++r) {
    for (uint8_t c = 0; c < kCols; ++c) {
      for (const Direction d : kAllDirections) {
        truth.setWall(Cell{r, c}, d, false);
      }
    }
  }
  for (uint8_t i = 0; i < count; ++i) {
    truth.setWall(walls[i].cell, walls[i].dir, true);
  }
  return truth;
}

/// Optional sensor corruption: returns the (possibly altered) wall reading.
using Corruption = bool (*)(Cell cell, Direction dir, bool trueWall);

struct RunResult {
  Status status;
  uint16_t moves;
  bool crashed;  // tried to drive through a real wall
};

RunResult explore(Explorer& explorer, const Maze& truth,
                  const Corruption corrupt = nullptr) {
  const uint16_t kSafetySteps = 1000;
  for (uint16_t step = 0; step < kSafetySteps; ++step) {
    if (explorer.atGoal()) {
      return RunResult{Status::kOk, explorer.cellsMoved(), false};
    }
    const Pose pose = explorer.pose();
    SideObservation obs[3];
    for (uint8_t i = 0; i < 3; ++i) {
      const Direction dir = rotate(pose.heading, kSides[i]);
      bool wall = truth.hasWall(pose.cell, dir);
      if (corrupt != nullptr) wall = corrupt(pose.cell, dir, wall);
      obs[i] = SideObservation{kSides[i], wall, true};
    }
    Status status = explorer.observe(obs, 3);
    if (!isOk(status)) return RunResult{status, explorer.cellsMoved(), false};

    Direction dir = Direction::kUp;
    status = explorer.plan(dir);
    if (!isOk(status)) return RunResult{status, explorer.cellsMoved(), false};
    if (truth.hasWall(pose.cell, dir)) {
      return RunResult{Status::kStall, explorer.cellsMoved(), true};
    }
    status = explorer.advance(dir);
    if (!isOk(status)) return RunResult{status, explorer.cellsMoved(), false};
  }
  return RunResult{Status::kExplorationLimit, explorer.cellsMoved(), false};
}

/// A mat: the sensors see only PHYSICAL walls (interior walls, plus the
/// boundary if `physicalBoundary`) and are completely silent (no echo at
/// all) facing anything else - the case that faulted on the real robot.
RunResult exploreSilent(Explorer& explorer, const Maze& truth,
                        const bool physicalBoundary) {
  const uint16_t kSafetySteps = 1000;
  for (uint16_t step = 0; step < kSafetySteps; ++step) {
    if (explorer.atGoal()) {
      return RunResult{Status::kOk, explorer.cellsMoved(), false};
    }
    const Pose pose = explorer.pose();
    SideObservation obs[3];
    for (uint8_t i = 0; i < 3; ++i) {
      const Direction dir = rotate(pose.heading, kSides[i]);
      const bool wall = truth.hasWall(pose.cell, dir) &&
                        (physicalBoundary || !truth.isBoundary(pose.cell, dir));
      obs[i] = SideObservation{kSides[i], wall, wall};
    }
    Status status = explorer.observe(obs, 3);
    if (!isOk(status)) return RunResult{status, explorer.cellsMoved(), false};
    Direction dir = Direction::kUp;
    status = explorer.plan(dir);
    if (!isOk(status)) return RunResult{status, explorer.cellsMoved(), false};
    if (truth.hasWall(pose.cell, dir)) {
      return RunResult{Status::kStall, explorer.cellsMoved(), true};
    }
    status = explorer.advance(dir);
    if (!isOk(status)) return RunResult{status, explorer.cellsMoved(), false};
  }
  return RunResult{Status::kExplorationLimit, explorer.cellsMoved(), false};
}

// Column 0 is a corridor closed on its right from row 0 to 3: going Up from
// the start (preferred: straight on, same optimistic distance) is a dead
// end at (0,0); the only way out is back down and right along row 4.
constexpr WallSpec kDeadEnd[] = {
    {{0, 0}, Direction::kRight},
    {{1, 0}, Direction::kRight},
    {{2, 0}, Direction::kRight},
    {{3, 0}, Direction::kRight},
};

// Serpentine: rows joined alternately at the right and the left end.
constexpr WallSpec kSerpentine[] = {
    {{3, 1}, Direction::kDown}, {{3, 2}, Direction::kDown},
    {{3, 3}, Direction::kDown},  // row 4 -> 3 only at col 0
    {{2, 0}, Direction::kDown}, {{2, 1}, Direction::kDown},
    {{2, 2}, Direction::kDown},  // row 3 -> 2 only at col 3
    {{1, 1}, Direction::kDown}, {{1, 2}, Direction::kDown},
    {{1, 3}, Direction::kDown},  // row 2 -> 1 only at col 0
    {{0, 0}, Direction::kDown}, {{0, 1}, Direction::kDown},
    {{0, 2}, Direction::kDown},  // row 1 -> 0 only at col 3
};

// Like kDeadEnd but the corridor is open to the right at row 2, which is
// where the flickering reading is injected.
constexpr WallSpec kFlickerMaze[] = {
    {{0, 0}, Direction::kRight},
    {{1, 0}, Direction::kRight},
    {{3, 0}, Direction::kRight},
};

// Goal (0,3) walled in on its two interior sides.
constexpr WallSpec kEnclosedGoal[] = {
    {{0, 3}, Direction::kLeft},
    {{0, 3}, Direction::kDown},
};

}  // namespace

void setUp() {}
void tearDown() {}

void test_open_maze_reaches_goal_by_shortest_path() {
  const Maze truth = makeTruth(nullptr, 0);
  Explorer explorer;
  TEST_ASSERT_TRUE(explorer.reset(kRows, kCols, kStart, kGoal, kParams) ==
                   Status::kOk);
  const RunResult r = explore(explorer, truth);
  TEST_ASSERT_TRUE(r.status == Status::kOk);
  TEST_ASSERT_EQUAL_UINT16(7, r.moves);  // Manhattan distance 4 + 3
}

void test_dead_end_is_backed_out_of() {
  const Maze truth = makeTruth(kDeadEnd, sizeof(kDeadEnd) / sizeof(WallSpec));
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kParams);
  const RunResult r = explore(explorer, truth);
  TEST_ASSERT_FALSE(r.crashed);
  TEST_ASSERT_TRUE(r.status == Status::kOk);
  TEST_ASSERT_TRUE(r.moves > 7);  // it did explore the dead end
  TEST_ASSERT_TRUE(r.moves <= explorer.budget());
}

void test_serpentine_reaches_goal() {
  const Maze truth =
      makeTruth(kSerpentine, sizeof(kSerpentine) / sizeof(WallSpec));
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kParams);
  const RunResult r = explore(explorer, truth);
  TEST_ASSERT_FALSE(r.crashed);
  TEST_ASSERT_TRUE(r.status == Status::kOk);
  TEST_ASSERT_EQUAL_UINT16(13, r.moves);  // the only path is 13 cells
  TEST_ASSERT_TRUE(r.moves <= explorer.budget());
}

void test_enclosed_goal_reports_no_path() {
  const Maze truth =
      makeTruth(kEnclosedGoal, sizeof(kEnclosedGoal) / sizeof(WallSpec));
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kParams);
  const RunResult r = explore(explorer, truth);
  TEST_ASSERT_FALSE(r.crashed);
  TEST_ASSERT_TRUE(r.status == Status::kNoPath);
}

namespace {
// The wall between (2,0) and (2,1) is really open; its reading alternates
// wall / open on every observation (starting open), from both sides.
uint8_t gFlickerCount = 0;
bool flicker(const Cell cell, const Direction dir, const bool trueWall) {
  const bool isFlickerWall =
      (cell == Cell{2, 0} && dir == Direction::kRight) ||
      (cell == Cell{2, 1} && dir == Direction::kLeft);
  if (!isFlickerWall) return trueWall;
  return (gFlickerCount++ % 2) == 1;
}
}  // namespace

void test_flickering_wall_does_not_livelock() {
  gFlickerCount = 0;
  const Maze truth =
      makeTruth(kFlickerMaze, sizeof(kFlickerMaze) / sizeof(WallSpec));
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kParams);
  const RunResult r = explore(explorer, truth, flicker);
  TEST_ASSERT_TRUE(gFlickerCount >= 2);  // the flicker was really seen
  TEST_ASSERT_FALSE(r.crashed);
  TEST_ASSERT_TRUE(r.status == Status::kOk);
  TEST_ASSERT_TRUE(r.moves <= explorer.budget());
}

void test_exploration_budget() {
  Explorer explorer;
  explorer.reset(3, 3, Pose{{2, 0}, Direction::kUp}, Cell{0, 2},
                 ExplorerParams{2, 3, 1});
  TEST_ASSERT_EQUAL_UINT16(9, explorer.budget());
  for (uint8_t i = 0; i < 10; ++i) {
    explorer.advance(i % 2 == 0 ? Direction::kUp : Direction::kDown);
  }
  Direction dir = Direction::kUp;
  TEST_ASSERT_TRUE(explorer.plan(dir) == Status::kExplorationLimit);
}

void test_silent_sensor_facing_known_wall_is_a_failure() {
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kParams);
  // Start (4,0) facing Up: the left side is the outer boundary, a physical
  // wall in this mode (kParams).
  const SideObservation silentLeft[1] = {{Relative::kLeft, false, false}};
  TEST_ASSERT_TRUE(explorer.observe(silentLeft, 1) == Status::kOk);
  TEST_ASSERT_TRUE(explorer.observe(silentLeft, 1) == Status::kOk);
  TEST_ASSERT_TRUE(explorer.observe(silentLeft, 1) == Status::kSensorFailure);
}

void test_echo_resets_failure_count_and_open_corridor_is_fine() {
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kParams);
  const SideObservation silentLeft[1] = {{Relative::kLeft, false, false}};
  const SideObservation echoLeft[1] = {{Relative::kLeft, true, true}};
  explorer.observe(silentLeft, 1);
  explorer.observe(silentLeft, 1);
  explorer.observe(echoLeft, 1);
  TEST_ASSERT_TRUE(explorer.observe(silentLeft, 1) == Status::kOk);
  TEST_ASSERT_TRUE(explorer.observe(silentLeft, 1) == Status::kOk);

  // Front faces an unknown interior side: silence is a legitimate open
  // corridor and never counts, however often it happens.
  const SideObservation silentFront[1] = {{Relative::kForward, false, false}};
  for (uint8_t i = 0; i < 20; ++i) {
    TEST_ASSERT_TRUE(explorer.observe(silentFront, 1) == Status::kOk);
  }
}

void test_silent_sensor_does_not_erase_known_wall() {
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, ExplorerParams{1, 5, 4});
  const SideObservation wallFront[1] = {{Relative::kForward, true, true}};
  explorer.observe(wallFront, 1);
  TEST_ASSERT_TRUE(explorer.maze().hasWall(Cell{4, 0}, Direction::kUp));
  // Even with "latest wins" (N = 1), silence does not remove the wall.
  const SideObservation silentFront[1] = {{Relative::kForward, false, false}};
  explorer.observe(silentFront, 1);
  TEST_ASSERT_TRUE(explorer.maze().hasWall(Cell{4, 0}, Direction::kUp));
}

void test_reset_validates_arguments() {
  Explorer explorer;
  TEST_ASSERT_TRUE(explorer.reset(kRows, kCols, Pose{{5, 0}, Direction::kUp},
                                  kGoal, kParams) == Status::kOutOfBounds);
  TEST_ASSERT_TRUE(explorer.reset(kRows, kCols, kStart, kGoal,
                                  ExplorerParams{0, 3, 4}) ==
                   Status::kInvalidArgument);
  TEST_ASSERT_TRUE(explorer.reset(kRows, kCols, kStart, kGoal,
                                  ExplorerParams{2, 0, 4}) ==
                   Status::kInvalidArgument);
  TEST_ASSERT_TRUE(explorer.reset(kRows, kCols, kStart, kGoal, kParams) ==
                   Status::kOk);
  TEST_ASSERT_TRUE(explorer.setGoal(Cell{9, 9}) == Status::kOutOfBounds);
  TEST_ASSERT_TRUE(explorer.setGoal(Cell{4, 0}) == Status::kOk);
  TEST_ASSERT_TRUE(explorer.atGoal());
}

void test_configured_maze_is_valid() {
  Explorer explorer;
  const ExplorerParams params = {config::kWallFlipObservations,
                                 config::kSensorFailurePasses,
                                 config::kExplorationBudgetFactor,
                                 config::kPhysicalBoundary};
  TEST_ASSERT_TRUE(
      explorer.reset(config::kMazeRows, config::kMazeCols,
                     Pose{config::kStartCell, config::kStartHeading},
                     config::kGoalCell, params) == Status::kOk);
}

// --- Open floor (virtual boundary) vs physical boundary ---------------------

namespace {
// The real-robot report: the configured 5x3 mat, start (4,0) facing Up,
// goal (0,2), no walls at all, kSensorFailurePasses = 3.
constexpr uint8_t kMatRows = 5;
constexpr uint8_t kMatCols = 3;
constexpr Pose kMatStart = {{4, 0}, Direction::kUp};
constexpr Cell kMatGoal = {0, 2};

Maze makeOpenTruth(const uint8_t rows, const uint8_t cols) {
  Maze truth;
  truth.reset(rows, cols, 1);
  for (uint8_t r = 0; r < rows; ++r) {
    for (uint8_t c = 0; c < cols; ++c) {
      for (const Direction d : kAllDirections) {
        truth.setWall(Cell{r, c}, d, false);
      }
    }
  }
  return truth;
}
}  // namespace

void test_default_mode_is_the_config() {
  const ExplorerParams params = {2, 3, 4};
  TEST_ASSERT_EQUAL(config::kPhysicalBoundary, params.physicalBoundary);
  TEST_ASSERT_FALSE(config::kPhysicalBoundary);  // open floor is the default
}

void test_open_floor_silent_sensors_reach_goal_without_fault() {
  const Maze truth = makeOpenTruth(kMatRows, kMatCols);
  Explorer explorer;
  TEST_ASSERT_TRUE(explorer.reset(kMatRows, kMatCols, kMatStart, kMatGoal,
                                  kOpenFloor) == Status::kOk);
  const RunResult r = exploreSilent(explorer, truth, false);
  TEST_ASSERT_TRUE(r.status == Status::kOk);
  TEST_ASSERT_FALSE(r.crashed);
  TEST_ASSERT_EQUAL_UINT16(6, r.moves);  // Manhattan distance 4 + 2
  // Never planned off the grid, and the edge is still a (virtual) wall.
  TEST_ASSERT_TRUE(explorer.maze().hasWall(Cell{4, 0}, Direction::kLeft));
  TEST_ASSERT_FALSE(explorer.physicalWall(Cell{4, 0}, Direction::kLeft));

  // Same on the larger test maze.
  Explorer big;
  big.reset(kRows, kCols, kStart, kGoal, kOpenFloor);
  const RunResult rb = exploreSilent(big, makeTruth(nullptr, 0), false);
  TEST_ASSERT_TRUE(rb.status == Status::kOk);
  TEST_ASSERT_EQUAL_UINT16(7, rb.moves);
}

void test_physical_boundary_silence_facing_boundary_faults() {
  // The old behaviour, reproducing the real-robot log: up column 0 the left
  // sensor faces the west boundary; silent 3 passes in a row -> FAULT
  // SENSOR_FAILURE at (2,0)@U after 2 moves.
  const Maze truth = makeOpenTruth(kMatRows, kMatCols);
  Explorer explorer;
  explorer.reset(kMatRows, kMatCols, kMatStart, kMatGoal, kParams);
  const RunResult r = exploreSilent(explorer, truth, false);
  TEST_ASSERT_TRUE(r.status == Status::kSensorFailure);
  TEST_ASSERT_EQUAL_UINT16(2, r.moves);
  TEST_ASSERT_TRUE(explorer.pose().cell == (Cell{2, 0}));
  TEST_ASSERT_TRUE(explorer.pose().heading == Direction::kUp);

  // With a real boundary that echoes, the same mat is fine.
  Explorer walled;
  walled.reset(kMatRows, kMatCols, kMatStart, kMatGoal, kParams);
  const RunResult rw = exploreSilent(walled, truth, true);
  TEST_ASSERT_TRUE(rw.status == Status::kOk);
  TEST_ASSERT_EQUAL_UINT16(6, rw.moves);
}

void test_open_floor_silence_at_virtual_boundary_is_neutral() {
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kOpenFloor);
  // Start (4,0) facing Up: left = west edge, right is interior.
  const SideObservation silentLeft[1] = {{Relative::kLeft, false, false}};
  for (uint8_t i = 0; i < 20; ++i) {
    TEST_ASSERT_TRUE(explorer.observe(silentLeft, 1) == Status::kOk);
  }
  // A "wall" reading at the virtual edge (e.g. a chair leg off the mat)
  // does not make it physical either; it stays impassable.
  const SideObservation wallLeft[1] = {{Relative::kLeft, true, true}};
  explorer.observe(wallLeft, 1);
  TEST_ASSERT_TRUE(explorer.maze().hasWall(Cell{4, 0}, Direction::kLeft));
  TEST_ASSERT_FALSE(explorer.maze().isKnown(Cell{4, 0}, Direction::kLeft));
  TEST_ASSERT_FALSE(explorer.physicalWallAt(Relative::kLeft));
  for (uint8_t i = 0; i < 20; ++i) {
    TEST_ASSERT_TRUE(explorer.observe(silentLeft, 1) == Status::kOk);
  }
}

void test_open_floor_interior_wall_still_arms_sensor_failure() {
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kOpenFloor);
  // A real (interior) wall ahead of the start: physical, so silence facing
  // it is implausible, exactly as in a walled maze.
  const SideObservation wallFront[1] = {{Relative::kForward, true, true}};
  TEST_ASSERT_TRUE(explorer.observe(wallFront, 1) == Status::kOk);
  TEST_ASSERT_TRUE(explorer.physicalWallAt(Relative::kForward));
  const SideObservation silentFront[1] = {{Relative::kForward, false, false}};
  TEST_ASSERT_TRUE(explorer.observe(silentFront, 1) == Status::kOk);
  TEST_ASSERT_TRUE(explorer.observe(silentFront, 1) == Status::kOk);
  TEST_ASSERT_TRUE(explorer.observe(silentFront, 1) ==
                   Status::kSensorFailure);
  // ... and the silence did not erase the wall.
  TEST_ASSERT_TRUE(explorer.maze().hasWall(Cell{4, 0}, Direction::kUp));
}

void test_open_floor_with_interior_walls_reaches_goal() {
  // Interior walls on an open mat: seen and mapped, dead end backed out of.
  const Maze truth = makeTruth(kDeadEnd, sizeof(kDeadEnd) / sizeof(WallSpec));
  Explorer explorer;
  explorer.reset(kRows, kCols, kStart, kGoal, kOpenFloor);
  const RunResult r = exploreSilent(explorer, truth, false);
  TEST_ASSERT_FALSE(r.crashed);
  TEST_ASSERT_TRUE(r.status == Status::kOk);
  TEST_ASSERT_TRUE(r.moves > 7);
  TEST_ASSERT_TRUE(explorer.physicalWall(Cell{1, 0}, Direction::kRight));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_open_maze_reaches_goal_by_shortest_path);
  RUN_TEST(test_dead_end_is_backed_out_of);
  RUN_TEST(test_serpentine_reaches_goal);
  RUN_TEST(test_enclosed_goal_reports_no_path);
  RUN_TEST(test_flickering_wall_does_not_livelock);
  RUN_TEST(test_exploration_budget);
  RUN_TEST(test_silent_sensor_facing_known_wall_is_a_failure);
  RUN_TEST(test_echo_resets_failure_count_and_open_corridor_is_fine);
  RUN_TEST(test_silent_sensor_does_not_erase_known_wall);
  RUN_TEST(test_reset_validates_arguments);
  RUN_TEST(test_configured_maze_is_valid);
  RUN_TEST(test_default_mode_is_the_config);
  RUN_TEST(test_open_floor_silent_sensors_reach_goal_without_fault);
  RUN_TEST(test_physical_boundary_silence_facing_boundary_faults);
  RUN_TEST(test_open_floor_silence_at_virtual_boundary_is_neutral);
  RUN_TEST(test_open_floor_interior_wall_still_arms_sensor_failure);
  RUN_TEST(test_open_floor_with_interior_walls_reaches_goal);
  return UNITY_END();
}
