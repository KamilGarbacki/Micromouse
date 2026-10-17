// Host tests: heading math, maze walls, flood fill and move selection.
#include <unity.h>

#include "navigation/flood_fill.h"
#include "navigation/maze.h"
#include "navigation/maze_render.h"
#include "navigation/pose.h"

#include <string.h>

using namespace micromouse;

void setUp() {}
void tearDown() {}

// --- Pose / heading math ---------------------------------------------------

void test_rotate_and_relative_are_inverse() {
  for (const Direction from : kAllDirections) {
    for (const Direction to : kAllDirections) {
      TEST_ASSERT_TRUE(rotate(from, relativeTurn(from, to)) == to);
    }
  }
}

void test_rotate_examples() {
  TEST_ASSERT_TRUE(rotate(Direction::kUp, Relative::kRight) ==
                   Direction::kRight);
  TEST_ASSERT_TRUE(rotate(Direction::kUp, Relative::kLeft) ==
                   Direction::kLeft);
  TEST_ASSERT_TRUE(rotate(Direction::kLeft, Relative::kRight) ==
                   Direction::kUp);
  TEST_ASSERT_TRUE(rotate(Direction::kRight, Relative::kBack) ==
                   Direction::kLeft);
  TEST_ASSERT_TRUE(opposite(Direction::kDown) == Direction::kUp);
}

void test_relative_turn_and_quarter_turns() {
  TEST_ASSERT_TRUE(relativeTurn(Direction::kUp, Direction::kUp) ==
                   Relative::kForward);
  TEST_ASSERT_TRUE(relativeTurn(Direction::kUp, Direction::kLeft) ==
                   Relative::kLeft);
  TEST_ASSERT_TRUE(relativeTurn(Direction::kLeft, Direction::kUp) ==
                   Relative::kRight);
  TEST_ASSERT_TRUE(relativeTurn(Direction::kDown, Direction::kUp) ==
                   Relative::kBack);
  TEST_ASSERT_EQUAL_UINT8(0, quarterTurns(Relative::kForward));
  TEST_ASSERT_EQUAL_UINT8(1, quarterTurns(Relative::kRight));
  TEST_ASSERT_EQUAL_UINT8(1, quarterTurns(Relative::kLeft));
  TEST_ASSERT_EQUAL_UINT8(2, quarterTurns(Relative::kBack));
}

void test_steps_follow_row_col_convention() {
  TEST_ASSERT_EQUAL_INT8(-1, rowStep(Direction::kUp));
  TEST_ASSERT_EQUAL_INT8(1, rowStep(Direction::kDown));
  TEST_ASSERT_EQUAL_INT8(1, colStep(Direction::kRight));
  TEST_ASSERT_EQUAL_INT8(-1, colStep(Direction::kLeft));
  TEST_ASSERT_EQUAL_INT8(0, colStep(Direction::kUp));
}

// --- Maze walls ------------------------------------------------------------

void test_reset_rejects_bad_sizes() {
  Maze maze;
  TEST_ASSERT_TRUE(maze.reset(0, 3) == Status::kInvalidArgument);
  TEST_ASSERT_TRUE(maze.reset(Maze::kMaxRows + 1, 3) ==
                   Status::kInvalidArgument);
  TEST_ASSERT_EQUAL_UINT8(0, maze.rows());
  TEST_ASSERT_TRUE(maze.reset(5, 4) == Status::kOk);
  TEST_ASSERT_EQUAL_UINT8(5, maze.rows());
  TEST_ASSERT_EQUAL_UINT8(4, maze.cols());
}

void test_boundary_is_known_wall_interior_unknown() {
  Maze maze;
  maze.reset(3, 3);
  TEST_ASSERT_TRUE(maze.hasWall(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_TRUE(maze.hasWall(Cell{0, 0}, Direction::kLeft));
  TEST_ASSERT_TRUE(maze.isKnown(Cell{2, 2}, Direction::kDown));
  TEST_ASSERT_FALSE(maze.hasWall(Cell{1, 1}, Direction::kUp));
  TEST_ASSERT_FALSE(maze.isKnown(Cell{1, 1}, Direction::kUp));
}

void test_virtual_boundary_blocks_travel_but_is_not_physical() {
  // Open floor (physicalBoundary = false): the edge still blocks travel
  // (hasWall), but it is not a known / physical wall a sensor could see.
  Maze maze;
  TEST_ASSERT_TRUE(maze.reset(3, 3, 2, false) == Status::kOk);
  TEST_ASSERT_TRUE(maze.isBoundary(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_TRUE(maze.isBoundary(Cell{0, 0}, Direction::kLeft));
  TEST_ASSERT_FALSE(maze.isBoundary(Cell{0, 0}, Direction::kRight));
  TEST_ASSERT_FALSE(maze.isBoundary(Cell{1, 1}, Direction::kUp));
  TEST_ASSERT_TRUE(maze.hasWall(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_FALSE(maze.isKnown(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_FALSE(maze.hasPhysicalWall(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_TRUE(maze.hasWall(Cell{2, 2}, Direction::kDown));
  TEST_ASSERT_FALSE(maze.hasPhysicalWall(Cell{2, 2}, Direction::kDown));

  // Readings at the virtual boundary (either way) change nothing.
  TEST_ASSERT_TRUE(maze.setWall(Cell{0, 0}, Direction::kUp, false) ==
                   Status::kOk);
  TEST_ASSERT_TRUE(maze.setWall(Cell{0, 0}, Direction::kLeft, true) ==
                   Status::kOk);
  TEST_ASSERT_TRUE(maze.hasWall(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_FALSE(maze.hasPhysicalWall(Cell{0, 0}, Direction::kLeft));
  TEST_ASSERT_FALSE(maze.isKnown(Cell{0, 0}, Direction::kLeft));

  // An observed interior wall is physical in both modes.
  maze.setWall(Cell{1, 1}, Direction::kRight, true);
  TEST_ASSERT_TRUE(maze.hasPhysicalWall(Cell{1, 1}, Direction::kRight));
  TEST_ASSERT_TRUE(maze.hasPhysicalWall(Cell{1, 2}, Direction::kLeft));

  // The planner never leaves the grid: a 1x3 strip from the left end goes
  // Right, never Up/Down/Left through the virtual edge.
  Maze strip;
  strip.reset(1, 3, 1, false);
  FloodFill flood;
  TEST_ASSERT_TRUE(flood.compute(strip, Cell{0, 2}) == Status::kOk);
  TEST_ASSERT_EQUAL_UINT8(2, flood.distance(Cell{0, 0}));
  Direction dir = Direction::kUp;
  TEST_ASSERT_TRUE(strip.hasWall(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_TRUE(flood.bestDirection(strip, Cell{0, 0}, Direction::kUp,
                                       dir) == Status::kOk);
  TEST_ASSERT_TRUE(dir == Direction::kRight);

  // Physical boundary (the default of Maze::reset): known, physical.
  Maze walled;
  walled.reset(3, 3);
  TEST_ASSERT_TRUE(walled.hasPhysicalWall(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_TRUE(walled.isKnown(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_FALSE(walled.hasPhysicalWall(Cell{1, 1}, Direction::kUp));
}

void test_set_wall_is_mirrored_to_neighbour() {
  Maze maze;
  maze.reset(3, 3);
  TEST_ASSERT_TRUE(maze.setWall(Cell{1, 1}, Direction::kRight, true) ==
                   Status::kOk);
  TEST_ASSERT_TRUE(maze.hasWall(Cell{1, 1}, Direction::kRight));
  TEST_ASSERT_TRUE(maze.hasWall(Cell{1, 2}, Direction::kLeft));
  TEST_ASSERT_TRUE(maze.isKnown(Cell{1, 2}, Direction::kLeft));

  // Contradictions from either side count towards the same wall (N = 2).
  maze.setWall(Cell{1, 2}, Direction::kLeft, false);
  TEST_ASSERT_TRUE(maze.hasWall(Cell{1, 1}, Direction::kRight));
  maze.setWall(Cell{1, 1}, Direction::kRight, false);
  TEST_ASSERT_FALSE(maze.hasWall(Cell{1, 1}, Direction::kRight));
  TEST_ASSERT_FALSE(maze.hasWall(Cell{1, 2}, Direction::kLeft));
  TEST_ASSERT_TRUE(maze.isKnown(Cell{1, 1}, Direction::kRight));

  // Row/col convention: Up of (1,1) is Down of (0,1).
  maze.setWall(Cell{1, 1}, Direction::kUp, true);
  TEST_ASSERT_TRUE(maze.hasWall(Cell{0, 1}, Direction::kDown));
  TEST_ASSERT_FALSE(maze.hasWall(Cell{1, 0}, Direction::kUp));
}

void test_boundary_walls_cannot_be_opened_and_bad_cells_rejected() {
  Maze maze;
  maze.reset(3, 3);
  TEST_ASSERT_TRUE(maze.setWall(Cell{0, 0}, Direction::kUp, false) ==
                   Status::kOk);
  TEST_ASSERT_TRUE(maze.hasWall(Cell{0, 0}, Direction::kUp));
  TEST_ASSERT_TRUE(maze.setWall(Cell{3, 0}, Direction::kUp, true) ==
                   Status::kOutOfBounds);
  Cell out{};
  TEST_ASSERT_FALSE(maze.neighbour(Cell{0, 2}, Direction::kRight, out));
  TEST_ASSERT_TRUE(maze.neighbour(Cell{0, 2}, Direction::kDown, out));
  TEST_ASSERT_TRUE(out == (Cell{1, 2}));
}

void test_sticky_wall_needs_consecutive_contradictions() {
  Maze maze;
  TEST_ASSERT_TRUE(maze.reset(3, 3, 3) == Status::kOk);
  const Cell c{1, 1};
  maze.setWall(c, Direction::kUp, false);  // unknown -> known: immediate
  TEST_ASSERT_TRUE(maze.isKnown(c, Direction::kUp));
  TEST_ASSERT_FALSE(maze.hasWall(c, Direction::kUp));
  // Flicker: wall, wall, open (resets), wall, wall -> never 3 in a row.
  maze.setWall(c, Direction::kUp, true);
  maze.setWall(c, Direction::kUp, true);
  maze.setWall(c, Direction::kUp, false);
  maze.setWall(c, Direction::kUp, true);
  maze.setWall(c, Direction::kUp, true);
  TEST_ASSERT_FALSE(maze.hasWall(c, Direction::kUp));
  maze.setWall(c, Direction::kUp, true);  // third in a row: flips
  TEST_ASSERT_TRUE(maze.hasWall(c, Direction::kUp));
  TEST_ASSERT_TRUE(maze.hasWall(Cell{0, 1}, Direction::kDown));
}

void test_flip_threshold_one_is_latest_wins_and_range_checked() {
  Maze maze;
  TEST_ASSERT_TRUE(maze.reset(3, 3, 1) == Status::kOk);
  maze.setWall(Cell{1, 1}, Direction::kLeft, true);
  maze.setWall(Cell{1, 1}, Direction::kLeft, false);
  TEST_ASSERT_FALSE(maze.hasWall(Cell{1, 1}, Direction::kLeft));
  TEST_ASSERT_TRUE(maze.reset(3, 3, 0) == Status::kInvalidArgument);
  TEST_ASSERT_TRUE(maze.reset(3, 3, Maze::kMaxFlipObservations + 1) ==
                   Status::kInvalidArgument);
}

void test_index_round_trip() {
  Maze maze;
  maze.reset(5, 4);
  for (uint8_t r = 0; r < 5; ++r) {
    for (uint8_t c = 0; c < 4; ++c) {
      const Cell cell{r, c};
      TEST_ASSERT_TRUE(maze.cellAt(maze.indexOf(cell)) == cell);
    }
  }
}

// --- Flood fill ------------------------------------------------------------

void test_open_maze_gives_manhattan_distance() {
  Maze maze;
  maze.reset(5, 4);
  FloodFill flood;
  TEST_ASSERT_TRUE(flood.compute(maze, Cell{0, 3}) == Status::kOk);
  for (uint8_t r = 0; r < 5; ++r) {
    for (uint8_t c = 0; c < 4; ++c) {
      TEST_ASSERT_EQUAL_UINT8(r + (3 - c), flood.distance(Cell{r, c}));
    }
  }
}

void test_walls_lengthen_paths() {
  // 3x3, goal (0,0). Wall between (0,0) and (0,1), and (0,0)/(1,0) open.
  Maze maze;
  maze.reset(3, 3);
  maze.setWall(Cell{0, 0}, Direction::kRight, true);
  FloodFill flood;
  flood.compute(maze, Cell{0, 0});
  TEST_ASSERT_EQUAL_UINT8(1, flood.distance(Cell{1, 0}));
  TEST_ASSERT_EQUAL_UINT8(3, flood.distance(Cell{0, 1}));  // around
}

void test_unreachable_goal_reports_no_path() {
  Maze maze;
  maze.reset(3, 3);
  // Box in the centre goal.
  for (const Direction dir : kAllDirections) {
    maze.setWall(Cell{1, 1}, dir, true);
  }
  FloodFill flood;
  TEST_ASSERT_TRUE(flood.compute(maze, Cell{1, 1}) == Status::kOk);
  TEST_ASSERT_EQUAL_UINT8(FloodFill::kUnreached, flood.distance(Cell{0, 0}));
  Direction dir = Direction::kUp;
  TEST_ASSERT_TRUE(flood.bestDirection(maze, Cell{0, 0}, Direction::kUp,
                                       dir) == Status::kNoPath);
}

void test_goal_out_of_bounds_and_uncomputed() {
  Maze maze;
  maze.reset(3, 3);
  FloodFill flood;
  Direction dir = Direction::kUp;
  TEST_ASSERT_TRUE(flood.bestDirection(maze, Cell{0, 0}, Direction::kUp,
                                       dir) == Status::kNotInitialized);
  TEST_ASSERT_TRUE(flood.compute(maze, Cell{3, 3}) == Status::kOutOfBounds);
  TEST_ASSERT_EQUAL_UINT8(FloodFill::kUnreached, flood.distance(Cell{0, 0}));
}

void test_best_direction_prefers_current_heading() {
  // 3x3 open, goal (2,2). From (0,0) both Down and Right are closer.
  Maze maze;
  maze.reset(3, 3);
  FloodFill flood;
  flood.compute(maze, Cell{2, 2});
  Direction dir = Direction::kUp;

  TEST_ASSERT_TRUE(flood.bestDirection(maze, Cell{0, 0}, Direction::kRight,
                                       dir) == Status::kOk);
  TEST_ASSERT_TRUE(dir == Direction::kRight);

  flood.bestDirection(maze, Cell{0, 0}, Direction::kDown, dir);
  TEST_ASSERT_TRUE(dir == Direction::kDown);

  // Facing Up: Right is a 90 deg turn, Down would be a 180 -> Right.
  flood.bestDirection(maze, Cell{0, 0}, Direction::kUp, dir);
  TEST_ASSERT_TRUE(dir == Direction::kRight);

  // Facing Left: Down is a 90 deg (left) turn, Right a 180 -> Down.
  flood.bestDirection(maze, Cell{0, 0}, Direction::kLeft, dir);
  TEST_ASSERT_TRUE(dir == Direction::kDown);
}

void test_best_direction_respects_walls() {
  Maze maze;
  maze.reset(3, 3);
  maze.setWall(Cell{0, 0}, Direction::kRight, true);
  FloodFill flood;
  flood.compute(maze, Cell{2, 2});
  Direction dir = Direction::kUp;
  TEST_ASSERT_TRUE(flood.bestDirection(maze, Cell{0, 0}, Direction::kRight,
                                       dir) == Status::kOk);
  TEST_ASSERT_TRUE(dir == Direction::kDown);
}

void test_at_goal_there_is_no_closer_cell() {
  Maze maze;
  maze.reset(3, 3);
  FloodFill flood;
  flood.compute(maze, Cell{1, 1});
  Direction dir = Direction::kUp;
  TEST_ASSERT_TRUE(flood.bestDirection(maze, Cell{1, 1}, Direction::kUp,
                                       dir) == Status::kNoPath);
}

void test_resize_invalidates_flood() {
  Maze maze;
  maze.reset(3, 3);
  FloodFill flood;
  flood.compute(maze, Cell{1, 1});
  maze.reset(4, 4);
  Direction dir = Direction::kUp;
  TEST_ASSERT_TRUE(flood.bestDirection(maze, Cell{0, 0}, Direction::kUp,
                                       dir) == Status::kNotInitialized);
}

void test_full_capacity_maze() {
  Maze maze;
  TEST_ASSERT_TRUE(maze.reset(Maze::kMaxRows, Maze::kMaxCols) == Status::kOk);
  FloodFill flood;
  TEST_ASSERT_TRUE(flood.compute(maze, Cell{0, 0}) == Status::kOk);
  TEST_ASSERT_EQUAL_UINT8(Maze::kMaxRows - 1 + Maze::kMaxCols - 1,
                          flood.distance(Cell{Maze::kMaxRows - 1,
                                              Maze::kMaxCols - 1}));
}

// --- Renderer -------------------------------------------------------------

struct Buffer {
  char text[256];
  size_t length;
};

void appendChar(const char c, void* context) {
  Buffer* buffer = static_cast<Buffer*>(context);
  if (buffer->length + 1 < sizeof(buffer->text)) {
    buffer->text[buffer->length++] = c;
    buffer->text[buffer->length] = '\0';
  }
}

void test_render_small_maze() {
  Maze maze;
  maze.reset(2, 2);
  maze.setWall(Cell{0, 0}, Direction::kRight, true);   // wall
  maze.setWall(Cell{0, 0}, Direction::kDown, false);   // known open
  FloodFill flood;
  flood.compute(maze, Cell{0, 1});
  Buffer buffer = {{0}, 0};
  renderMaze(maze, flood, Cell{1, 0}, appendChar, &buffer);
  const char* expected =
      "+---+---+\n"
      "|  3|  0|\n"
      "+   + . +\n"
      "| @ :  1|\n"
      "+---+---+\n";
  TEST_ASSERT_EQUAL_STRING(expected, buffer.text);
}

void test_render_unreached_cell() {
  Maze maze;
  maze.reset(1, 2);
  maze.setWall(Cell{0, 0}, Direction::kRight, true);
  FloodFill flood;
  flood.compute(maze, Cell{0, 1});
  Buffer buffer = {{0}, 0};
  renderMaze(maze, flood, Cell{0, 1}, appendChar, &buffer);
  TEST_ASSERT_EQUAL_STRING("+---+---+\n| - | @ |\n+---+---+\n", buffer.text);
}

void test_render_draws_virtual_boundary_as_wall() {
  // The maze dump shows the PLANNING map: the virtual edge blocks travel,
  // so it is drawn like any wall.
  Maze maze;
  maze.reset(1, 2, 1, false);
  maze.setWall(Cell{0, 0}, Direction::kRight, true);
  FloodFill flood;
  flood.compute(maze, Cell{0, 1});
  Buffer buffer = {{0}, 0};
  renderMaze(maze, flood, Cell{0, 1}, appendChar, &buffer);
  TEST_ASSERT_EQUAL_STRING("+---+---+\n| - | @ |\n+---+---+\n", buffer.text);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_rotate_and_relative_are_inverse);
  RUN_TEST(test_rotate_examples);
  RUN_TEST(test_relative_turn_and_quarter_turns);
  RUN_TEST(test_steps_follow_row_col_convention);
  RUN_TEST(test_reset_rejects_bad_sizes);
  RUN_TEST(test_boundary_is_known_wall_interior_unknown);
  RUN_TEST(test_virtual_boundary_blocks_travel_but_is_not_physical);
  RUN_TEST(test_set_wall_is_mirrored_to_neighbour);
  RUN_TEST(test_boundary_walls_cannot_be_opened_and_bad_cells_rejected);
  RUN_TEST(test_sticky_wall_needs_consecutive_contradictions);
  RUN_TEST(test_flip_threshold_one_is_latest_wins_and_range_checked);
  RUN_TEST(test_index_round_trip);
  RUN_TEST(test_open_maze_gives_manhattan_distance);
  RUN_TEST(test_walls_lengthen_paths);
  RUN_TEST(test_unreachable_goal_reports_no_path);
  RUN_TEST(test_goal_out_of_bounds_and_uncomputed);
  RUN_TEST(test_best_direction_prefers_current_heading);
  RUN_TEST(test_best_direction_respects_walls);
  RUN_TEST(test_at_goal_there_is_no_closer_cell);
  RUN_TEST(test_resize_invalidates_flood);
  RUN_TEST(test_full_capacity_maze);
  RUN_TEST(test_render_small_maze);
  RUN_TEST(test_render_unreached_cell);
  RUN_TEST(test_render_draws_virtual_boundary_as_wall);
  return UNITY_END();
}
