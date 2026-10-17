// Host tests: navigation/run_log - step encode/decode, absolute-wall
// derivation, header CRC, interrupted runs, truncation, invalid/blank
// EEPROM and the EEPROM region not overlapping the calibration records.
#include <stddef.h>
#include <string.h>
#include <unity.h>

#include "config/config.h"
#include "control/calibration.h"
#include "navigation/run_log.h"

using namespace micromouse;

namespace {

// A fake EEPROM: a RAM buffer the size of the real one, read/write
// functions matching RunLogReadFn/RunLogWriteFn exactly (same signatures
// hal::eepromRead/eepromWrite use on the device).
uint8_t gEeprom[hal::kEepromSize];

void fakeRead(const uint16_t address, void* const out, const uint16_t size) {
  uint8_t* bytes = static_cast<uint8_t*>(out);
  for (uint16_t i = 0; i < size; ++i) {
    const uint16_t at = static_cast<uint16_t>(address + i);
    bytes[i] = at < sizeof(gEeprom) ? gEeprom[at] : 0xFF;
  }
}

void fakeWrite(const uint16_t address, const void* const data,
              const uint16_t size) {
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  for (uint16_t i = 0; i < size; ++i) {
    const uint16_t at = static_cast<uint16_t>(address + i);
    if (at < sizeof(gEeprom)) gEeprom[at] = bytes[i];
  }
}

RunStep makeStep(const uint8_t row, const uint8_t col, const Direction heading,
                 const bool l, const bool f, const bool r, const Direction move,
                 const bool noMove = false) {
  RunStep step{};
  step.cell = {row, col};
  step.heading = heading;
  step.wallLeft = l;
  step.wallFront = f;
  step.wallRight = r;
  step.move = move;
  step.noMove = noMove;
  return step;
}

}  // namespace

void setUp() { memset(gEeprom, 0xFF, sizeof(gEeprom)); }
void tearDown() {}

// --- Step encode / decode -----------------------------------------------

void test_step_encode_decode_round_trip() {
  for (uint8_t row = 0; row < 16; row += 3) {
    for (uint8_t col = 0; col < 16; col += 5) {
      for (const Direction heading : kAllDirections) {
        for (const Direction move : kAllDirections) {
          for (uint8_t mask = 0; mask < 8; ++mask) {
            for (uint8_t noMove = 0; noMove < 2; ++noMove) {
              const RunStep step =
                  makeStep(row, col, heading, (mask & 1) != 0,
                          (mask & 2) != 0, (mask & 4) != 0, move,
                          noMove != 0);
              const RunStep decoded = decodeStep(encodeStep(step));
              TEST_ASSERT_EQUAL_UINT8(step.cell.row, decoded.cell.row);
              TEST_ASSERT_EQUAL_UINT8(step.cell.col, decoded.cell.col);
              TEST_ASSERT_TRUE(step.heading == decoded.heading);
              TEST_ASSERT_EQUAL(step.wallLeft, decoded.wallLeft);
              TEST_ASSERT_EQUAL(step.wallFront, decoded.wallFront);
              TEST_ASSERT_EQUAL(step.wallRight, decoded.wallRight);
              TEST_ASSERT_TRUE(step.move == decoded.move);
              TEST_ASSERT_EQUAL(step.noMove, decoded.noMove);
            }
          }
        }
      }
    }
  }
}

void test_absolute_walls_use_boundary_for_the_unsensed_back_side() {
  // (4,0) in a 5x3 maze, facing Up: sensed L=wall F=open R=open; the back
  // side (South) is the maze's bottom edge - a wall even though not sensed
  // from here (matches the README "Run log" worked example).
  bool out[kDirectionCount];
  absoluteWalls({4, 0}, Direction::kUp, true, false, false, 5, 3, out);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kUp)]);     // N open
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kRight)]);  // E open
  TEST_ASSERT_TRUE(out[static_cast<uint8_t>(Direction::kDown)]);    // S wall
  TEST_ASSERT_TRUE(out[static_cast<uint8_t>(Direction::kLeft)]);    // W wall

  // An interior cell's unsensed back side is reported open (optimistic,
  // like an unobserved Maze side).
  absoluteWalls({2, 1}, Direction::kRight, false, false, false, 5, 3, out);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kLeft)]);  // back

  // The sensed sides always win, even where they coincide with a boundary.
  absoluteWalls({0, 1}, Direction::kUp, false, false, false, 5, 3, out);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kUp)]);  // front = N
}

/// Matches the worked example of the README "Run log" section exactly (a
/// 5x3 maze, start (4,0) facing Up, goal (0,2), 6 moves U,U,R,U,R,U): if
/// this ever stops matching, fix the README, not this test.
void test_readme_run_log_example_abs_walls() {
  bool out[kDirectionCount];
  // 1: pos 4,0@U, sensed L=wall F=open R=open -> abs N=open E=open S=wall
  // W=wall ("..##").
  absoluteWalls({4, 0}, Direction::kUp, true, false, false, 5, 3, out);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kUp)]);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kRight)]);
  TEST_ASSERT_TRUE(out[static_cast<uint8_t>(Direction::kDown)]);
  TEST_ASSERT_TRUE(out[static_cast<uint8_t>(Direction::kLeft)]);
  // 2: pos 3,0@U, L=wall F=open R=open -> abs "...#" (only W is a wall).
  absoluteWalls({3, 0}, Direction::kUp, true, false, false, 5, 3, out);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kUp)]);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kRight)]);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kDown)]);
  TEST_ASSERT_TRUE(out[static_cast<uint8_t>(Direction::kLeft)]);
  // 3: pos 2,0@U, L=wall F=wall R=open, move R -> abs "#..#" (N and W).
  absoluteWalls({2, 0}, Direction::kUp, true, true, false, 5, 3, out);
  TEST_ASSERT_TRUE(out[static_cast<uint8_t>(Direction::kUp)]);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kRight)]);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kDown)]);
  TEST_ASSERT_TRUE(out[static_cast<uint8_t>(Direction::kLeft)]);
  // 4: pos 2,1@R, L=open F=open R=open, move U -> abs "...." (interior).
  absoluteWalls({2, 1}, Direction::kRight, false, false, false, 5, 3, out);
  for (const bool wall : out) TEST_ASSERT_FALSE(wall);
  // 5: pos 1,1@U, L=open F=open R=open, move R -> abs "...." (interior).
  absoluteWalls({1, 1}, Direction::kUp, false, false, false, 5, 3, out);
  for (const bool wall : out) TEST_ASSERT_FALSE(wall);
  // 6: pos 1,2@R, L=open F=wall R=open, move U -> abs ".#.." (only E, the
  // last column's boundary, which the front sensor facing East confirms).
  absoluteWalls({1, 2}, Direction::kRight, false, true, false, 5, 3, out);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kUp)]);
  TEST_ASSERT_TRUE(out[static_cast<uint8_t>(Direction::kRight)]);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kDown)]);
  TEST_ASSERT_FALSE(out[static_cast<uint8_t>(Direction::kLeft)]);
}

void test_boundary_sides_and_wall_symbols() {
  const auto bit = [](const Direction d) {
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(d));
  };
  // 5x3: (4,0) bottom-left corner -> S and W; (0,2) top-right -> N and E;
  // (2,1) interior -> none; a 1x1 maze -> all four.
  TEST_ASSERT_EQUAL_HEX8(bit(Direction::kDown) | bit(Direction::kLeft),
                         boundarySides({4, 0}, 5, 3));
  TEST_ASSERT_EQUAL_HEX8(bit(Direction::kUp) | bit(Direction::kRight),
                         boundarySides({0, 2}, 5, 3));
  TEST_ASSERT_EQUAL_HEX8(0, boundarySides({2, 1}, 5, 3));
  TEST_ASSERT_EQUAL_HEX8(0x0F, boundarySides({0, 0}, 1, 1));

  // '+' only where the virtual mask says so, regardless of the stored bit;
  // otherwise '#' / '.'.
  const uint8_t virtualSides = boundarySides({4, 0}, 5, 3);
  TEST_ASSERT_EQUAL_CHAR('+', runLogWallSymbol(virtualSides, 3, false));
  TEST_ASSERT_EQUAL_CHAR('+', runLogWallSymbol(virtualSides, 2, true));
  TEST_ASSERT_EQUAL_CHAR('.', runLogWallSymbol(virtualSides, 0, false));
  TEST_ASSERT_EQUAL_CHAR('#', runLogWallSymbol(virtualSides, 1, true));
  // Physical-boundary log: no virtual sides, the stored bit decides.
  TEST_ASSERT_EQUAL_CHAR('#', runLogWallSymbol(0, 3, true));
  TEST_ASSERT_EQUAL_CHAR('.', runLogWallSymbol(0, 3, false));
}

void test_open_floor_flag_is_stored_and_crc_valid() {
  RunLog log(fakeRead, fakeWrite, kRunLogAddress, kRunLogCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2},
               /*openFloor=*/true);
  log.appendStep(makeStep(4, 0, Direction::kUp, false, false, false,
                          Direction::kUp));
  log.finishGoal({0, 2}, Direction::kUp);
  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, kRunLogAddress, kRunLogCapacity,
                               header));
  TEST_ASSERT_EQUAL_HEX8(kRunLogOpenFloorFlag, header.flags);

  // The default (and every log written before the flag existed): clear.
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, kRunLogAddress, kRunLogCapacity,
                               header));
  TEST_ASSERT_EQUAL_HEX8(0, header.flags);
}

// --- Header CRC / validity -----------------------------------------------

void test_header_crc_detects_corruption() {
  RunLog log(fakeRead, fakeWrite, 0, 16);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.finishGoal({0, 2}, Direction::kUp);

  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, 0, 16, header));

  // Flip a byte inside the step region: the header CRC covers it too.
  gEeprom[sizeof(RunLogHeader)] ^= 0x01;
  TEST_ASSERT_EQUAL(RunLogValidity::kBadCrc,
                    readRunLog(fakeRead, 0, 16, header));

  setUp();
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.finishGoal({0, 2}, Direction::kUp);
  gEeprom[9] ^= 0x01;  // goalRow, inside the CRC-covered header prefix
  TEST_ASSERT_EQUAL(RunLogValidity::kBadCrc,
                    readRunLog(fakeRead, 0, 16, header));
}

// --- Torn-write recovery (power loss mid-write) -----------------------------
//
// hal::eepromWrite (and the fake below) commits bytes in ascending address
// order, so within one RunLog::appendStep()/finishGoal()/finishFault() call
// the crc (the header's last field) is always the LAST byte pair written.
// These tests forge exactly that: the fields before the crc already
// reflect the new write, but the crc field still holds the PREVIOUS,
// fully-written value - and check that readRunLog() recovers the previous,
// consistent state instead of reporting kBadCrc (see the comment in
// navigation/run_log.cc).

void test_torn_append_step_recovers_previous_step_count() {
  RunLog log(fakeRead, fakeWrite, 0, 16);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  RunLogHeader before{};  // stepCount = 2, crc valid for 2 steps
  fakeRead(0, &before, sizeof(before));

  // The 3rd step's bytes and the new stepCount (3) commit; the crc write
  // never arrives.
  log.appendStep(makeStep(2, 0, Direction::kUp, true, true, false,
                         Direction::kRight));
  fakeWrite(offsetof(RunLogHeader, crc), &before.crc, sizeof(before.crc));

  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, 0, 16, header));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RunLogState::kInProgress),
                          header.state);
  TEST_ASSERT_EQUAL_UINT16(2, header.stepCount);  // the 3rd step is dropped
  TEST_ASSERT_EQUAL_UINT8(4, readRunLogStep(fakeRead, 0, 0).cell.row);
  TEST_ASSERT_EQUAL_UINT8(3, readRunLogStep(fakeRead, 0, 1).cell.row);
}

void test_torn_finish_goal_recovers_interrupted_run() {
  RunLog log(fakeRead, fakeWrite, 0, 16);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  RunLogHeader before{};  // in progress, crc valid for 2 steps
  fakeRead(0, &before, sizeof(before));

  // state/outcome commit; the crc write never arrives.
  log.finishGoal({0, 2}, Direction::kUp);
  fakeWrite(offsetof(RunLogHeader, crc), &before.crc, sizeof(before.crc));

  RunLogHeader header{};
  // readRunLog() recovers the pre-finish header: state is back to
  // "in progress", so printStoredRunLog() prints "(interrupted)" instead
  // of an END RUN line (see test_interrupted_run_has_no_end_marker above
  // for the same state -> marker link; printing itself needs hal::log,
  // which is not available on the host).
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, 0, 16, header));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RunLogState::kInProgress),
                          header.state);
  TEST_ASSERT_EQUAL_UINT16(2, header.stepCount);
}

void test_torn_finish_fault_after_truncation_recovers_interrupted_run() {
  // Combines a torn finish with a truncated log: the recovered header must
  // still show truncated (flags is untouched by finish*()) alongside the
  // reverted state/outcome/fault fields.
  constexpr uint16_t kCapacity = 2;
  RunLog log(fakeRead, fakeWrite, 0, kCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.appendStep(makeStep(2, 0, Direction::kUp, true, true, false,
                         Direction::kRight));  // beyond capacity: truncated
  TEST_ASSERT_TRUE(log.truncated());
  RunLogHeader before{};  // in progress, truncated, crc valid for 2 steps
  fakeRead(0, &before, sizeof(before));

  // state/outcome/fault fields commit; the crc write never arrives.
  log.finishFault(Status::kExplorationLimit, Cell{2, 0}, Direction::kUp);
  fakeWrite(offsetof(RunLogHeader, crc), &before.crc, sizeof(before.crc));

  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, 0, kCapacity, header));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RunLogState::kInProgress),
                          header.state);
  TEST_ASSERT_TRUE((header.flags & kRunLogTruncatedFlag) != 0);
  TEST_ASSERT_EQUAL_UINT16(kCapacity, header.stepCount);
}

// --- Interrupted run -------------------------------------------------------

void test_interrupted_run_has_no_end_marker() {
  RunLog log(fakeRead, fakeWrite, 0, 16);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  // No finishGoal()/finishFault(): power was "lost" here.

  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, 0, 16, header));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RunLogState::kInProgress),
                          header.state);
  TEST_ASSERT_EQUAL_UINT16(2, header.stepCount);
}

// --- Truncation -------------------------------------------------------------

void test_truncation_at_capacity_keeps_the_run_going() {
  constexpr uint16_t kCapacity = 3;
  RunLog log(fakeRead, fakeWrite, 0, kCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  for (uint8_t i = 0; i < 5; ++i) {
    log.appendStep(makeStep(static_cast<uint8_t>(4 - i), 0, Direction::kUp,
                           false, false, false, Direction::kUp));
  }
  TEST_ASSERT_EQUAL_UINT16(kCapacity, log.stepCount());
  TEST_ASSERT_TRUE(log.truncated());
  log.finishGoal({0, 2}, Direction::kUp);

  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, 0, kCapacity, header));
  TEST_ASSERT_EQUAL_UINT16(kCapacity, header.stepCount);
  TEST_ASSERT_TRUE((header.flags & kRunLogTruncatedFlag) != 0);
  // The two steps beyond capacity were simply not written.
  TEST_ASSERT_EQUAL_UINT8(2, readRunLogStep(fakeRead, 0, 2).cell.row);
}

void test_truncation_crc_is_valid_before_any_finish() {
  // Covers the fix in RunLog::appendStep: the truncation branch must
  // recompute the crc (flags is CRC-covered), or a read right after hitting
  // capacity - before finishGoal()/finishFault() ever runs - would wrongly
  // report kBadCrc even though nothing is actually corrupt.
  constexpr uint16_t kCapacity = 2;
  RunLog log(fakeRead, fakeWrite, 0, kCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  log.appendStep(makeStep(2, 0, Direction::kUp, true, true, false,
                         Direction::kRight));  // beyond capacity: truncated
  TEST_ASSERT_TRUE(log.truncated());
  // No finishGoal()/finishFault(): read the log exactly as it stands right
  // after truncation.

  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, 0, kCapacity, header));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RunLogState::kInProgress),
                          header.state);
  TEST_ASSERT_TRUE((header.flags & kRunLogTruncatedFlag) != 0);
  TEST_ASSERT_EQUAL_UINT16(kCapacity, header.stepCount);
}

// --- Invalid / blank EEPROM -------------------------------------------------

void test_blank_and_foreign_eeprom_are_rejected() {
  RunLogHeader header{};
  // Freshly erased (setUp() already filled 0xFF).
  TEST_ASSERT_EQUAL(RunLogValidity::kEmpty,
                    readRunLog(fakeRead, 0, 16, header));

  RunLogHeader bad{};
  bad.magic = 0x1234;
  fakeWrite(0, &bad, sizeof(bad));
  TEST_ASSERT_EQUAL(RunLogValidity::kBadMagic,
                    readRunLog(fakeRead, 0, 16, header));

  RunLog log(fakeRead, fakeWrite, 0, 16);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.finishGoal({0, 2}, Direction::kUp);
  RunLogHeader good{};
  fakeRead(0, &good, sizeof(good));
  good.version = static_cast<uint8_t>(kRunLogVersion + 1);
  good.crc = computeRunLogCrc(fakeRead, 0, good);  // re-seal, still rejected
  fakeWrite(0, &good, sizeof(good));
  TEST_ASSERT_EQUAL(RunLogValidity::kBadVersion,
                    readRunLog(fakeRead, 0, 16, header));
}

// --- EEPROM layout ----------------------------------------------------------

void test_run_log_region_does_not_overlap_calibration_and_fits() {
  TEST_ASSERT_GREATER_OR_EQUAL_UINT16(
      config::kLearnedRecordAddress + sizeof(LearnedRecord), kRunLogAddress);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(
      hal::kEepromSize,
      static_cast<uint32_t>(kRunLogAddress) + kRunLogRegionSize);
  // The worst-case exploration budget at the maximum static maze size.
  TEST_ASSERT_EQUAL_UINT16(
      static_cast<uint16_t>(config::kMazeMaxRows) * config::kMazeMaxCols *
              config::kExplorationBudgetFactor +
          1u,
      kRunLogCapacity);
}

// --- Full run round trip ----------------------------------------------------

void test_full_run_round_trip_goal_reached() {
  RunLog log(fakeRead, fakeWrite, kRunLogAddress, kRunLogCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  const RunStep steps[] = {
      makeStep(4, 0, Direction::kUp, true, false, false, Direction::kUp),
      makeStep(3, 0, Direction::kUp, true, false, false, Direction::kUp),
      makeStep(2, 0, Direction::kUp, true, true, false, Direction::kRight),
      makeStep(2, 1, Direction::kRight, false, false, false, Direction::kUp),
      makeStep(1, 1, Direction::kUp, false, false, false, Direction::kRight),
      makeStep(1, 2, Direction::kRight, true, false, false, Direction::kUp),
  };
  for (const RunStep& s : steps) log.appendStep(s);
  log.finishGoal({0, 2}, Direction::kUp);

  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, kRunLogAddress, kRunLogCapacity,
                              header));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RunLogState::kComplete),
                          header.state);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RunOutcome::kGoalReached),
                          header.outcome);
  TEST_ASSERT_EQUAL_UINT16(6, header.stepCount);
  TEST_ASSERT_FALSE(log.truncated());
  for (uint16_t i = 0; i < 6; ++i) {
    const RunStep decoded = readRunLogStep(fakeRead, kRunLogAddress, i);
    TEST_ASSERT_EQUAL_UINT8(steps[i].cell.row, decoded.cell.row);
    TEST_ASSERT_EQUAL_UINT8(steps[i].cell.col, decoded.cell.col);
    TEST_ASSERT_TRUE(steps[i].move == decoded.move);
  }
}

void test_full_run_round_trip_fault_with_no_move_record() {
  RunLog log(fakeRead, fakeWrite, kRunLogAddress, kRunLogCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                         Direction::kUp));
  // Sensing/planning halted at (3,0) before a move could be attempted.
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                         Direction::kUp, /*noMove=*/true));
  log.finishFault(Status::kNoPath, {3, 0}, Direction::kUp);

  RunLogHeader header{};
  TEST_ASSERT_EQUAL(RunLogValidity::kValid,
                    readRunLog(fakeRead, kRunLogAddress, kRunLogCapacity,
                              header));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RunOutcome::kFault),
                          header.outcome);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::kNoPath),
                          header.faultStatus);
  TEST_ASSERT_EQUAL_UINT8(3, header.faultRow);
  TEST_ASSERT_EQUAL_UINT8(0, header.faultCol);
  TEST_ASSERT_EQUAL_UINT16(2, header.stepCount);
  TEST_ASSERT_TRUE(readRunLogStep(fakeRead, kRunLogAddress, 1).noMove);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_step_encode_decode_round_trip);
  RUN_TEST(test_absolute_walls_use_boundary_for_the_unsensed_back_side);
  RUN_TEST(test_readme_run_log_example_abs_walls);
  RUN_TEST(test_boundary_sides_and_wall_symbols);
  RUN_TEST(test_open_floor_flag_is_stored_and_crc_valid);
  RUN_TEST(test_header_crc_detects_corruption);
  RUN_TEST(test_torn_append_step_recovers_previous_step_count);
  RUN_TEST(test_torn_finish_goal_recovers_interrupted_run);
  RUN_TEST(test_torn_finish_fault_after_truncation_recovers_interrupted_run);
  RUN_TEST(test_interrupted_run_has_no_end_marker);
  RUN_TEST(test_truncation_at_capacity_keeps_the_run_going);
  RUN_TEST(test_truncation_crc_is_valid_before_any_finish);
  RUN_TEST(test_blank_and_foreign_eeprom_are_rejected);
  RUN_TEST(test_run_log_region_does_not_overlap_calibration_and_fits);
  RUN_TEST(test_full_run_round_trip_goal_reached);
  RUN_TEST(test_full_run_round_trip_fault_with_no_move_record);
  return UNITY_END();
}
