// Host tests: the Serial rendering of the stored run log (app/
// run_log_printer) - the record numbering (the final "no move" record gets
// the next number) and the wall legend ('#' wall, '.' open, '+' virtual
// open-floor boundary). The printer and its status-name table are compiled
// into this suite only, against a fake hal::log that captures the text.
#include <string.h>
#include <unity.h>

#include <string>

#include "app/run_log_printer.cc"  // NOLINT: compiled into this suite only
#include "app/status_name.cc"      // NOLINT

using namespace micromouse;

namespace {

std::string gOut;
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

std::string printed() {
  gOut.clear();
  printStoredRunLog(fakeRead, kRunLogAddress, kRunLogCapacity);
  return gOut;
}

}  // namespace

// Fake Serial: everything the printer uses, appended to gOut ("\n" lines).
namespace micromouse::hal::log {
void begin(uint32_t) {}
void print(const __FlashStringHelper* text) {
  gOut += reinterpret_cast<const char*>(text);
}
void print(const char c) { gOut += c; }
void print(const int value) { gOut += std::to_string(value); }
void print(const unsigned int value) { gOut += std::to_string(value); }
void print(const long value) { gOut += std::to_string(value); }
void print(const unsigned long value) { gOut += std::to_string(value); }
void print(const float value, uint8_t) { gOut += std::to_string(value); }
void println() { gOut += '\n'; }
}  // namespace micromouse::hal::log

void setUp() { memset(gEeprom, 0xFF, sizeof(gEeprom)); }
void tearDown() {}

void test_final_no_move_record_gets_the_next_number() {
  // The real-robot report (physical-boundary mode): 2 moves up column 0,
  // then SENSOR_FAILURE before the 3rd - its "no move" record is number 3,
  // not a repeated 2, while END RUN still counts the 2 moves driven.
  RunLog log(fakeRead, fakeWrite, kRunLogAddress, kRunLogCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                          Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                          Direction::kUp));
  log.appendStep(makeStep(2, 0, Direction::kUp, true, false, false,
                          Direction::kUp, /*noMove=*/true));
  log.finishFault(Status::kSensorFailure, {2, 0}, Direction::kUp);
  TEST_ASSERT_EQUAL_STRING(
      "=== LAST RUN LOG ===\n"
      "START RUN 5x3 4,0@U g=0,2\n"
      " 1 4,0@U #.. ..## U\n"
      " 2 3,0@U #.. ...# U\n"
      " 3 2,0@U #.. ...# -\n"
      "END RUN FAULT SENSOR_FAILURE at 2,0@U n=2\n"
      "=== END LOG ===\n",
      printed().c_str());
}

void test_goal_run_numbers_every_move() {
  // README "Run log" worked example (physical boundary).
  RunLog log(fakeRead, fakeWrite, kRunLogAddress, kRunLogCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2});
  log.appendStep(makeStep(4, 0, Direction::kUp, true, false, false,
                          Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, true, false, false,
                          Direction::kUp));
  log.appendStep(makeStep(2, 0, Direction::kUp, true, true, false,
                          Direction::kRight));
  log.appendStep(makeStep(2, 1, Direction::kRight, false, false, false,
                          Direction::kUp));
  log.appendStep(makeStep(1, 1, Direction::kUp, false, false, false,
                          Direction::kRight));
  log.appendStep(makeStep(1, 2, Direction::kRight, false, true, false,
                          Direction::kUp));
  log.finishGoal({0, 2}, Direction::kUp);
  TEST_ASSERT_EQUAL_STRING(
      "=== LAST RUN LOG ===\n"
      "START RUN 5x3 4,0@U g=0,2\n"
      " 1 4,0@U #.. ..## U\n"
      " 2 3,0@U #.. ...# U\n"
      " 3 2,0@U ##. #..# R\n"
      " 4 2,1@R ... .... U\n"
      " 5 1,1@U ... .... R\n"
      " 6 1,2@R .#. .#.. U\n"
      "END RUN GOAL OK at 0,2@U n=6\n"
      "=== END LOG ===\n",
      printed().c_str());
}

void test_open_floor_log_marks_virtual_boundary() {
  // Open floor: the stored bits are the PHYSICAL walls (a virtual edge is
  // stored as open), and every edge side prints '+', sensed or not.
  RunLog log(fakeRead, fakeWrite, kRunLogAddress, kRunLogCapacity);
  log.startRun(5, 3, Pose{{4, 0}, Direction::kUp}, Cell{0, 2},
               /*openFloor=*/true);
  log.appendStep(makeStep(4, 0, Direction::kUp, false, false, false,
                          Direction::kUp));
  log.appendStep(makeStep(3, 0, Direction::kUp, false, false, false,
                          Direction::kRight));
  // An interior wall that WAS sensed ahead: '#'.
  log.appendStep(makeStep(3, 1, Direction::kRight, false, true, false,
                          Direction::kUp));
  log.appendStep(makeStep(0, 1, Direction::kRight, false, false, false,
                          Direction::kRight));
  log.finishGoal({0, 2}, Direction::kUp);
  TEST_ASSERT_EQUAL_STRING(
      "=== LAST RUN LOG ===\n"
      "START RUN 5x3 4,0@U g=0,2\n"
      " 1 4,0@U +.. ..++ U\n"
      " 2 3,0@U +.. ...+ R\n"
      " 3 3,1@R .#. .#.. U\n"
      " 4 0,1@R +.. +... R\n"
      "END RUN GOAL OK at 0,2@U n=4\n"
      "=== END LOG ===\n",
      printed().c_str());
}

void test_blank_eeprom_prints_no_log() {
  TEST_ASSERT_EQUAL_STRING(
      "=== LAST RUN LOG ===\nno log stored\n=== END LOG ===\n",
      printed().c_str());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_final_no_move_record_gets_the_next_number);
  RUN_TEST(test_goal_run_numbers_every_move);
  RUN_TEST(test_open_floor_log_marks_virtual_boundary);
  RUN_TEST(test_blank_eeprom_prints_no_log);
  return UNITY_END();
}
