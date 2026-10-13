#include "app/run_log_printer.h"

#include "app/status_name.h"
#include "config/config.h"
#include "hal/log.h"

namespace micromouse {

namespace {

namespace log = hal::log;
namespace cfg = config;
using config::LogLevel;

// Flash budget (README "Flash budget"): the debug-log builds (uno_debug,
// uno_setup) have little room left for the per-step line on top of their
// own debug printing, so they print the relative L/F/R walls only; the
// plain info build (uno) also gets the absolute N/E/S/W sides.
constexpr bool kPrintAbsoluteWalls = !cfg::logEnabled(LogLevel::kDebug);

// Flash is tight (see README "Flash budget"): one char per wall ('#'/'.')
// instead of a "WALL"/"open" word, so a step line costs plain %c
// conversions, not %s + a helper returning a flash string.
char wallChar(const bool wall) { return wall ? '#' : '.'; }

void printStep(const uint16_t moveNumber, const RunStep& step,
              const uint8_t rows, const uint8_t cols) {
  // Plain print() calls, not fmt(): a long %-format with many conversions
  // costs more code to marshal on the AVR than a sequence of simple,
  // non-variadic calls (measured; see README "Flash budget").
  log::print(' ');
  log::print(static_cast<unsigned int>(moveNumber));
  log::print(' ');
  log::print(static_cast<unsigned int>(step.cell.row));
  log::print(',');
  log::print(static_cast<unsigned int>(step.cell.col));
  log::print('@');
  log::print(directionLetter(step.heading));
  log::print(F("  "));
  log::print(wallChar(step.wallLeft));
  log::print(wallChar(step.wallFront));
  log::print(wallChar(step.wallRight));
  log::print(F("  "));
  if constexpr (kPrintAbsoluteWalls) {
    bool abs[kDirectionCount];
    absoluteWalls(step.cell, step.heading, step.wallLeft, step.wallFront,
                 step.wallRight, rows, cols, abs);
    log::print(wallChar(abs[static_cast<uint8_t>(Direction::kUp)]));
    log::print(wallChar(abs[static_cast<uint8_t>(Direction::kRight)]));
    log::print(wallChar(abs[static_cast<uint8_t>(Direction::kDown)]));
    log::print(wallChar(abs[static_cast<uint8_t>(Direction::kLeft)]));
    log::print(F("  "));
  } else {
    (void)rows;
    (void)cols;
  }
  log::print(step.noMove ? '-' : directionLetter(step.move));
  log::println();
}

}  // namespace

void printRunLogStart(const uint8_t rows, const uint8_t cols,
                      const Pose start, const Cell goal) {
  log::print(F("START RUN "));
  log::print(static_cast<unsigned int>(rows));
  log::print('x');
  log::print(static_cast<unsigned int>(cols));
  log::print(' ');
  log::print(static_cast<unsigned int>(start.cell.row));
  log::print(',');
  log::print(static_cast<unsigned int>(start.cell.col));
  log::print('@');
  log::print(directionLetter(start.heading));
  log::print(F(" g="));
  log::print(static_cast<unsigned int>(goal.row));
  log::print(',');
  log::print(static_cast<unsigned int>(goal.col));
  log::println();
}

void printRunLogEnd(const RunOutcome outcome, const Status fault,
                    const Cell faultCell, const Direction faultHeading,
                    const uint16_t moves) {
  // A non-fault end prints "OK" instead of a status name, and the
  // cell/heading it is given (the goal).
  const bool isFault = outcome == RunOutcome::kFault;
  log::print(F("END RUN result "));
  log::print(isFault ? F("FAULT") : F("GOAL_REACHED"));
  log::print(' ');
  log::print(isFault ? statusName(fault) : F("OK"));
  log::print(F(" at "));
  log::print(static_cast<unsigned int>(faultCell.row));
  log::print(',');
  log::print(static_cast<unsigned int>(faultCell.col));
  log::print('@');
  log::print(directionLetter(faultHeading));
  log::print(F(" n="));
  log::print(static_cast<unsigned int>(moves));
  log::println();
}

void printStoredRunLog(const RunLogReadFn read, const uint16_t address,
                       const uint16_t capacity) {
  log::print(F("=== LAST RUN LOG ==="));
  log::println();
  RunLogHeader header{};
  const RunLogValidity validity = readRunLog(read, address, capacity, header);
  if (validity != RunLogValidity::kValid) {
    log::print(F("no log stored"));
    log::println();
    log::print(F("=== END LOG ==="));
    log::println();
    return;
  }
  const Pose start{{header.startRow, header.startCol},
                   static_cast<Direction>(header.startHeading)};
  const Cell goal{header.goalRow, header.goalCol};
  printRunLogStart(header.rows, header.cols, start, goal);
  // Step line format (README "Run log"): n, pos row,col@heading, the
  // relative walls L F R (+ the absolute walls N E S W where flash allows,
  // see kPrintAbsoluteWalls), the move direction ('-' for the one possible
  // final "no move" record). # = wall, . = open.
  uint16_t moveNumber = 0;
  for (uint16_t i = 0; i < header.stepCount; ++i) {
    const RunStep step = readRunLogStep(read, address, i);
    if (!step.noMove) ++moveNumber;
    printStep(moveNumber, step, header.rows, header.cols);
  }
  if ((header.flags & kRunLogTruncatedFlag) != 0) {
    log::print(F("(truncated)"));
    log::println();
  }
  if (header.state == static_cast<uint8_t>(RunLogState::kInProgress)) {
    log::print(F("(interrupted)"));
    log::println();
  } else {
    const Cell faultCell{header.faultRow, header.faultCol};
    printRunLogEnd(static_cast<RunOutcome>(header.outcome),
                  static_cast<Status>(header.faultStatus), faultCell,
                  static_cast<Direction>(header.faultHeading), moveNumber);
  }
  log::print(F("=== END LOG ==="));
  log::println();
}

}  // namespace micromouse
