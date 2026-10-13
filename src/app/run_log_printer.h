// Human-readable Serial rendering of navigation/run_log (the words live
// here, in flash F() strings, not in the pure navigation module: see
// app/maze_printer.h for the same split with navigation/maze_render.h).
#pragma once

#include <stdint.h>

#include "navigation/run_log.h"

namespace micromouse {

/// Prints "START RUN RxC r,c@H g=r,c" (info level): the live marker when a
/// run begins, and the first line of the stored log.
void printRunLogStart(uint8_t rows, uint8_t cols, Pose start, Cell goal);

/// Prints "END RUN result OUTCOME STATUS at r,c@H n=N" (info level): the
/// live marker when a run ends, and the last line of the stored log.
/// `fault`/`faultCell`/`faultHeading` are only used if outcome == kFault
/// (otherwise "OK" and the goal cell/heading are printed instead).
void printRunLogEnd(RunOutcome outcome, Status fault, Cell faultCell,
                    Direction faultHeading, uint16_t moves);

/// Prints the whole stored log, bracketed by "=== LAST RUN LOG ===" /
/// "=== END LOG ===": "no log stored" for a blank or invalid EEPROM, the
/// START RUN / numbered step / END RUN lines for a complete run, or
/// "(interrupted)" after the last step of an in-progress one (a power loss
/// mid-run), or "(truncated)" if the capacity was reached. Reads through
/// `read` at `address` (see kRunLogAddress / kRunLogCapacity).
void printStoredRunLog(RunLogReadFn read, uint16_t address,
                       uint16_t capacity);

}  // namespace micromouse
