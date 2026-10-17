// Compact, host-testable record of the LAST maze run (see README "Run
// log"): what the robot sensed and did in every cell, framed by a header
// with the outcome. Stored in EEPROM right after the learned-calibration
// record (control/calibration.h) and read back at boot to print the
// previous run before a new one starts and overwrites it.
//
// Pure (no Arduino.h): the app wires this to hal/eeprom through the
// RunLogReadFn/RunLogWriteFn callbacks (hal::eepromRead / hal::eepromWrite
// match their signatures exactly), and tests fake them with a RAM buffer.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config/config.h"
#include "control/calibration.h"  // sizeof(LearnedRecord) only: a header-only use
#include "hal/eeprom.h"           // kEepromSize only: header-only use
#include "navigation/pose.h"
#include "util/status.h"

namespace micromouse {

/// One step: the pose BEFORE the move, the relative PHYSICAL walls known
/// there after the SENSE (Explorer::physicalWall(): a virtual open-floor
/// boundary is stored as 0, a physical boundary as 1), and the direction
/// driven. `noMove` marks
/// the (at most one, final) record of a run that halts in FAULT before a
/// move could be attempted (a SENSE or PLAN failure): the robot stopped at
/// `cell`/`heading` with whatever walls are known there, and `move` is
/// meaningless.
struct RunStep {
  Cell cell;
  Direction heading;
  bool wallLeft;
  bool wallFront;
  bool wallRight;
  Direction move;
  bool noMove;
};

/// Packs a RunStep into 16 bits: row/col (4 bits each), heading (2 bits),
/// wall L/F/R (3 bits), move direction (2 bits), no-move flag (1 bit) -
/// exactly 2 bytes per step, no spare bits.
uint16_t encodeStep(const RunStep& step);
RunStep decodeStep(uint16_t packed);

/// Absolute wall presence (indexed by Direction: kUp=N, kRight=E, kDown=S,
/// kLeft=W) of `cell`, for the "(abs N=.. E=.. S=.. W=..)" part of a
/// printed step. The 3 sensed sides (front/left/right of `heading`) come
/// from the reading; the 4th (back) side was not sensed from here, so it
/// falls back to whether it is an outer-boundary wall of a `rows` x `cols`
/// maze (always a wall, known even before it is ever sensed - see
/// navigation/maze.h) - interior back sides are reported open, same as an
/// unobserved side in Maze.
void absoluteWalls(Cell cell, Direction heading, bool wallLeft,
                   bool wallFront, bool wallRight, uint8_t rows,
                   uint8_t cols, bool out[kDirectionCount]);

/// Outer-boundary sides of `cell` in a `rows` x `cols` maze: bit d set
/// (d = static_cast<uint8_t>(Direction)) if that side is on the grid edge.
uint8_t boundarySides(Cell cell, uint8_t rows, uint8_t cols);

/// Printed symbol of side `dir` (README "Run log"): '+' if its bit is set
/// in `virtualSides` (boundarySides() of the cell in a run logged in
/// open-floor mode, else 0: the grid edge is impassable for planning, but
/// no physical wall was there to sense), otherwise '#' for `wall`, '.'
/// for open.
constexpr char runLogWallSymbol(const uint8_t virtualSides,
                                const uint8_t dir, const bool wall) {
  return ((virtualSides >> dir) & 1u) != 0 ? '+' : (wall ? '#' : '.');
}

enum class RunLogState : uint8_t { kInProgress = 1, kComplete = 2 };
enum class RunOutcome : uint8_t { kNone = 0, kGoalReached = 1, kFault = 2 };

/// Validity of a stored header and its steps; see readRunLog(). kValid
/// covers a header recovered from a torn write (see readRunLog()): the
/// prefix of the run up to the last write that fully completed.
enum class RunLogValidity : uint8_t {
  kValid = 0,
  kEmpty,       ///< erased EEPROM (magic 0xFFFF)
  kBadMagic,    ///< another / older layout
  kBadVersion,
  kBadCrc,      ///< not explained by a known torn-write pattern either
  kOutOfRange,  ///< state / stepCount / outcome outside what can be stored
};

/// On-disk header, immediately followed by `stepCount` packed step records
/// (<= kRunLogCapacity) at kRunLogAddress + sizeof(RunLogHeader). No
/// interior padding on the AVR or the host (every 16 bit field starts at an
/// even offset). The CRC covers every byte before it (magic..faultHeading,
/// reserved) AND all `stepCount` step records that follow in EEPROM (see
/// computeRunLogCrc()) - so it is NOT invalidated merely by `state` /
/// `outcome` / fault fields changing at the end of a run that already has
/// its steps written.
struct RunLogHeader {
  uint16_t magic;
  uint8_t version;
  uint8_t state;         ///< RunLogState
  uint8_t rows;
  uint8_t cols;
  uint8_t startRow;
  uint8_t startCol;
  uint8_t startHeading;  ///< Direction
  uint8_t goalRow;
  uint8_t goalCol;
  uint8_t flags;         ///< bit 0: truncated (capacity was reached),
                         ///< bit 1: open floor (virtual boundary)
  uint16_t stepCount;    ///< records stored, <= capacity (moves + at most
                         ///< one final "no move" record; the app counts
                         ///< actual moves itself, see Explorer::cellsMoved)
  uint8_t outcome;       ///< RunOutcome, meaningful once state == kComplete
  uint8_t faultStatus;   ///< Status, meaningful if outcome == kFault
  uint8_t faultRow;      ///< END pose of either outcome (the fault cell,
  uint8_t faultCol;      ///< or the goal cell), once state == kComplete
  uint8_t faultHeading;  ///< Direction
  uint8_t reserved;      ///< 0; keeps crc at an even offset
  uint16_t crc;
};

constexpr uint16_t kRunLogMagic = 0x524C;  // "RL"
constexpr uint8_t kRunLogVersion = 1;
constexpr uint8_t kRunLogTruncatedFlag = 0x01;
/// The run was in open-floor mode (ExplorerParams::physicalBoundary false):
/// boundary sides print as '+' (runLogWallSymbol). Logs written before this
/// flag existed have it clear, which matches what they stored (boundary
/// walls as 1), so the version is unchanged.
constexpr uint8_t kRunLogOpenFloorFlag = 0x02;
constexpr uint16_t kRunLogErasedMagic = 0xFFFF;  // erased EEPROM

constexpr size_t kRunLogCrcOffset = offsetof(RunLogHeader, crc);
static_assert(kRunLogCrcOffset == 20, "unexpected padding in RunLogHeader");
#if defined(__AVR__)
static_assert(sizeof(RunLogHeader) == 22, "RunLogHeader layout changed");
#endif

/// Place of the run log in EEPROM: right after the learned-calibration
/// record (control/calibration.h), so it can never overlap the two
/// calibration records.
constexpr uint16_t kRunLogAddress =
    config::kLearnedRecordAddress + sizeof(LearnedRecord);
/// Worst case step count: the exploration budget at the maximum STATIC
/// maze size (config::kMazeMaxRows/kMazeMaxCols, not just the configured
/// kMazeRows/kMazeCols), plus one spare record for a possible final
/// "no move" fault record. If a future kMazeRows/kMazeCols/
/// kExplorationBudgetFactor change makes this not fit in the EEPROM, the
/// static_assert below fails at compile time instead of silently
/// truncating every run.
constexpr uint16_t kRunLogCapacity =
    static_cast<uint16_t>(config::kMazeMaxRows) * config::kMazeMaxCols *
        config::kExplorationBudgetFactor +
    1u;
constexpr uint16_t kRunLogRegionSize =
    static_cast<uint16_t>(sizeof(RunLogHeader)) +
    kRunLogCapacity * static_cast<uint16_t>(sizeof(uint16_t));
static_assert(kRunLogAddress >=
                  config::kLearnedRecordAddress + sizeof(LearnedRecord),
              "run log must start after the learned-calibration record");
static_assert(static_cast<uint32_t>(kRunLogAddress) + kRunLogRegionSize <=
                  hal::kEepromSize,
              "run log overlaps the calibration records or exceeds the "
              "1 KB EEPROM");
/// Ties the capacity above to the exploration budget it is sized from (the
/// actually CONFIGURED maze, not just the maximum static one): truncation
/// (RunLog::appendStep, the `truncated` flag), and with it the two
/// unrecovered torn-write edge cases documented at readRunLog() in the
/// .cc (a torn write of the truncated flag itself, and a stepCount tear on
/// a byte-boundary increment such as 255 -> 256), can only happen once
/// `stepCount` approaches `kRunLogCapacity` - i.e. only for a maze near the
/// maximum static size (config::kMazeMaxRows/kMazeMaxCols). The configured
/// maze (config::kMazeRows/kMazeCols) is far smaller, so this assert always
/// holds with a wide margin in practice; it exists to catch the day it
/// stops holding.
static_assert(static_cast<uint32_t>(config::kMazeRows) * config::kMazeCols *
                      config::kExplorationBudgetFactor + 1u <=
                  kRunLogCapacity,
              "the configured maze's exploration budget must fit in the "
              "run log capacity");

/// Byte-level EEPROM access the run log writes/reads through;
/// hal::eepromRead / hal::eepromWrite match these signatures exactly.
/// Tests fake both with a RAM buffer.
using RunLogReadFn = void (*)(uint16_t address, void* out, uint16_t size);
using RunLogWriteFn = void (*)(uint16_t address, const void* data,
                               uint16_t size);

/// Recomputes the CRC of `header` (any state) against the `header.stepCount`
/// step records stored at `address`, read through `read`.
uint16_t computeRunLogCrc(RunLogReadFn read, uint16_t address,
                          const RunLogHeader& header);

/// Reads and validates the header at `address` (magic, version, state,
/// `stepCount` within `capacity`, outcome range, CRC over the header and its
/// steps). kValid covers both an in-progress (interrupted) and a complete
/// run; the caller tells them apart with `out.state`. If the stored CRC
/// does not match, this also tries the two torn-write states a power loss
/// can leave (see the comment in the .cc) before giving up with kBadCrc;
/// when one matches, `out` is the recovered (slightly older, but internally
/// consistent) header, not the raw bytes on disk.
RunLogValidity readRunLog(RunLogReadFn read, uint16_t address,
                          uint16_t capacity, RunLogHeader& out);

/// Reads and decodes step `index` (< header.stepCount).
RunStep readRunLogStep(RunLogReadFn read, uint16_t address, uint16_t index);

/// Writes one run's progress to EEPROM. Wear per run: one header write at
/// the start, one per step (but EEPROM.update only actually rewrites the
/// bytes that changed - stepCount and crc, 4 bytes - since the rest of the
/// header is unchanged after the first write), and one at the end; see
/// README "Run log" for the lifetime estimate. A power loss mid-write only
/// loses that one write (readRunLog() recovers the previous, complete
/// state) - see readRunLog() and the comment in the .cc.
class RunLog {
 public:
  constexpr RunLog(const RunLogReadFn read, const RunLogWriteFn write,
                   const uint16_t address, const uint16_t capacity)
      : read_(read), write_(write), address_(address), capacity_(capacity) {}

  /// START RUN: begins a new run, overwriting whatever was stored (the
  /// caller must have already read out the previous log). Writes the header
  /// "in progress, 0 steps" - this call IS the START RUN marker.
  /// `openFloor` sets kRunLogOpenFloorFlag (virtual boundary).
  void startRun(uint8_t rows, uint8_t cols, Pose start, Cell goal,
                bool openFloor = false);

  /// One step, appended while the robot is stopped (SENSE/PLAN; never from
  /// the 100 Hz move loop). No-op beyond `capacity` other than setting the
  /// truncated flag once; the run keeps going.
  void appendStep(const RunStep& step);

  /// END RUN: the goal was reached at `cell`/`heading` (stored in the
  /// fault* pose fields, which hold the END pose of either outcome).
  void finishGoal(Cell cell, Direction heading) {
    finish(RunOutcome::kGoalReached, Status::kOk, cell, heading);
  }
  /// END RUN: a fault occurred at `cell`/`heading`.
  void finishFault(const Status status, const Cell cell,
                   const Direction heading) {
    finish(RunOutcome::kFault, status, cell, heading);
  }

  bool truncated() const {
    return (header_.flags & kRunLogTruncatedFlag) != 0;
  }
  uint16_t stepCount() const { return header_.stepCount; }

 private:
  void writeHeader() { write_(address_, &header_, sizeof(header_)); }
  void finish(RunOutcome outcome, Status status, Cell cell,
              Direction heading);
  uint16_t stepAddress(const uint16_t index) const {
    return static_cast<uint16_t>(address_ + sizeof(RunLogHeader) +
                                 index * sizeof(uint16_t));
  }

  RunLogReadFn read_;
  RunLogWriteFn write_;
  uint16_t address_;
  uint16_t capacity_;
  RunLogHeader header_{};
};

}  // namespace micromouse
