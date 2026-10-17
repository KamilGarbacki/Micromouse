#include "navigation/run_log.h"

#include "util/crc16.h"

namespace micromouse {

namespace {

constexpr uint16_t kRowMask = 0xFu;
constexpr uint16_t kColShift = 4u;
constexpr uint16_t kColMask = 0xFu;
constexpr uint16_t kHeadingShift = 8u;
constexpr uint16_t kHeadingMask = 0x3u;
constexpr uint16_t kWallLeftBit = 1u << 10;
constexpr uint16_t kWallFrontBit = 1u << 11;
constexpr uint16_t kWallRightBit = 1u << 12;
constexpr uint16_t kMoveShift = 13u;
constexpr uint16_t kMoveMask = 0x3u;
constexpr uint16_t kNoMoveBit = 1u << 15;

uint16_t stepAddressOf(const uint16_t address, const uint16_t index) {
  return static_cast<uint16_t>(address + sizeof(RunLogHeader) +
                               index * sizeof(uint16_t));
}

/// Structural check (not the CRC): state is one of the two known values,
/// and outcome is one of the two known values whenever state == kComplete.
bool headerLooksValid(const RunLogHeader& h) {
  if (h.state != static_cast<uint8_t>(RunLogState::kInProgress) &&
      h.state != static_cast<uint8_t>(RunLogState::kComplete)) {
    return false;
  }
  return h.state != static_cast<uint8_t>(RunLogState::kComplete) ||
         h.outcome == static_cast<uint8_t>(RunOutcome::kGoalReached) ||
         h.outcome == static_cast<uint8_t>(RunOutcome::kFault);
}

}  // namespace

uint16_t encodeStep(const RunStep& step) {
  uint16_t packed = static_cast<uint16_t>(step.cell.row & kRowMask);
  packed = static_cast<uint16_t>(
      packed | static_cast<uint16_t>((step.cell.col & kColMask) << kColShift));
  packed = static_cast<uint16_t>(
      packed | static_cast<uint16_t>(
                   (static_cast<uint8_t>(step.heading) & kHeadingMask)
                   << kHeadingShift));
  if (step.wallLeft) packed |= kWallLeftBit;
  if (step.wallFront) packed |= kWallFrontBit;
  if (step.wallRight) packed |= kWallRightBit;
  packed = static_cast<uint16_t>(
      packed | static_cast<uint16_t>(
                   (static_cast<uint8_t>(step.move) & kMoveMask)
                   << kMoveShift));
  if (step.noMove) packed |= kNoMoveBit;
  return packed;
}

RunStep decodeStep(const uint16_t packed) {
  RunStep step{};
  step.cell.row = static_cast<uint8_t>(packed & kRowMask);
  step.cell.col = static_cast<uint8_t>((packed >> kColShift) & kColMask);
  step.heading =
      static_cast<Direction>((packed >> kHeadingShift) & kHeadingMask);
  step.wallLeft = (packed & kWallLeftBit) != 0;
  step.wallFront = (packed & kWallFrontBit) != 0;
  step.wallRight = (packed & kWallRightBit) != 0;
  step.move = static_cast<Direction>((packed >> kMoveShift) & kMoveMask);
  step.noMove = (packed & kNoMoveBit) != 0;
  return step;
}

uint8_t boundarySides(const Cell cell, const uint8_t rows,
                      const uint8_t cols) {
  uint8_t sides = 0;
  if (cell.row == 0) sides |= 1u << static_cast<uint8_t>(Direction::kUp);
  if (static_cast<uint8_t>(cell.col + 1) >= cols) {
    sides |= 1u << static_cast<uint8_t>(Direction::kRight);
  }
  if (static_cast<uint8_t>(cell.row + 1) >= rows) {
    sides |= 1u << static_cast<uint8_t>(Direction::kDown);
  }
  if (cell.col == 0) sides |= 1u << static_cast<uint8_t>(Direction::kLeft);
  return sides;
}

void absoluteWalls(const Cell cell, const Direction heading,
                   const bool wallLeft, const bool wallFront,
                   const bool wallRight, const uint8_t rows,
                   const uint8_t cols, bool out[kDirectionCount]) {
  const uint8_t boundary = boundarySides(cell, rows, cols);
  for (uint8_t d = 0; d < kDirectionCount; ++d) {
    out[d] = ((boundary >> d) & 1u) != 0;
  }
  out[static_cast<uint8_t>(heading)] = wallFront;
  out[static_cast<uint8_t>(rotate(heading, Relative::kLeft))] = wallLeft;
  out[static_cast<uint8_t>(rotate(heading, Relative::kRight))] = wallRight;
}

uint16_t computeRunLogCrc(const RunLogReadFn read, const uint16_t address,
                          const RunLogHeader& header) {
  uint16_t crc = util::crc16(&header, kRunLogCrcOffset);
  for (uint16_t i = 0; i < header.stepCount; ++i) {
    uint16_t packed = 0;
    read(stepAddressOf(address, i), &packed, sizeof(packed));
    crc = util::crc16(&packed, sizeof(packed), crc);
  }
  return crc;
}

RunLogValidity readRunLog(const RunLogReadFn read, const uint16_t address,
                          const uint16_t capacity, RunLogHeader& out) {
  read(address, &out, sizeof(out));
  if (out.magic == kRunLogErasedMagic) return RunLogValidity::kEmpty;
  if (out.magic != kRunLogMagic) return RunLogValidity::kBadMagic;
  if (out.version != kRunLogVersion) return RunLogValidity::kBadVersion;
  if (out.stepCount > capacity) return RunLogValidity::kOutOfRange;

  // A power loss mid-write (hal::eepromWrite commits bytes in offset order,
  // so the lowest-offset changed byte lands before the crc at the end)
  // leaves one of two torn states, each matching exactly the crc of the
  // PREVIOUS, fully-written header - so trying them in turn recovers the
  // last good write instead of discarding the whole log:
  //  (a) RunLog::appendStep commits the new stepCount (offset 12) before
  //      the crc (offset 20): the stored crc is still the one computed for
  //      stepCount - 1 steps.
  //  (b) RunLog::finishGoal/finishFault commit state/outcome/fault (offset
  //      3..18) before the crc: the stored crc is still the one computed
  //      with the run still "in progress" (the startRun() defaults).
  // A single torn write can only be mid-flight in one of these two calls,
  // never both, so trying them in turn covers both, and `out` is mutated
  // in place (one candidate at a time) rather than copied, to keep this
  // small.
  //
  // Two further torn-write patterns are NOT recovered here, by design (the
  // AVR flash budget does not allow more recovery code, see README "Flash
  // budget", and both are unreachable at the configured maze size): they
  // fail safe as kBadCrc (the whole log is lost), never as a false accept.
  //  (i) A torn write of the TRUNCATED flag itself (RunLog::appendStep: the
  //      flags byte, offset 11, commits but the crc does not): neither
  //      candidate below reverts `flags`, so this is not matched. Only
  //      possible once `stepCount` has reached `capacity` (kRunLogCapacity).
  //  (ii) A stepCount tear exactly on a byte-boundary increment (255 ->
  //      256: low byte 0xFF -> 0x00 and high byte 0x00 -> 0x01 both change,
  //      offsets 12 and 13): if only the low byte lands, stepCount reads as
  //      0, not `stepCount - 1`, so candidate (a) below does not match it
  //      either. Only possible once `stepCount` is at least 255, i.e. deep
  //      into a near-maximum-capacity run.
  // Both therefore need `stepCount` within a handful of `kRunLogCapacity`
  // (257, the exploration budget at the maximum STATIC maze size - see
  // kRunLogCapacity below) to even be reachable; the configured maze here
  // (5x3, budget 60) never gets close.
  bool matched = out.crc == computeRunLogCrc(read, address, out);
  if (!matched && out.stepCount > 0) {
    const uint16_t fullStepCount = out.stepCount;
    --out.stepCount;
    matched = out.crc == computeRunLogCrc(read, address, out);
    if (!matched) out.stepCount = fullStepCount;
  }
  if (matched) {
    return headerLooksValid(out) ? RunLogValidity::kValid
                                 : RunLogValidity::kOutOfRange;
  }
  const uint8_t state = out.state;
  const uint8_t outcome = out.outcome;
  const uint8_t faultStatus = out.faultStatus;
  const uint8_t faultRow = out.faultRow;
  const uint8_t faultCol = out.faultCol;
  const uint8_t faultHeading = out.faultHeading;
  out.state = static_cast<uint8_t>(RunLogState::kInProgress);
  out.outcome = static_cast<uint8_t>(RunOutcome::kNone);
  out.faultStatus = static_cast<uint8_t>(Status::kOk);
  out.faultRow = 0;
  out.faultCol = 0;
  out.faultHeading = 0;
  // Always structurally valid by construction: no headerLooksValid() gate.
  if (out.crc == computeRunLogCrc(read, address, out)) return RunLogValidity::kValid;
  out.state = state;
  out.outcome = outcome;
  out.faultStatus = faultStatus;
  out.faultRow = faultRow;
  out.faultCol = faultCol;
  out.faultHeading = faultHeading;
  return headerLooksValid(out) ? RunLogValidity::kBadCrc
                               : RunLogValidity::kOutOfRange;
}

RunStep readRunLogStep(const RunLogReadFn read, const uint16_t address,
                       const uint16_t index) {
  uint16_t packed = 0;
  read(stepAddressOf(address, index), &packed, sizeof(packed));
  return decodeStep(packed);
}

void RunLog::startRun(const uint8_t rows, const uint8_t cols,
                      const Pose start, const Cell goal,
                      const bool openFloor) {
  header_ = RunLogHeader{};
  header_.magic = kRunLogMagic;
  header_.version = kRunLogVersion;
  header_.state = static_cast<uint8_t>(RunLogState::kInProgress);
  header_.rows = rows;
  header_.cols = cols;
  header_.startRow = start.cell.row;
  header_.startCol = start.cell.col;
  header_.startHeading = static_cast<uint8_t>(start.heading);
  header_.goalRow = goal.row;
  header_.goalCol = goal.col;
  header_.flags = openFloor ? kRunLogOpenFloorFlag : 0;
  header_.stepCount = 0;
  header_.outcome = static_cast<uint8_t>(RunOutcome::kNone);
  header_.faultStatus = static_cast<uint8_t>(Status::kOk);
  header_.faultRow = 0;
  header_.faultCol = 0;
  header_.faultHeading = 0;
  header_.reserved = 0;
  header_.crc = computeRunLogCrc(read_, address_, header_);
  writeHeader();
}

void RunLog::appendStep(const RunStep& step) {
  if (header_.stepCount >= capacity_) {
    if (!truncated()) {
      // flags is covered by the crc (it is part of the header prefix): it
      // must be recomputed here too, or a read right after truncation (and
      // before the next finish*()) would see a crc mismatch.
      header_.flags |= kRunLogTruncatedFlag;
      header_.crc = computeRunLogCrc(read_, address_, header_);
      writeHeader();
    }
    return;
  }
  const uint16_t packed = encodeStep(step);
  write_(stepAddress(header_.stepCount), &packed, sizeof(packed));
  ++header_.stepCount;
  header_.crc = computeRunLogCrc(read_, address_, header_);
  writeHeader();
}

void RunLog::finish(const RunOutcome outcome, const Status status,
                    const Cell cell, const Direction heading) {
  header_.state = static_cast<uint8_t>(RunLogState::kComplete);
  header_.outcome = static_cast<uint8_t>(outcome);
  header_.faultStatus = static_cast<uint8_t>(status);
  header_.faultRow = cell.row;
  header_.faultCol = cell.col;
  header_.faultHeading = static_cast<uint8_t>(heading);
  header_.crc = computeRunLogCrc(read_, address_, header_);
  writeHeader();
}

}  // namespace micromouse
