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

bool isBoundaryWall(const Cell cell, const Direction dir, const uint8_t rows,
                    const uint8_t cols) {
  switch (dir) {
    case Direction::kUp:
      return cell.row == 0;
    case Direction::kDown:
      return static_cast<uint8_t>(cell.row + 1) >= rows;
    case Direction::kLeft:
      return cell.col == 0;
    case Direction::kRight:
      return static_cast<uint8_t>(cell.col + 1) >= cols;
  }
  return false;
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

void absoluteWalls(const Cell cell, const Direction heading,
                   const bool wallLeft, const bool wallFront,
                   const bool wallRight, const uint8_t rows,
                   const uint8_t cols, bool out[kDirectionCount]) {
  for (uint8_t d = 0; d < kDirectionCount; ++d) {
    out[d] = isBoundaryWall(cell, static_cast<Direction>(d), rows, cols);
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
  if (out.state != static_cast<uint8_t>(RunLogState::kInProgress) &&
      out.state != static_cast<uint8_t>(RunLogState::kComplete)) {
    return RunLogValidity::kOutOfRange;
  }
  if (out.stepCount > capacity) return RunLogValidity::kOutOfRange;
  if (out.state == static_cast<uint8_t>(RunLogState::kComplete) &&
      out.outcome != static_cast<uint8_t>(RunOutcome::kGoalReached) &&
      out.outcome != static_cast<uint8_t>(RunOutcome::kFault)) {
    return RunLogValidity::kOutOfRange;
  }
  if (out.crc != computeRunLogCrc(read, address, out)) {
    return RunLogValidity::kBadCrc;
  }
  return RunLogValidity::kValid;
}

RunStep readRunLogStep(const RunLogReadFn read, const uint16_t address,
                       const uint16_t index) {
  uint16_t packed = 0;
  read(stepAddressOf(address, index), &packed, sizeof(packed));
  return decodeStep(packed);
}

void RunLog::startRun(const uint8_t rows, const uint8_t cols,
                      const Pose start, const Cell goal) {
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
  header_.flags = 0;
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
      header_.flags |= kRunLogTruncatedFlag;
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

void RunLog::finishGoal() {
  header_.state = static_cast<uint8_t>(RunLogState::kComplete);
  header_.outcome = static_cast<uint8_t>(RunOutcome::kGoalReached);
  header_.crc = computeRunLogCrc(read_, address_, header_);
  writeHeader();
}

void RunLog::finishFault(const Status status, const Cell cell,
                         const Direction heading) {
  header_.state = static_cast<uint8_t>(RunLogState::kComplete);
  header_.outcome = static_cast<uint8_t>(RunOutcome::kFault);
  header_.faultStatus = static_cast<uint8_t>(status);
  header_.faultRow = cell.row;
  header_.faultCol = cell.col;
  header_.faultHeading = static_cast<uint8_t>(heading);
  header_.crc = computeRunLogCrc(read_, address_, header_);
  writeHeader();
}

}  // namespace micromouse
