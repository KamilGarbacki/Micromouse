#include "control/calibration.h"

#include "config/config.h"
#include "control/learning.h"
#include "util/flash.h"
#include "util/crc16.h"

namespace micromouse::calibration {

namespace cfg = config;

namespace {

constexpr uint16_t kTicksPerWheelRev =
    static_cast<uint16_t>(cfg::kTicksPerWheelRev + 0.5f);

/// lo <= x <= hi; false for NaN.
bool inRange(const float x, const float lo, const float hi) {
  return x >= lo && x <= hi;
}

bool wheelValid(const WheelCalibration& w) {
  const float maxPwm = static_cast<float>(cfg::kMaxPwm);
  return inRange(w.staticPwm, 0.0f, maxPwm) &&
         inRange(w.kineticPwm, 0.0f, w.staticPwm) &&
         inRange(w.ffOffsetPwm, cfg::kMinFfOffsetPwm, maxPwm) &&
         inRange(w.ffSlopePwmPerTpp, cfg::kMinFfSlope, cfg::kMaxFfSlope);
}

const WheelCalibration kDefaultWheels[2] MM_FLASH = {
    {cfg::forward::kDefaultStaticPwm, cfg::forward::kDefaultKineticPwm,
     cfg::forward::kDefaultFfOffsetPwm, cfg::forward::kDefaultFfSlope},
    {cfg::turn::kDefaultStaticPwm, cfg::turn::kDefaultKineticPwm,
     cfg::turn::kDefaultFfOffsetPwm, cfg::turn::kDefaultFfSlope},
};

WheelCalibration defaultWheel(const MoveKind kind) {
  return util::loadFlash(kDefaultWheels[static_cast<uint8_t>(kind)]);
}

}  // namespace

MotorRecord defaultMotor() {
  MotorRecord r = {};
  r.wheel[0] = defaultWheel(MoveKind::kForward);
  r.wheel[1] = defaultWheel(MoveKind::kForward);
  seal(r);
  return r;
}

LearnedRecord defaultLearned() {
  LearnedRecord r = {};
  r.mmPerTick = cfg::kMmPerTick;
  r.cellPitchMm = cfg::kCellSizeMm;
  r.ticksPer90 = static_cast<float>(cfg::kTicksPer90);
  r.brakeLeadForward = static_cast<float>(cfg::forward::kBrakeLeadTicks);
  r.brakeLeadTurn = static_cast<float>(cfg::turn::kBrakeLeadTicks);
  seal(r);
  return r;
}

Calibration defaults() { return {defaultMotor(), defaultLearned(), false}; }

uint16_t computeCrc(const MotorRecord& record) {
  return util::crc16(&record, kMotorCrcOffset);
}

uint16_t computeCrc(const LearnedRecord& record) {
  return util::crc16(&record, kLearnedCrcOffset);
}

void seal(MotorRecord& record) {
  record.magic = kMotorMagic;
  record.version = kMotorVersion;
  record.ticksPerWheelRev = kTicksPerWheelRev;
  record.crc = computeCrc(record);
}

void seal(LearnedRecord& record) {
  record.magic = kLearnedMagic;
  record.version = kLearnedVersion;
  record.geometryTicksPerCell = static_cast<uint16_t>(cfg::kTicksPerCell);
  record.geometryTicksPer90 = static_cast<uint16_t>(cfg::kTicksPer90);
  record.crc = computeCrc(record);
}

Validity validate(const MotorRecord& record) {
  if (record.magic == kErasedMagic) return Validity::kEmpty;
  if (record.magic != kMotorMagic) return Validity::kBadMagic;
  if (record.version != kMotorVersion) return Validity::kBadVersion;
  if (record.crc != computeCrc(record)) return Validity::kBadCrc;
  if (record.ticksPerWheelRev != kTicksPerWheelRev) {
    return Validity::kGeometryChanged;
  }
  return wheelValid(record.wheel[0]) && wheelValid(record.wheel[1])
             ? Validity::kValid
             : Validity::kOutOfRange;
}

Validity validate(const LearnedRecord& record) {
  if (record.magic == kErasedMagic) return Validity::kEmpty;
  if (record.magic != kLearnedMagic) return Validity::kBadMagic;
  if (record.version != kLearnedVersion) return Validity::kBadVersion;
  if (record.crc != computeCrc(record)) return Validity::kBadCrc;
  if (record.geometryTicksPerCell !=
          static_cast<uint16_t>(cfg::kTicksPerCell) ||
      record.geometryTicksPer90 != static_cast<uint16_t>(cfg::kTicksPer90)) {
    return Validity::kGeometryChanged;
  }
  constexpr float kHuge = 1e30f;
  bool sumsValid = record.pitchSums.v[learning::PitchSums::kAA] >= 0.0f &&
                   record.pitchSums.v[learning::PitchSums::kBB] >= 0.0f;
  for (const float v : record.pitchSums.v) {
    sumsValid = sumsValid && inRange(v, -kHuge, kHuge);
  }
  const bool valid = sumsValid &&
      (record.flags & ~learning::kPitchStateMask) == 0 &&
      inRange(record.mmPerTick, cfg::kMmPerTickMin, cfg::kMmPerTickMax) &&
      inRange(record.cellPitchMm, cfg::kMinCellSizeMm, cfg::kMaxCellSizeMm) &&
      inRange(record.wallOffsetMm, -cfg::kMaxWallOffsetMm,
              cfg::kMaxWallOffsetMm) &&

      inRange(record.ticksPer90, cfg::kTicksPer90Min, cfg::kTicksPer90Max) &&
      inRange(record.brakeLeadForward, 0.0f,
              static_cast<float>(cfg::forward::kMaxBrakeLeadTicks)) &&
      inRange(record.brakeLeadTurn, 0.0f,
              static_cast<float>(cfg::turn::kMaxBrakeLeadTicks));
  return valid ? Validity::kValid : Validity::kOutOfRange;
}

void restartIncompleteSeeds(LearnedRecord& record) {
  if (record.scaleSamples < cfg::kScaleLearnMinSamples) {
    record.scaleSamples = 0;
  }
  if (record.pitchSamples < cfg::kPitchLearnMinSamples) {
    record.pitchSamples = 0;
  }
  if (record.turnSamples < cfg::kTurnLearnMinSamples) record.turnSamples = 0;
}

float mmPerTick(const Calibration& cal) {
  return learning::learnedOrPrior(cal.learned.mmPerTick,
                                  cal.learned.scaleSamples, cfg::kMmPerTick,
                                  cfg::kScaleLearnMinSamples);
}

bool pitchJoint(const Calibration& cal) {
  return (cal.learned.flags & kLearnedFlagJoint) != 0;
}

float cellPitchMm(const Calibration& cal) {
  if (pitchJoint(cal)) return cal.learned.cellPitchMm;
  return learning::learnedOrPrior(cal.learned.cellPitchMm,
                                  cal.learned.pitchSamples, cfg::kCellSizeMm,
                                  cfg::kPitchLearnMinSamples);
}

float wallOffsetMm(const Calibration& cal) {
  return pitchJoint(cal) ? cal.learned.wallOffsetMm : 0.0f;
}

int32_t ticksPerCell(const Calibration& cal) {
  return static_cast<int32_t>(cellPitchMm(cal) / mmPerTick(cal) + 0.5f);
}

int32_t ticksPer90(const Calibration& cal) {
  const float ticks = learning::learnedOrPrior(
      cal.learned.ticksPer90, cal.learned.turnSamples,
      static_cast<float>(cfg::kTicksPer90), cfg::kTurnLearnMinSamples);
  return static_cast<int32_t>(ticks + 0.5f);
}

WheelCalibration applyLoad(const WheelCalibration& lifted, const float scale,
                           const float offsetPwm) {
  return {lifted.staticPwm * scale + offsetPwm,
          lifted.kineticPwm * scale + offsetPwm,
          lifted.ffOffsetPwm * scale + offsetPwm, lifted.ffSlopePwmPerTpp};
}

WheelCalibration wheelModel(const Calibration& cal, const Wheel wheel,
                            const MoveKind kind) {
  if (!cal.motorsCalibrated) return defaultWheel(kind);
  const WheelCalibration& lifted = cal.motor.wheel[static_cast<uint8_t>(wheel)];
  return kind == MoveKind::kTurn
             ? applyLoad(lifted, cfg::kTurnLoadScale, cfg::kTurnLoadOffsetPwm)
             : applyLoad(lifted, cfg::kForwardLoadScale,
                         cfg::kForwardLoadOffsetPwm);
}

}  // namespace micromouse::calibration
