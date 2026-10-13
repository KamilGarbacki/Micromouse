#include "app/online_learning.h"

#include "control/heading_carry.h"
#include "hal/log.h"
#include "util/flash.h"
#include "util/math_utils.h"

namespace micromouse {

namespace {

namespace cfg = config;
namespace log = hal::log;
using config::LogLevel;
using learning::LearnOutcome;
using learning::LearnParams;

const LearnParams kScaleParams MM_FLASH = {
    cfg::kMmPerTickMin,           cfg::kMmPerTickMax,
    cfg::kScaleLearnOutlierFraction, cfg::kScaleLearnGain,
    cfg::kScaleLearnMinSamples,   cfg::kLearnReacquireCount};
const LearnParams kPitchParams MM_FLASH = {
    cfg::kMinCellSizeMm,          cfg::kMaxCellSizeMm,
    cfg::kPitchLearnOutlierFraction, cfg::kPitchLearnGain,
    cfg::kPitchLearnMinSamples,   cfg::kLearnReacquireCount};
const LearnParams kTurnParams MM_FLASH = {
    cfg::kTicksPer90Min,          cfg::kTicksPer90Max,
    cfg::kTurnLearnOutlierFraction, cfg::kTurnLearnGain,
    cfg::kTurnLearnMinSamples,    cfg::kLearnReacquireCount};
const learning::PitchParams kJointParams MM_FLASH = {
    cfg::kMinCellSizeMm,          cfg::kMaxCellSizeMm,
    cfg::kMaxWallOffsetMm,        cfg::kPitchLearnOutlierFraction,
    cfg::kPitchJointForget,       cfg::kPitchMinDecorrelation,
    cfg::kLearnReacquireCount,    cfg::kPitchMinRows};

bool inRange(const float x, const float lo, const float hi) {
  return x >= lo && x <= hi;
}

/// One-letter outcome tag for the debug log: + accepted, S seeding,
/// O outlier, R re-acquired / restarted, ! invalid.
[[maybe_unused]] char outcomeTag(const LearnOutcome outcome) {
  switch (outcome) {
    case LearnOutcome::kAccepted:
      return '+';
    case LearnOutcome::kSeeding:
      return 'S';
    case LearnOutcome::kOutlier:
      return 'O';
    case LearnOutcome::kReacquired:
      return 'R';
    case LearnOutcome::kInvalid:
      break;
  }
  return '!';
}

}  // namespace

void OnlineLearning::reset() {
  turnLearner_.reset();
  pendingScale_ = 0.0f;
  pendingPitchCells_ = 0;
  pendingMoveMm_ = 0.0f;
  pendingAdjustMm_ = 0;
  pendingEpsMask_ = 0;
  // The start cell: placed centred by hand, facing the start heading.
  refValid_ = true;
  refAligned_ = false;
  refCells_ = 0;
  refMm_ = 0.0f;
}

void OnlineLearning::onFailedMove() {
  reset();
  refValid_ = false;
}

void OnlineLearning::onTurn(const MoveReport& report, const int8_t sign,
                            const uint8_t quarterTurns) {
  if (!isOk(report.status) || !report.settled) {
    onFailedMove();
    return;
  }
  LearnedRecord& rec = calibration_.learned;
  rec.brakeLeadTurn = learning::adaptBrakeLead(
      rec.brakeLeadTurn, (report.left + report.right) / 2 - report.target,
      cfg::kBrakeLeadGain, static_cast<float>(cfg::turn::kMaxBrakeLeadTicks),
      cfg::kTurnBrakeMaxOverTicks);
  dirty_ = true;

  // The encoders' belief about the turn is lost if the carry was clipped
  // (heading::carryAfterTurn): no turn sample can be attributed then.
  const int32_t ticksPer90 = calibration::ticksPer90(calibration_);
  const int32_t unclipped = heading::carryAfterTurnUnclamped(
      report.carryBefore, sign, report.left, report.right,
      static_cast<int32_t>(quarterTurns) * ticksPer90);
  if (util::absValue(unclipped) > cfg::kHeadingCarryLimit) {
    turnLearner_.reset();
  } else {
    turnLearner_.onTurn(
        sign, 0.5f * static_cast<float>(report.left + report.right),
        static_cast<float>(ticksPer90));
  }
  // A 180 deg turn keeps the axis, and the position along it is still the
  // reference only if the robot has not moved since (a U-turn right after
  // the alignment); otherwise - and after any 90 deg turn - drop it.
  if (quarterTurns != 2 || refCells_ != 0) refValid_ = false;
}

void OnlineLearning::onForward(const MoveReport& report,
                               const ForwardObservation& obs) {
  pendingScale_ = 0.0f;
  pendingPitchCells_ = 0;
  pendingMoveMm_ = 0.0f;
  pendingEpsMask_ = 0;
  if (!isOk(report.status) || !report.settled) {
    onFailedMove();
    return;
  }
  LearnedRecord& rec = calibration_.learned;
  const int32_t travelled = (report.left + report.right) / 2;
  rec.brakeLeadForward = learning::adaptBrakeLead(
      rec.brakeLeadForward, travelled - report.target, cfg::kBrakeLeadGain,
      static_cast<float>(cfg::forward::kMaxBrakeLeadTicks),
      cfg::kForwardBrakeMaxOverTicks);
  dirty_ = true;
  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    log::fmtln(F("brake f=%1 t=%1"),
               static_cast<double>(rec.brakeLeadForward),
               static_cast<double>(rec.brakeLeadTurn));
  }

  const float mmPerTick = calibration::mmPerTick(calibration_);
  const learning::StraightMove move = {report.carryBefore, report.left,
                                       report.right, report.headingIntegral};
  pendingMoveMm_ = learning::straightDistanceMm(move, mmPerTick);
  // The wall right ahead (pitch / alignment): down to the sensor minimum.
  const bool frontWallNow =
      inRange(obs.frontAfterMm, cfg::kLearnFrontNearMinMm,
              cfg::kWallThresholdCm * 10.0f);

  // (a) Wheel scale: a plain straight move (the SENSE front reading was
  // taken in the same heading) with the wall ahead in RULER range both
  // before and after (the near gate above is for the pitch only).
  if (!obs.turnedBefore &&
      inRange(obs.frontAfterMm, cfg::kLearnFrontMinMm,
              cfg::kLearnFrontMaxMm) &&
      inRange(obs.frontBeforeMm, cfg::kLearnFrontMinMm,
              cfg::kLearnFrontMaxMm)) {
    pendingScale_ =
        learning::mmPerTickSample(obs.frontBeforeMm, obs.frontAfterMm,
                                  travelled);
  }

  // (b) Cell pitch / front offset: the front reading at a wall ahead after
  // `refCells_` straight cells along the reference axis, within the
  // alignment window of the centred reading in force.
  if (refValid_) {
    if (refCells_ < 0xFF) ++refCells_;
    refMm_ += pendingMoveMm_;
    const float centred =
        cfg::frontWallNominalMm(calibration::cellPitchMm(calibration_)) +
        calibration::wallOffsetMm(calibration_);
    if (frontWallNow &&
        util::absValue(obs.frontAfterMm - centred) <= cfg::kAlignMaxErrorMm) {
      const learning::PitchObservation o = learning::pitchObservation(
          refMm_, obs.frontAfterMm, refCells_, refAligned_, refFrontMm_,
          cfg::kWallThicknessMm, cfg::kFrontSensorToCenterMm);
      pendingPitchY_ = o.y;
      pendingPitchCells_ = refCells_;
      pendingPitchAligned_ = refAligned_;
    }
  }

  // (c) Invisible heading error from the drift along a side wall.
  if (util::absValue(report.carryBefore) > cfg::kTurnLearnMaxCarryTicks ||
      travelled <= 0) {
    return;
  }
  const float ticksPer90 =
      static_cast<float>(calibration::ticksPer90(calibration_));
  for (uint8_t side = 0; side < 2; ++side) {
    const uint8_t bit = side == 0 ? kSideLeftBit : kSideRightBit;
    if ((obs.sideMask & bit) == 0) continue;
    const float start = obs.sideStartMm[side];
    const float end = obs.sideEndMm[side];
    if (!inRange(start, cfg::kLearnSideMinMm, cfg::kLearnSideMaxMm) ||
        !inRange(end, cfg::kLearnSideMinMm, cfg::kLearnSideMaxMm) ||
        util::absValue(end - start) > cfg::kLearnMaxSideDriftMm) {
      continue;
    }
    const float eps = learning::invisibleHeadingRad(learning::wallDriftOf(
        move, side == 0, start, end, mmPerTick, ticksPer90,
        cfg::kSideSensorAheadOfCenterMm));
    // |eps| <= 25 mm drift / >= 100 mm = 0.25 rad fits 16 bits at 10 urad.
    pendingEps_[side] = static_cast<int16_t>(
        util::clamp(eps * 1e5f, -32767.0f, 32767.0f));
    pendingEpsMask_ |= bit;
  }
}

void OnlineLearning::onAdjusted(const float signedMm) {
  const float total = pendingAdjustMm_ + util::absValue(signedMm) + 0.5f;
  pendingAdjustMm_ = static_cast<uint8_t>(total > 255.0f ? 255.0f : total);
}

void OnlineLearning::onAligned(const AlignResult result,
                               const float finalFrontMm) {
  switch (result) {
    case AlignResult::kNotAttempted:
      // No wall right ahead: the robot did not move, the reference (and
      // its cell count) stays valid.
      break;
    case AlignResult::kCentred:
      refCells_ = 0;
      refMm_ = 0.0f;
      refValid_ = finalFrontMm > 0.0f;
      refAligned_ = true;
      refFrontMm_ = finalFrontMm;
      break;
    case AlignResult::kLost:
      // Moved, but the final position is unknown.
      refValid_ = false;
      break;
  }
}

void OnlineLearning::confirm(const bool frontWall, const bool leftWall,
                             const bool rightWall) {
  LearnedRecord& rec = calibration_.learned;
  if (frontWall && pendingScale_ > 0.0f) {
    learn(scaleEstimator_, rec.mmPerTick, rec.scaleSamples, pendingScale_,
          cfg::kMmPerTick, kScaleParams, F("scale"));
  }
  if (frontWall && pendingPitchCells_ > 0) learnPitch();
  pendingScale_ = 0.0f;
  pendingPitchCells_ = 0;

  float sum = 0.0f;
  uint8_t count = 0;
  if ((pendingEpsMask_ & kSideLeftBit) && leftWall) {
    sum += static_cast<float>(pendingEps_[0]) * 1e-5f;
    ++count;
  }
  if ((pendingEpsMask_ & kSideRightBit) && rightWall) {
    sum += static_cast<float>(pendingEps_[1]) * 1e-5f;
    ++count;
  }
  pendingEpsMask_ = 0;
  const float moveMm = pendingMoveMm_;
  const float adjustMm = static_cast<float>(pendingAdjustMm_);
  pendingMoveMm_ = 0.0f;
  pendingAdjustMm_ = 0;
  if (count == 0) {
    turnLearner_.onStraight(moveMm + adjustMm);
    return;
  }
  const float eps = sum / count;
  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    log::fmtln(F("learn eps=%2 curve=%3"),
               static_cast<double>(eps * 1000.0f),
               static_cast<double>(turnLearner_.curvatureRadPerMm() * 1e6f));
  }
  float sample = 0.0f;
  const bool produced = turnLearner_.onMeasurement(eps, moveMm, sample);
  // The alignment after the measured move also drives (curvature).
  turnLearner_.onStraight(adjustMm);
  if (produced) {
    learn(turnEstimator_, rec.ticksPer90, rec.turnSamples, sample,
          static_cast<float>(cfg::kTicksPer90), kTurnParams, F("t90"));
  }
}

void OnlineLearning::learnPitch() {
  LearnedRecord& rec = calibration_.learned;
  const learning::PitchObservation o = {
      static_cast<float>(pendingPitchCells_) +
          (pendingPitchAligned_ ? 1.0f : 0.5f),
      pendingPitchAligned_ ? 2.0f : 1.0f, pendingPitchY_};
  const LearnOutcome outcome = pitchOffsetEstimator_.update(
      rec.pitchSums, rec.cellPitchMm, rec.wallOffsetMm, rec.flags, o,
      util::loadFlash(kJointParams));
  const bool joint = calibration::pitchJoint(calibration_);
  if (outcome == LearnOutcome::kReacquired) {
    // Restarted: the provisional estimate starts from scratch as well.
    rec.pitchSamples = 0;
    pitchEstimator_.reset();
  }
  noteOutcome(outcome);
  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    // j=0: still accumulating (P and delta not yet separable); n=<cells>
    // a = from an alignment, s = from the start cell.
    log::fmtln(F("learn pitch %c n=%u%c y/a=%2 P=%2 d=%2 j=%u use=%l"),
               outcomeTag(outcome),
               static_cast<unsigned int>(pendingPitchCells_),
               pendingPitchAligned_ ? 'a' : 's',
               static_cast<double>(o.y / o.a),
               static_cast<double>(calibration::cellPitchMm(calibration_)),
               static_cast<double>(calibration::wallOffsetMm(calibration_)),
               joint ? 1u : 0u,
               static_cast<long>(calibration::ticksPerCell(calibration_)));
  }
  // Provisional path (delta = 0 assumed) until the joint solve is in
  // force: only runs of kPitchProvisionalCells or more, where an unknown
  // delta biases P least (by +delta / (n + 1/2) or +2 delta / (n + 1)).
  if (!joint && pendingPitchCells_ >= cfg::kPitchProvisionalCells &&
      outcome != LearnOutcome::kOutlier) {
    learn(pitchEstimator_, rec.cellPitchMm, rec.pitchSamples, o.y / o.a,
          cfg::kCellSizeMm, kPitchParams, F("pitch(prov)"));
  }
}

void OnlineLearning::learn(learning::RobustEstimator& estimator, float& value,
                           uint16_t& samples, const float sample,
                           const float prior, const LearnParams& flashParams,
                           const __FlashStringHelper* name) {
  const LearnOutcome outcome = estimator.update(
      value, samples, sample, prior, util::loadFlash(flashParams));
  noteOutcome(outcome);
  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    // The wheel scale is printed in um per tick (3 decimals of mm are too
    // coarse for it). The estimate is in force once n >= minSamples.
    const float unit = prior < 1.0f ? 1000.0f : 1.0f;
    log::fmtln(F("learn %s %c sample=%2 est=%2 n=%u"), name,
               outcomeTag(outcome), static_cast<double>(sample * unit),
               static_cast<double>(value * unit),
               static_cast<unsigned int>(samples));
  } else {
    (void)name;
  }
}

void OnlineLearning::noteOutcome(const LearnOutcome outcome) {
  if (outcome == LearnOutcome::kAccepted ||
      outcome == LearnOutcome::kReacquired) {
    ++updatesSinceSave_;
    dirty_ = true;
  } else if (outcome == LearnOutcome::kSeeding) {
    dirty_ = true;
  }
}

}  // namespace micromouse
