#include "control/learning.h"

#include "config/config.h"
#include "util/math_utils.h"
#include "util/median.h"

namespace micromouse::learning {

namespace {

constexpr float kHalfPi = 1.57079633f;
constexpr float kQuarterPi = 0.785398163f;
constexpr float kHuge = 1e30f;
constexpr float kRelScale = 10000.0f;  // buffer unit: 0.01 % of the prior

static_assert(config::kLearnReacquireCount >= 1 &&
                  config::kLearnReacquireCount <= kLearnBuffer &&
                  config::kScaleLearnMinSamples <= kLearnBuffer &&
                  config::kPitchLearnMinSamples <= kLearnBuffer &&
                  config::kTurnLearnMinSamples <= kLearnBuffer &&
                  config::kScaleLearnMinSamples >= 1 &&
                  config::kPitchLearnMinSamples >= 1 &&
                  config::kTurnLearnMinSamples >= 1,
              "seed / re-acquisition sizes must fit the estimator buffer");

/// False for NaN and +-infinity (no <cmath> isfinite on the AVR).
bool finite(const float x) { return x == x && x < kHuge && x > -kHuge; }

int16_t encode(const float sample, const float prior) {
  const float rel = (sample / prior - 1.0f) * kRelScale;
  if (rel > 32767.0f) return 32767;
  if (rel < -32767.0f) return -32767;
  return static_cast<int16_t>(rel);
}

float decode(const int16_t rel, const float prior) {
  return prior + prior * (static_cast<float>(rel) / kRelScale);
}

}  // namespace

float learnedOrPrior(const float value, const uint16_t samples,
                     const float prior, const uint16_t minSamples) {
  return samples >= minSamples ? value : prior;
}

float RobustEstimator::bufferedMedian(const float prior) {
  int16_t sorted[kLearnBuffer];
  for (uint8_t i = 0; i < count_; ++i) sorted[i] = buffer_[i];
  return decode(util::medianInPlace(sorted, count_), prior);
}

LearnOutcome RobustEstimator::update(float& value, uint16_t& samples,
                                     const float sample, const float prior,
                                     const LearnParams& p) {
  // `!(x > 0)` also rejects NaN.
  if (!(sample > 0.0f) || !(sample < kHuge) || !(prior > 0.0f)) {
    return LearnOutcome::kInvalid;
  }
  const bool seeding = samples < p.minSamples;
  const bool inBounds = sample >= p.lo && sample <= p.hi;
  if (seeding) {
    // Seeding: only the hard bounds gate; the estimate is the median.
    if (!inBounds) return LearnOutcome::kOutlier;
  } else if (util::absValue(sample - value) <= p.outlierFraction * value) {
    const float next = value + p.gain * (sample - value);
    value = next < p.lo ? p.lo : (next > p.hi ? p.hi : next);
    if (samples < 0xFFFF) ++samples;
    count_ = 0;  // outliers must be CONSECUTIVE to re-acquire
    return LearnOutcome::kAccepted;
  }

  // Buffer the sample: a seed, or the latest of consecutive outliers.
  const uint8_t window =
      seeding ? static_cast<uint8_t>(p.minSamples) : p.reacquireCount;
  if (count_ >= window) {
    for (uint8_t i = 1; i < count_; ++i) buffer_[i - 1] = buffer_[i];
    count_ = static_cast<uint8_t>(window - 1);
  }
  buffer_[count_++] = encode(sample, prior);
  if (seeding) samples = count_;
  if (count_ < window) {
    return seeding ? LearnOutcome::kSeeding : LearnOutcome::kOutlier;
  }
  const float median = bufferedMedian(prior);
  if (!seeding) {
    // Re-acquire only if the outliers agree with each other.
    if (median < p.lo || median > p.hi) return LearnOutcome::kOutlier;
    for (uint8_t i = 0; i < count_; ++i) {
      if (util::absValue(decode(buffer_[i], prior) - median) >
          p.outlierFraction * median) {
        return LearnOutcome::kOutlier;
      }
    }
  }
  value = median < p.lo ? p.lo : (median > p.hi ? p.hi : median);
  samples = p.minSamples;
  count_ = 0;
  return seeding ? LearnOutcome::kAccepted : LearnOutcome::kReacquired;
}

float mmPerTickSample(const float frontBeforeMm, const float frontAfterMm,
                      const int32_t ticksDriven) {
  if (ticksDriven <= 0 || !(frontAfterMm > 0.0f) ||
      !(frontBeforeMm > frontAfterMm)) {
    return 0.0f;
  }
  return (frontBeforeMm - frontAfterMm) / static_cast<float>(ticksDriven);
}

PitchObservation pitchObservation(const float drivenMm,
                                  const float frontEndMm, const uint8_t cells,
                                  const bool alignedReference,
                                  const float frontRefMm,
                                  const float configuredWallMm,
                                  const float configuredSensorMm) {
  const float half = 0.5f * configuredWallMm + configuredSensorMm;
  if (alignedReference) {
    return {static_cast<float>(cells) + 1.0f, 2.0f,
            drivenMm + frontEndMm + frontRefMm + 2.0f * half};
  }
  return {static_cast<float>(cells) + 0.5f, 1.0f,
          drivenMm + frontEndMm + half};
}

bool solvePitch(const PitchSums& sums, const float minDecorrelation,
                float& pitch, float& offset) {
  const float* s = sums.v;
  const float product = s[PitchSums::kAA] * s[PitchSums::kBB];
  if (!(product > 0.0f)) return false;
  const float det = product - s[PitchSums::kAB] * s[PitchSums::kAB];
  if (!(det >= minDecorrelation * product)) return false;
  pitch = (s[PitchSums::kBB] * s[PitchSums::kAY] -
           s[PitchSums::kAB] * s[PitchSums::kBY]) /
          det;
  offset = (s[PitchSums::kAA] * s[PitchSums::kBY] -
            s[PitchSums::kAB] * s[PitchSums::kAY]) /
           det;
  return true;
}

LearnOutcome PitchOffsetEstimator::update(PitchSums& sums, float& pitch,
                                          float& offset, uint8_t& state,
                                          const PitchObservation& o,
                                          const PitchParams& p) {
  if (!(o.a > 0.0f) || !finite(o.y)) return LearnOutcome::kInvalid;
  const float span = util::absValue(o.b) * p.offsetMax;
  if (o.y < o.a * p.pitchMin - span || o.y > o.a * p.pitchMax + span) {
    return LearnOutcome::kOutlier;
  }
  bool restarted = false;
  if (pitchJointInForce(state)) {
    const float residual = o.y - (o.a * pitch + o.b * offset);
    if (util::absValue(residual) > p.gateFraction * pitch * o.a) {
      if (++outliers_ < p.reacquireCount) return LearnOutcome::kOutlier;
      // Restart, seeded with this (consistent) observation.
      sums = PitchSums{};
      state = 0;
      restarted = true;
    }
  }
  outliers_ = 0;
  const float row[PitchSums::kCount] = {o.a * o.a, o.a * o.b, o.b * o.b,
                                       o.a * o.y, o.b * o.y};
  for (uint8_t i = 0; i < PitchSums::kCount; ++i) {
    sums.v[i] = p.forget * sums.v[i] + row[i];
  }
  uint8_t rows = pitchRows(state);
  if (rows < (kPitchRowsMask >> kPitchRowsShift)) ++rows;
  state = static_cast<uint8_t>((state & kPitchJointBit) |
                               (rows << kPitchRowsShift));
  float solvedPitch = 0.0f;
  float solvedOffset = 0.0f;
  if (rows >= p.minRows &&
      solvePitch(sums, p.minDecorrelation, solvedPitch, solvedOffset)) {
    pitch = util::clamp(solvedPitch, p.pitchMin, p.pitchMax);
    offset = util::clamp(solvedOffset, -p.offsetMax, p.offsetMax);
    state |= kPitchJointBit;
  }
  if (restarted) return LearnOutcome::kReacquired;
  return pitchJointInForce(state) ? LearnOutcome::kAccepted
                                  : LearnOutcome::kSeeding;
}

float straightDistanceMm(const StraightMove& move, const float mmPerTick) {
  return 0.5f * static_cast<float>(move.left + move.right) * mmPerTick;
}

WallDrift wallDriftOf(const StraightMove& move, const bool leftWall,
                      const float startMm, const float endMm,
                      const float mmPerTick, const float ticksPer90,
                      const float leverMm) {
  const float k = kQuarterPi / ticksPer90;  // rad per (L - R) tick
  WallDrift d = {};
  d.startMm = startMm;
  d.endMm = endMm;
  d.sigma = leftWall ? 1 : -1;
  d.distanceMm = straightDistanceMm(move, mmPerTick);
  d.encStartRad = static_cast<float>(move.carryBefore) * k;
  d.encEndRad =
      static_cast<float>(move.carryBefore + move.left - move.right) * k;
  d.encIntegralMmRad = move.headingIntegral * k * mmPerTick;
  d.leverMm = leverMm;
  return d;
}

float invisibleHeadingRad(const WallDrift& drift) {
  if (!(drift.distanceMm > 0.0f)) return 0.0f;
  const float lateral = static_cast<float>(drift.sigma) *
                            (drift.endMm - drift.startMm) -
                        drift.leverMm * (drift.encEndRad - drift.encStartRad);
  // atan(x) ~= x: |x| <= ~0.12 here (25 mm drift over >= half a cell), where
  // the error is < 0.5 % of the angle - far below the sensor noise - and it
  // saves libm's atan in flash.
  return (lateral - drift.encIntegralMmRad) / drift.distanceMm;
}

float ticksPer90Sample(const float deltaEpsRad, const int8_t turnSign,
                       const float turnTicksPerWheel,
                       const float ticksPer90Used) {
  if ((turnSign != 1 && turnSign != -1) || !(turnTicksPerWheel > 0.0f) ||
      !(ticksPer90Used > 0.0f) || !finite(deltaEpsRad)) {
    return 0.0f;
  }
  const float inverse =
      1.0f / ticksPer90Used +
      deltaEpsRad /
          (static_cast<float>(turnSign) * kHalfPi * turnTicksPerWheel);
  return inverse > 0.0f ? 1.0f / inverse : 0.0f;
}

void TurnScaleLearner::reset() {
  haveBaseline_ = false;
  turns_ = 0;
  sinceMm_ = 0.0f;
}

void TurnScaleLearner::onTurn(const int8_t sign, const float ticksPerWheel,
                              const float ticksPer90Used) {
  if (turns_ < 0xFF) ++turns_;
  turnSign_ = sign;
  turnTicks_ = static_cast<uint16_t>(ticksPerWheel + 0.5f);
  turnTicksPer90_ = static_cast<uint16_t>(ticksPer90Used + 0.5f);
}

void TurnScaleLearner::onStraight(const float mm) { sinceMm_ += mm; }

float TurnScaleLearner::curvatureRadPerMm() const {
  return curveSamples_ >= config::kCurveLearnMinSamples ? curve_ : 0.0f;
}

bool TurnScaleLearner::onMeasurement(const float epsRad, const float moveMm,
                                     float& sample) {
  bool produced = false;
  const float distance = sinceMm_ + 0.5f * moveMm;
  if (haveBaseline_ && distance > 0.0f) {
    const float delta = epsRad - baselineRad_;
    if (turns_ == 0) {
      const float curve = delta / distance;
      if (util::absValue(curve) <= config::kMaxCurveRadPerMm) {
        if (curveSamples_ < 0xFF) ++curveSamples_;
        const float meanGain = 1.0f / static_cast<float>(curveSamples_);
        const float gain =
            meanGain > config::kCurveLearnGain ? meanGain
                                               : config::kCurveLearnGain;
        curve_ += gain * (curve - curve_);
      }
    } else if (turns_ == 1) {
      sample = ticksPer90Sample(delta - curvatureRadPerMm() * distance,
                                turnSign_, static_cast<float>(turnTicks_),
                                static_cast<float>(turnTicksPer90_));
      produced = sample > 0.0f;
    }
  }
  baselineRad_ = epsRad;
  haveBaseline_ = true;
  turns_ = 0;
  sinceMm_ = 0.5f * moveMm;
  return produced;
}

float adaptBrakeLead(const float lead, const int32_t over, const float gain,
                     const float maxLead, const int32_t maxAbsOver) {
  if (util::absValue(over) > maxAbsOver) return lead;
  return util::clamp(lead + gain * static_cast<float>(over), 0.0f, maxLead);
}

int32_t alignmentTicks(const float frontMm, const float mmPerTick,
                       const AlignParams& p) {
  if (!(frontMm > 0.0f) || !(mmPerTick > 0.0f)) return 0;
  const float error = frontMm - p.nominalMm;  // + = too far: drive forward
  const float magnitude = util::absValue(error);
  if (magnitude < p.deadbandMm || magnitude > p.maxErrorMm) return 0;
  const float mm = util::clamp(error, -p.maxCorrectionMm, p.maxCorrectionMm);
  const float ticks = mm / mmPerTick;
  return static_cast<int32_t>(ticks >= 0.0f ? ticks + 0.5f : ticks - 0.5f);
}

int32_t alignmentBrakeLead(const int32_t moveTicks, const int32_t leadTicks) {
  const int32_t half = util::absValue(moveTicks) / 2;
  return util::clamp<int32_t>(leadTicks, 0, half);
}

}  // namespace micromouse::learning
