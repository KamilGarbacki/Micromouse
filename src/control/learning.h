// Online calibration estimators (pure, host-tested): wheel scale and cell
// pitch from the front sensor, ticks per 90 deg from side-wall drift,
// brake-lead adaptation, and the front-wall alignment math. The app gathers
// the readings and gates them with the maze map; everything numeric is here.
#pragma once

#include <stdint.h>

namespace micromouse::learning {

// --- Robust running estimate -------------------------------------------------

struct LearnParams {
  float lo;               ///< hard bounds of the estimate (and of seeds)
  float hi;
  float outlierFraction;  ///< tracking: reject |sample - value| > this*value
  float gain;             ///< tracking: EMA gain
  uint16_t minSamples;    ///< seed size (median), 1..kLearnBuffer
  uint8_t reacquireCount; ///< consecutive agreeing outliers, 1..kLearnBuffer
};

constexpr uint8_t kLearnBuffer = 4;

enum class LearnOutcome : uint8_t {
  kAccepted,    ///< seed completed or EMA update: the estimate changed
  kSeeding,     ///< stored as a seed sample, estimate not applied yet
  kOutlier,     ///< rejected (outside the bounds / the gate)
  kReacquired,  ///< re-seeded from consecutive agreeing outliers
  kInvalid,     ///< not finite or <= 0
};

/// The value in use: the estimate once it has `minSamples` samples, else the
/// prior (the geometry default).
float learnedOrPrior(float value, uint16_t samples, float prior,
                     uint16_t minSamples);

/// Robust estimator. The estimate (`value`, `samples`) lives in the stored
/// record; the seed / outlier buffer is RAM only (relative to the prior).
///  - Seeding (samples < minSamples): only the hard bounds [lo, hi] gate.
///    After minSamples samples the estimate is their MEDIAN (one wild early
///    sample cannot bias it).
///  - Tracking: samples within outlierFraction of the estimate move it by
///    the EMA gain (clamped to [lo, hi]). Other samples are outliers; once
///    `reacquireCount` CONSECUTIVE outliers agree with each other (all within
///    outlierFraction of their median, and the median within [lo, hi]) the
///    estimate is re-seeded from that median: a stale stored value (hardware
///    changed) or a bad early estimate cannot lock the learning out.
class RobustEstimator {
 public:
  LearnOutcome update(float& value, uint16_t& samples, float sample,
                      float prior, const LearnParams& params);

  /// Forgets the buffered seed / outlier samples.
  void reset() { count_ = 0; }

 private:
  float bufferedMedian(float prior);

  int16_t buffer_[kLearnBuffer] = {};
  uint8_t count_ = 0;
};

// --- Wheel scale and cell pitch (front sensor as the ruler) ------------------

/// Wheel-scale sample [mm per tick] of a straight move towards a wall: the
/// front distance shrank from `frontBeforeMm` to `frontAfterMm` while the
/// wheels turned `ticksDriven` (average of both). 0 (invalid) unless
/// before > after > 0 and ticks > 0.
float mmPerTickSample(float frontBeforeMm, float frontAfterMm,
                      int32_t ticksDriven);

// Cell pitch P and front-offset error delta, estimated JOINTLY.
//
// Along the axis of travel the wall centre lines are at multiples of P. A
// wall of TRUE thickness w has its face w / 2 in front of its centre line,
// and the front sensor sits s ahead of the rotation centre. The firmware
// only knows the configured w_c and s_c; the error that matters is
//   delta = (w_c - w) / 2 + (s_c - s)    [mm, configured minus true].
// (A robot is truly centred when its front reading is
//  nominal(P) + delta, nominal(P) = P / 2 - w_c / 2 - s_c.)
// From a reference the robot drives `n` straight cells (encoder distance D)
// to a wall ahead and reads r_e before aligning:
//  - start cell (placed centred by hand, centre at P / 2):
//      r_e = (n + 1) P - w / 2 - P / 2 - D - s
//      => y = D + r_e + w_c / 2 + s_c = (n + 1/2) P + delta
//  - aligned reference (the robot was aligned on the wall BEHIND it, which
//    read r_f there - including any residual of the alignment - and turned
//    180 deg; centre at w / 2 + r_f + s):
//      r_e = (n + 1) P - w / 2 - (w / 2 + r_f + s) - D - s
//      => y = D + r_e + r_f + w_c + 2 s_c = (n + 1) P + 2 delta
// (Thicker real walls than configured, delta < 0, bring the wall face
// closer: smaller readings, smaller y.) Each observation is linear in
// (P, delta): a * P + b * delta = y with (a, b) = (n + 1/2, +1) or
// (n + 1, +2). One alone cannot separate P from delta - assuming delta = 0
// biases P by +delta / (n + 1/2) or +2 delta / (n + 1), e.g. 12 mm walls
// configured as 0 (delta = -6) make a 220 mm maze look like 214 mm from
// aligned 1-cell runs (216 mm from a 1-cell run from the start cell) - so
// they are solved together. That needs observations with a different
// ratio a / b, not just a different n: an aligned run of 2k cells
// (a / b = (2k + 1) / 2) is collinear with a start run of k cells
// (k + 1/2); the determinant check of solvePitch() decides. Measuring w
// and s well still helps: delta is then small, the provisional pitch
// (below) is nearly unbiased and the joint solve converges faster.

/// One pitch observation a * P + b * delta = y.
struct PitchObservation {
  float a;
  float b;
  float y;
};

/// The observation of a run of `cells` straight cells (> 0) over
/// `drivenMm`, ending with the front reading `frontEndMm`, from the start
/// cell or (`alignedReference`) from an alignment that read `frontRefMm`.
/// `configuredWallMm` / `configuredSensorMm` are w_c and s_c.
PitchObservation pitchObservation(float drivenMm, float frontEndMm,
                                  uint8_t cells, bool alignedReference,
                                  float frontRefMm, float configuredWallMm,
                                  float configuredSensorMm);

/// Normal equations of the joint least squares (exponentially forgetting):
/// v = {sum a^2, sum a b, sum b^2, sum a y, sum b y}.
struct PitchSums {
  static constexpr uint8_t kAA = 0;
  static constexpr uint8_t kAB = 1;
  static constexpr uint8_t kBB = 2;
  static constexpr uint8_t kAY = 3;
  static constexpr uint8_t kBY = 4;
  static constexpr uint8_t kCount = 5;
  float v[kCount];
};

struct PitchParams {
  float pitchMin;          ///< bounds of P [mm]
  float pitchMax;
  float offsetMax;         ///< bound of |delta| [mm]
  float gateFraction;      ///< residual gate per cell, fraction of P
  float forget;            ///< sums *= forget before each observation
  float minDecorrelation;  ///< det / (saa * sbb) needed to solve
  uint8_t reacquireCount;  ///< consecutive outliers -> restart
  uint8_t minRows;         ///< accepted rows since a restart before solving
};

/// Solves the normal equations. False if they are not conditioned (det /
/// (sum a^2 * sum b^2) < minDecorrelation, i.e. all observations have
/// (nearly) the same ratio a / b).
bool solvePitch(const PitchSums& sums, float minDecorrelation, float& pitch,
                float& offset);

/// Packed estimator state (one byte of the stored record): bit 0 = a joint
/// solution is in force, bits 1..3 = accepted rows since the last restart
/// (saturating at 7).
constexpr uint8_t kPitchJointBit = 0x01;
constexpr uint8_t kPitchRowsShift = 1;
constexpr uint8_t kPitchRowsMask = 0x0E;
constexpr uint8_t kPitchStateMask = kPitchJointBit | kPitchRowsMask;

constexpr bool pitchJointInForce(const uint8_t state) {
  return (state & kPitchJointBit) != 0;
}
constexpr uint8_t pitchRows(const uint8_t state) {
  return static_cast<uint8_t>((state & kPitchRowsMask) >> kPitchRowsShift);
}

/// Joint robust estimator of (P, delta) (b > 0). `sums`, `pitch`, `offset`
/// and `state` (see kPitchJointBit) live in the stored record.
///  - Hard bounds: an observation that no (P, delta) inside the bounds can
///    explain is an outlier.
///  - While a solution is in force, the residual y - (a P + b delta) must be
///    within gateFraction * P * a; reacquireCount consecutive outliers
///    restart the estimator (the hardware or the maze changed): the sums
///    are cleared and re-seeded with the observation that triggered it.
///  - Accepted observations enter the forgetting sums; once there are
///    minRows of them since the last restart AND they are conditioned, the
///    solution (clamped to the bounds) is put in force (two rows alone
///    would give P only to ~4 mm).
/// kAccepted: a solution is in force (updated if solvable); kSeeding:
/// accumulated, not yet in force; kOutlier; kReacquired: restarted;
/// kInvalid.
class PitchOffsetEstimator {
 public:
  LearnOutcome update(PitchSums& sums, float& pitch, float& offset,
                      uint8_t& state, const PitchObservation& obs,
                      const PitchParams& params);
  void reset() { outliers_ = 0; }

 private:
  uint8_t outliers_ = 0;
};

// --- Ticks per 90 deg (side-wall drift) --------------------------------------
//
// Conventions (also those of control/heading_carry.h):
//  - heading angles are + CLOCKWISE (robot rotated to the right);
//  - y is the lateral position, + to the RIGHT; dy/ds = heading;
//  - the encoder heading theta_enc is the carry plus the L - R ticks of the
//    current move, converted with pi / (4 * ticksPer90) rad per tick
//    (a 90 deg spin is L - R = 2 * ticksPer90 signed ticks);
//  - eps = true heading - encoder heading: the error the encoders cannot
//    see (turn scale / scrub, and straight-line curvature).
//
// A side sensor `lever` mm ahead of the rotation centre reads
//   left wall:  d_L = W_L + y + lever * theta
//   right wall: d_R = W_R - y - lever * theta
// so over a straight move  dy = sigma * (d_end - d_start) - lever * dtheta
// with sigma = +1 for the left, -1 for the right wall, and dtheta (the
// change of the TRUE heading) equals the change of the encoder heading
// (eps changes negligibly within one move). With
//   dy = integral(theta_enc ds) + eps * D
// the invisible error (its average over the move) is
//   eps = atan((dy - integral(theta_enc ds)) / D) ~= (dy - ...) / D.
// Subtracting integral(theta_enc ds) and lever * dtheta removes everything
// the encoders saw (the carry correction at the start of the move, and the
// wall-centering steering if enabled, which acts only through the heading
// loop): only the residual is left, so nothing is counted twice.

struct WallDrift {
  float startMm;      ///< side distance before the move
  float endMm;        ///< side distance after the move
  int8_t sigma;       ///< +1 left wall, -1 right wall
  float distanceMm;   ///< driven distance D (> 0)
  float encStartRad;  ///< encoder heading at the start
  float encEndRad;    ///< encoder heading at the end
  float encIntegralMmRad;  ///< integral of theta_enc over the distance
  float leverMm;      ///< side sensor ahead of the rotation centre
};

/// Encoder-side record of one forward move (from MoveReport).
struct StraightMove {
  int32_t carryBefore;    ///< encoder heading at the start [L - R ticks]
  int32_t left;           ///< final counts
  int32_t right;
  float headingIntegral;  ///< sum of heading x distance increments [ticks^2]
};

/// Average distance driven [mm].
float straightDistanceMm(const StraightMove& move, float mmPerTick);

/// The WallDrift of `move` along the left (`leftWall`) or right wall, with
/// the conversions above (k = pi / (4 * ticksPer90)).
WallDrift wallDriftOf(const StraightMove& move, bool leftWall, float startMm,
                      float endMm, float mmPerTick, float ticksPer90,
                      float leverMm);

/// The invisible heading error eps [rad, + = clockwise] during the move
/// (small-angle form of the atan).
float invisibleHeadingRad(const WallDrift& drift);

/// Ticks-per-90 sample from the change of eps caused by ONE turn.
/// A turn of `turnSign` (+1 CW, -1 CCW) with `turnTicksPerWheel` = (L+R)/2,
/// executed while `ticksPer90Used` was in force, is believed by the encoders
/// to rotate sign * (pi / 2) * N / T_used; it truly rotated
/// sign * (pi / 2) * N / T_true. Hence
///   deltaEps = sign * (pi / 2) * N * (1 / T_true - 1 / T_used)
///   T_true   = 1 / (1 / T_used + deltaEps / (sign * (pi / 2) * N)).
/// Over-rotating (deltaEps with the sign of the turn) -> fewer ticks per 90.
/// Returns 0 (invalid) for a non-positive result or bad inputs.
float ticksPer90Sample(float deltaEpsRad, int8_t turnSign,
                       float turnTicksPerWheel, float ticksPer90Used);

/// Attributes eps measurements to turns without double counting. eps is
/// absolute (accumulated), so a turn sample is the CHANGE of eps between the
/// measurement before and after EXACTLY ONE turn, minus the straight-line
/// curvature accumulated over the distance between the two measurements
/// (each measurement is the average over its move, i.e. at its midpoint).
/// The curvature [rad per mm] itself is learned from consecutive
/// measurements with NO turn in between (wheel-diameter mismatch).
class TurnScaleLearner {
 public:
  /// Forgets the baseline (start of a run, failed move). The curvature
  /// estimate is kept (it is a property of the wheels).
  void reset();

  /// Records an executed turn.
  void onTurn(int8_t sign, float ticksPerWheel, float ticksPer90Used);

  /// A straight move of `mm` without a measurement.
  void onStraight(float mm);

  /// Feeds the eps measurement of a straight move of `moveMm`. Returns true
  /// and writes `sample` (ticks per 90) if there was a baseline and exactly
  /// one turn since it; with no turn it updates the curvature instead. The
  /// measurement becomes the new baseline in every case.
  bool onMeasurement(float epsRad, float moveMm, float& sample);

  /// The applied curvature [rad per mm]: 0 until kCurveLearnMinSamples.
  float curvatureRadPerMm() const;

 private:
  float baselineRad_ = 0.0f;
  float sinceMm_ = 0.0f;  // from the baseline's move midpoint
  uint16_t turnTicks_ = 0;
  uint16_t turnTicksPer90_ = 0;
  float curve_ = 0.0f;
  uint8_t curveSamples_ = 0;
  uint8_t turns_ = 0;
  int8_t turnSign_ = 0;
  bool haveBaseline_ = false;
};

// --- Brake lead --------------------------------------------------------------

/// lead + gain * over, clamped to [0, maxLead]; `over` = (L+R)/2 - target
/// [ticks] (> 0: overshoot -> brake earlier). |over| > maxAbsOver is a slip
/// or bump, not a brake-lead error: lead unchanged.
float adaptBrakeLead(float lead, int32_t over, float gain, float maxLead,
                     int32_t maxAbsOver);

// --- Front-wall alignment ----------------------------------------------------

struct AlignParams {
  float nominalMm;        ///< front reading of a centred robot
  float deadbandMm;       ///< |error| below: no move
  float maxCorrectionMm;  ///< one adjustment is clamped to +- this
  float maxErrorMm;       ///< |error| above: not plausible, no move
};

/// Signed ticks to drive (+ forward, - backward) so that the front reading
/// becomes nominal; 0 for no echo (frontMm <= 0), within the deadband or an
/// implausible error.
int32_t alignmentTicks(float frontMm, float mmPerTick, const AlignParams& p);

/// Brake lead of a tiny alignment move of |moveTicks|: the learned forward
/// lead `leadTicks` (it brakes from the same creep speed), but at most half
/// the move, so the move is never swallowed by the lead.
int32_t alignmentBrakeLead(int32_t moveTicks, int32_t leadTicks);

}  // namespace micromouse::learning
