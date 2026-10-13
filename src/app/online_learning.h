// Online refinement of the calibration while driving the maze: gathers the
// move reports and sensor readings, gates them, feeds the pure estimators
// (control/learning.h) and decides when the learned record is worth saving.
#pragma once

#include <stdint.h>

#include "config/config.h"
#include "control/calibration.h"
#include "control/learning.h"
#include "control/motion_controller.h"
#include "hal/log.h"

namespace micromouse {

/// Outcome of the front-wall alignment after a forward move.
enum class AlignResult : uint8_t {
  kNotAttempted,  ///< no wall right ahead (or out of the window): no move
  kCentred,       ///< centred on the wall ahead (re-measured)
  kLost,          ///< adjusted, then the front echo was lost
};

/// Side walls used for the drift measurement: bit 0 left, bit 1 right.
constexpr uint8_t kSideLeftBit = 0x1;
constexpr uint8_t kSideRightBit = 0x2;

/// Sensor readings around one forward move [mm, 0 = none]. Index 0 = left,
/// 1 = right for the side arrays.
struct ForwardObservation {
  bool turnedBefore;       ///< the move started with a turn
  float frontBeforeMm;     ///< SENSE front reading before the move
  float frontAfterMm;      ///< fresh front reading after it (pre-alignment)
  uint8_t sideMask;        ///< sides with a known wall in the start cell
  float sideStartMm[2];    ///< side readings at the start of the move
  float sideEndMm[2];      ///< side readings at the end (pre-alignment)
};

class OnlineLearning {
 public:
  explicit constexpr OnlineLearning(Calibration& calibration)
      : calibration_(calibration) {}

  /// Start of a run: forgets the turn baseline and pending samples; the
  /// start cell (robot placed centred by hand) is the pitch reference.
  void reset();

  /// A move failed: the turn baseline and the pitch reference are lost.
  void onFailedMove();

  /// After a completed turn (brake lead, turn record, pitch reference).
  void onTurn(const MoveReport& report, int8_t sign, uint8_t quarterTurns);

  /// After a completed one-cell forward move: adapts the forward brake lead
  /// and computes the wheel-scale, pitch and heading observations, which
  /// stay PENDING until confirm() (the map must confirm the walls first).
  void onForward(const MoveReport& report, const ForwardObservation& obs);

  /// After each alignment adjustment of `signedMm` (straight distance for
  /// the curvature bookkeeping of the turn learner).
  void onAdjusted(float signedMm);

  /// After the front-wall alignment step (also when it was not attempted):
  ///  - kCentred: `finalFrontMm` (re-measured after the last adjustment,
  ///    residual included) makes this the new pitch reference;
  ///  - kNotAttempted: no wall right ahead, the robot did not move - the
  ///    reference stays and keeps counting cells;
  ///  - kLost: the reference is dropped.
  void onAligned(AlignResult result, float finalFrontMm);

  /// After the map update in the new cell (SENSE, or the goal check):
  /// `frontWall` / `leftWall` / `rightWall` = the map has a wall there.
  /// Commits the pending samples whose walls are confirmed, drops the rest.
  void confirm(bool frontWall, bool leftWall, bool rightWall);

  /// True if the learned record changed since the last save.
  bool dirty() const { return dirty_; }
  /// True if enough samples were accepted to save (wear policy).
  bool saveDue() const {
    return updatesSinceSave_ >= config::kLearnSaveEveryUpdates;
  }
  void markSaved() {
    dirty_ = false;
    updatesSinceSave_ = 0;
  }

 private:
  void learn(learning::RobustEstimator& estimator, float& value,
             uint16_t& samples, float sample, float prior,
             const learning::LearnParams& flashParams,
             const __FlashStringHelper* name);
  void learnPitch();
  void noteOutcome(learning::LearnOutcome outcome);

  Calibration& calibration_;
  learning::RobustEstimator scaleEstimator_;
  learning::RobustEstimator pitchEstimator_;  // provisional pitch
  learning::RobustEstimator turnEstimator_;
  learning::PitchOffsetEstimator pitchOffsetEstimator_;
  learning::TurnScaleLearner turnLearner_;
  // Pending observations of the last forward move (0 = none).
  float pendingScale_ = 0.0f;
  float pendingPitchY_ = 0.0f;
  float pendingMoveMm_ = 0.0f;
  int16_t pendingEps_[2] = {0, 0};  // [10 urad] (RAM)
  uint8_t pendingAdjustMm_ = 0;     // |alignment moves| [mm], saturating
  uint8_t pendingPitchCells_ = 0;  // 0 = no pitch observation
  bool pendingPitchAligned_ = false;
  uint8_t pendingEpsMask_ = 0;
  // Pitch reference: straight cells and distance since it.
  float refMm_ = 0.0f;
  float refFrontMm_ = 0.0f;  // final front reading of an aligned reference
  uint8_t refCells_ = 0;
  bool refValid_ = false;
  bool refAligned_ = false;
  uint8_t updatesSinceSave_ = 0;
  bool dirty_ = false;
};

}  // namespace micromouse
