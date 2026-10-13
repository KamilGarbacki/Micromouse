// Host tests of the app-level learning glue (app/online_learning): fake move
// reports and sensor readings of a simulated robot driving corridors whose
// cells are open between two end walls, through onForward / onAligned /
// onTurn / confirm, exactly in the order RobotApp calls them.
#include <unity.h>

#include "app/online_learning.h"
#include "config/config.h"
#include "control/calibration.h"

using namespace micromouse;

namespace {

/// TRUE geometry (the firmware only knows the configured values). With the
/// default 220 mm the robot overdrives 4 mm per cell at first; runs that
/// end closer than the sensor minimum (kLearnFrontNearMinMm) are gated
/// out until the estimate improves, as on the real robot.
constexpr float kTruePitch = 216.0f;
constexpr float kTrueWall = 8.0f;     // configured: 0
constexpr float kTrueSensor = 60.0f;  // configured: 60
// delta = (w_c - w) / 2 + (s_c - s) = -4 mm.
constexpr float kTrueOffset =
    (config::kWallThicknessMm - kTrueWall) / 2.0f +
    (config::kFrontSensorToCenterMm - kTrueSensor);

/// A robot in a straight corridor of `length` cells (wall centre lines at
/// 0 and length * P along the axis), driven like RobotApp drives it.
struct World {
  Calibration cal = calibration::defaults();
  OnlineLearning learning{cal};
  int length = 1;
  float y = 0.0f;  // true centre along the axis [mm]
  int dir = 1;     // +1 towards length * P, -1 towards 0
  int pitchObservations = 0;
  float noise = 0.0f;

  /// True front reading [mm] (0 = nothing within the mapping range).
  float frontReading() const {
    const float face = dir > 0 ? length * kTruePitch - kTrueWall / 2.0f - y
                               : y - kTrueWall / 2.0f;
    const float reading = face - kTrueSensor;
    return reading < config::kMappingMaxCm * 10.0f ? reading : 0.0f;
  }

  static MoveReport report(const int32_t ticks) {
    return MoveReport{Status::kOk, true, ticks, ticks, ticks, 0, 0.0f};
  }

  /// One cell forward, the readings, the alignment and the SENSE confirm.
  void forward() {
    const int32_t ticks = calibration::ticksPerCell(cal);
    // The wheel scale is exact here (only pitch / offset are wrong).
    y += dir * ticks * config::kMmPerTick;
    ForwardObservation obs = {};
    obs.turnedBefore = true;  // no wheel-scale samples in this test
    obs.frontAfterMm = frontReading() + noise;
    const float rowsBefore = cal.learned.pitchSums.v[learning::PitchSums::kAA];
    learning.onForward(report(ticks), obs);

    // Alignment as RobotApp::alignToFrontWall does it (perfect moves).
    const float target =
        config::frontWallNominalMm(calibration::cellPitchMm(cal)) +
        calibration::wallOffsetMm(cal);
    const float reading = obs.frontAfterMm;
    const float error = reading - target;
    if (reading > 0.0f && error <= config::kAlignMaxErrorMm &&
        error >= -config::kAlignMaxErrorMm) {
      if (error > config::kAlignDeadbandMm ||
          error < -config::kAlignDeadbandMm) {
        y += dir * error;
        learning.onAdjusted(error);
      }
      learning.onAligned(AlignResult::kCentred, frontReading() + 0.5f * noise);
    } else {
      learning.onAligned(AlignResult::kNotAttempted, 0.0f);
    }
    const bool wallAhead =
        frontReading() > 0.0f &&
        frontReading() < config::kWallThresholdCm * 10.0f;
    learning.confirm(wallAhead, false, false);
    if (cal.learned.pitchSums.v[learning::PitchSums::kAA] != rowsBefore ||
        calibration::pitchJoint(cal)) {
      ++pitchObservations;
    }
  }

  void turn(const uint8_t quarters) {
    const int32_t n = quarters * calibration::ticksPer90(cal);
    learning.onTurn(report(n), 1, quarters);
  }

  /// Drives to the wall ahead (the corridor's cells are open between).
  void driveToWall() {
    const int cells = dir > 0
                          ? length - 1 - static_cast<int>(y / kTruePitch)
                          : static_cast<int>(y / kTruePitch);
    for (int i = 0; i < cells; ++i) forward();
  }

  /// 90 deg turn into a new corridor of `newLength` cells, in its middle
  /// cell (not a pitch reference: the axis changed).
  void enterCorridor(const int newLength, const float lateralMm) {
    turn(1);
    length = newLength;
    y = (newLength / 2 + 0.5f) * kTruePitch + lateralMm;
    dir = 1;
  }

  /// U-turn at the wall and back to the other end.
  void bounce() {
    turn(2);
    dir = -dir;
    driveToWall();
  }
};

}  // namespace

void setUp() {}
void tearDown() {}

void test_open_cells_keep_the_reference_multi_cell_observation() {
  // From the start cell (placed centred) along a 3-cell corridor whose
  // middle cell is open: the reference must survive the open cell, so the
  // first observation is a 2-cell run (n >= 2) and feeds the provisional
  // pitch.
  World w;
  w.learning.reset();
  w.length = 3;
  w.y = kTruePitch / 2.0f;
  w.driveToWall();
  TEST_ASSERT_EQUAL_INT(1, w.pitchObservations);
  TEST_ASSERT_EQUAL_UINT8(1, learning::pitchRows(w.cal.learned.flags));
  // a = n + 1/2 = 2.5 for the start-cell run of 2 cells.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.5f * 2.5f,
                           w.cal.learned.pitchSums.v[learning::PitchSums::kAA]);
  TEST_ASSERT_EQUAL_UINT16(1, w.cal.learned.pitchSamples);  // provisional
}

void test_provisional_then_joint_convergence_in_a_maze() {
  World w;
  w.learning.reset();
  w.length = 3;
  w.y = kTruePitch / 2.0f;
  w.driveToWall();
  // Corridors of different lengths, bouncing between their end walls;
  // U-turns keep the reference (aligned runs), 90 deg turns drop it.
  const int lengths[] = {4, 3, 5, 2, 4};
  const float noise[] = {0.8f, -0.6f, 0.3f, -1.0f, 0.5f};
  bool provisionalSeen = false;
  int run = 0;
  for (int pass = 0; pass < 6; ++pass) {
    for (const int length : lengths) {
      w.enterCorridor(length, 3.0f);
      w.driveToWall();  // align at the end: the new reference
      for (int b = 0; b < 2; ++b) {
        w.noise = noise[run++ % 5];
        w.bounce();
        // The provisional path receives the n >= 2 runs until the joint
        // solve is in force.
        if (!calibration::pitchJoint(w.cal) &&
            w.cal.learned.pitchSamples >= 1) {
          provisionalSeen = true;
        }
      }
    }
  }
  TEST_ASSERT_TRUE(provisionalSeen);
  TEST_ASSERT_TRUE(w.pitchObservations > 40);
  TEST_ASSERT_TRUE(calibration::pitchJoint(w.cal));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, kTruePitch, calibration::cellPitchMm(w.cal));
  TEST_ASSERT_FLOAT_WITHIN(1.5f, kTrueOffset,
                           calibration::wallOffsetMm(w.cal));
  // 216 / 0.21271 = 1015 ticks per cell (1034 with the default 220 mm).
  TEST_ASSERT_INT32_WITHIN(5, 1015, calibration::ticksPerCell(w.cal));
  TEST_ASSERT_TRUE(w.learning.dirty());
}

void test_provisional_pitch_when_runs_are_all_alike() {
  // Only one corridor length and aligned runs only (all observations have
  // the same a / b): the joint solve stays unconditioned and the
  // provisional pitch (delta = 0 assumed, n >= 2) is applied - biased by
  // b delta / a = 2 * (-4) / 3 = -2.7 mm, but far better than the 220 mm
  // default.
  World w;
  w.learning.reset();
  w.enterCorridor(3, 0.0f);  // 90 deg: the start reference is dropped
  w.driveToWall();
  for (int i = 0; i < 12; ++i) w.bounce();
  TEST_ASSERT_FALSE(calibration::pitchJoint(w.cal));
  TEST_ASSERT_TRUE(w.cal.learned.pitchSamples >= config::kPitchLearnMinSamples);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, kTruePitch + 2.0f * kTrueOffset / 3.0f,
                           calibration::cellPitchMm(w.cal));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, calibration::wallOffsetMm(w.cal));
}

namespace {

/// One plain straight cell move (no turn before it) from an aligned
/// reference whose final reading was `refFrontMm`, ending with `afterMm`
/// in front (and `afterMm` + the cell before it). Returns the record.
LearnedRecord plainMoveTowardsWall(const float refFrontMm,
                                   const float afterMm) {
  Calibration cal = calibration::defaults();
  OnlineLearning learning(cal);
  learning.reset();
  learning.onAligned(AlignResult::kCentred, refFrontMm);
  const int32_t ticks = calibration::ticksPerCell(cal);
  ForwardObservation obs = {};
  obs.turnedBefore = false;
  obs.frontAfterMm = afterMm;
  obs.frontBeforeMm = afterMm + ticks * calibration::mmPerTick(cal);
  learning.onForward(MoveReport{Status::kOk, true, ticks, ticks, ticks, 0,
                                0.0f},
                     obs);
  learning.confirm(true, false, false);
  return cal.learned;
}

}  // namespace

void test_scale_ruler_gate_vs_near_wall_gate() {
  // ~25 mm after the move: below the 30 mm ruler minimum - no wheel-scale
  // sample - but above the 20 mm sensor minimum: the pitch observation is
  // made (aligned reference r_f = 50 mm, in the hard bounds).
  LearnedRecord near = plainMoveTowardsWall(50.0f, 25.0f);
  TEST_ASSERT_EQUAL_UINT16(0, near.scaleSamples);
  TEST_ASSERT_EQUAL_UINT8(1, learning::pitchRows(near.flags));
  // ~100 mm: in ruler range - a wheel-scale sample (seed) - and still in
  // the alignment window of the centred reading (50 mm): a pitch
  // observation too (r_f = 20 mm keeps it in the hard bounds).
  LearnedRecord far = plainMoveTowardsWall(20.0f, 100.0f);
  TEST_ASSERT_EQUAL_UINT16(1, far.scaleSamples);
  TEST_ASSERT_EQUAL_UINT8(1, learning::pitchRows(far.flags));
}

void test_lost_alignment_and_90_deg_turn_drop_the_reference() {
  World w;
  w.learning.reset();
  w.length = 3;
  w.y = kTruePitch / 2.0f;
  w.learning.onAligned(AlignResult::kLost, 0.0f);  // e.g. echo lost
  w.driveToWall();
  TEST_ASSERT_EQUAL_INT(0, w.pitchObservations);
  // A new reference by the alignment at the wall, then a 90 deg turn:
  // dropped again, the next wall gives no observation.
  w.enterCorridor(3, 0.0f);
  w.driveToWall();
  TEST_ASSERT_EQUAL_INT(0, w.pitchObservations);
  // ... but a U-turn right after the alignment keeps it.
  w.bounce();
  TEST_ASSERT_EQUAL_INT(1, w.pitchObservations);
}

void test_u_turn_mid_corridor_drops_the_reference() {
  // A 180 deg turn after cells were driven since the reference gives
  // zero-information samples: the reference is dropped.
  World w;
  w.learning.reset();
  w.length = 5;
  w.y = kTruePitch / 2.0f;
  w.forward();
  w.forward();  // 2 cells from the start, corridor still open ahead
  w.turn(2);
  w.dir = -1;
  w.driveToWall();
  TEST_ASSERT_EQUAL_INT(0, w.pitchObservations);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_open_cells_keep_the_reference_multi_cell_observation);
  RUN_TEST(test_provisional_then_joint_convergence_in_a_maze);
  RUN_TEST(test_provisional_pitch_when_runs_are_all_alike);
  RUN_TEST(test_lost_alignment_and_90_deg_turn_drop_the_reference);
  RUN_TEST(test_u_turn_mid_corridor_drops_the_reference);
  RUN_TEST(test_scale_ruler_gate_vs_near_wall_gate);
  return UNITY_END();
}
