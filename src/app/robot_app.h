// Top-level application: the robot's explicit state machine.
#pragma once

#include <stdint.h>

#include "app/online_learning.h"
#include "config/config.h"
#include "control/calibration.h"
#include "control/motion_controller.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/ultrasonic_sensor.h"
#include "hal/eeprom.h"
#include "navigation/explorer.h"
#include "navigation/pose.h"
#include "navigation/run_log.h"
#include "util/status.h"

namespace micromouse {

/// States of the robot.
///
///   INIT -> SELECT_MODE -> START_RUN -> SENSE -> PLAN -> MOVE -> SENSE ...
///        -> GOAL_REACHED -> IDLE
///   any failure (no path, stall, move timeout, sensor failure, exploration
///   budget exceeded, bad config) -> FAULT
///
/// INIT brings up the hardware and loads the calibration from EEPROM.
/// SELECT_MODE prints a menu and waits kModeSelectTimeoutMs for a key:
/// 'e' encoder check, 'c' motor self-calibration, 'r' reset calibration,
/// 'p' print calibration, 'l' print the stored run log (README "Run log",
/// each returns to the menu, which then waits without timeout), 'g' or the
/// timeout -> START_RUN (maze reset).
/// SENSE checks for the goal first (the goal cell is not sensed), then maps
/// the three walls around the robot while it is stopped. PLAN runs the
/// flood fill and picks the next direction; MOVE turns and drives one cell.
/// The decisions themselves live in the pure Explorer
/// (navigation/explorer.h); this class sequences them with the hardware.
/// FAULT keeps the motors braked and reports the error code on Serial.
enum class AppState : uint8_t {
  kInit,
  kSelectMode,
  kStartRun,
  kSense,
  kPlan,
  kMove,
  kGoalReached,
  kIdle,
  kFault,
};

/// What the robot is trying to do; selects the goal and what happens on
/// arrival. Extension point: a kReturnToStart mission
/// (Explorer::setGoal(start), same SENSE/PLAN/MOVE loop) and a kSpeedRun
/// mission (PLAN on known walls only, faster MoveParams) can be added here
/// and dispatched from onGoalReached().
enum class Mission : uint8_t { kExploreToGoal };

class RobotApp {
 public:
  /// constexpr: the one global instance is constant-initialised (no
  /// start-up code); the hardware is set up in INIT.
  constexpr RobotApp()
      : leftMotor_(config::pins::kLeftMotorPwm, config::pins::kLeftMotorDir,
                   MountSide::kLeft),
        rightMotor_(config::pins::kRightMotorPwm,
                    config::pins::kRightMotorDir, MountSide::kRight),
        leftEncoder_(hal::EncoderChannel::kLeft),
        rightEncoder_(hal::EncoderChannel::kRight),
        leftSonar_(SonarId::kLeft),
        frontSonar_(SonarId::kFront),
        rightSonar_(SonarId::kRight),
        calibration_(),
        motion_(leftMotor_, rightMotor_, leftEncoder_, rightEncoder_,
                leftSonar_, rightSonar_, calibration_),
        learning_(calibration_),
        runLog_(hal::eepromRead, hal::eepromWrite, kRunLogAddress,
               kRunLogCapacity) {}

  /// Opens the serial log. Call once from setup().
  void begin();

  /// Runs one state of the state machine. Call repeatedly from loop().
  void step();

  AppState state() const { return state_; }

 private:
  void runInit();
  void runSelectMode();
  void runStartRun();
  void runSense();
  void runPlan();
  void runMove();
  void loadCalibration();
  void saveLearned();
  void confirmLearning();
  /// Aligns on the wall ahead (if any, see AlignResult); `finalFrontMm` =
  /// the final (re-measured) front reading when kCentred.
  AlignResult alignToFrontWall(float frontMm, float& finalFrontMm);
  void onGoalReached();
  void runFault();
  void enterFault(Status status);
  CenteringWalls centeringWallsFor(Direction heading) const;
  /// Appends one run-log record at the current pose (SENSE/PLAN, robot
  /// stopped): `move`/`noMove` as explained in navigation/run_log.h.
  void logRunStep(Direction move, bool noMove);

  // Drivers.
  Motor leftMotor_;
  Motor rightMotor_;
  Encoder leftEncoder_;
  Encoder rightEncoder_;
  UltrasonicSensor leftSonar_;
  UltrasonicSensor frontSonar_;
  UltrasonicSensor rightSonar_;

  // Run-time calibration (EEPROM record or defaults). Declared before the
  // users that keep a reference to it.
  Calibration calibration_;
  MotionController motion_;
  Explorer explorer_;
  OnlineLearning learning_;
  RunLog runLog_;
  // SENSE readings [mm, 0 = no echo] in sensor order left, front, right:
  // the "before" readings of the next straight move.
  // (tenths of a mm in 16 bits: RAM; 0 = no echo)
  uint16_t senseTenthMm_[kSonarCount] = {0, 0, 0};
  float senseMm(const uint8_t i) const {
    return static_cast<float>(senseTenthMm_[i]) * 0.1f;
  }

  Mission mission_ = Mission::kExploreToGoal;
  Direction plannedDirection_ = Direction::kUp;
  AppState state_ = AppState::kInit;
  Status fault_ = Status::kOk;
  uint32_t lastFaultReportMs_ = 0;
  calibration::Validity motorValidity_ = calibration::Validity::kEmpty;
  calibration::Validity learnedValidity_ = calibration::Validity::kEmpty;
  bool faultReported_ = false;
};

}  // namespace micromouse
