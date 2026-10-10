// Top-level application: the robot's explicit state machine.
#pragma once

#include <stdint.h>

#include "control/motion_controller.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/ultrasonic_sensor.h"
#include "navigation/explorer.h"
#include "navigation/pose.h"
#include "util/status.h"

namespace micromouse {

/// States of the robot.
///
///   INIT -> SENSE -> PLAN -> MOVE -> SENSE -> ... -> GOAL_REACHED -> IDLE
///   any failure (no path, stall, move timeout, sensor failure, exploration
///   budget exceeded, bad config) -> FAULT
///
/// SENSE checks for the goal first (the goal cell is not sensed), then maps
/// the three walls around the robot while it is stopped. PLAN runs the
/// flood fill and picks the next direction; MOVE turns and drives one cell.
/// The decisions themselves live in the pure Explorer
/// (navigation/explorer.h); this class sequences them with the hardware.
/// FAULT keeps the motors braked and reports the error code on Serial.
enum class AppState : uint8_t {
  kInit,
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
  RobotApp();

  /// Opens the serial log. Call once from setup().
  void begin();

  /// Runs one state of the state machine. Call repeatedly from loop().
  void step();

  AppState state() const { return state_; }

 private:
  void runInit();
  void runSense();
  void runPlan();
  void runMove();
  void onGoalReached();
  void runFault();
  void enterFault(Status status);
  CenteringWalls centeringWallsFor(Direction heading) const;

  // Drivers.
  Motor leftMotor_;
  Motor rightMotor_;
  Encoder leftEncoder_;
  Encoder rightEncoder_;
  UltrasonicSensor leftSonar_;
  UltrasonicSensor frontSonar_;
  UltrasonicSensor rightSonar_;

  MotionController motion_;
  Explorer explorer_;

  Mission mission_ = Mission::kExploreToGoal;
  Direction plannedDirection_ = Direction::kUp;
  AppState state_ = AppState::kInit;
  AppState faultState_ = AppState::kInit;  // state in which FAULT occurred
  Status fault_ = Status::kOk;
  uint32_t lastFaultReportMs_ = 0;
  bool faultReported_ = false;
};

}  // namespace micromouse
