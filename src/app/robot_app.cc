#include "app/robot_app.h"

#include "app/maze_printer.h"
#include "app/wall_sensing.h"
#include "config/config.h"
#include "hal/log.h"
#include "hal/time.h"

namespace micromouse {

namespace {

namespace cfg = config;
namespace log = hal::log;
using config::LogLevel;

/// Sensor order used for mapping: left, front, right.
constexpr Relative kSonarSides[kSonarCount] = {
    Relative::kLeft, Relative::kForward, Relative::kRight};

const __FlashStringHelper* statusName(const Status status) {
  switch (status) {
    case Status::kOk:
      return F("OK");
    case Status::kInvalidArgument:
      return F("INVALID_ARGUMENT");
    case Status::kOutOfBounds:
      return F("OUT_OF_BOUNDS");
    case Status::kNoPath:
      return F("NO_PATH");
    case Status::kStall:
      return F("STALL");
    case Status::kMoveTimeout:
      return F("MOVE_TIMEOUT");
    case Status::kNotInitialized:
      return F("NOT_INITIALIZED");
    case Status::kQueueOverflow:
      return F("QUEUE_OVERFLOW");
    case Status::kSensorFailure:
      return F("SENSOR_FAILURE");
    case Status::kExplorationLimit:
      return F("EXPLORATION_LIMIT");
  }
  return F("UNKNOWN");
}

const __FlashStringHelper* stateName(const AppState state) {
  switch (state) {
    case AppState::kInit:
      return F("INIT");
    case AppState::kSense:
      return F("SENSE");
    case AppState::kPlan:
      return F("PLAN");
    case AppState::kMove:
      return F("MOVE");
    case AppState::kGoalReached:
      return F("GOAL_REACHED");
    case AppState::kIdle:
      return F("IDLE");
    case AppState::kFault:
      return F("FAULT");
  }
  return F("UNKNOWN");
}

char directionLetter(const Direction dir) {
  switch (dir) {
    case Direction::kUp:
      return 'U';
    case Direction::kRight:
      return 'R';
    case Direction::kDown:
      return 'D';
    case Direction::kLeft:
      return 'L';
  }
  return '?';
}

void printCell(const Cell cell) {
  log::print('(');
  log::print(static_cast<unsigned int>(cell.row));
  log::print(',');
  log::print(static_cast<unsigned int>(cell.col));
  log::print(')');
}

void printPose(const Pose& pose) {
  printCell(pose.cell);
  log::print(F(" facing "));
  log::print(directionLetter(pose.heading));
}

}  // namespace

RobotApp::RobotApp()
    : leftMotor_(cfg::pins::kLeftMotorPwm, cfg::pins::kLeftMotorDir,
                 MountSide::kLeft),
      rightMotor_(cfg::pins::kRightMotorPwm, cfg::pins::kRightMotorDir,
                  MountSide::kRight),
      leftEncoder_(hal::EncoderChannel::kLeft),
      rightEncoder_(hal::EncoderChannel::kRight),
      leftSonar_(SonarId::kLeft),
      frontSonar_(SonarId::kFront),
      rightSonar_(SonarId::kRight),
      motion_(leftMotor_, rightMotor_, leftEncoder_, rightEncoder_,
              leftSonar_, rightSonar_) {}

void RobotApp::begin() {
  if constexpr (cfg::logEnabled(LogLevel::kError)) {
    log::begin(cfg::kSerialBaud);
    log::println();
  }
  state_ = AppState::kInit;
}

void RobotApp::step() {
  switch (state_) {
    case AppState::kInit:
      runInit();
      break;
    case AppState::kSense:
      runSense();
      break;
    case AppState::kPlan:
      runPlan();
      break;
    case AppState::kMove:
      runMove();
      break;
    case AppState::kGoalReached:
      onGoalReached();
      break;
    case AppState::kIdle:
      break;
    case AppState::kFault:
      runFault();
      break;
  }
}

void RobotApp::runInit() {
  motion_.begin();  // motors braked first
  leftSonar_.begin();
  frontSonar_.begin();
  rightSonar_.begin();

  const ExplorerParams params = {cfg::kWallFlipObservations,
                                 cfg::kSensorFailurePasses,
                                 cfg::kExplorationBudgetFactor};
  const Status status =
      explorer_.reset(cfg::kMazeRows, cfg::kMazeCols,
                      Pose{cfg::kStartCell, cfg::kStartHeading},
                      cfg::kGoalCell, params);
  if (!isOk(status)) {
    enterFault(status);
    return;
  }
  if constexpr (cfg::logEnabled(LogLevel::kInfo)) {
    log::print(F("Micromouse: maze "));
    log::print(static_cast<unsigned int>(cfg::kMazeRows));
    log::print('x');
    log::print(static_cast<unsigned int>(cfg::kMazeCols));
    log::print(F(" start "));
    printPose(explorer_.pose());
    log::print(F(" goal "));
    printCell(explorer_.goal());
    log::println();
  }
  state_ = AppState::kSense;
}

void RobotApp::runSense() {
  if (explorer_.atGoal()) {
    state_ = AppState::kGoalReached;
    return;
  }

  const UltrasonicSensor* const sensors[kSonarCount] = {
      &leftSonar_, &frontSonar_, &rightSonar_};
  WallReading readings[kSonarCount];
  senseWalls(sensors, kSonarSides, readings);

  SideObservation observations[kSonarCount];
  for (uint8_t i = 0; i < kSonarCount; ++i) {
    observations[i] = SideObservation{readings[i].side, readings[i].wall,
                                      readings[i].anyEcho};
  }
  const Status status = explorer_.observe(observations, kSonarCount);

  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    log::print(F("sense L/F/R cm="));
    for (const WallReading& reading : readings) {
      if (reading.medianCm == kFarCm) {
        log::print(F(" far"));
      } else {
        log::print(' ');
        log::print(static_cast<unsigned int>(reading.medianCm));
      }
      log::print(reading.wall ? F("(W)") : F("(-)"));
    }
    log::println();
  }
  if (!isOk(status)) {
    enterFault(status);
    return;
  }
  state_ = AppState::kPlan;
}

void RobotApp::runPlan() {
  const Status status = explorer_.plan(plannedDirection_);
  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    printMaze(explorer_.maze(), explorer_.flood(), explorer_.pose().cell);
  }
  if (!isOk(status)) {
    enterFault(status);
    return;
  }
  state_ = AppState::kMove;
}

CenteringWalls RobotApp::centeringWallsFor(const Direction heading) const {
  const Direction left = rotate(heading, Relative::kLeft);
  const Direction right = rotate(heading, Relative::kRight);
  const Maze& maze = explorer_.maze();
  const Cell cell = explorer_.pose().cell;
  CenteringWalls walls = {{false, false}, {false, false}};
  walls.current =
      SideWalls{maze.hasWall(cell, left), maze.hasWall(cell, right)};
  Cell next{};
  if (maze.neighbour(cell, heading, next)) {
    walls.next =
        SideWalls{maze.hasWall(next, left), maze.hasWall(next, right)};
  }
  return walls;
}

void RobotApp::runMove() {
  Cell next{};
  if (!explorer_.maze().neighbour(explorer_.pose().cell, plannedDirection_,
                                  next)) {
    enterFault(Status::kOutOfBounds);  // planner bug: never drive off-grid
    return;
  }

  Status status = Status::kOk;
  switch (relativeTurn(explorer_.pose().heading, plannedDirection_)) {
    case Relative::kForward:
      break;
    case Relative::kRight:
      status = motion_.turn(TurnDirection::kClockwise, 1);
      break;
    case Relative::kBack:
      status = motion_.turn(TurnDirection::kClockwise, 2);
      break;
    case Relative::kLeft:
      status = motion_.turn(TurnDirection::kCounterClockwise, 1);
      break;
  }
  if (isOk(status)) {
    status = motion_.driveForward(centeringWallsFor(plannedDirection_));
  }
  if (!isOk(status)) {
    // The position is unknown now: do not update the pose, stop everything.
    enterFault(status);
    return;
  }

  hal::delayMs(cfg::kPostMoveDelayMs);
  status = explorer_.advance(plannedDirection_);
  if (!isOk(status)) {
    enterFault(status);
    return;
  }

  if constexpr (cfg::logEnabled(LogLevel::kInfo)) {
    log::print(F("at "));
    printPose(explorer_.pose());
    log::println();
  }
  state_ = AppState::kSense;
}

void RobotApp::onGoalReached() {
  motion_.stop();
  if constexpr (cfg::logEnabled(LogLevel::kInfo)) {
    log::print(F("Solved: goal "));
    printCell(explorer_.goal());
    log::print(F(" reached after "));
    log::print(static_cast<unsigned int>(explorer_.cellsMoved()));
    log::print(F(" cells"));
    log::println();
    if constexpr (cfg::kPrintMazeOnGoal) {
      printMaze(explorer_.maze(), explorer_.flood(), explorer_.pose().cell);
    }
  }

  // Extension point: dispatch the next mission here (e.g.
  // explorer_.setGoal(start) and go to kSense for a return run).
  switch (mission_) {
    case Mission::kExploreToGoal:
      state_ = AppState::kIdle;
      break;
  }
}

void RobotApp::enterFault(const Status status) {
  motion_.stop();
  fault_ = status;
  faultState_ = state_;
  faultReported_ = false;
  state_ = AppState::kFault;
}

void RobotApp::runFault() {
  motion_.stop();  // keep the motors braked, whatever happened
  if constexpr (cfg::logEnabled(LogLevel::kError)) {
    const uint32_t now = hal::nowMs();
    if (faultReported_ &&
        now - lastFaultReportMs_ < cfg::kFaultReportIntervalMs) {
      return;
    }
    faultReported_ = true;
    lastFaultReportMs_ = now;
    log::print(F("FAULT code="));
    log::print(static_cast<unsigned int>(fault_));
    log::print(F(" ("));
    log::print(statusName(fault_));
    log::print(F(") in "));
    log::print(stateName(faultState_));
    log::print(F(" at "));
    printPose(explorer_.pose());
    log::print(F(" moves="));
    log::print(static_cast<unsigned int>(explorer_.cellsMoved()));
    log::print(F(" - halted"));
    log::println();
  }
}

}  // namespace micromouse
