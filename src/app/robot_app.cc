#include "app/robot_app.h"

#include "app/calibration_tools.h"
#include "app/maze_printer.h"
#include "app/run_log_printer.h"
#include "app/status_name.h"
#include "app/wall_sensing.h"
#include "config/config.h"
#include "control/learning.h"
#include "hal/eeprom.h"
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
constexpr uint8_t kLeftIndex = 0;
constexpr uint8_t kFrontIndex = 1;
constexpr uint8_t kRightIndex = 2;
constexpr uint8_t kLeftMask = 1u << kLeftIndex;
constexpr uint8_t kFrontMask = 1u << kFrontIndex;
constexpr uint8_t kRightMask = 1u << kRightIndex;
constexpr uint32_t kDrainMs = 50;

}  // namespace

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
    case AppState::kSelectMode:
      runSelectMode();
      break;
    case AppState::kStartRun:
      runStartRun();
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
  loadCalibration();
  state_ = cfg::kModeMenuEnabled ? AppState::kSelectMode : AppState::kStartRun;
}

void RobotApp::loadCalibration() {
  calibration_ = calibration::defaults();
  MotorRecord motor;
  hal::eepromRead(cfg::kMotorRecordAddress, &motor, sizeof(motor));
  motorValidity_ = calibration::validate(motor);
  if (motorValidity_ == calibration::Validity::kValid) {
    calibration_.motor = motor;
    calibration_.motorsCalibrated = true;
  }
  LearnedRecord learned;
  hal::eepromRead(cfg::kLearnedRecordAddress, &learned, sizeof(learned));
  learnedValidity_ = calibration::validate(learned);
  if (learnedValidity_ == calibration::Validity::kValid) {
    calibration::restartIncompleteSeeds(learned);
    calibration_.learned = learned;
  }
  if constexpr (cfg::logEnabled(LogLevel::kError)) {
    if (motorValidity_ == calibration::Validity::kGeometryChanged) {
      // The record is keyed on the CONFIGURED resolution. The model itself
      // (measured in real counted ticks) may well still be right, but it is
      // not trusted across a config change: rerun 'c' to confirm it.
      log::fmtln(F("cal: WARNING motor record is for another configured "
                   "resolution (%u ticks/rev, now %u): rerun 'c' to confirm"),
                 static_cast<unsigned int>(motor.ticksPerWheelRev),
                 static_cast<unsigned int>(cfg::kTicksPerWheelRev + 0.5f));
    } else if (!calibration_.motorsCalibrated) {
      log::fmtln(F("cal: WARNING motor model %s - defaults; run 'c' "
                   "(env uno_setup)"),
                 validityName(motorValidity_));
    }
    if (learnedValidity_ != calibration::Validity::kValid) {
      // GEOMETRY_CHANGED: other default ticks per cell / 90 (geometry or
      // encoder resolution): discarded, relearnt while driving.
      log::fmtln(F("cal: learned values %s - defaults"),
                 validityName(learnedValidity_));
    }
  }
  if constexpr (cfg::logEnabled(LogLevel::kInfo)) {
    log::fmtln(F("cal: ticks/cell=%l ticks/90=%l"),
               static_cast<long>(calibration::ticksPerCell(calibration_)),
               static_cast<long>(calibration::ticksPer90(calibration_)));
  }
}

void RobotApp::saveLearned() {
  calibration::seal(calibration_.learned);
  hal::eepromWrite(cfg::kLearnedRecordAddress, &calibration_.learned,
                   sizeof(calibration_.learned));
  learnedValidity_ = calibration::Validity::kValid;
  learning_.markSaved();
  if constexpr (cfg::logEnabled(LogLevel::kInfo)) {
    log::fmtln(F("cal: saved"));
  }
}

void RobotApp::runSelectMode() {
  if constexpr (!cfg::kModeMenuEnabled) {
    state_ = AppState::kStartRun;
    return;
  } else {
    const uint32_t now = hal::nowMs();
    // Menu state lives here, so it only exists (in RAM) in the menu build.
    static uint32_t menuStartMs_ = 0;
    static bool menuShown_ = false;
    static bool menuTimeout_ = true;  // the boot menu times out into RUN
    if (!menuShown_) {
      log::fmt(F("MENU: e=encoder check c=motor self-cal (LIFTED) "
                 "r=reset cal p=print cal l=print run log g=run"));
      if (menuTimeout_) {
        log::fmt(F(" (RUN in %u s)"),
                 static_cast<unsigned int>(cfg::kModeSelectTimeoutMs / 1000));
      }
      log::println();
      menuShown_ = true;
      menuStartMs_ = now;
    }
    const int16_t key = log::readChar();
    if (key < 0) {
      if (menuTimeout_ && now - menuStartMs_ >= cfg::kModeSelectTimeoutMs) {
        state_ = AppState::kStartRun;
      }
      return;
    }
    // Only a valid key cancels the timeout: line noise or a stray byte on
    // an unconnected Serial line must not keep the robot in the menu.
    if (key != 'e' && key != 'c' && key != 'r' && key != 'p' && key != 'g' &&
        key != 'l') {
      return;
    }
    log::drainInput(kDrainMs);
    // After a tool the menu waits for the next key without timeout (the
    // robot may still be lifted).
    menuTimeout_ = false;
    menuShown_ = false;
    switch (key) {
      case 'e':
        runEncoderCheck(leftEncoder_, rightEncoder_);
        break;
      case 'c':
        if (runMotorSelfCalibration(leftMotor_, rightMotor_, leftEncoder_,
                                    rightEncoder_, calibration_.motor)) {
          calibration_.motorsCalibrated = true;
          hal::eepromWrite(cfg::kMotorRecordAddress, &calibration_.motor,
                           sizeof(calibration_.motor));
          motorValidity_ = calibration::Validity::kValid;
        }
        break;
      case 'r': {
        // Invalidate both stored records (magic) and go back to defaults.
        const uint16_t erased = 0xFFFF;
        hal::eepromWrite(cfg::kMotorRecordAddress, &erased, sizeof(erased));
        hal::eepromWrite(cfg::kLearnedRecordAddress, &erased, sizeof(erased));
        motorValidity_ = calibration::Validity::kEmpty;
        learnedValidity_ = calibration::Validity::kEmpty;
        calibration_ = calibration::defaults();
        log::fmtln(F("cal: reset - defaults"));
        break;
      }
      case 'p':
        printCalibration(calibration_, motorValidity_, learnedValidity_);
        break;
      case 'l':
        printStoredRunLog(hal::eepromRead, kRunLogAddress, kRunLogCapacity);
        break;
      default:  // 'g'
        state_ = AppState::kStartRun;
        break;
    }
  }
}

void RobotApp::runStartRun() {
  // Boot behaviour (README "Run log"): print whatever the LAST run left in
  // EEPROM before it is overwritten below, then give the user kPreRunDelayMs
  // to read it / lift the robot.
  printStoredRunLog(hal::eepromRead, kRunLogAddress, kRunLogCapacity);
  if constexpr (cfg::kPreRunDelayMs > 0) {
    hal::delayMs(cfg::kPreRunDelayMs);
  }

  learning_.reset();
  // START RUN: write the header "in progress, 0 steps" first, so even a
  // reset()/budget failure below is captured as a FAULT in the log.
  runLog_.startRun(cfg::kMazeRows, cfg::kMazeCols,
                  Pose{cfg::kStartCell, cfg::kStartHeading}, cfg::kGoalCell);
  if constexpr (cfg::logEnabled(LogLevel::kInfo)) {
    printRunLogStart(cfg::kMazeRows, cfg::kMazeCols,
                    Pose{cfg::kStartCell, cfg::kStartHeading},
                    cfg::kGoalCell);
  }

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
  state_ = AppState::kSense;
}

void RobotApp::confirmLearning() {
  if (!cfg::kOnlineLearningEnabled) return;
  // The map knows the walls around the current cell: commit the samples of
  // the last move whose walls it confirms.
  const Maze& maze = explorer_.maze();
  const Pose pose = explorer_.pose();
  learning_.confirm(
      maze.hasWall(pose.cell, pose.heading),
      maze.hasWall(pose.cell, rotate(pose.heading, Relative::kLeft)),
      maze.hasWall(pose.cell, rotate(pose.heading, Relative::kRight)));
  if (learning_.saveDue()) saveLearned();
}

void RobotApp::runSense() {
  if (explorer_.atGoal()) {
    // The goal cell is not sensed: commit what the map already confirms.
    confirmLearning();
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
    senseTenthMm_[i] =
        static_cast<uint16_t>(echoToMm(readings[i].medianUs) * 10.0f + 0.5f);
  }
  const Status status = explorer_.observe(observations, kSonarCount);

  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    log::print(F("sense L/F/R cm="));
    for (uint8_t i = 0; i < kSonarCount; ++i) {
      if (readings[i].medianUs == kFarUs) {
        log::print(F(" far"));
      } else {
        log::fmt(F(" %1"), static_cast<double>(senseMm(i) / 10.0f));
      }
      log::print(readings[i].wall ? F("(W)") : F("(-)"));
    }
    log::println();
  }
  if (!isOk(status)) {
    // Halted before a move could even be planned: log the cell with
    // whatever walls are known (the implausible observation above was not
    // applied to the map).
    logRunStep(explorer_.pose().heading, /*noMove=*/true);
    enterFault(status);
    return;
  }

  confirmLearning();
  state_ = AppState::kPlan;
}

void RobotApp::runPlan() {
  const Status status = explorer_.plan(plannedDirection_);
  if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
    printMaze(explorer_.maze(), explorer_.flood(), explorer_.pose().cell);
  }
  if (!isOk(status)) {
    logRunStep(explorer_.pose().heading, /*noMove=*/true);
    enterFault(status);
    return;
  }
  // The regular run-log record: pose before the move, its walls (just
  // sensed), and the chosen direction. Appended here (SENSE/PLAN, robot
  // stopped), never from the 100 Hz move loop.
  logRunStep(plannedDirection_, /*noMove=*/false);
  state_ = AppState::kMove;
}

void RobotApp::logRunStep(const Direction move, const bool noMove) {
  const Pose pose = explorer_.pose();
  const Maze& maze = explorer_.maze();
  RunStep step;
  step.cell = pose.cell;
  step.heading = pose.heading;
  step.wallLeft = maze.hasWall(pose.cell, rotate(pose.heading, Relative::kLeft));
  step.wallFront = maze.hasWall(pose.cell, pose.heading);
  step.wallRight =
      maze.hasWall(pose.cell, rotate(pose.heading, Relative::kRight));
  step.move = move;
  step.noMove = noMove;
  runLog_.appendStep(step);
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

  const UltrasonicSensor* const sensors[kSonarCount] = {
      &leftSonar_, &frontSonar_, &rightSonar_};
  const Relative relative =
      relativeTurn(explorer_.pose().heading, plannedDirection_);
  MoveReport report = {};
  Status status = Status::kOk;
  const bool turned = relative != Relative::kForward;
  if (turned) {
    const TurnDirection direction = relative == Relative::kLeft
                                        ? TurnDirection::kCounterClockwise
                                        : TurnDirection::kClockwise;
    const uint8_t quarters = quarterTurns(relative);
    status = motion_.turn(direction, quarters, report);
    if (isOk(status) && cfg::kOnlineLearningEnabled) {
      learning_.onTurn(report, turnSign(direction), quarters);
    }
  }

  // Learning: side walls known beside the start cell in the new heading,
  // and their distances at the start of the straight move (after a turn
  // the SENSE readings face the wrong way, so range again).
  ForwardObservation obs = {};
  obs.turnedBefore = turned;
  obs.frontBeforeMm = senseMm(kFrontIndex);
  if (cfg::kOnlineLearningEnabled && isOk(status)) {
    const Maze& maze = explorer_.maze();
    const Cell cell = explorer_.pose().cell;
    if (maze.hasWall(cell, rotate(plannedDirection_, Relative::kLeft))) {
      obs.sideMask |= kSideLeftBit;
    }
    if (maze.hasWall(cell, rotate(plannedDirection_, Relative::kRight))) {
      obs.sideMask |= kSideRightBit;
    }
    float start[kSonarCount] = {senseMm(0), senseMm(1), senseMm(2)};
    if (turned && obs.sideMask != 0) {
      uint8_t mask = 0;
      if (obs.sideMask & kSideLeftBit) mask |= kLeftMask;
      if (obs.sideMask & kSideRightBit) mask |= kRightMask;
      rangeSonars(sensors, mask, start);
    }
    obs.sideStartMm[0] = start[kLeftIndex];
    obs.sideStartMm[1] = start[kRightIndex];
  }

  if (isOk(status)) {
    status = motion_.driveForward(centeringWallsFor(plannedDirection_),
                                  report);
  }
  if (!isOk(status)) {
    // The position is unknown now: do not update the pose, stop everything.
    learning_.onFailedMove();
    enterFault(status);
    return;
  }

  hal::delayMs(cfg::kPostMoveDelayMs);

  // Fresh readings after the move, BEFORE any alignment.
  uint8_t mask = 0;
  if (cfg::kFrontAlignEnabled || cfg::kOnlineLearningEnabled) {
    mask |= kFrontMask;
  }
  if (cfg::kOnlineLearningEnabled) {
    if (obs.sideMask & kSideLeftBit) mask |= kLeftMask;
    if (obs.sideMask & kSideRightBit) mask |= kRightMask;
  }
  float after[kSonarCount] = {0.0f, 0.0f, 0.0f};
  if (mask != 0) rangeSonars(sensors, mask, after);
  if (cfg::kOnlineLearningEnabled) {
    obs.frontAfterMm = after[kFrontIndex];
    obs.sideEndMm[0] = after[kLeftIndex];
    obs.sideEndMm[1] = after[kRightIndex];
    learning_.onForward(report, obs);
  }
  if (cfg::kFrontAlignEnabled) {
    float finalFrontMm = 0.0f;
    const AlignResult aligned =
        alignToFrontWall(after[kFrontIndex], finalFrontMm);
    if (state_ == AppState::kFault) return;
    if (cfg::kOnlineLearningEnabled) {
      learning_.onAligned(aligned, finalFrontMm);
    }
  }

  status = explorer_.advance(plannedDirection_);
  if (!isOk(status)) {
    enterFault(status);
    return;
  }

  // (the per-move "at (r,c) facing H" info line was dropped to make room
  // for the run log, see README "Flash budget": the same position/heading
  // is now in the run log, printed at the next boot.)
  state_ = AppState::kSense;
}

AlignResult RobotApp::alignToFrontWall(float frontMm, float& finalFrontMm) {
  // Truly centred reading: the configured nominal plus the learned front
  // offset error delta (control/learning.h).
  const learning::AlignParams align = {
      cfg::frontWallNominalMm(calibration::cellPitchMm(calibration_)) +
          calibration::wallOffsetMm(calibration_),
      cfg::kAlignDeadbandMm, cfg::kAlignMaxCorrectionMm,
      cfg::kAlignMaxErrorMm};
  finalFrontMm = 0.0f;
  // Only a reading inside the alignment window is "the wall right ahead".
  const float error = frontMm - align.nominalMm;
  if (!(frontMm > 0.0f) || error > align.maxErrorMm ||
      error < -align.maxErrorMm) {
    if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
      if (frontMm > 0.0f && frontMm < cfg::kWallThresholdCm * 10.0f) {
        log::fmtln(F("align skip %1"),
                   static_cast<double>(frontMm));
      }
    }
    return AlignResult::kNotAttempted;
  }
  const UltrasonicSensor* const sensors[kSonarCount] = {
      &leftSonar_, &frontSonar_, &rightSonar_};
  for (uint8_t round = 0; round < cfg::kAlignMaxIterations; ++round) {
    const int32_t ticks = learning::alignmentTicks(
        frontMm, calibration::mmPerTick(calibration_), align);
    if (ticks == 0) break;
    if constexpr (cfg::logEnabled(LogLevel::kDebug)) {
      log::fmtln(F("align %1 adj=%l"),
                 static_cast<double>(frontMm), static_cast<long>(ticks));
    }
    MoveReport report = {};
    const Status status = motion_.adjustStraight(ticks, report);
    if (!isOk(status)) {
      learning_.onFailedMove();
      enterFault(status);
      return AlignResult::kLost;
    }
    if (cfg::kOnlineLearningEnabled) {
      learning_.onAdjusted(static_cast<float>(ticks) *
                           calibration::mmPerTick(calibration_));
    }
    // Re-measure after every adjustment (also the last one): the coast is
    // not exact, and the final reading is the reference of the pitch
    // learning (residual included).
    float mm[kSonarCount] = {0.0f, 0.0f, 0.0f};
    rangeSonars(sensors, kFrontMask, mm);
    frontMm = mm[kFrontIndex];
    if (!(frontMm > 0.0f)) return AlignResult::kLost;
  }
  finalFrontMm = frontMm;
  return AlignResult::kCentred;
}

void RobotApp::onGoalReached() {
  motion_.stop();
  // END RUN: goal reached (README "Run log").
  runLog_.finishGoal();
  // Wear policy: the learned values are written here (and every
  // kLearnSaveEveryUpdates accepted samples), never per move.
  if (learning_.dirty()) saveLearned();
  if constexpr (cfg::logEnabled(LogLevel::kInfo)) {
    printRunLogEnd(RunOutcome::kGoalReached, Status::kOk, explorer_.goal(),
                  explorer_.pose().heading, explorer_.cellsMoved());
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
  faultReported_ = false;
  // END RUN: a fault occurred (README "Run log"). explorer_.pose() is the
  // last known-good cell (a failed move does not update it, see runMove()).
  const Pose pose = explorer_.pose();
  runLog_.finishFault(status, pose.cell, pose.heading);
  if constexpr (cfg::logEnabled(LogLevel::kInfo)) {
    printRunLogEnd(RunOutcome::kFault, status, pose.cell, pose.heading,
                  explorer_.cellsMoved());
  }
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
    // Repeats the END RUN line (so a serial monitor opened late still sees
    // it): same wording as the live marker in enterFault(), no extra flash
    // strings (flash is tight, see README "Flash budget").
    const Pose pose = explorer_.pose();
    printRunLogEnd(RunOutcome::kFault, fault_, pose.cell, pose.heading,
                  explorer_.cellsMoved());
  }
}

}  // namespace micromouse
