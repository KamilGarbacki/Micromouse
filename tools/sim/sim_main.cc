// Host simulator: runs the REAL navigation::Explorer (same SENSE -> PLAN ->
// MOVE sequencing as app::RobotApp, see src/app/robot_app.cc) against a
// maze read from a simple ASCII text file, and writes a self-contained HTML
// animation of the run. No motor physics: this is a navigation-level replay
// (cell-to-cell moves only), not a hardware simulation.
//
// Usage: sim <maze.txt> [--noise p] [--seed n] [--dead-sensor L|F|R]
//            [--out file.html]
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <sstream>
#include <string>

#include "ascii_maze.h"
#include "config/config.h"
#include "html_writer.h"
#include "navigation/explorer.h"
#include "navigation/maze_render.h"

namespace {

using namespace micromouse;

namespace cfg = micromouse::config;

// directionLetter(Direction) is shared from navigation/pose.h.

const char* statusName(const Status status) {
  switch (status) {
    case Status::kOk:
      return "OK";
    case Status::kInvalidArgument:
      return "INVALID_ARGUMENT";
    case Status::kOutOfBounds:
      return "OUT_OF_BOUNDS";
    case Status::kNoPath:
      return "NO_PATH";
    case Status::kStall:
      return "STALL";
    case Status::kMoveTimeout:
      return "MOVE_TIMEOUT";
    case Status::kNotInitialized:
      return "NOT_INITIALIZED";
    case Status::kQueueOverflow:
      return "QUEUE_OVERFLOW";
    case Status::kSensorFailure:
      return "SENSOR_FAILURE";
    case Status::kExplorationLimit:
      return "EXPLORATION_LIMIT";
    case Status::kEncoderFault:
      return "ENCODER_FAULT";
  }
  return "UNKNOWN";
}

void printUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0
      << " <maze.txt> [--noise p] [--seed n] [--dead-sensor L|F|R] "
         "[--out file.html]\n"
         "  maze.txt       ASCII maze (see tools/sim/mazes/*.txt), S/G mark\n"
         "                 the start/goal; if absent, the firmware config's\n"
         "                 start cell/heading and goal are used.\n"
         "  --noise p      probability (0..1) of flipping a wall reading per\n"
         "                 observation (exercises sticky walls / flicker)\n"
         "  --seed n       RNG seed for --noise (default 1)\n"
         "  --dead-sensor  L|F|R: that sensor never reports an echo\n"
         "                 (exercises SENSOR_FAILURE)\n"
         "  --out file     output HTML path (default tools/sim/out/<maze>.html)\n";
}

std::string baseName(const std::string& path) {
  const size_t slash = path.find_last_of("/\\");
  std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
  const size_t dot = name.find_last_of('.');
  if (dot != std::string::npos) name = name.substr(0, dot);
  return name;
}

std::vector<uint8_t> encodeWalls(const Maze& maze) {
  std::vector<uint8_t> out(static_cast<size_t>(maze.rows()) * maze.cols() * 4,
                           0);
  for (uint8_t r = 0; r < maze.rows(); ++r) {
    for (uint8_t c = 0; c < maze.cols(); ++c) {
      const Cell cell{r, c};
      for (uint8_t d = 0; d < kDirectionCount; ++d) {
        const Direction dir = kAllDirections[d];
        uint8_t code = 0;
        if (maze.isKnown(cell, dir)) {
          code = maze.hasWall(cell, dir) ? 2 : 1;
        }
        out[(static_cast<size_t>(r) * maze.cols() + c) * 4 + d] = code;
      }
    }
  }
  return out;
}

std::vector<uint8_t> encodeTrueWalls(const Maze& truth) {
  std::vector<uint8_t> out(
      static_cast<size_t>(truth.rows()) * truth.cols() * 4, 1);
  for (uint8_t r = 0; r < truth.rows(); ++r) {
    for (uint8_t c = 0; c < truth.cols(); ++c) {
      const Cell cell{r, c};
      for (uint8_t d = 0; d < kDirectionCount; ++d) {
        const Direction dir = kAllDirections[d];
        out[(static_cast<size_t>(r) * truth.cols() + c) * 4 + d] =
            truth.hasWall(cell, dir) ? 2 : 1;
      }
    }
  }
  return out;
}

std::vector<int16_t> encodeFlood(const Explorer& explorer) {
  const Maze& maze = explorer.maze();
  std::vector<int16_t> out(static_cast<size_t>(maze.rows()) * maze.cols());
  for (uint8_t r = 0; r < maze.rows(); ++r) {
    for (uint8_t c = 0; c < maze.cols(); ++c) {
      const Distance d = explorer.flood().distance(Cell{r, c});
      out[static_cast<size_t>(r) * maze.cols() + c] =
          d == FloodFill::kUnreached ? -1 : static_cast<int16_t>(d);
    }
  }
  return out;
}

sim::TraceStep makeStep(const char* state, const Explorer& explorer,
                        bool hasPlanned, Direction planned, bool hasFault,
                        Status fault) {
  sim::TraceStep step;
  step.state = state;
  step.cell = explorer.pose().cell;
  step.heading = explorer.pose().heading;
  step.moves = explorer.cellsMoved();
  step.hasPlanned = hasPlanned;
  step.planned = planned;
  step.wallCodes = encodeWalls(explorer.maze());
  step.flood = encodeFlood(explorer);
  step.hasFault = hasFault;
  if (hasFault) {
    step.faultCode = static_cast<int>(fault);
    step.faultName = statusName(fault);
  }
  return step;
}

void sinkToString(char c, void* context) {
  static_cast<std::string*>(context)->push_back(c);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  std::string mazePath;
  double noise = 0.0;
  uint32_t seed = 1;
  std::optional<Relative> deadSensor;
  std::string deadSensorLabel;
  std::string outPath;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "error: " << name << " needs a value\n";
        std::exit(1);
      }
      return argv[++i];
    };
    if (arg == "--noise") {
      noise = std::stod(next("--noise"));
    } else if (arg == "--seed") {
      seed = static_cast<uint32_t>(std::stoul(next("--seed")));
    } else if (arg == "--dead-sensor") {
      deadSensorLabel = next("--dead-sensor");
      if (deadSensorLabel == "L" || deadSensorLabel == "l") {
        deadSensor = Relative::kLeft;
        deadSensorLabel = "L";
      } else if (deadSensorLabel == "F" || deadSensorLabel == "f") {
        deadSensor = Relative::kForward;
        deadSensorLabel = "F";
      } else if (deadSensorLabel == "R" || deadSensorLabel == "r") {
        deadSensor = Relative::kRight;
        deadSensorLabel = "R";
      } else {
        std::cerr << "error: --dead-sensor must be L, F or R\n";
        return 1;
      }
    } else if (arg == "--out") {
      outPath = next("--out");
    } else if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "error: unknown option " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    } else {
      mazePath = arg;
    }
  }
  if (mazePath.empty()) {
    printUsage(argv[0]);
    return 1;
  }
  if (outPath.empty()) {
    outPath = "tools/sim/out/" + baseName(mazePath) + ".html";
  }

  std::ifstream in(mazePath, std::ios::binary);
  if (!in) {
    std::cerr << "error: cannot read " << mazePath << "\n";
    return 2;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  const std::string text = ss.str();

  sim::ParsedMaze pm;
  std::vector<std::string> errors;
  if (!sim::parseMazeText(text, pm, errors)) {
    std::cerr << "error: invalid maze " << mazePath << "\n";
    for (const std::string& e : errors) std::cerr << "  " << e << "\n";
    return 3;
  }

  const Cell start = pm.hasStart ? pm.start : cfg::kStartCell;
  const Direction startHeading = cfg::kStartHeading;
  const Cell goal = pm.hasGoal ? pm.goal : cfg::kGoalCell;
  if (start.row >= pm.rows || start.col >= pm.cols) {
    std::cerr << "error: start cell (" << static_cast<int>(start.row) << ","
              << static_cast<int>(start.col) << ") is outside the parsed "
              << static_cast<int>(pm.rows) << "x" << static_cast<int>(pm.cols)
              << " maze; mark it with 'S'\n";
    return 3;
  }
  if (goal.row >= pm.rows || goal.col >= pm.cols) {
    std::cerr << "error: goal cell (" << static_cast<int>(goal.row) << ","
              << static_cast<int>(goal.col) << ") is outside the parsed "
              << static_cast<int>(pm.rows) << "x" << static_cast<int>(pm.cols)
              << " maze; mark it with 'G'\n";
    return 3;
  }

  const ExplorerParams params = {cfg::kWallFlipObservations,
                                 cfg::kSensorFailurePasses,
                                 cfg::kExplorationBudgetFactor};
  Explorer explorer;
  Status status =
      explorer.reset(pm.rows, pm.cols, Pose{start, startHeading}, goal, params);
  if (!isOk(status)) {
    std::cerr << "error: Explorer::reset failed: " << statusName(status)
              << "\n";
    return 4;
  }

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  constexpr Relative kSides[3] = {Relative::kLeft, Relative::kForward,
                                  Relative::kRight};

  sim::SimData data;
  data.rows = pm.rows;
  data.cols = pm.cols;
  data.start = start;
  data.goal = goal;
  data.trueWalls = encodeTrueWalls(pm.truth);
  data.mazeFile = mazePath;
  data.noise = noise;
  data.seed = seed;
  data.deadSensor = deadSensorLabel;

  constexpr uint32_t kSafetySteps = 20000;
  bool done = false;
  Status outcomeStatus = Status::kOk;
  bool goalReached = false;

  for (uint32_t iter = 0; iter < kSafetySteps && !done; ++iter) {
    if (explorer.atGoal()) {
      data.steps.push_back(
          makeStep("GOAL_REACHED", explorer, false, Direction::kUp, false,
                  Status::kOk));
      goalReached = true;
      done = true;
      break;
    }

    // SENSE: observe the three sensors (left/front/right relative to
    // heading), mirroring RobotApp::runSense().
    const Pose pose = explorer.pose();
    SideObservation obs[3];
    for (uint8_t i = 0; i < 3; ++i) {
      const Direction absDir = rotate(pose.heading, kSides[i]);
      const bool trueWall = pm.truth.hasWall(pose.cell, absDir);
      bool wall = trueWall;
      bool anyEcho = true;
      if (deadSensor.has_value() && *deadSensor == kSides[i]) {
        anyEcho = false;
        wall = false;
      } else if (noise > 0.0 && uni(rng) < noise) {
        wall = !wall;
      }
      obs[i] = SideObservation{kSides[i], wall, anyEcho};
    }
    status = explorer.observe(obs, 3);
    if (!isOk(status)) {
      data.steps.push_back(makeStep("FAULT", explorer, false, Direction::kUp,
                                    true, status));
      outcomeStatus = status;
      done = true;
      break;
    }
    data.steps.push_back(
        makeStep("SENSE", explorer, false, Direction::kUp, false, Status::kOk));

    // PLAN: flood fill + direction choice, mirroring RobotApp::runPlan().
    Direction dir = Direction::kUp;
    status = explorer.plan(dir);
    if (!isOk(status)) {
      data.steps.push_back(makeStep("FAULT", explorer, false, Direction::kUp,
                                    true, status));
      outcomeStatus = status;
      done = true;
      break;
    }
    data.steps.push_back(makeStep("PLAN", explorer, true, dir, false, Status::kOk));

    // MOVE: turn + drive one cell, mirroring RobotApp::runMove(). No motor
    // physics here: if the planned direction is really blocked (e.g. the
    // map was tricked open by injected noise), that is what would stall the
    // real robot against the wall it failed to see.
    if (pm.truth.hasWall(pose.cell, dir)) {
      data.steps.push_back(
          makeStep("FAULT", explorer, true, dir, true, Status::kStall));
      outcomeStatus = Status::kStall;
      done = true;
      break;
    }
    status = explorer.advance(dir);
    if (!isOk(status)) {
      data.steps.push_back(
          makeStep("FAULT", explorer, true, dir, true, status));
      outcomeStatus = status;
      done = true;
      break;
    }
    data.steps.push_back(
        makeStep("MOVE", explorer, true, dir, false, Status::kOk));
  }

  if (!done) {
    std::cerr << "error: internal safety limit (" << kSafetySteps
              << " iterations) reached without a result\n";
    return 5;
  }

  if (goalReached) {
    data.outcomeText =
        "GOAL_REACHED after " + std::to_string(explorer.cellsMoved()) +
        " moves";
  } else {
    data.outcomeText = std::string("FAULT ") + statusName(outcomeStatus) +
                       " at (" + std::to_string(explorer.pose().cell.row) +
                       "," + std::to_string(explorer.pose().cell.col) +
                       ") facing " +
                       std::string(1, directionLetter(explorer.pose().heading)) +
                       " after " + std::to_string(explorer.cellsMoved()) +
                       " moves";
  }

  // Text summary to stdout.
  std::cout << "Micromouse simulator\n";
  std::cout << "maze:  " << mazePath << " (" << static_cast<int>(pm.rows)
            << "x" << static_cast<int>(pm.cols) << ")\n";
  std::cout << "start: (" << static_cast<int>(start.row) << ","
            << static_cast<int>(start.col) << ") facing "
            << directionLetter(startHeading) << "\n";
  std::cout << "goal:  (" << static_cast<int>(goal.row) << ","
            << static_cast<int>(goal.col) << ")\n";
  std::cout << "noise=" << noise << " seed=" << seed
            << (deadSensor.has_value() ? " dead-sensor=" + deadSensorLabel : "")
            << "\n";
  std::cout << "result: " << data.outcomeText << "\n\n";

  std::string mazeDump;
  renderMaze(explorer.maze(), explorer.flood(), explorer.pose().cell,
            sinkToString, &mazeDump);
  std::cout << "Discovered maze:\n" << mazeDump << "\n";

  if (!sim::writeHtml(outPath, data)) return 6;
  std::cout << "wrote " << outPath << " (" << data.steps.size()
            << " animation frames)\n";
  return goalReached ? 0 : 1;
}
