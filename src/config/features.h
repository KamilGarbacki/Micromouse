// Compile-time feature flags and the log level.
#pragma once

#include <stdint.h>

// Log level, selectable with -D MM_LOG_LEVEL=<n> in platformio.ini:
//   0 = none, 1 = errors (FAULT reports), 2 = info (progress, goal),
//   3 = debug (per-move diagnostics + maze after every plan; this replaces
//       the phase-1 ENGINE_DEBUG flag).
#define MM_LOG_LEVEL_NONE 0
#define MM_LOG_LEVEL_ERROR 1
#define MM_LOG_LEVEL_INFO 2
#define MM_LOG_LEVEL_DEBUG 3

#ifndef MM_LOG_LEVEL
#define MM_LOG_LEVEL MM_LOG_LEVEL_INFO
#endif

// Side-wall centering during straight moves (see README): 0 = off
// (default), 1 = on. Also selectable with -D MM_WALL_CENTERING=1.
#ifndef MM_WALL_CENTERING
#define MM_WALL_CENTERING 0
#endif

// Front-wall alignment after straight moves (see README): 1 = on (default).
#ifndef MM_FRONT_ALIGN
#define MM_FRONT_ALIGN 1
#endif

// Maze walls (see README "Open floor vs physical boundary"):
//   0 = OPEN FLOOR (default): a mat with the cells outlined, no physical
//       walls at all, not even the outer boundary. The grid edge is still
//       impassable for planning, but it is a VIRTUAL wall: no sensor is
//       expected to see it (no sensor-failure check, no learning, no
//       centering against it). Interior walls, if any, are physical.
//   1 = PHYSICAL BOUNDARY (competition maze): the outer boundary is a real
//       wall that the sensors see, like every interior wall.
// Also selectable with -D MM_PHYSICAL_BOUNDARY=1.
#ifndef MM_PHYSICAL_BOUNDARY
#define MM_PHYSICAL_BOUNDARY 0
#endif

// Startup menu with the bench tools (encoder check, motor self-calibration,
// reset / print calibration): 1 = on. Off in the competition env `uno`
// (saves ~3 KB of flash); on in `uno_setup`. Needs MM_LOG_LEVEL >= 1.
#ifndef MM_MENU
#define MM_MENU 0
#endif

// Online learning of wheel scale, cell pitch, ticks per 90 deg and the brake
// leads while driving the maze (see README): 1 = on (default).
#ifndef MM_LEARNING
#define MM_LEARNING 1
#endif

namespace micromouse::config {

enum class LogLevel : uint8_t {
  kNone = MM_LOG_LEVEL_NONE,
  kError = MM_LOG_LEVEL_ERROR,
  kInfo = MM_LOG_LEVEL_INFO,
  kDebug = MM_LOG_LEVEL_DEBUG,
};

constexpr LogLevel kLogLevel = static_cast<LogLevel>(MM_LOG_LEVEL);

/// True if messages of `level` are compiled in. Use with `if constexpr`.
constexpr bool logEnabled(const LogLevel level) { return kLogLevel >= level; }

constexpr uint32_t kSerialBaud = 115200;  // [baud] match monitor_speed

/// Side-wall centering during straight moves (MM_WALL_CENTERING).
constexpr bool kWallCenteringEnabled = MM_WALL_CENTERING != 0;

/// Front-wall alignment after straight moves (MM_FRONT_ALIGN).
constexpr bool kFrontAlignEnabled = MM_FRONT_ALIGN != 0;

/// The outer boundary is a physical wall (MM_PHYSICAL_BOUNDARY); false =
/// open floor (virtual boundary). The default of ExplorerParams.
constexpr bool kPhysicalBoundary = MM_PHYSICAL_BOUNDARY != 0;

/// Online learning while driving (MM_LEARNING).
constexpr bool kOnlineLearningEnabled = MM_LEARNING != 0;

/// Startup mode menu (MM_MENU). It needs Serial, i.e. a log level of at
/// least kError; otherwise the robot starts RUN at once.
constexpr bool kModeMenuEnabled =
    MM_MENU != 0 && logEnabled(LogLevel::kError);

// Print the discovered maze when the goal is reached.
constexpr bool kPrintMazeOnGoal = true;

// While in FAULT, repeat the error report this often, so a serial monitor
// opened late still sees it. [ms]
constexpr uint32_t kFaultReportIntervalMs = 5000;

}  // namespace micromouse::config
