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

// Print the discovered maze when the goal is reached.
constexpr bool kPrintMazeOnGoal = true;

// While in FAULT, repeat the error report this often, so a serial monitor
// opened late still sees it. [ms]
constexpr uint32_t kFaultReportIntervalMs = 5000;

}  // namespace micromouse::config
