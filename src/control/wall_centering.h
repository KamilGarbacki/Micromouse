// Optional side-wall centering: turns side-sensor distances into a bounded
// correction of the heading error. Pure logic; the caller does the pings.
#pragma once

#include <stdint.h>

namespace micromouse {

/// Which side walls are known to be present beside the robot.
struct SideWalls {
  bool left;
  bool right;
};

enum class Side : uint8_t { kNone, kLeft, kRight };

struct CenteringParams {
  float nominalLeftCm;    ///< [cm] left reading when centred
  float nominalRightCm;   ///< [cm] right reading when centred
  float gainTicksPerCm;   ///< [ticks per cm] heading-error ticks per offset
  float maxTicks;         ///< [ticks] bound of the correction
  float maxDeviationCm;   ///< [cm] |reading - nominal| above this: ignored
  float filter;           ///< [-] low-pass coefficient per side, (0, 1]
  uint8_t pingPeriods;    ///< [periods] between pings of one side / 2, >= 1
};

/// Sign convention: the heading error is (left - right) ticks; a positive
/// correction steers the robot to the LEFT. So a robot that is right of
/// centre (left reading > nominal, or right reading < nominal) gets a
/// positive correction.
class WallCentering {
 public:
  explicit WallCentering(const CenteringParams& params) : params_(params) {}

  /// Sets which sides may be used. Readings of a side that changes are
  /// discarded. Does not reset the left/right alternation.
  void setWalls(SideWalls walls);

  /// Clears all readings, walls and the alternation.
  void reset();

  /// The side to ping this period (call once per period). Cycle of
  /// 2 * pingPeriods periods: left in slot 0, right in slot pingPeriods,
  /// kNone otherwise and when the side due has no known wall.
  Side nextSide();

  /// Feeds a reading [cm] for `side` (<= 0 = no echo). The side's term is
  /// low-pass filtered; an implausible reading resets it to 0 and marks the
  /// side invalid until the next good one.
  void addReading(Side side, float distanceCm);

  /// The correction to add to the heading error [ticks], bounded to
  /// +-maxTicks; 0 without valid readings. Averages both sides if both valid.
  float correctionTicks() const;

 private:
  CenteringParams params_;
  SideWalls walls_ = {false, false};
  bool leftValid_ = false;
  bool rightValid_ = false;
  float leftTerm_ = 0.0f;
  float rightTerm_ = 0.0f;
  uint8_t slot_ = 0;
};

}  // namespace micromouse
