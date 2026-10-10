#include "control/wall_centering.h"

#include "util/math_utils.h"

namespace micromouse {

void WallCentering::setWalls(const SideWalls walls) {
  if (walls.left != walls_.left) {
    leftValid_ = false;
    leftTerm_ = 0.0f;
  }
  if (walls.right != walls_.right) {
    rightValid_ = false;
    rightTerm_ = 0.0f;
  }
  walls_ = walls;
}

void WallCentering::reset() {
  walls_ = SideWalls{false, false};
  leftValid_ = false;
  rightValid_ = false;
  leftTerm_ = 0.0f;
  rightTerm_ = 0.0f;
  slot_ = 0;
}

Side WallCentering::nextSide() {
  const uint8_t periods = params_.pingPeriods > 0 ? params_.pingPeriods : 1;
  const uint8_t slot = slot_;
  slot_ = static_cast<uint8_t>(slot_ + 1 >= 2 * periods ? 0 : slot_ + 1);
  if (slot == 0) return walls_.left ? Side::kLeft : Side::kNone;
  if (slot == periods) return walls_.right ? Side::kRight : Side::kNone;
  return Side::kNone;
}

void WallCentering::addReading(const Side side, const float distanceCm) {
  if (side == Side::kNone) return;
  const bool isLeft = side == Side::kLeft;
  const float nominal =
      isLeft ? params_.nominalLeftCm : params_.nominalRightCm;
  const float offset = distanceCm - nominal;
  const bool valid = distanceCm > 0.0f &&
                     (isLeft ? walls_.left : walls_.right) &&
                     util::absValue(offset) <= params_.maxDeviationCm;
  // Left wall further away than nominal => robot is right of centre =>
  // steer left (positive). The right sensor sees the mirror image.
  const float raw = (isLeft ? offset : -offset) * params_.gainTicksPerCm;
  bool& sideValid = isLeft ? leftValid_ : rightValid_;
  float& term = isLeft ? leftTerm_ : rightTerm_;
  if (valid) {
    term += params_.filter * (raw - term);  // starts from 0: gentle ramp-in
  } else {
    term = 0.0f;
  }
  sideValid = valid;
}

float WallCentering::correctionTicks() const {
  float sum = 0.0f;
  uint8_t count = 0;
  if (leftValid_) {
    sum += leftTerm_;
    ++count;
  }
  if (rightValid_) {
    sum += rightTerm_;
    ++count;
  }
  if (count == 0) return 0.0f;
  return util::clamp(sum / count, -params_.maxTicks, params_.maxTicks);
}

}  // namespace micromouse
