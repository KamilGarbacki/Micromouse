// Host tests: wheel mixer, wall centering, median and ring buffer.
#include <unity.h>

#include "control/heading_carry.h"
#include "control/stiction.h"
#include "control/wall_centering.h"
#include "control/wheel_mixer.h"
#include "util/median.h"
#include "util/ring_buffer.h"

using namespace micromouse;

namespace {
constexpr float kEps = 1e-4f;
// No filter, ping every period: raw behaviour, easy to reason about.
constexpr CenteringParams kCentering = {6.0f, 6.0f, 8.0f, 40.0f,
                                        5.0f, 1.0f, 1};
}  // namespace

void setUp() {}
void tearDown() {}

// --- Wheel mixer -----------------------------------------------------------

void test_mixer_applies_trim_and_correction() {
  const WheelMix m = mixWheels(100.0f, 10.0f, 0.9f, 1.0f, 20.0f, 255.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 80.0f, m.left);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 110.0f, m.right);
  TEST_ASSERT_FALSE(m.speedClamp.high || m.speedClamp.low);
  TEST_ASSERT_FALSE(m.headingClamp.high || m.headingClamp.low);
}

void test_mixer_floor_keeps_differential() {
  // left = 30 - 25 = 5 < 20: raised to 20, the 15 moves to the right wheel.
  const WheelMix m = mixWheels(30.0f, 25.0f, 1.0f, 1.0f, 20.0f, 255.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 20.0f, m.left);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 70.0f, m.right);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 50.0f, m.right - m.left);
}

void test_mixer_overflow_keeps_differential() {
  // right = 270 > 255: both lowered by 15, differential 40 kept.
  const WheelMix r = mixWheels(250.0f, 20.0f, 1.0f, 1.0f, 20.0f, 255.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 255.0f, r.right);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 215.0f, r.left);
  TEST_ASSERT_TRUE(r.speedClamp.high);
  TEST_ASSERT_FALSE(r.headingClamp.high || r.headingClamp.low);

  const WheelMix l = mixWheels(250.0f, -20.0f, 1.0f, 1.0f, 20.0f, 255.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 255.0f, l.left);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 215.0f, l.right);
  TEST_ASSERT_TRUE(l.speedClamp.high);
  TEST_ASSERT_FALSE(l.headingClamp.high || l.headingClamp.low);
}

void test_mixer_reports_heading_saturation_when_differential_too_big() {
  // Differential 400 > 255 - 20: lower wheel held at the floor.
  const WheelMix r = mixWheels(250.0f, 200.0f, 1.0f, 1.0f, 20.0f, 255.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 255.0f, r.right);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 20.0f, r.left);
  TEST_ASSERT_TRUE(r.headingClamp.high);
  TEST_ASSERT_FALSE(r.headingClamp.low);

  const WheelMix l = mixWheels(250.0f, -200.0f, 1.0f, 1.0f, 20.0f, 255.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 255.0f, l.left);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 20.0f, l.right);
  TEST_ASSERT_TRUE(l.headingClamp.low);
  TEST_ASSERT_FALSE(l.headingClamp.high);
}

// --- Wall centering --------------------------------------------------------

void test_centering_alternates_and_skips_sides_without_wall() {
  WallCentering c(kCentering);
  c.setWalls(SideWalls{true, false});
  TEST_ASSERT_TRUE(c.nextSide() == Side::kLeft);
  TEST_ASSERT_TRUE(c.nextSide() == Side::kNone);  // right: no wall
  TEST_ASSERT_TRUE(c.nextSide() == Side::kLeft);
  c.setWalls(SideWalls{true, true});
  TEST_ASSERT_TRUE(c.nextSide() == Side::kRight);
  TEST_ASSERT_TRUE(c.nextSide() == Side::kLeft);
}

void test_centering_ping_spacing() {
  CenteringParams params = kCentering;
  params.pingPeriods = 2;  // L, -, R, -, L, ...
  WallCentering c(params);
  c.setWalls(SideWalls{true, true});
  TEST_ASSERT_TRUE(c.nextSide() == Side::kLeft);
  TEST_ASSERT_TRUE(c.nextSide() == Side::kNone);
  TEST_ASSERT_TRUE(c.nextSide() == Side::kRight);
  TEST_ASSERT_TRUE(c.nextSide() == Side::kNone);
  TEST_ASSERT_TRUE(c.nextSide() == Side::kLeft);
}

void test_centering_low_pass_and_sub_cm_resolution() {
  CenteringParams params = kCentering;
  params.filter = 0.5f;
  WallCentering c(params);
  c.setWalls(SideWalls{true, false});
  c.addReading(Side::kLeft, 6.5f);  // raw +0.5 cm * 8 = 4, filtered 2
  TEST_ASSERT_FLOAT_WITHIN(kEps, 2.0f, c.correctionTicks());
  c.addReading(Side::kLeft, 6.5f);  // 2 + 0.5 * (4 - 2) = 3
  TEST_ASSERT_FLOAT_WITHIN(kEps, 3.0f, c.correctionTicks());
  c.addReading(Side::kLeft, 0.0f);  // no echo: reset
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, c.correctionTicks());
}

void test_centering_sign_convention() {
  WallCentering c(kCentering);
  c.setWalls(SideWalls{true, true});
  // Robot 2 cm right of centre: left reads 8, right reads 4 -> steer left.
  c.addReading(Side::kLeft, 8);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 16.0f, c.correctionTicks());
  c.addReading(Side::kRight, 4);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 16.0f, c.correctionTicks());
  // Robot left of centre -> negative.
  c.addReading(Side::kLeft, 4);
  c.addReading(Side::kRight, 8);
  TEST_ASSERT_FLOAT_WITHIN(kEps, -16.0f, c.correctionTicks());
}

void test_centering_is_bounded() {
  CenteringParams params = kCentering;
  params.gainTicksPerCm = 100.0f;
  WallCentering c(params);
  c.setWalls(SideWalls{true, false});
  c.addReading(Side::kLeft, 10);  // +4 cm * 100 -> clamped to 40
  TEST_ASSERT_FLOAT_WITHIN(kEps, 40.0f, c.correctionTicks());
}

void test_centering_rejects_implausible_readings() {
  WallCentering c(kCentering);
  c.setWalls(SideWalls{true, true});
  c.addReading(Side::kLeft, 8);
  c.addReading(Side::kLeft, 0);  // no echo
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, c.correctionTicks());
  c.addReading(Side::kLeft, 30);  // 24 cm off nominal: a gap in the wall
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, c.correctionTicks());
}

void test_centering_ignores_sides_without_known_wall() {
  WallCentering c(kCentering);
  c.setWalls(SideWalls{false, false});
  c.addReading(Side::kLeft, 8);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, c.correctionTicks());
  // Losing the wall discards the held reading.
  c.setWalls(SideWalls{true, false});
  c.addReading(Side::kLeft, 8);
  c.setWalls(SideWalls{false, false});
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, c.correctionTicks());
}

// --- Heading carry -----------------------------------------------------------

void test_heading_carry_forward() {
  TEST_ASSERT_EQUAL_INT32(15, heading::carryAfterForward(10, 105, 100, 60));
  TEST_ASSERT_EQUAL_INT32(60, heading::carryAfterForward(50, 130, 100, 60));
  TEST_ASSERT_EQUAL_INT32(-60, heading::carryAfterForward(-50, 0, 30, 60));
}

void test_heading_turn_target_absorbs_carry() {
  // Rotated clockwise by 20 ticks: turn 10 less clockwise, 10 more CCW.
  TEST_ASSERT_EQUAL_INT32(544, heading::turnTarget(554, 1, 20));
  TEST_ASSERT_EQUAL_INT32(564, heading::turnTarget(554, -1, 20));
  // Clamped to [nominal / 2, 1.5 * nominal].
  TEST_ASSERT_EQUAL_INT32(277, heading::turnTarget(554, 1, 10000));
  TEST_ASSERT_EQUAL_INT32(831, heading::turnTarget(554, -1, 10000));
}

void test_heading_carry_after_turn() {
  // Clockwise, wheels 560 + 558 vs nominal 2 * 554: over-rotated by 10.
  TEST_ASSERT_EQUAL_INT32(10,
                          heading::carryAfterTurn(0, 1, 560, 558, 554, 60));
  // Same counts counter-clockwise: -10.
  TEST_ASSERT_EQUAL_INT32(-10,
                          heading::carryAfterTurn(0, -1, 560, 558, 554, 60));
}

void test_heading_carry_after_settle_timeout_uses_counts() {
  // A forward move seeded with carry 30 drives L - R towards -30. Even if it
  // did not settle, the counts at the timeout are applied: the new carry is
  // the small residual, not the stale 30 (which would be a phantom error).
  const int32_t carry = 30;
  const int32_t left = 2068 - 14;
  const int32_t right = 2068 + 14;  // L - R = -28
  TEST_ASSERT_EQUAL_INT32(2, heading::carryAfterForward(carry, left, right,
                                                        60));
  // A turn that consumed the carry: target 554 - 15 = 539 per wheel.
  const int32_t target = heading::turnTarget(554, 1, carry);
  TEST_ASSERT_EQUAL_INT32(0, heading::carryAfterTurn(carry, 1, target, target,
                                                     554, 60));
}

void test_stiction_hysteresis() {
  bool rolling = false;
  rolling = updateRolling(rolling, 0.5f, 1.0f, 0.25f);  // one backlash tick
  TEST_ASSERT_FALSE(rolling);
  rolling = updateRolling(rolling, 1.0f, 1.0f, 0.25f);
  TEST_ASSERT_TRUE(rolling);
  rolling = updateRolling(rolling, 0.3f, 1.0f, 0.25f);  // still rolling
  TEST_ASSERT_TRUE(rolling);
  rolling = updateRolling(rolling, 0.2f, 1.0f, 0.25f);
  TEST_ASSERT_FALSE(rolling);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 35.0f, stictionPwm(false, 35.0f, 25.0f));
  TEST_ASSERT_FLOAT_WITHIN(kEps, 25.0f, stictionPwm(true, 35.0f, 25.0f));
}

// --- Median / ring buffer --------------------------------------------------

void test_median_odd_and_outlier() {
  uint16_t a[] = {30, 5, 12};
  TEST_ASSERT_EQUAL_UINT16(12, util::medianInPlace(a, 3));
  uint16_t b[] = {15, 0xFFFF, 14};  // one dropout mapped to "far"
  TEST_ASSERT_EQUAL_UINT16(15, util::medianInPlace(b, 3));
  uint16_t c[] = {7};
  TEST_ASSERT_EQUAL_UINT16(7, util::medianInPlace(c, 1));
  uint16_t d[] = {9, 1, 5, 3, 7};
  TEST_ASSERT_EQUAL_UINT16(5, util::medianInPlace(d, 5));
}

void test_ring_buffer_fifo_wraps_and_rejects_overflow() {
  util::RingBuffer<uint8_t, 3> q;
  uint8_t v = 0;
  TEST_ASSERT_TRUE(q.empty());
  TEST_ASSERT_FALSE(q.pop(v));
  TEST_ASSERT_TRUE(q.push(1));
  TEST_ASSERT_TRUE(q.push(2));
  TEST_ASSERT_TRUE(q.push(3));
  TEST_ASSERT_FALSE(q.push(4));
  TEST_ASSERT_TRUE(q.pop(v));
  TEST_ASSERT_EQUAL_UINT8(1, v);
  TEST_ASSERT_TRUE(q.push(4));  // wraps
  TEST_ASSERT_TRUE(q.pop(v));
  TEST_ASSERT_EQUAL_UINT8(2, v);
  TEST_ASSERT_TRUE(q.pop(v));
  TEST_ASSERT_EQUAL_UINT8(3, v);
  TEST_ASSERT_TRUE(q.pop(v));
  TEST_ASSERT_EQUAL_UINT8(4, v);
  TEST_ASSERT_TRUE(q.empty());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_mixer_applies_trim_and_correction);
  RUN_TEST(test_mixer_floor_keeps_differential);
  RUN_TEST(test_mixer_overflow_keeps_differential);
  RUN_TEST(test_mixer_reports_heading_saturation_when_differential_too_big);
  RUN_TEST(test_centering_alternates_and_skips_sides_without_wall);
  RUN_TEST(test_centering_ping_spacing);
  RUN_TEST(test_centering_low_pass_and_sub_cm_resolution);
  RUN_TEST(test_heading_carry_forward);
  RUN_TEST(test_heading_turn_target_absorbs_carry);
  RUN_TEST(test_heading_carry_after_turn);
  RUN_TEST(test_heading_carry_after_settle_timeout_uses_counts);
  RUN_TEST(test_stiction_hysteresis);
  RUN_TEST(test_centering_sign_convention);
  RUN_TEST(test_centering_is_bounded);
  RUN_TEST(test_centering_rejects_implausible_readings);
  RUN_TEST(test_centering_ignores_sides_without_known_wall);
  RUN_TEST(test_median_odd_and_outlier);
  RUN_TEST(test_ring_buffer_fifo_wraps_and_rejects_overflow);
  return UNITY_END();
}
