// Host tests: velocity profile shape, using the configured profiles and an
// ideal plant (the robot moves exactly at the target speed).
#include <math.h>
#include <unity.h>

#include "config/config.h"
#include "control/motion_profile.h"

using namespace micromouse;

namespace {

constexpr float kEps = 1e-3f;
constexpr int kMaxSteps = 10000;

constexpr ProfileParams kForward = {
    config::forward::kCruiseSpeed, config::forward::kCreepSpeed,
    config::forward::kAccel,       config::forward::kDecel,
    config::forward::kCreepTicks,  config::forward::kBrakeLeadTicks};

constexpr ProfileParams kTurn = {
    config::turn::kCruiseSpeed, config::turn::kCreepSpeed,
    config::turn::kAccel,       config::turn::kDecel,
    config::turn::kCreepTicks,  config::turn::kBrakeLeadTicks};

struct RunResult {
  int steps;
  float maxSpeed;
  float maxStepChange;
  bool creptBeforeStop;  // reached creep speed before the brake point
  int creepSteps;        // periods spent in [creepStart, stop)
  float maxSpeedInCreepZone;
};

RunResult simulate(const ProfileParams& params, const int32_t target) {
  MotionProfile profile(params, target);
  RunResult r = {0, 0.0f, 0.0f, false, 0, 0.0f};
  float position = 0.0f;
  float prev = 0.0f;
  while (r.steps < kMaxSteps) {
    const int32_t travelled = static_cast<int32_t>(position);
    if (profile.brakePointReached(travelled)) break;
    const float v = profile.next(travelled);
    if (v > r.maxSpeed) r.maxSpeed = v;
    const float change = fabsf(v - prev);
    if (r.steps > 0 && change > r.maxStepChange) r.maxStepChange = change;
    if (travelled >= profile.creepStartTicks()) {
      ++r.creepSteps;
      if (v > r.maxSpeedInCreepZone) r.maxSpeedInCreepZone = v;
      if (fabsf(v - params.creepSpeed) < kEps) r.creptBeforeStop = true;
    }
    prev = v;
    position += v;
    ++r.steps;
  }
  return r;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_stop_and_creep_points() {
  MotionProfile profile(kForward, 2068);
  TEST_ASSERT_EQUAL_INT32(2068 - kForward.brakeLeadTicks, profile.stopTicks());
  TEST_ASSERT_EQUAL_INT32(2068 - kForward.brakeLeadTicks - kForward.creepTicks,
                          profile.creepStartTicks());
  TEST_ASSERT_FALSE(profile.brakePointReached(profile.stopTicks() - 1));
  TEST_ASSERT_TRUE(profile.brakePointReached(profile.stopTicks()));
}

void test_first_step_is_one_accel_increment() {
  MotionProfile profile(kForward, 2068);
  TEST_ASSERT_FLOAT_WITHIN(kEps, kForward.accel, profile.next(0));
  TEST_ASSERT_FLOAT_WITHIN(kEps, 2.0f * kForward.accel, profile.next(0));
}

void test_decel_limit_is_sqrt_curve_and_monotonic() {
  MotionProfile profile(kForward, 2068);
  const int32_t creepStart = profile.creepStartTicks();
  TEST_ASSERT_FLOAT_WITHIN(kEps, kForward.creepSpeed,
                           profile.decelLimit(creepStart));
  TEST_ASSERT_FLOAT_WITHIN(kEps, kForward.creepSpeed,
                           profile.decelLimit(creepStart + 50));
  const float expected = sqrtf(kForward.creepSpeed * kForward.creepSpeed +
                               2.0f * kForward.decel * 100.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, expected,
                           profile.decelLimit(creepStart - 100));
  float prev = profile.decelLimit(0);
  for (int32_t x = 1; x <= profile.stopTicks(); ++x) {
    const float v = profile.decelLimit(x);
    TEST_ASSERT_TRUE(v <= prev + kEps);
    prev = v;
  }
}

void test_forward_cell_reaches_cruise_and_creeps_before_stop() {
  const RunResult r = simulate(kForward, config::kTicksPerCell);
  TEST_ASSERT_TRUE(r.steps < kMaxSteps);
  TEST_ASSERT_FLOAT_WITHIN(kEps, kForward.cruiseSpeed, r.maxSpeed);
  TEST_ASSERT_TRUE(r.creptBeforeStop);
  TEST_ASSERT_TRUE(r.maxSpeedInCreepZone <= kForward.creepSpeed + kEps);
  // creepTicks at creepSpeed takes about creepTicks / creepSpeed periods.
  TEST_ASSERT_TRUE(r.creepSteps >=
                   static_cast<int>(kForward.creepTicks / kForward.creepSpeed) -
                       1);
}

void test_profile_is_continuous() {
  // With an ideal plant the target changes by at most ~accel / ~decel per
  // period (a little more on the first sqrt samples: position quantisation).
  const RunResult r = simulate(kForward, config::kTicksPerCell);
  const float bound =
      1.5f * (kForward.accel > kForward.decel ? kForward.accel
                                              : kForward.decel);
  TEST_ASSERT_TRUE(r.maxStepChange <= bound);

  const RunResult t = simulate(kTurn, config::kTicksPer90);
  const float turnBound =
      1.5f * (kTurn.accel > kTurn.decel ? kTurn.accel : kTurn.decel);
  TEST_ASSERT_TRUE(t.maxStepChange <= turnBound);
}

void test_turn_profiles_creep_before_stop() {
  // Shortest (target clamped to half) to longest (1.5 x 180 deg) turn.
  const int32_t targets[] = {config::kTicksPer90 / 2, config::kTicksPer90,
                             2 * config::kTicksPer90,
                             3 * config::kTicksPer90};
  for (const int32_t target : targets) {
    const RunResult r = simulate(kTurn, target);
    TEST_ASSERT_TRUE(r.steps < kMaxSteps);
    TEST_ASSERT_TRUE(r.creptBeforeStop);
    TEST_ASSERT_TRUE(r.maxSpeedInCreepZone <= kTurn.creepSpeed + kEps);
  }
}

void test_short_move_never_exceeds_creep() {
  // Shorter than creepTicks + brakeLead: the whole move is a creep.
  const int32_t target = kForward.creepTicks / 2;
  const RunResult r = simulate(kForward, target);
  TEST_ASSERT_TRUE(r.steps > 0);
  TEST_ASSERT_TRUE(r.maxSpeed <= kForward.creepSpeed + kEps);
}

void test_move_shorter_than_brake_lead_brakes_at_once() {
  MotionProfile profile(kForward, kForward.brakeLeadTicks);
  TEST_ASSERT_TRUE(profile.brakePointReached(0));
  const RunResult r = simulate(kForward, kForward.brakeLeadTicks - 1);
  TEST_ASSERT_EQUAL_INT(0, r.steps);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_stop_and_creep_points);
  RUN_TEST(test_first_step_is_one_accel_increment);
  RUN_TEST(test_decel_limit_is_sqrt_curve_and_monotonic);
  RUN_TEST(test_forward_cell_reaches_cruise_and_creeps_before_stop);
  RUN_TEST(test_profile_is_continuous);
  RUN_TEST(test_turn_profiles_creep_before_stop);
  RUN_TEST(test_short_move_never_exceeds_creep);
  RUN_TEST(test_move_shorter_than_brake_lead_brakes_at_once);
  return UNITY_END();
}
