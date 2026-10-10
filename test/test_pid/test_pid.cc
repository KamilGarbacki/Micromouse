// Host tests: PID controller including conditional-integration anti-windup.
#include <unity.h>

#include "control/pid.h"

using namespace micromouse;

namespace {
constexpr float kDt = 0.01f;
constexpr float kEps = 1e-4f;
constexpr PidClamp kNoClamp = {false, false};
}  // namespace

void setUp() {}
void tearDown() {}

void test_proportional_only() {
  Pid pid(PidGains{2.0f, 0.0f, 0.0f, 0.0f, 100.0f}, kDt);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 6.0f, pid.update(3.0f));
  TEST_ASSERT_FLOAT_WITHIN(kEps, -4.0f, pid.update(-2.0f));
}

void test_output_limit() {
  Pid pid(PidGains{10.0f, 0.0f, 0.0f, 0.0f, 5.0f}, kDt);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 5.0f, pid.update(3.0f));
  TEST_ASSERT_FLOAT_WITHIN(kEps, -5.0f, pid.update(-3.0f));
}

void test_integral_accumulates_with_dt() {
  Pid pid(PidGains{0.0f, 10.0f, 0.0f, 100.0f, 100.0f}, kDt);
  float out = 0.0f;
  for (int i = 0; i < 10; ++i) out = pid.update(1.0f);
  // ki * sum(e * dt) = 10 * 10 * 0.01 = 1
  TEST_ASSERT_FLOAT_WITHIN(kEps, 1.0f, out);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 1.0f, pid.integralTerm());
}

void test_integral_limit() {
  Pid pid(PidGains{0.0f, 10.0f, 0.0f, 2.0f, 100.0f}, kDt);
  for (int i = 0; i < 1000; ++i) pid.update(1.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 2.0f, pid.integralTerm());
}

void test_derivative_uses_precomputed_inverse_dt() {
  Pid pid(PidGains{0.0f, 0.0f, 0.5f, 0.0f, 1000.0f}, kDt);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, pid.update(1.0f));  // no history
  // kd * (3 - 1) / 0.01 = 100
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 100.0f, pid.update(3.0f));
}

void test_no_windup_while_saturated_by_own_limit() {
  // P alone saturates the output; the integrator must not wind up.
  Pid pid(PidGains{10.0f, 5.0f, 0.0f, 50.0f, 10.0f}, kDt);
  for (int i = 0; i < 500; ++i) pid.update(5.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, pid.integralTerm());
  // So the output responds to a reversed error immediately.
  TEST_ASSERT_TRUE(pid.update(-0.5f) < 0.0f);
}

void test_integrator_stops_at_output_limit_and_recovers() {
  // Small P: the integrator itself drives the output into the limit.
  Pid pid(PidGains{0.1f, 5.0f, 0.0f, 50.0f, 1.0f}, kDt);
  for (int i = 0; i < 1000; ++i) pid.update(1.0f);
  // Frozen just below outputLimit - p (0.9), not at integralLimit (50).
  TEST_ASSERT_TRUE(pid.integralTerm() <= 0.9f + kEps);
  TEST_ASSERT_TRUE(pid.integralTerm() > 0.8f);
  // Reversed error integrates down at once and leaves saturation.
  const float before = pid.integralTerm();
  const float out = pid.update(-1.0f);
  TEST_ASSERT_TRUE(pid.integralTerm() < before);
  TEST_ASSERT_TRUE(out < 1.0f);
}

void test_external_high_clamp_blocks_positive_integration() {
  Pid pid(PidGains{0.1f, 5.0f, 0.0f, 50.0f, 100.0f}, kDt);
  for (int i = 0; i < 100; ++i) pid.update(1.0f, PidClamp{true, false});
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, pid.integralTerm());
  // A negative error still integrates (it moves away from the clamp).
  pid.update(-1.0f, PidClamp{true, false});
  TEST_ASSERT_TRUE(pid.integralTerm() < 0.0f);
}

void test_external_low_clamp_blocks_negative_integration() {
  Pid pid(PidGains{0.1f, 5.0f, 0.0f, 50.0f, 100.0f}, kDt);
  for (int i = 0; i < 100; ++i) pid.update(-1.0f, PidClamp{false, true});
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, pid.integralTerm());
  pid.update(1.0f, PidClamp{false, true});
  TEST_ASSERT_TRUE(pid.integralTerm() > 0.0f);
}

void test_unclamped_matches_plain_pi() {
  Pid pid(PidGains{1.0f, 2.0f, 0.0f, 100.0f, 100.0f}, kDt);
  float integral = 0.0f;
  float out = 0.0f;
  for (int i = 0; i < 20; ++i) {
    const float e = (i % 2 == 0) ? 1.0f : 0.5f;
    integral += e * kDt;
    out = pid.update(e, kNoClamp);
    TEST_ASSERT_FLOAT_WITHIN(kEps, 1.0f * e + 2.0f * integral, out);
  }
}

void test_reset_clears_state() {
  Pid pid(PidGains{0.0f, 10.0f, 1.0f, 100.0f, 100.0f}, kDt);
  for (int i = 0; i < 10; ++i) pid.update(1.0f);
  pid.reset();
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, pid.integralTerm());
  // No derivative kick after reset: first update has no history.
  TEST_ASSERT_FLOAT_WITHIN(kEps, 10.0f * 5.0f * kDt, pid.update(5.0f));
}

void test_invalid_dt_outputs_zero() {
  Pid pid(PidGains{1.0f, 1.0f, 1.0f, 10.0f, 10.0f}, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 0.0f, pid.update(5.0f));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_proportional_only);
  RUN_TEST(test_output_limit);
  RUN_TEST(test_integral_accumulates_with_dt);
  RUN_TEST(test_integral_limit);
  RUN_TEST(test_derivative_uses_precomputed_inverse_dt);
  RUN_TEST(test_no_windup_while_saturated_by_own_limit);
  RUN_TEST(test_integrator_stops_at_output_limit_and_recovers);
  RUN_TEST(test_external_high_clamp_blocks_positive_integration);
  RUN_TEST(test_external_low_clamp_blocks_negative_integration);
  RUN_TEST(test_unclamped_matches_plain_pi);
  RUN_TEST(test_reset_clears_state);
  RUN_TEST(test_invalid_dt_outputs_zero);
  return UNITY_END();
}
