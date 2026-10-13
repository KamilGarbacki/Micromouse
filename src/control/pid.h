// Fixed-period PID controller with conditional-integration anti-windup.
#pragma once

namespace micromouse {

/// PID gains and limits (all in output units unless noted).
struct PidGains {
  float kp;             ///< [out / err]
  float ki;             ///< [out / (err * s)]
  float kd;             ///< [out / (err / s)]
  float integralLimit;  ///< max |ki * integral|
  float outputLimit;    ///< max |output|
};

/// Saturation of the actuator downstream of the PID, as seen by the caller
/// (e.g. a PWM floor or ceiling applied after update()). `high` means the
/// output could not be increased any further, `low` that it could not be
/// decreased.
struct PidClamp {
  bool high;
  bool low;
};

/// output = kp * e + ki * integral(e dt) + kd * de/dt, called every `dt`.
///
/// Anti-windup: |ki * integral| <= integralLimit, and the integrator is
/// frozen whenever integrating would push the output further into a
/// saturation - either its own outputLimit or an external clamp reported by
/// the caller. The external clamp is the one observed on the previous
/// period (one period of latency).
class Pid {
 public:
  /// `dt` [s] must be > 0; otherwise update() always returns 0.
  /// constexpr, so that the owning objects can be constant-initialised
  /// (no start-up code).
  constexpr Pid(const PidGains& gains, const float dt)
      : kp_(gains.kp),
        ki_(gains.ki),
        kd_(gains.kd),
        outputLimit_(gains.outputLimit),
        dt_(dt > 0.0f ? dt : 0.0f),
        invDt_(dt > 0.0f ? 1.0f / dt : 0.0f),
        integralMax_(gains.ki != 0.0f
                         ? gains.integralLimit /
                               (gains.ki < 0.0f ? -gains.ki : gains.ki)
                         : 0.0f) {}

  /// One control step. `external` is the downstream saturation observed
  /// after the previous update().
  float update(float error, PidClamp external = {false, false});

  /// Clears the integrator and the derivative history.
  void reset();

  /// Current integral contribution ki * integral (for tests / diagnostics).
  float integralTerm() const { return ki_ * integral_; }

 private:
  float kp_;
  float ki_;
  float kd_;
  float outputLimit_;
  float dt_;
  float invDt_;            // precomputed 1 / dt
  float integralMax_;      // precomputed integralLimit / |ki| (state units)
  float integral_ = 0.0f;  // integral of the error [err * s]
  float prevError_ = 0.0f;
  bool hasPrev_ = false;
};

}  // namespace micromouse
