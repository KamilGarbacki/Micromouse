#include "app/calibration_tools.h"

#include "config/config.h"
#include "control/ff_fit.h"
#include "hal/log.h"
#include "hal/time.h"

namespace micromouse {

namespace {

namespace cfg = config;
namespace log = hal::log;

constexpr uint32_t kDrainMs = 50;
constexpr uint32_t kRestQuietMs = 300;  // wheel counted as at rest [ms]
constexpr int16_t kRampFailed = -1;
constexpr int16_t kRampAborted = -2;

bool keyPressed() { return log::readChar() >= 0; }

/// Ticks of `encoder` during `ms`; -1 if a key was pressed (abort).
int32_t countTicks(const Encoder& encoder, const uint32_t ms) {
  encoder.reset();
  const uint32_t start = hal::nowMs();
  while (hal::nowMs() - start < ms) {
    if (keyPressed()) return -1;
  }
  return encoder.count();
}

/// Steps the PWM by 1 every `stepMs` from `from` to `to` (inclusive).
/// Up: returns the first PWM of kSelfCalMovingSteps consecutive moving steps
/// (>= kSelfCalMovingTicks each; a single burst can be gearbox backlash).
/// Down: returns the last PWM at which the wheel still moved once it stops
/// (or `to` if it never stops). kRampFailed if the condition is never met
/// (up) or the wheel did not move at `from` (down), kRampAborted on a key.
int16_t ramp(const Motor& motor, const Encoder& encoder, const int16_t from,
             const int16_t to, const uint32_t stepMs, const bool up) {
  uint8_t streak = 0;
  int16_t first = kRampFailed;
  int16_t lastMoving = kRampFailed;
  for (int16_t pwm = from; up ? pwm <= to : pwm >= to;
       pwm = static_cast<int16_t>(pwm + (up ? 1 : -1))) {
    motor.setDuty(pwm);
    const int32_t ticks = countTicks(encoder, stepMs);
    if (ticks < 0) return kRampAborted;
    const bool moving = ticks >= cfg::kSelfCalMovingTicks;
    if (up) {
      if (!moving) {
        streak = 0;
        continue;
      }
      if (streak++ == 0) first = pwm;
      if (streak >= cfg::kSelfCalMovingSteps) return first;
    } else {
      if (!moving) return lastMoving;
      lastMoving = pwm;
    }
  }
  return up ? kRampFailed : lastMoving;
}

void printWheel(const char name, const WheelCalibration& w) {
  log::fmtln(F("%c: static=%1 kinetic=%1 ff=%2+%3*tpp"), name,
             static_cast<double>(w.staticPwm),
             static_cast<double>(w.kineticPwm),
             static_cast<double>(w.ffOffsetPwm),
             static_cast<double>(w.ffSlopePwmPerTpp));
}

/// Self-calibration of one wheel (the other one stays braked).
bool calibrateWheel(const char name, const Motor& motor,
                    const Encoder& encoder, WheelCalibration& out) {
  motor.brake();
  if (countTicks(encoder, kRestQuietMs) < 0) return false;

  // 1. staticPwm: ramp up from rest until the wheel moves.
  const int16_t breakaway =
      ramp(motor, encoder, cfg::kSelfCalStartPwm, cfg::kSelfCalMaxStartPwm,
           cfg::kSelfCalStepMs, true);
  if (breakaway < 0) {
    motor.brake();
    if (breakaway == kRampFailed) log::fmtln(F("%c: no motion"), name);
    return false;
  }

  // 2. Feed-forward line: steady-state speed at levels from the top down.
  const int16_t low =
      static_cast<int16_t>(breakaway + cfg::kSelfCalLowAboveStaticPwm);
  float pwmAt[cfg::kSelfCalLevels];
  float tppAt[cfg::kSelfCalLevels];
  for (uint8_t i = 0; i < cfg::kSelfCalLevels; ++i) {
    const int16_t pwm = static_cast<int16_t>(
        cfg::kSelfCalTopPwm -
        (cfg::kSelfCalTopPwm - low) * i / (cfg::kSelfCalLevels - 1));
    motor.setDuty(pwm);
    if (countTicks(encoder, cfg::kSelfCalSettleMs) < 0) return false;
    const int32_t ticks = countTicks(encoder, cfg::kSelfCalMeasureMs);
    if (ticks < 0) return false;
    pwmAt[i] = static_cast<float>(pwm);
    tppAt[i] = static_cast<float>(ticks) * (cfg::kControlPeriodUs / 1000.0f) /
               static_cast<float>(cfg::kSelfCalMeasureMs);
  }
  const LineFit fit = fitLine(tppAt, pwmAt, cfg::kSelfCalLevels);

  // 3. kineticPwm: from the lowest level ramp down (never above it, never
  // below 1) until the wheel stops.
  const int16_t kinetic =
      ramp(motor, encoder, low, 1, cfg::kSelfCalKineticStepMs, false);
  motor.brake();
  if (kinetic == kRampAborted) return false;

  log::fmtln(F("%c: break=%d stop=%d r2=%3"), name,
             static_cast<int>(breakaway), static_cast<int>(kinetic),
             static_cast<double>(fit.r2));
  if (!fit.valid || fit.r2 < cfg::kSelfCalMinR2 || kinetic < 0) {
    log::fmtln(F("%c: bad fit/stop"), name);
    return false;
  }
  const float staticPwm =
      static_cast<float>(breakaway) + cfg::kSelfCalStaticMarginPwm;
  float kineticPwm =
      static_cast<float>(kinetic) + cfg::kSelfCalKineticMarginPwm;
  if (kineticPwm > staticPwm) kineticPwm = staticPwm;
  out = WheelCalibration{staticPwm, kineticPwm, fit.offset, fit.slope};
  return true;
}

}  // namespace

const __FlashStringHelper* validityName(const calibration::Validity validity) {
  using calibration::Validity;
  switch (validity) {
    case Validity::kValid:
      return F("VALID");
    case Validity::kEmpty:
      return F("EMPTY");
    case Validity::kBadMagic:
      return F("INVALID/OLD");
    case Validity::kBadVersion:
      return F("OLD_VERSION");
    case Validity::kBadCrc:
      return F("BAD_CRC");
    case Validity::kGeometryChanged:
      return F("GEOMETRY_CHANGED");
    case Validity::kOutOfRange:
      return F("OUT_OF_RANGE");
  }
  return F("UNKNOWN");
}

void runEncoderCheck(const Encoder& left, const Encoder& right) {
  constexpr long kExpected = static_cast<long>(cfg::kTicksPerWheelRev + 0.5f);
  log::fmtln(F("ENCODER CHECK: turn each wheel exactly 1 rev by hand (mark "
               "the tyre). Expect %l ticks = ratio 1.00. Ratio ~2: "
               "kEncoderPulsesPerMotorRev x2, ~0.5: /2, one wheel ~0: dead "
               "encoder. Key exits."),
             kExpected);
  left.reset();
  right.reset();
  int32_t last = -1;
  uint32_t lastPrint = hal::nowMs();
  while (!keyPressed()) {
    const uint32_t now = hal::nowMs();
    if (now - lastPrint < cfg::kEncoderCheckPrintMs) continue;
    lastPrint = now;
    const int32_t l = left.count();
    const int32_t r = right.count();
    if (l + r == last) continue;  // both only count up: sum detects change
    last = l + r;
    // Counts, expected ticks per wheel revolution, measured / expected.
    log::fmtln(F("enc L=%l R=%l exp=%l L/exp=%2 R/exp=%2"),
               static_cast<long>(l), static_cast<long>(r), kExpected,
               static_cast<double>(static_cast<float>(l) /
                                   cfg::kTicksPerWheelRev),
               static_cast<double>(static_cast<float>(r) /
                                   cfg::kTicksPerWheelRev));
  }
  log::drainInput(kDrainMs);
}

bool runMotorSelfCalibration(const Motor& leftMotor, const Motor& rightMotor,
                             const Encoder& leftEncoder,
                             const Encoder& rightEncoder,
                             MotorRecord& record) {
  log::fmtln(F("SELF-CAL: LIFT THE ROBOT (wheels spin ~40 s). y=start, any "
               "key aborts"));
  int16_t key = -1;
  while (key < 0) key = log::readChar();
  log::drainInput(kDrainMs);
  if (key != 'y' && key != 'Y') return false;

  MotorRecord updated = record;
  leftMotor.brake();
  rightMotor.brake();
  bool ok = calibrateWheel('L', leftMotor, leftEncoder, updated.wheel[0]);
  leftMotor.brake();
  if (ok) ok = calibrateWheel('R', rightMotor, rightEncoder, updated.wheel[1]);
  rightMotor.brake();
  log::drainInput(kDrainMs);
  if (ok) {
    calibration::seal(updated);
    ok = calibration::validate(updated) == calibration::Validity::kValid;
  }
  if (!ok) {
    log::fmtln(F("SELF-CAL FAILED, nothing stored"));
    return false;
  }
  record = updated;
  log::fmtln(F("SELF-CAL OK (lifted):"));
  printWheel('L', updated.wheel[0]);
  printWheel('R', updated.wheel[1]);
  return true;
}

void printCalibration(const Calibration& cal,
                      const calibration::Validity motor,
                      const calibration::Validity learned) {
  log::fmtln(F("motor record %s, learned record %s"), validityName(motor),
             validityName(learned));
  if (cal.motorsCalibrated) {
    printWheel('L', cal.motor.wheel[0]);
    printWheel('R', cal.motor.wheel[1]);
  }
  const LearnedRecord& r = cal.learned;
  // d = front-offset error delta, j = joint solution in force (else the
  // pitch is provisional / default and d is not applied).
  log::fmtln(F("scale=%3 um/tick n=%u pitch=%1 mm n=%u d=%1 j=%u rows=%u -> "
               "ticks/cell=%l"),
             static_cast<double>(calibration::mmPerTick(cal) * 1000.0f),
             static_cast<unsigned int>(r.scaleSamples),
             static_cast<double>(calibration::cellPitchMm(cal)),
             static_cast<unsigned int>(r.pitchSamples),
             static_cast<double>(r.wallOffsetMm),
             calibration::pitchJoint(cal) ? 1u : 0u,
             static_cast<unsigned int>(learning::pitchRows(r.flags)),
             static_cast<long>(calibration::ticksPerCell(cal)));
  log::fmtln(F("ticks/90=%l n=%u brake fwd=%1 turn=%1"),
             static_cast<long>(calibration::ticksPer90(cal)),
             static_cast<unsigned int>(r.turnSamples),
             static_cast<double>(r.brakeLeadForward),
             static_cast<double>(r.brakeLeadTurn));
}

}  // namespace micromouse
